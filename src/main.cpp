/* ESP32 controller: host, ODrive, battery and IMU run independently. */
#include <Arduino.h>
#include "ODriveArduino.h"
#include "controller_state.h"
#include <Wire.h>
#include "ina226.h"
#include <Adafruit_MPU6050.h>
#include "checked_wire.h"
#include "imu_calibration.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <inttypes.h>

constexpr int LED_PIN = 2;
constexpr int PIN_RESET_ODRIVE = 15;
constexpr uint32_t ODRIVE_PERIOD_MS = 20;
constexpr uint32_t BATTERY_PERIOD_MS = 200;
constexpr uint32_t IMU_PERIOD_MS = 20;
constexpr uint32_t IMU_RETRY_MS = 1000;
constexpr uint32_t IMU_MAX_AGE_US = 100000;
#ifndef MPU6050_ADDRESS
#define MPU6050_ADDRESS 0x68
#endif
constexpr uint8_t IMU_ADDRESS = MPU6050_ADDRESS;
constexpr uint32_t ODRIVE_READ_TIMEOUT_MS = 12;
constexpr size_t MAX_COMMAND_LENGTH = 200;

ODriveArduino odrive(Serial2);
Adafruit_MPU6050 mpu;
CheckedWire sensorWire;
Ina226<CheckedWire> ina226(sensorWire, INA226_I2C_ADDRESS, INA226_SHUNT_RESISTANCE_OHMS);
ImuCalibration imuCalibration;
// Setup owns the IMU before task creation; imuTask owns it afterwards.
bool imuInitialized = false;

struct SensorSnapshot {
  float encoder_left = 0;
  float encoder_right = 0;
  float battery_voltage = 0;
  float battery_current_mA = 0;
  uint32_t sample_sequence = 0;
  uint32_t read_failures = 0;
  bool watchdog_active = true;
  float acceleration[3] = {};
  float angular_velocity[3] = {};
  uint64_t imu_time_us = 0; // Used internally to reject stale IMU data.
  uint32_t imu_sequence = 0;
  uint32_t imu_failures = 0;
  bool imu_valid = false;
};

SensorSnapshot snapshot;
SemaphoreHandle_t snapshotMutex;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t odriveReady;
QueueHandle_t motorQueue;

void processCommand(String command) {
  command.trim();
  digitalWrite(LED_PIN, HIGH);
  if (command == "") {
    Serial.print("\r\n");
  } else if (command == "e") {
    xSemaphoreTake(snapshotMutex, portMAX_DELAY);
    const SensorSnapshot copy = snapshot;
    xSemaphoreGive(snapshotMutex);
    const bool imu_fresh = copy.imu_valid &&
        uint64_t(esp_timer_get_time()) - copy.imu_time_us <= IMU_MAX_AGE_US;
    // Preserve the original protocol layout; debug timing fields are zero.
    Serial.printf("%.6f %.6f %.2f %.2f "
                  "0 0 0 0 0 0 %" PRIu32 " 0 %" PRIu32 " 0 %u "
                  "%.5f %.5f %.5f %.5f %.5f %.5f "
                  "0 %" PRIu32 " %u %" PRIu32 "\r\n",
                  copy.encoder_left, copy.encoder_right, copy.battery_voltage,
                  copy.battery_current_mA, copy.sample_sequence, copy.read_failures,
                  unsigned(copy.watchdog_active),
                  copy.acceleration[0], copy.acceleration[1], copy.acceleration[2],
                  copy.angular_velocity[0], copy.angular_velocity[1], copy.angular_velocity[2],
                  copy.imu_sequence, unsigned(imu_fresh), copy.imu_failures);
  } else if (command.startsWith("m ")) {
    MotorCommand motor;
    if (parseMotorCommand(command.c_str(), millis(), motor) &&
        xQueueOverwrite(motorQueue, &motor) == pdPASS) {
      // Acknowledge queue acceptance; the owner applies it on its next cycle.
      Serial.print("\r\n");
    } else {
      Serial.print("Invalid motor command format\r\n");
    }
  } else {
    Serial.print("Unknown command: ");
    Serial.print(command.c_str());
    Serial.print("\r\n");
  }
  digitalWrite(LED_PIN, LOW);
}

// Called only by the host task. Handle every CR separately, even in a burst.
void serviceHostSerial() {
  static String input;
  static bool discarding = false;
  // Bound each batch so an RX flood still lets this task yield.
  for (size_t count = 0; count < 256 && Serial.available(); ++count) {
    const char c = Serial.read();
    if (c == '\r') {
      if (discarding) Serial.print("Command too long\r\n");
      else processCommand(input);
      input = "";
      discarding = false;
    } else if (c != '\n' && !discarding) {
      if (c == '\0' || input.length() >= MAX_COMMAND_LENGTH) discarding = true;
      else input += c;
    }
  }
}

void hostTask(void *) {
  xSemaphoreTake(odriveReady, portMAX_DELAY);
  Serial.println("ESP32 SBEM controller ready");
  for (;;) {
    serviceHostSerial();
    vTaskDelay(1);
  }
}

// Only the ODrive task invokes this function and ODriveArduino methods.
void activateOdrive() {
  digitalWrite(PIN_RESET_ODRIVE, LOW);
  delay(100);
  digitalWrite(PIN_RESET_ODRIVE, HIGH);
  delay(1000);
  // Keep the existing hardware activation sequence.
  for (int axis = 0; axis < 2; ++axis) {
    for (int attempt = 0; attempt < 500; ++attempt) {
      odrive.run_state(axis, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false);
      delay(10);
    }
  }
}

void runOdriveCycle() {
  static MotorCommandState motors;
  static bool resync = false;
  static uint32_t quiet_since_ms = 0;

  MotorCommand queued;
  if (xQueueReceive(motorQueue, &queued, 0) == pdTRUE) motors.accept(queued);
  const uint32_t now_ms = millis();
  const MotorCommand target = motors.current(now_ms);
  odrive.SetVelocity(0, target.right); // Axis 0 is right, axis 1 is left.
  odrive.SetVelocity(1, target.left);
  Serial2.flush(); // Finish motor TX before starting position-read timeouts.

  // After a bad/partial reply, discard late bytes until the link has been quiet
  // for a whole period. An old reply must not become the next wheel's sample.
  if (resync) {
    for (size_t n = 0; n < 256 && Serial2.available(); ++n) {
      Serial2.read();
      quiet_since_ms = millis();
    }
    if (!Serial2.available() && uint32_t(millis() - quiet_since_ms) >= ODRIVE_PERIOD_MS)
      resync = false;
  }

  float right = 0, left = 0;
  bool complete = false;
  bool failed = false;
  if (!resync) {
    const bool right_ok = odrive.GetPosition(0, right, ODRIVE_READ_TIMEOUT_MS);
    if (right_ok) {
      complete = odrive.GetPosition(1, left, ODRIVE_READ_TIMEOUT_MS);
    }
    if (!complete) {
      failed = true;
      resync = true;
      quiet_since_ms = millis();
    }
  }

  xSemaphoreTake(snapshotMutex, portMAX_DELAY);
  if (complete) {
    snapshot.encoder_right = right;
    snapshot.encoder_left = left;
    ++snapshot.sample_sequence;
  }
  snapshot.watchdog_active = motors.expired(now_ms);
  if (failed) ++snapshot.read_failures;
  xSemaphoreGive(snapshotMutex);
}

// Use an absolute schedule normally. On an overrun, yield and start a new
// schedule instead of spinning through missed releases at high priority.
void waitForNextPeriod(TickType_t &last_wake, TickType_t period) {
  if (TickType_t(xTaskGetTickCount() - last_wake) >= period) {
    vTaskDelay(1);
    last_wake = xTaskGetTickCount();
  } else {
    vTaskDelayUntil(&last_wake, period);
  }
}

void odriveTask(void *) {
  activateOdrive();
  TickType_t last_wake = xTaskGetTickCount();
  bool ready = false;
  for (;;) {
    runOdriveCycle();
    if (!ready) {
      xSemaphoreGive(odriveReady);
      ready = true;
    }
    waitForNextPeriod(last_wake, pdMS_TO_TICKS(ODRIVE_PERIOD_MS));
  }
}

void readBattery() {
  // The battery task owns the filter. Average only collected samples
  // so the existing 50-sample filter does not ramp from zero for 10 s at startup.
  static float readings[50] = {};
  static float total = 0;
  static size_t index = 0, count = 0;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  sensorWire.clearError();
  Ina226Sample sample{};
  const Ina226Status status = ina226.read(sample);
  const bool valid = status == Ina226Status::Ok && !sensorWire.failed();
  xSemaphoreGive(i2cMutex);
  if (!valid) return; // Keep the last good snapshot and filter on invalid reads.
  // Retain the existing supply-voltage estimate (VBUS connected to IN-).
  const float voltage = sample.bus_V + sample.shunt_mV / 1000.0f;
  total -= readings[index];
  readings[index] = voltage;
  total += voltage;
  index = (index + 1) % 50;
  if (count < 50) ++count;

  xSemaphoreTake(snapshotMutex, portMAX_DELAY);
  snapshot.battery_voltage = total / count;
  // used a minus sign in current because when the 
  // charger is connected, the current is negative,
  // so to make it more intuitive, we report the current as positive when charging
  snapshot.battery_current_mA = - sample.current_mA; 
  xSemaphoreGive(snapshotMutex);
}

void batteryTask(void *) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    readBattery();
    waitForNextPeriod(last_wake, pdMS_TO_TICKS(BATTERY_PERIOD_MS));
  }
}

bool initializeImu() {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  sensorWire.clearError();
  bool initialized = mpu.begin(IMU_ADDRESS, &sensorWire);
  if (initialized) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu.setSampleRateDivisor(19); // 1 kHz / (1 + 19) = 50 Hz with DLPF enabled.
  }
  initialized = initialized && !sensorWire.failed();
  xSemaphoreGive(i2cMutex);
  return initialized;
}

bool readImuSample(float acceleration[3], float angular_velocity[3]) {
  sensors_event_t accel{}, gyro{}, temperature{};
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  sensorWire.clearError();
  const bool event_ok = mpu.getEvent(&accel, &gyro, &temperature);
  const bool bus_ok = !sensorWire.failed();
  xSemaphoreGive(i2cMutex);
  if (!event_ok || !bus_ok) return false;
  acceleration[0] = accel.acceleration.x;
  acceleration[1] = accel.acceleration.y;
  acceleration[2] = accel.acceleration.z;
  angular_velocity[0] = gyro.gyro.x;
  angular_velocity[1] = gyro.gyro.y;
  angular_velocity[2] = gyro.gyro.z;
  for (unsigned axis = 0; axis < 3; ++axis)
    if (!std::isfinite(acceleration[axis]) || !std::isfinite(angular_velocity[axis]))
      return false;
  return true;
}

void calibrateImuAtStartup() {
  imuInitialized = initializeImu();
  if (!imuInitialized) {
    Serial.println("MPU6050 calibration skipped: IMU initialization failed");
    return;
  }
  Serial.println("MPU6050 startup calibration: keep robot still and level, sensor Z up");
  delay(1000); // Let the sensor and low-pass filter settle; release I2C meanwhile.
  const uint64_t deadline_us = uint64_t(esp_timer_get_time()) + 10000000;
  for (unsigned n = 0; n < ImuCalibration::SAMPLE_COUNT; ++n) {
    if (uint64_t(esp_timer_get_time()) >= deadline_us) {
      Serial.println("MPU6050 calibration skipped: sampling timed out");
      return;
    }
    float acceleration[3], angular_velocity[3];
    if (!readImuSample(acceleration, angular_velocity)) {
      imuInitialized = false; // The runtime task will retry initialization.
      Serial.println("MPU6050 calibration skipped: invalid sample or I2C failure");
      return;
    }
    if (uint64_t(esp_timer_get_time()) >= deadline_us) {
      Serial.println("MPU6050 calibration skipped: sampling timed out");
      return;
    }
    if (!imuCalibration.add(acceleration, angular_velocity)) {
      Serial.println("MPU6050 calibration skipped: excessive rotation or invalid sample");
      return;
    }
    delay(IMU_PERIOD_MS);
  }
  if (imuCalibration.finish())
    Serial.println("MPU6050 calibration complete: accel and gyro offsets applied, gravity retained");
  else
    Serial.println("MPU6050 calibration skipped: unstable samples or unexpected level pose");
}

void runImuCycle() {
  // Runtime initialization/reconnection never recalibrates a possibly moving robot.
  static uint64_t retry_after_us = 0;
  const uint64_t now_us = esp_timer_get_time();
  if (!imuInitialized && now_us < retry_after_us) return;

  float acceleration[3] = {}, angular_velocity[3] = {};
  if (!imuInitialized) {
    imuInitialized = initializeImu();
    retry_after_us = uint64_t(esp_timer_get_time()) + IMU_RETRY_MS * 1000;
    if (imuInitialized) return; // First sample on the next scheduled cycle.
  } else {
    imuInitialized = readImuSample(acceleration, angular_velocity);
    if (imuInitialized)
      imuCalibration.apply(acceleration, angular_velocity);
    else
      retry_after_us = uint64_t(esp_timer_get_time()) + IMU_RETRY_MS * 1000;
  }

  const uint64_t completed_us = esp_timer_get_time();
  xSemaphoreTake(snapshotMutex, portMAX_DELAY);
  snapshot.imu_valid = imuInitialized;
  if (imuInitialized) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      snapshot.acceleration[axis] = acceleration[axis];
      snapshot.angular_velocity[axis] = angular_velocity[axis];
    }
    snapshot.imu_time_us = completed_us;
    ++snapshot.imu_sequence;
  } else {
    ++snapshot.imu_failures;
  }
  xSemaphoreGive(snapshotMutex);
}

void imuTask(void *) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    runImuCycle();
    waitForNextPeriod(last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
  }
}

void startupFailed(const char *reason) {
  Serial.println(reason);
  // ODrive task is created last: no activation occurs on allocation failure.
  for (;;) delay(1000);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_RESET_ODRIVE, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(PIN_RESET_ODRIVE, LOW); // Hold reset throughout startup calibration.
  
  snapshotMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  odriveReady = xSemaphoreCreateBinary();
  motorQueue = xQueueCreate(1, sizeof(MotorCommand));
  if (!snapshotMutex || !i2cMutex || !odriveReady || !motorQueue)
    startupFailed("Controller allocation failed");
  sensorWire.begin(21, 22);
  sensorWire.setTimeOut(10);
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  sensorWire.clearError();
  const bool batteryReady = ina226.begin() == Ina226Status::Ok && !sensorWire.failed();
  xSemaphoreGive(i2cMutex);
  if (!batteryReady)
    Serial.println("INA226 initialization failed; continuing without battery monitoring");
  calibrateImuAtStartup();
  if (xTaskCreate(hostTask, "host", 4096, nullptr, 2, nullptr) != pdPASS)
    startupFailed("Host task creation failed");
  if (batteryReady &&
      xTaskCreate(batteryTask, "battery", 3072, nullptr, 1, nullptr) != pdPASS)
    startupFailed("Battery task creation failed");
  if (xTaskCreate(imuTask, "imu", 4096, nullptr, 1, nullptr) != pdPASS)
    startupFailed("IMU task creation failed");
  if (xTaskCreate(odriveTask, "odrive", 4096, nullptr, 3, nullptr) != pdPASS)
    startupFailed("ODrive task creation failed");
}

void loop() {
  // Arduino's loop task has no serialEvent handler; the host task owns Serial.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
