/* ESP32 controller: host, ODrive and battery acquisition run independently. */
#include <Arduino.h>
#include "ODriveArduino.h"
#include "controller_state.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
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
constexpr uint32_t ODRIVE_READ_TIMEOUT_MS = 12;
constexpr size_t MAX_COMMAND_LENGTH = 200;

ODriveArduino odrive(Serial2);
Adafruit_INA219 ina219;

struct SensorSnapshot {
  float encoder_left = 0;
  float encoder_right = 0;
  float battery_voltage = 0;
  float battery_current_mA = 0;
  uint32_t right_read_us = 0;
  uint32_t left_read_us = 0;
  uint32_t task_period_us = 0;
  uint32_t task_work_us = 0;
  uint32_t motor_write_us = 0;
  uint64_t sample_time_us = 0; // Completion of the older (right) wheel read.
  uint32_t sample_interval_us = 0;
  uint32_t sample_sequence = 0;
  uint32_t overruns = 0;
  uint32_t read_failures = 0;
  bool wheels_valid = false;
  bool watchdog_active = true;
};

SensorSnapshot snapshot;
SemaphoreHandle_t snapshotMutex;
SemaphoreHandle_t odriveReady;
QueueHandle_t motorQueue;

uint32_t saturatedMicroseconds(uint64_t elapsed_us) {
  return elapsed_us >= UINT32_MAX ? UINT32_MAX : uint32_t(elapsed_us);
}

void processCommand(String command) {
  command.trim();
  digitalWrite(LED_PIN, HIGH);
  if (command == "") {
    Serial.print("\r\n");
  } else if (command == "e") {
    xSemaphoreTake(snapshotMutex, portMAX_DELAY);
    const SensorSnapshot copy = snapshot;
    xSemaphoreGive(snapshotMutex);
    const uint32_t age_us = copy.wheels_valid
        ? saturatedMicroseconds(esp_timer_get_time() - copy.sample_time_us) : UINT32_MAX;
    // First four values and the following two read timings retain their order.
    Serial.printf("%.2f %.2f %.2f %.2f "
                  "%" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " "
                  "%" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %u\r\n",
                  copy.encoder_left, copy.encoder_right, copy.battery_voltage,
                  copy.battery_current_mA, copy.right_read_us, copy.left_read_us,
                  copy.task_period_us, copy.task_work_us, copy.motor_write_us,
                  age_us, copy.sample_sequence, copy.overruns, copy.read_failures,
                  copy.sample_interval_us, unsigned(copy.watchdog_active));
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
  static uint32_t previous_start_us = 0;
  static bool have_previous_start = false;
  static bool resync = false;
  static uint32_t quiet_since_ms = 0;

  const uint32_t started_us = micros();
  const uint32_t period_us = have_previous_start
      ? uint32_t(started_us - previous_start_us) : 0;
  previous_start_us = started_us;
  have_previous_start = true;

  MotorCommand queued;
  if (xQueueReceive(motorQueue, &queued, 0) == pdTRUE) motors.accept(queued);
  const uint32_t now_ms = millis();
  const MotorCommand target = motors.current(now_ms);
  const uint32_t motor_start_us = micros();
  odrive.SetVelocity(0, target.right); // Axis 0 is right, axis 1 is left.
  odrive.SetVelocity(1, target.left);
  Serial2.flush(); // Include physical TX completion in motor-write timing.
  const uint32_t motor_us = micros() - motor_start_us;

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
  uint32_t right_us = 0, left_us = 0;
  uint64_t right_done_us = 0;
  bool complete = false;
  bool failed = false;
  if (!resync) {
    const uint32_t right_start_us = micros();
    const bool right_ok = odrive.GetPosition(0, right, ODRIVE_READ_TIMEOUT_MS);
    right_us = micros() - right_start_us;
    right_done_us = esp_timer_get_time();
    if (right_ok) {
      const uint32_t left_start_us = micros();
      complete = odrive.GetPosition(1, left, ODRIVE_READ_TIMEOUT_MS);
      left_us = micros() - left_start_us;
    }
    if (!complete) {
      failed = true;
      resync = true;
      quiet_since_ms = millis();
    }
  }

  const uint32_t work_us = micros() - started_us;
  xSemaphoreTake(snapshotMutex, portMAX_DELAY);
  if (complete) {
    snapshot.sample_interval_us = snapshot.wheels_valid
        ? saturatedMicroseconds(right_done_us - snapshot.sample_time_us) : 0;
    snapshot.encoder_right = right;
    snapshot.encoder_left = left;
    snapshot.sample_time_us = right_done_us;
    ++snapshot.sample_sequence;
    snapshot.wheels_valid = true;
  }
  snapshot.right_read_us = right_us;
  snapshot.left_read_us = left_us;
  snapshot.task_period_us = period_us;
  snapshot.task_work_us = work_us;
  snapshot.motor_write_us = motor_us;
  snapshot.watchdog_active = motors.expired(now_ms);
  if (failed) ++snapshot.read_failures;
  if (work_us > ODRIVE_PERIOD_MS * 1000) ++snapshot.overruns;
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
  // Battery task alone owns I2C and the filter. Average only collected samples
  // so the existing 50-sample filter does not ramp from zero for 10 s at startup.
  static float readings[50] = {};
  static float total = 0;
  static size_t index = 0, count = 0;
  const float shunt_mV = ina219.getShuntVoltage_mV();
  const float voltage = ina219.getBusVoltage_V() + shunt_mV / 1000.0f;
  const float current_mA = ina219.getCurrent_mA();
  total -= readings[index];
  readings[index] = voltage;
  total += voltage;
  index = (index + 1) % 50;
  if (count < 50) ++count;

  xSemaphoreTake(snapshotMutex, portMAX_DELAY);
  snapshot.battery_voltage = total / count;
  snapshot.battery_current_mA = current_mA;
  xSemaphoreGive(snapshotMutex);
}

void batteryTask(void *) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    readBattery();
    waitForNextPeriod(last_wake, pdMS_TO_TICKS(BATTERY_PERIOD_MS));
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
  
  if (!ina219.begin()) startupFailed("INA219 initialization failed");

  snapshotMutex = xSemaphoreCreateMutex();
  odriveReady = xSemaphoreCreateBinary();
  motorQueue = xQueueCreate(1, sizeof(MotorCommand));
  if (!snapshotMutex || !odriveReady || !motorQueue)
    startupFailed("Controller allocation failed");
  if (xTaskCreate(hostTask, "host", 4096, nullptr, 2, nullptr) != pdPASS)
    startupFailed("Host task creation failed");
  if (xTaskCreate(batteryTask, "battery", 3072, nullptr, 1, nullptr) != pdPASS)
    startupFailed("Battery task creation failed");
  if (xTaskCreate(odriveTask, "odrive", 4096, nullptr, 3, nullptr) != pdPASS)
    startupFailed("ODrive task creation failed");
}

void loop() {
  // Arduino's loop task has no serialEvent handler; the host task owns Serial.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
