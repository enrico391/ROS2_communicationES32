// Standalone sensor diagnostics. Build with: pio run -e sensor-debug
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include "checked_wire.h"

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint8_t INA_I2C_ADDRESS = 0x40;

// One loop owns the bus: no FreeRTOS tasks, mutexes, ROS protocol or motor UART.
CheckedWire sensorBus;
Adafruit_INA219 ina219(INA_I2C_ADDRESS);
Adafruit_MPU6050 mpu;
bool inaReady = false;
bool mpuReady = false;
uint8_t mpuAddress = 0x68;

bool responds(uint8_t address) {
  sensorBus.beginTransmission(address);
  return sensorBus.endTransmission() == 0;
}

void scanAndInitialize() {
  Serial.println("\nScanning I2C (SDA=21, SCL=22, 100 kHz)...");
  unsigned found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    if (responds(address)) {
      Serial.printf("  Found device at 0x%02X\n", unsigned(address));
      ++found;
    }
  }
  if (found == 0) Serial.println("  No devices found. Check power, GND, SDA and SCL.");

  sensorBus.clearError();
  inaReady = ina219.begin(&sensorBus);
  inaReady = inaReady && !sensorBus.failed();
  Serial.printf("INA219 at 0x%02X: %s\n", unsigned(INA_I2C_ADDRESS),
                inaReady ? "initialized" : "INITIALIZATION FAILED");

  // AD0 low is 0x68; AD0 high is 0x69. Check both automatically.
  mpuAddress = responds(0x68) ? 0x68 : 0x69;
  sensorBus.clearError();
  mpuReady = mpu.begin(mpuAddress, &sensorBus);
  if (mpuReady) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  mpuReady = mpuReady && !sensorBus.failed();
  Serial.printf("MPU6050 at 0x%02X: %s\n", unsigned(mpuAddress),
                mpuReady ? "initialized" : "INITIALIZATION FAILED");
  Serial.println("Readings every 500 ms. Send 'r' to rescan and initialize again.\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nINA219 + Adafruit MPU6050 sensor debug");
  if (!sensorBus.begin(SDA_PIN, SCL_PIN, 100000)) {
    Serial.println("I2C bus initialization FAILED");
    for (;;) delay(1000);
  }
  sensorBus.setTimeOut(10);
  scanAndInitialize();
}

void loop() {
  while (Serial.available()) {
    const char command = Serial.read();
    if (command == 'r' || command == 'R') scanAndInitialize();
  }

  if (inaReady) {
    sensorBus.clearError();
    const float shunt_mV = ina219.getShuntVoltage_mV();
    const float bus_V = ina219.getBusVoltage_V();
    const float current_mA = ina219.getCurrent_mA();
    if (sensorBus.failed()) {
      Serial.println("INA219: I2C READ FAILED (send 'r' to rescan)");
    } else {
      Serial.printf("INA219: bus=%.3f V  shunt=%.3f mV  load=%.3f V  current=%.3f mA\n",
                    bus_V, shunt_mV, bus_V + shunt_mV / 1000.0f, current_mA);
    }
  } else {
    Serial.println("INA219: unavailable (send 'r' to retry initialization)");
  }

  if (mpuReady) {
    sensors_event_t acceleration{}, gyro{}, temperature{};
    sensorBus.clearError();
    const bool ok = mpu.getEvent(&acceleration, &gyro, &temperature);
    if (!ok || sensorBus.failed()) {
      Serial.println("MPU6050: I2C READ FAILED (send 'r' to rescan)");
    } else {
      Serial.printf("MPU6050: accel [m/s^2] X=%.4f Y=%.4f Z=%.4f\n",
                    acceleration.acceleration.x, acceleration.acceleration.y,
                    acceleration.acceleration.z);
      Serial.printf("         gyro [rad/s] X=%.5f Y=%.5f Z=%.5f  temp=%.2f C\n",
                    gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, temperature.temperature);
    }
  } else {
    Serial.println("MPU6050: unavailable (send 'r' to retry initialization)");
  }
  Serial.println();
  delay(500);
}
