/*
 * SBEM ESP32 Controller for ROS2 control
 *
 * Features:
 * - Serial communication with ROS2 hardware interface
 * - Encoder reading with quadrature decoding
 * - Motor control with PWM and direction
 * - PID control support

*/

//for Odrive communication
#include "ODriveArduino.h"
#include <Wire.h>
#include <Adafruit_INA219.h>



#define LED_PIN         2   // Built-in LED
#define PIN_RESET_ODRIVE 15
#define BATTERY_PIN 35

const int RESISTOR_1{28500};
const int RESISTOR_2{5100};

// ODrive object
ODriveArduino odrive(Serial2);

// INA219 sensor object
Adafruit_INA219 ina219;


// declaration of functions
void processCommand(String command);
void setMotorSpeeds(float left_speed, float right_speed);
void activeOdrive();
void resetOdrive();
void led_blink();
void encoders_read();
//void read_battery_voltage();
void read_battery_voltage_ina219();


// motor 0 is right
// motor 1 is left

// Encoder variables
float encoder_left = 0;
float encoder_right = 0;

// variable to manage battery reading
float voltage = 0;
float current = 0;
float averageV = 0;
double average_current = 0.0;
float total = 0.0; //total voltage
int readIndex = 0;          // the index of the current reading
const int numReadings = 50;
float voltage_readings[numReadings];  // the readings from the analog input

// variables for battery voltage reading using INA219
float shuntvoltage = 0;
float loadvoltage = 0;
float current_mA = 0;
float busvoltage = 0;

// Communication variables
String inputString = "";
boolean stringComplete = false;

// Timing variables
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;  // 1 second


void setup() {
  // Initialize serial communication
  Serial.begin(115200);  // Higher baud rate for ESP32
  
  // serial for ODrive communication
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn on LED during setup
  
  //set pinMode for reset odrive
  pinMode(PIN_RESET_ODRIVE, OUTPUT);

  // pin mode for battery voltage reading
  pinMode(BATTERY_PIN, INPUT);
  
  // Reserve string buffer
  inputString.reserve(200);
  
  digitalWrite(LED_PIN, LOW);  // Turn off LED after setup


  // initialize ina219 sensor
  if (! ina219.begin()) {
    while (1) { delay(10); }
  }

  // Initialize ODrive
  activeOdrive();
  
  // Send ready message
  Serial.println("ESP32 SBEM controller ready");
  delay(100);
}

void loop() {
  encoders_read();  // Read encoders
  read_battery_voltage_ina219(); // Read battery voltage

  // Check for incoming serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Small delay to prevent overwhelming the system
  delay(1);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\r') {
      stringComplete = true;
    } else if (inChar != '\n') {  // Ignore line feeds
      inputString += inChar;
    }
  }
}

void processCommand(String command) {
  command.trim();
  
  // Blink LED on command reception
  digitalWrite(LED_PIN, HIGH);
  
  if (command == "") {
    // Empty command - heartbeat response
    Serial.print("\r\n");
  }
  else if (command == "e") {
    //led_blink();
    
    // Send encoder values and battery voltage
    Serial.print(encoder_left);
    Serial.print(" ");
    Serial.print(encoder_right);
    Serial.print(" ");
    Serial.print(averageV);//Serial.print(averageV);
    Serial.print(" ");
    Serial.print(average_current);
    Serial.print("\r\n");
  }
  else if (command.startsWith("m ")) {
    // Set motor values: "m left_speed right_speed"
    int spaceIndex = command.indexOf(' ', 2);
    if (spaceIndex > 0) {
      float left_speed = command.substring(2, spaceIndex).toFloat();
      float right_speed = command.substring(spaceIndex + 1).toFloat();
      
      setMotorSpeeds(left_speed, right_speed);
      Serial.print("\r\n");
    } else {
      Serial.print("Invalid motor command format\r\n");
    }
  }
  else {
    // Unknown command
    Serial.print("Unknown command: ");
    Serial.print(command);
    Serial.print("\r\n");
  }
  
  digitalWrite(LED_PIN, LOW);
}


// void read_battery_voltage() {
//   constexpr float ADC_MAX = 4095.0f;
//   constexpr float VREF = 3.3f; // calibrated ADC reference
//   // read ADC and convert to measured ADC voltage
//   float raw = static_cast<float>(analogRead(BATTERY_PIN));
//   float v_adc = (raw / ADC_MAX) * VREF;

//   // compute actual battery voltage using resistor divider
//   float v_bat = v_adc * (static_cast<float>(RESISTOR_1 + RESISTOR_2) / RESISTOR_2);

//   // update circular buffer / running total for moving average
//   total -= voltage_readings[readIndex];
//   voltage_readings[readIndex] = v_bat;
//   total += voltage_readings[readIndex];

//   readIndex = (readIndex + 1) % numReadings;

//   // averaged values
//   averageV = total / numReadings;
//   // current through the divider (A). Multiply by 1000 for mA if desired.
//   average_current = averageV / static_cast<double>(RESISTOR_1 + RESISTOR_2);
// }

void read_battery_voltage_ina219() {
  
  // read voltage and current from INA219
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  loadvoltage = ina219.getBusVoltage_V() + (shuntvoltage / 1000.0f); // Convert shunt voltage to volts and add to bus voltage
  current_mA = ina219.getCurrent_mA();

  // update circular buffer / running total for moving average
  total -= voltage_readings[readIndex];
  voltage_readings[readIndex] = loadvoltage;
  total += voltage_readings[readIndex];

  readIndex = (readIndex + 1) % numReadings;

  // averaged values
  averageV = total / numReadings;
  average_current = current_mA;
}


void led_blink() {
  // Heartbeat LED blink
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeat > HEARTBEAT_INTERVAL) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastHeartbeat = currentTime;
  }
}


void setMotorSpeeds(float left_speed, float right_speed) {
  // set wheels' speed
  odrive.SetVelocity(0,right_speed);
  odrive.SetVelocity(1,left_speed);
}


// Interrupt service routine for left encoder (quadrature decoding)
void encoders_read() {
  encoder_right = 1*odrive.GetPosition(0);//(int16_t)odrive.GetPosition(0);
  encoder_left = 1*odrive.GetPosition(1);//(int16_t)odrive.GetPosition(1);
}


void activeOdrive(){
  // Setup ODrive
  //enable odrive
  digitalWrite(PIN_RESET_ODRIVE, LOW);
  delay(100);
  digitalWrite(PIN_RESET_ODRIVE, HIGH);
  delay(1000); // Wait for ODrive to boot up


  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  
  int c = 0;
  while (c++ < 500) {
    odrive.run_state(0, requested_state, false); // don't wait 
    delay(10);
  }

  c = 0;
  while (c++ < 500) {
    odrive.run_state(1, requested_state, false); // don't wait 
    delay(10);
  }
}

void resetOdrive(){
  digitalWrite(PIN_RESET_ODRIVE, LOW);
}