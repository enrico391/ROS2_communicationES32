/*
 * SBEM ESP32 Controller with MicroROS Support
 * 
 * This ESP32 script communicates with the ROS2 hardware interface
 * and can also run MicroROS for direct ROS2 communication.
 * 
 * Features:
 * - Serial communication with ROS2 hardware interface
 * - Encoder reading with quadrature decoding
 * - Motor control with PWM and direction
 * - PID control support

*/

//for Odrive communication
#include "ODriveArduino.h"


#define LED_PIN         2   // Built-in LED
#define PIN_RESET_ODRIVE 15
#define BATTERY_PIN 35
#define RESISTOR_1 22000
#define RESISTOR_2 5100

// ODrive object
ODriveArduino odrive(Serial2);


// declaration of functions
void processCommand(String command);
void setMotorSpeeds(float left_speed, float right_speed);
bool parsePIDValues(String pidString);
void activeOdrive();
void resetOdrive();
void led_blink();
void encoders_read();
void read_battery_voltage();


// motor 0 is right
// motor 1 is left

// Encoder variables
float encoder_left = 0;
float encoder_right = 0;

// Motor control variables
int motor_left_speed = 0;   // -255 to 255
int motor_right_speed = 0;  // -255 to 255

// variable to manage battery reading
float voltage = 0;
float current = 0;
float averageV = 0;
float total = 0;              // the running total
int readIndex = 0;          // the index of the current reading
const int numReadings = 20;
float readings[numReadings];  // the readings from the analog input


// PID variables
struct PIDParams {
  int kp = 100;
  int kd = 10;
  int ki = 5;
  int ko = 50;
} pid;

// Communication variables
String inputString = "";
boolean stringComplete = false;

// Timing variables
unsigned long lastHeartbeat = 0;
unsigned long lastEncoderPrint = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;  // 1 second
const unsigned long ENCODER_PRINT_INTERVAL = 100;  // 100ms


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
  //set pinmode Odrive HIGH to enable it
  digitalWrite(PIN_RESET_ODRIVE, HIGH);
  
  // Reserve string buffer
  inputString.reserve(200);
  
  digitalWrite(LED_PIN, LOW);  // Turn off LED after setup

  // Initialize ODrive
  int i = 0;
  while(i < 500){
    activeOdrive();
    i++;
  }
  

  
  // Send ready message
  Serial.println("ESP32 SBEM controller ready");
  delay(100);
}

void loop() {
  encoders_read();  // Read encoders
  read_battery_voltage(); // Read battery voltage
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
    led_blink();
    // Read encoder values
    //noInterrupts();
    //float left_enc = encoder_left;
    //float right_enc = encoder_right;
    //interrupts();
    // string will be like: "left_enc right_enc battery_voltage"
    Serial.print(encoder_left);
    Serial.print(" ");
    Serial.print(encoder_right);
    Serial.print(" ");
    Serial.print(averageV);//Serial.print(averageV);
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


void read_battery_voltage() {
  voltage = analogRead(BATTERY_PIN);
  voltage = (voltage / 4095 ) * 3.45; // 3.3 but with 3.6 more real value

  //voltage partitor
  current = voltage / RESISTOR_2;
  voltage = current * (RESISTOR_1 + RESISTOR_2);
    
  //AVERAGE VOLTAGE
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = voltage;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average voltage and current:
  averageV = total / numReadings;
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
  digitalWrite(PIN_RESET_ODRIVE, HIGH);

  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  
  odrive.run_state(1, requested_state, false); // don't wait 
  odrive.run_state(0, requested_state, false); // don't wait 
}

void resetOdrive(){
  digitalWrite(PIN_RESET_ODRIVE, LOW);
}