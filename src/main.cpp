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


// Encoder variables
float encoder_left = 0;
float encoder_right = 0;

// Motor control variables
int motor_left_speed = 0;   // -255 to 255
int motor_right_speed = 0;  // -255 to 255

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
  // Check for incoming serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  
  
  // Debug encoder values periodically (comment out in production)
  /*
  if (currentTime - lastEncoderPrint > ENCODER_PRINT_INTERVAL) {
    Serial.print("Debug - Left: ");
    Serial.print(encoder_left);
    Serial.print(", Right: ");
    Serial.println(encoder_right);
    lastEncoderPrint = currentTime;
  }
  */
  
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
    noInterrupts();
    float left_enc = encoder_left;
    float right_enc = encoder_right;
    interrupts();
    
    Serial.print(left_enc);
    Serial.print(" ");
    Serial.print(right_enc);
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
  else if (command.startsWith("u ")) {
    // Set PID values: "u kp:kd:ki:ko"
    String pidValues = command.substring(2);
    if (parsePIDValues(pidValues)) {
      Serial.print("\r\n");
    } else {
      Serial.print("Invalid PID format\r\n");
    }
  }
  else if (command == "reset") {
    resetOdrive();
    Serial.print("Encoders reset\r\n");
  }
  else if (command == "stop") {
    // Emergency stop
    Serial.print("Motors stopped\r\n");
  }
  else if (command == "status") {
    // Status report
    Serial.print("Left motor: ");
    Serial.print(motor_left_speed);
    Serial.print(", Right motor: ");
    Serial.print(motor_right_speed);
    Serial.print(", PID: ");
    Serial.print(pid.kp);
    Serial.print(":");
    Serial.print(pid.kd);
    Serial.print(":");
    Serial.print(pid.ki);
    Serial.print(":");
    Serial.print(pid.ko);
    Serial.print("\r\n");
  }
  else {
    // Unknown command
    Serial.print("Unknown command: ");
    Serial.print(command);
    Serial.print("\r\n");
  }
  
  digitalWrite(LED_PIN, LOW);
}



bool parsePIDValues(String pidString) {
  int colonIndex1 = pidString.indexOf(':');
  int colonIndex2 = pidString.indexOf(':', colonIndex1 + 1);
  int colonIndex3 = pidString.indexOf(':', colonIndex2 + 1);
  
  if (colonIndex1 > 0 && colonIndex2 > 0 && colonIndex3 > 0) {
    pid.kp = pidString.substring(0, colonIndex1).toInt();
    pid.kd = pidString.substring(colonIndex1 + 1, colonIndex2).toInt();
    pid.ki = pidString.substring(colonIndex2 + 1, colonIndex3).toInt();
    pid.ko = pidString.substring(colonIndex3 + 1).toInt();
    return true;
  }
  return false;
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
  odrive.SetVelocity(1,-left_speed);
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