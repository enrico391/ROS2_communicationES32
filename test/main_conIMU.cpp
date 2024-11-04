//MAIN with state_machine 
#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>


#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>


#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <example_interfaces/srv/set_bool.h>
#include <rmw_microros/rmw_microros.h>

//for Odrive communication
#include "ODriveArduino.h"

//for mpu6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LED_PIN 13
#define BATTERY_PIN 35

#define RESISTOR_1 22000
#define RESISTOR_2 5100


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;


bool micro_ros_init_successful;

float vel_motor0_linear;
float vel_motor1_linear;

bool restartFlag = false;


//voltage and current battery
float voltage;
float current;

float percentage; //percentage battery
int power_supply_status; //status : 1 charging    3 not charging   4 full 

const int numReadings = 20;
float readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
float total = 0;              // the running total
float averageV = 0;            // the average



//SUBSCRIBER
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

//PUBLISHER battery status
rcl_publisher_t publisher_battery;
sensor_msgs__msg__BatteryState battery;

//PUBLISHER R wheel
rcl_publisher_t publisher_l;
std_msgs__msg__Float32 wheel_l;

//PUBLISHER L wheel
rcl_publisher_t publisher_r;
std_msgs__msg__Float32 wheel_r;

//PUBLISHER imu
rcl_publisher_t publisher_imu;
sensor_msgs__msg__Imu imu_msg;

//services values
rcl_service_t service_restart;



example_interfaces__srv__SetBool_Request req;
example_interfaces__srv__SetBool_Response res;

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print& obj, T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print& obj, float arg) { obj.print(arg, 4); return obj; }

//mpu object
Adafruit_MPU6050 mpu;


// ODrive object
ODriveArduino odrive(Serial2);

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//
//void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
//{
//  (void) last_call_time;
//  if (timer != NULL) {
//    wheel_l.data = -1*odrive.GetPosition(1);//(int16_t)odrive.GetPosition(1);
//    wheel_r.data = 1*odrive.GetPosition(0);//(int16_t)odrive.GetPosition(0);
//    
//    rcl_publish(&publisher_l,&wheel_l, NULL);
//    rcl_publish(&publisher_r,&wheel_r, NULL);
//  }
//}


//function to convert euler to quaternion
const void euler_to_quat(float x, float y, float z, double* q) {
    float c1 = cos((y*3.14/180.0)/2);
    float c2 = cos((z*3.14/180.0)/2);
    float c3 = cos((x*3.14/180.0)/2);

    float s1 = sin((y*3.14/180.0)/2);
    float s2 = sin((z*3.14/180.0)/2);
    float s3 = sin((x*3.14/180.0)/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}


float map_percentage(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void publishImu(){
  double q[4];

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //euler_to_quat(g.gyro.x, g.gyro.y, g.gyro.z, q);
  char* content_frame_id = "imu_frame";
  rcl_time_point_value_t now;



  imu_msg.header.frame_id.data = "imu";
  imu_msg.header.frame_id.size = 3;

  imu_msg.header.stamp.sec = rmw_uros_epoch_millis() * 1000;
  imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();;

  imu_msg.angular_velocity.x = g.gyro.x;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;
  
  rcl_publish(&publisher_imu, &imu_msg , NULL);
}


//function that publish battery status 
void publishBattery(){
  //calculate voltage from voltage partitor
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

  current = averageV / RESISTOR_2;

  //choose status battery in according with voltage level
  if(voltage  > 14.5){
    power_supply_status = 4;
  } 
  if(voltage <= 14.5 && voltage > 13.3 ){
    power_supply_status = 1;
  }
  if(voltage <= 13.3){
    power_supply_status = 3;
  }

  //map the voltage to percentage battery TODO
  percentage = map_percentage(voltage,12.5,14.5,0.0,1.0);

  battery.power_supply_status = power_supply_status;
  battery.percentage = percentage;
  battery.voltage = averageV;
  battery.current = current;

  
  rcl_publish(&publisher_battery, &battery , NULL);
}



//function 
void publishPosition(){
  
  wheel_l.data = -1*odrive.GetPosition(1);//(int16_t)odrive.GetPosition(1);
  wheel_r.data = 1*odrive.GetPosition(0);//(int16_t)odrive.GetPosition(0);

  rcl_publish(&publisher_l,&wheel_l, NULL);
  rcl_publish(&publisher_r,&wheel_r, NULL);

}

void activeOdrive(){
  // Setup ODrive
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  
  odrive.run_state(1, requested_state, false); // don't wait 
  odrive.run_state(0, requested_state, false); // don't wait 
}

void disableOdrive(){
  // Setup ODrive
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_UNDEFINED;
  
  odrive.run_state(1, requested_state, false); // don't wait 
  odrive.run_state(0, requested_state, false); // don't wait 
}


//function to check current mode in Odrive
bool checkOdriveMode(){
  float getMode = odrive.GetParameter(0,ODriveArduino::PARAM_INT_CONTROL_MODE);
}


//CALLBACK FOR SUBSCRIBE NODE
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  //right motors --> motor0
  //left motors --> motor1
 
  //right linear
  vel_motor0_linear = msg->linear.x + ((0.285/2) * msg->angular.z); 
  
  //left linear
  vel_motor1_linear = msg->linear.x - ((0.285/2) * msg->angular.z);

  // set wheels' speed
  odrive.SetVelocity(0,vel_motor0_linear);
  odrive.SetVelocity(1,-vel_motor1_linear); // invertire segno forse

}


//callback for service that restart esp32
void service_callback(const void * req, void * res){
  example_interfaces__srv__SetBool_Request * req_in = (example_interfaces__srv__SetBool_Request *) req;
  example_interfaces__srv__SetBool_Response * res_in = (example_interfaces__srv__SetBool_Response *) res;

  //printf("");
  if(req_in->data){
    
    res_in->success = true;
    
    //TO DO (make responso works)
    res_in->message.size = 20;
    res_in->message.capacity = 21;
    char * status = "Restart....";
    res_in->message.data = status;

    restartFlag = true;
    //restartESP();
  }
}


// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber cmd vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  //create publisher left wheel
  RCCHECK(rclc_publisher_init_default(
    &publisher_l,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "lwheel"));

  //create publisher right wheel
    RCCHECK(rclc_publisher_init_default(
    &publisher_r,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rwheel"));

  // create publisher imu
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"));


  //create publisher battery
  RCCHECK(rclc_publisher_init_default(
    &publisher_battery,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "battery_state"));

  // create timer,
  //const unsigned int timer_timeout = 1000;
  //RCCHECK(rclc_timer_init_default(
  //  &timer,
  //  &support,
  //  RCL_MS_TO_NS(timer_timeout),
  //  timer_callback));

  // create service
  RCCHECK(rclc_service_init_default(&service_restart, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, SetBool), "/restartESP"));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  //RCCHECK(rclc_executor_add_timer(&executor, &timer));
  

  //add subscriber
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  //add service
  RCCHECK(rclc_executor_add_service(&executor, &service_restart, &req, &res, service_callback));

  return true;
}


void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher_battery, &node);
  rcl_publisher_fini(&publisher_l, &node);
  rcl_publisher_fini(&publisher_r, &node);
  rcl_publisher_fini(&publisher_imu, &node);
  rcl_subscription_fini(&subscriber,&node);
  rcl_service_fini(&service_restart,&node);
  //rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports();
  //pinMode(LED_PIN, OUTPUT);
  //activeOdrive();
  state = WAITING_AGENT;

  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }


  //set pinmode for pin that reads battery voltage
  pinMode(BATTERY_PIN, INPUT);


  //setup for mpu6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpu.begin();
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }else{
        activeOdrive();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      disableOdrive();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    publishPosition();
    publishImu();
    publishBattery();

    if(restartFlag){
      ESP.restart();
    }
  }
}




