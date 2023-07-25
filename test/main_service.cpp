// MAIN funzionante con servizio

#include <Arduino.h>
//#include <micro_ros_platformio.h>
#include "ODriveArduino.h"
#include <micro_ros_arduino.h>

//for mpu6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <example_interfaces/srv/set_bool.h>




// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print& obj, T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print& obj, float arg) { obj.print(arg, 4); return obj; }

//mpu object
Adafruit_MPU6050 mpu;


// ODrive object
ODriveArduino odrive(Serial2);

float vel_motor0_linear;
float vel_motor1_linear;
float vel_motor0_angular;
float vel_motor1_angular;
float vel_motor0;
float vel_motor1;

//SUBSCRIBER
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

//PUBLISHER R
rcl_publisher_t publisher_l;
std_msgs__msg__Float32 wheel_l;

//PUBLISHER L
rcl_publisher_t publisher_r;
std_msgs__msg__Float32 wheel_r;




rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

//services values
rcl_service_t service;

example_interfaces__srv__SetBool_Request req;
example_interfaces__srv__SetBool_Response res;

bool restartFlag = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void restartESP(){
  ESP.restart();
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

// //callback function to publish NODE 
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
// 	if (timer != NULL) {
// 		wheel_l.data = -1*odrive.GetPosition(1);//(int16_t)odrive.GetPosition(1);
//    wheel_r.data = 1*odrive.GetPosition(0);//(int16_t)odrive.GetPosition(0);
// 	}

//  RCCHECK(rcl_publish(&publisher_l,&wheel_l, NULL));
//  RCCHECK(rcl_publish(&publisher_r,&wheel_r, NULL));
// }


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
    char * status = "Riavvio in corso....";
    res_in->message.data = status;

    restartFlag = true;
    //restartESP();
  }
}


//function 
void publishPosition(){
  
  wheel_l.data = -1*odrive.GetPosition(1);//(int16_t)odrive.GetPosition(1);
  wheel_r.data = 1*odrive.GetPosition(0);//(int16_t)odrive.GetPosition(0);

  RCCHECK(rcl_publish(&publisher_l,&wheel_l, NULL));
  RCCHECK(rcl_publish(&publisher_r,&wheel_r, NULL));

}



void setup() {
  
  // Setup ODrive
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  
  odrive.run_state(1, requested_state, false); // don't wait 
  odrive.run_state(0, requested_state, false); // don't wait 

  

  //Setup for ROS2
  // Configure serial transport
  Serial.begin(115200);
  set_microros_transports();
  //set_microros_serial_transports(Serial);

  delay(2000);

  //create allocator
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  
  //init publisher left wheel
  RCCHECK(rclc_publisher_init_default(
  &publisher_l,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "lwheel"));


  //init publisher right wheel
  RCCHECK(rclc_publisher_init_default(
  &publisher_r,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "rwheel"));

  
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  
  //add subscriber
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));


  // create service
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, SetBool), "/restartESP"));

  //add service
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));


  // //create timer
  // const unsigned int timer_period = RCL_MS_TO_NS(1000);
  // rcl_timer_t timer;
  // rcl_ret_t rc = rclc_timer_init_default(&timer, &support, timer_period, timer_callback);
  // //RCCHECK(rclc_executor_add_timer(&executor,&timer));


  //setup for mpu6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}

void loop() {

  // TO DO FARE METODO CON TIMER 
  
  //function that publishes encoders values
  publishPosition();
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  //flag to restart ESP32 if there is a call in service restart
  if(restartFlag){
    restartESP();
  }

}




