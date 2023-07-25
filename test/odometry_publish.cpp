// MAIN funzionante con due topic: cmd_vel come topic che pubblica velocità e che viene seguito dal codice e uno che pubblica valori encoders

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include "ODriveArduino.h"

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

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>




#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif


// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print& obj, T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print& obj, float arg) { obj.print(arg, 4); return obj; }

//mpu object
Adafruit_MPU6050 mpu;


// ODrive object
ODriveArduino odrive(Serial2);

float vel_motor0;
float vel_motor1;

//SUBSCRIBER
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

//PUBLISHER
rcl_publisher_t publisher_l;
std_msgs__msg__Int16 wheel_l;

//PUBLISHER
rcl_publisher_t publisher_r;
std_msgs__msg__Int16 wheel_r;

//PUBLISHER
rcl_publisher_t publisher_odom;
nav_msgs__msg__Odometry odometry;



rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

//CALLBACK FOR SUBSCRIBE NODE
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x > 5) ? LOW : HIGH);

  //value from twist 
  vel_motor0 = msg->linear.x;
  vel_motor1 = msg->linear.y;

  // set wheels' speed
  odrive.SetVelocity(0,vel_motor0);
  odrive.SetVelocity(1,-vel_motor1);

}

// //callback function to publish NODE 
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
	
// 	if (timer != NULL) {
// 		send_message.angular.x = odrive.GetPosition(0);
// 	}

//   RCCHECK(rcl_publish(&publisher,&send_message, NULL));
// }




void setup() {
  // Setup ODrive
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  int requested_state;

  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(1, requested_state, false); // don't wait 

  odrive.run_state(0, requested_state, false); // don't wait 

  // Led status 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  //Setup for ROS2
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  delay(2000);

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

  RCCHECK(rclc_publisher_init_default(
  &publisher_l,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
  "lwheel"));

  RCCHECK(rclc_publisher_init_default(
  &publisher_r,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
  "rwheel"));

  RCCHECK(rclc_publisher_init_default(
  &publisher_odom,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
  "/odom"));

  
   // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));


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
float pos0_diff, pos1_diff, pos0_old, pos1_old, pos0_mm_diff, pos1_mm_diff, pos_average_mm_diff, pos_total_mm, theta, x, y;

void loop() {
  delay(100);

  //public odometry
  float pos0 = odrive.GetPosition(0);
  float pos1 = odrive.GetPosition(1);

  pos0_diff = pos0 - pos0_old;
  pos1_diff = pos1 - pos1_old;            
  pos0_old = pos0;
  pos1_old = pos1;
  // calc mm from encoder counts
  pos0_mm_diff = pos0_diff / 279 ;
  pos1_mm_diff = pos1_diff / 279 ;
  // calc distance travelled based on average of both wheels
  pos_average_mm_diff = (pos0_mm_diff + pos1_mm_diff) / 2;   // difference in each cycle
  pos_total_mm += pos_average_mm_diff;                       // calc total running total dist
  // calc angle or rotation to broadcast with tf
  theta += (pos1_mm_diff - pos0_mm_diff) / 360;
  
  if (theta > PI)
  theta -= TWO_PI;
  if (theta < (-PI))
  theta += TWO_PI;
  // calc x and y to broadcast wit
  y += pos_average_mm_diff * sin(theta);
  x += pos_average_mm_diff * cos(theta);

  odometry.header.stamp = ;
  odometry.header.frame_id = "odom";
  odometry.pose.pose.position.x = x/1000;
  odometry.pose.pose.position.y = y/1000;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
  odometry.child_frame_id = base_link;
  odometry.twist.twist.linear.x = ((pos0_mm_diff + pos1_mm_diff) / 2)/10;          // forward linear velovity
  odometry.twist.twist.linear.y = 0.0;                                            // robot does not move sideways
  odometry.twist.twist.angular.z = ((pos1_mm_diff - pos0_mm_diff) / 360)*100;      // anglular velocity

  RCCHECK(rcl_publish(&publisher_odom,&odometry, NULL));

  //FARE METODO CON TIMER 
  
  wheel_l.data = odrive.GetPosition(0);
  wheel_r.data = odrive.GetPosition(1);

  

  RCCHECK(rcl_publish(&publisher_l,&wheel_l, NULL));
  RCCHECK(rcl_publish(&publisher_r,&wheel_r, NULL));
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
}






