#include <Arduino.h>

#include <micro_ros_arduino.h>

//for mpu6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <sensor_msgs/msg/imu.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


sensor_msgs__msg__Imu imu;



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);


//mpu object
Adafruit_MPU6050 mpu;

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

void error_loop(){
  while(1){
    
    delay(100);
  }
}



void setup() {
  set_microros_transports();

  //setup for mpu6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu"));

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

}

void loop() {
  
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  double q[4];

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  euler_to_quat(g.gyro.x, g.gyro.y, g.gyro.z, q);

  imu.header.frame_id.size = 3;
  imu.header.frame_id.capacity = 4;
  imu.header.frame_id.data = "imu";
    
  //imu.header.stamp.sec = rmw_uros_epoch_nanos();;
  //imu.header.stamp.nanosec = rmw_uros_epoch_millis();;
  imu.orientation.w = q[1];
  imu.orientation.w = q[2];
  imu.orientation.w = q[3];
  imu.orientation.w = q[0];

  imu.angular_velocity.x = g.gyro.x;
  imu.angular_velocity.y = g.gyro.y;
  imu.angular_velocity.z = g.gyro.z;

  imu.linear_acceleration.x = a.acceleration.x;
  imu.linear_acceleration.y = a.acceleration.y;
  imu.linear_acceleration.z = a.acceleration.z;


  RCSOFTCHECK(rcl_publish(&publisher, &imu , NULL));

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}