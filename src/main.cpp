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
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/battery_state.h>


#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <example_interfaces/srv/set_bool.h>
#include <rmw_microros/rmw_microros.h>

//for Odrive communication
#include "ODriveArduino.h"


#define LED_PIN 13
#define BATTERY_PIN 35

#define COEFF_FILTER 0.1
#define RESISTOR_1 22000
#define RESISTOR_2 5100

#define PIN_RESET_ODRIVE 15


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;


bool odrive_init = false;;

float vel_motor0_linear;
float vel_motor1_linear;

bool restartFlag = false;


//voltage and current battery
float voltage;
float voltage_filtered = 0;
float current;
float current_filtered = 0;
float num_samples = 30;

float percentage; //percentage battery
int power_supply_status; //status : 1 charging  3 not charging  4 full 


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

//services values
rcl_service_t service_restart;



example_interfaces__srv__SetBool_Request req;
example_interfaces__srv__SetBool_Response res;

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print& obj, T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print& obj, float arg) { obj.print(arg, 4); return obj; }


// ODrive object
ODriveArduino odrive(Serial2);

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


enum batteryStatus{
  NOT_CHARGING = 0,
  CHARGING = 1,
  FULL = 4
} battery_status;



//function that publish battery status 
void publishBattery(){
  //calculate voltage from voltage partitor

  float sum = 0;
  for (int i = 0; i < num_samples; i++){
    voltage = analogRead(BATTERY_PIN);  
    sum += voltage;
  }
  voltage = sum / num_samples;
  
  voltage = (voltage / 4095) * 3.45; // 3.3 but with 3.6 more real value
  
  voltage = (voltage / RESISTOR_2) * (RESISTOR_1 + RESISTOR_2);

  // IIR filter
  voltage_filtered = voltage_filtered * (1 - COEFF_FILTER) + voltage * COEFF_FILTER;

  current_filtered = voltage_filtered / RESISTOR_2;
  

  //choose status battery in according with voltage level
  if(voltage_filtered  > 14.5){
    power_supply_status = FULL;
  } 
  if(voltage_filtered <= 14.5 && voltage_filtered > 13.3 ){
    power_supply_status = CHARGING;
  }
  if(voltage_filtered <= 13.3){
    power_supply_status = NOT_CHARGING;
  }


  //to obtain correct timestamp
  RCSOFTCHECK(rmw_uros_sync_session(1000));
  if(rmw_uros_epoch_synchronized()){
    battery.header.stamp.sec = rmw_uros_epoch_millis()/1000;
    battery.header.stamp.nanosec = rmw_uros_epoch_nanos();
  }


  battery.power_supply_status = power_supply_status;
  //battery.percentage = percentage;
  battery.voltage = voltage_filtered;
  battery.current = current_filtered;

  
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
  //enable odrive
  digitalWrite(PIN_RESET_ODRIVE, HIGH);

  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  
  odrive.run_state(1, requested_state, false); // don't wait 
  odrive.run_state(0, requested_state, false); // don't wait 
}

void setZeroSpeedOdrive(){
  //set zero speed
  odrive.SetVelocity(0,0);
  odrive.SetVelocity(1,0);
}


void resetOdrive(){
  digitalWrite(PIN_RESET_ODRIVE, LOW);
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
  odrive.SetVelocity(1,-vel_motor1_linear);

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
    char status[] = "Restart.... \n";
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
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_l,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "lwheel"));

  //create publisher right wheel
    RCCHECK(rclc_publisher_init_best_effort(
    &publisher_r,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rwheel"));


  //create publisher battery
  RCCHECK(rclc_publisher_init_default(
    &publisher_battery,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "battery_state"));


  // create service
  RCCHECK(rclc_service_init_default(&service_restart, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, SetBool), "/restartESP"));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  
  
  //add subscriber
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA)); // TRY ALWAYS and remove setZeroSpeed

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
  rcl_subscription_fini(&subscriber,&node);
  rcl_service_fini(&service_restart,&node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);



}

void setup() {
  set_microros_transports();
  //pinMode(LED_PIN, OUTPUT);
  //activeOdrive();
  state = WAITING_AGENT;

  //set pinmode for pin that reads battery voltage
  pinMode(BATTERY_PIN, INPUT);

  //set pinMode for reset odrive
  pinMode(PIN_RESET_ODRIVE, OUTPUT);
  //set pinmode Odrive HIGH to enable it
  digitalWrite(PIN_RESET_ODRIVE, HIGH);

}



void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      //enable odrive for new setup
      digitalWrite(PIN_RESET_ODRIVE, HIGH);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      // set HIGH to enable odrive
      //activeOdrive();
      if (state == AGENT_CONNECTED) {
        
        // activate odrive the first time in the loop
        if(odrive_init == false){
          activeOdrive();
          odrive_init = true;
        }

        publishPosition();
        //publishImu();
        publishBattery();

        if(restartFlag){
          ESP.restart();
        }
        
        //to prevent errors in movements
        setZeroSpeedOdrive();
        
        //spin 
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); //100
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      setZeroSpeedOdrive();
      resetOdrive();
      odrive_init = false;
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}




