#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>
#include <std_srvs/srv/trigger.h>

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// ---- MICROROS ENTITIES -----
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t feedback_publisher;
std_msgs__msg__Int16 feedback_pub_msg;

rcl_subscription_t value_subscriber;
std_msgs__msg__Int16 value_sub_msg;

rcl_subscription_t turn_subscriber;
std_msgs__msg__Bool turn_sub_msg;

rcl_service_t state_service;
std_srvs__srv__Trigger_Request state_request_msg;
std_srvs__srv__Trigger_Response state_response_msg;

rclc_executor_t executor;

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT, 
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

bool microros_setup();
bool microros_add_pubs();
bool microros_add_subs();
void value_sub_callback(const void * msgin);
void turn_sub_callback(const void * msgin);
bool microros_add_srvs();
void state_srv_callback(const void * req_msg, void * res_msg);
bool microros_add_executor();
bool microros_create_entities();
void microros_destroy_entities();
void microros_loop();
void error_loop();