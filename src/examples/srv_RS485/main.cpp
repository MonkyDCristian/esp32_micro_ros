/*
This example is a combination of publisher, subscriber and timer running in the same 
microros node using micro_ros_platformio with raspberry PI and Arduino framework

Steps:

  Add to your /.pio/libdeps/esp32doit-devkit-v1/micro_ros_platformio/metas/colcon.meta file:
  "microxrcedds_client": {
      "cmake-args": [
          "-DUCLIENT_HARD_LIVELINESS_CHECK=ON",
          "-DUCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT=1000"
          "-RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0"
          "-UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3"
      ]
  }

  Uninstall brltty for ubuntu computer connection
    sudo apt remove brltty
  
  check USB ports
    sudo dmesg | grep usb
 
  Edit build_src_filter in platformio.ini to build this example
    build_src_filter = +<examples/reconnection/*> -<.git/> -<.svn/> 
  
  Terminal1:
    # Find your serial [device name]:
    ls /dev/serial/by-id/*
    
    # Start micro_ros_agent:
    ros2 run micro_ros_agent micro_ros_agent serial --dev [device name]
  
  Terminal2:
    # visualice msgs
    rqt 

  Terminal3:
    # Call the service
    ros2 service call /motor_right/state std_srvs/srv/Trigger {}\

    ros2 topic pub /motor_right/value std_msgs/msg/Int16 data:\ 100\

    ros2 topic pub /motor_right/turn std_msgs/msg/Bool data:\ false\

reference:
  micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
  micro_ros_agent: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
  handle_reconnections: https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/handle_reconnections/handle_reconnections.html
*/

#include "variables.hpp"

unsigned int num_handles = 3;   // 2 subscriber, 1 service

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH);

  int bound_rate = 115200;
  Serial2.begin(bound_rate);
  set_microros_serial_transports(Serial2);
  delay(1000); 

  state = WAITING_AGENT;
}

void loop() {
  microros_loop();
}

// ---- MICROROS SETUP ----- 
bool microros_setup(){
  // Configure serial transport, you can use Serial/1/2
  const char *node_name = "micro_ros_platformio_node";
  const char *node_ns = ""; //namespace
  
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
  RCCHECK(rclc_node_init_default(&node, node_name, node_ns, &support)); // create node
  
  return true;
}

// ---- MICROROS PUBLISHERS -----
bool microros_add_pubs(){
  RCCHECK(rclc_publisher_init_default( // create publisher
    &feedback_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "motor_right/feedback"));
  
  return true;
}

// ---- MICROROS SUBSCRIBERS -----
bool microros_add_subs(){
  RCCHECK(rclc_subscription_init_default( // create subscriber
    &value_subscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
    "motor_right/value"));

  RCCHECK(rclc_subscription_init_default( // create subscriber
    &turn_subscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), 
    "motor_right/turn"));

  return true;
}

void value_sub_callback(const void * msgin){
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *) msgin;
  value_sub_msg.data = msg->data;
  feedback_pub_msg.data = value_sub_msg.data;
}

void turn_sub_callback(const void * msgin){
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *) msgin;
  turn_sub_msg.data = msg->data;
  digitalWrite(LED_BUILTIN, (turn_sub_msg.data) ? HIGH : LOW); 
}

// ---- MICROROS SERVICES -----
bool microros_add_srvs(){
  const char * service_name = "motor_right/state";
  const rosidl_service_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
  RCCHECK(rclc_service_init_default(&state_service, &node, type_support, service_name));

  return true;
}

void state_srv_callback(const void * req_msg, void * res_msg){
  // Cast messages to expected types
  std_srvs__srv__Trigger_Request * req_in =(std_srvs__srv__Trigger_Request *) req_msg;
  std_srvs__srv__Trigger_Response * res_out = (std_srvs__srv__Trigger_Response *) res_msg;
  
  // microros string message response
  const char * str = "Temp: ";
  rosidl_runtime_c__String ros_str = micro_ros_string_utilities_init(str);

  String fabc(temperatureRead(), 2);
  const char* t_str = fabc.c_str();
 
  ros_str = micro_ros_string_utilities_append(ros_str, t_str);
  ros_str = micro_ros_string_utilities_append(ros_str, "°C");
  
  state_response_msg.message.data = ros_str.data;

  // Handle request message and set the response message values
  res_out->success = true;
  res_out->message.data = state_response_msg.message.data;
}

// ---- MICROROS EXECUTOR -----
bool microros_add_executor()
{
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &value_subscriber, &value_sub_msg, &value_sub_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &turn_subscriber, &turn_sub_msg, &turn_sub_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&executor, &state_service, &state_request_msg, &state_response_msg, state_srv_callback));

  return true;
}

bool microros_create_entities()
{
  microros_setup();
  microros_add_pubs();
  microros_add_subs();
  microros_add_srvs();
  microros_add_executor();
  
  return true;
}

void microros_destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&feedback_publisher, &node);
  rcl_subscription_fini(&value_subscriber, &node);
  rcl_subscription_fini(&turn_subscriber, &node);
  rcl_service_fini(&state_service, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void microros_loop()
{
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 8)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == microros_create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        microros_destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 8)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        if (Serial2.available() > 0) {
          RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
          RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_pub_msg, NULL));
        }
      }
      break;
      
    case AGENT_DISCONNECTED:
      microros_destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}

void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}
