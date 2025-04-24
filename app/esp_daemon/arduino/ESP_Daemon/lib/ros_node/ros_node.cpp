#include "ros_node.h"
#include "config.h"

#include "espnow_comm.h"
#include "led_control.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>

// micro-ROS essential components
rcl_publisher_t                  counter_publisher;
rcl_publisher_t          battery_voltage_publisher;
rcl_subscription_t          sima_command_subscriber;
rcl_subscription_t             emergency_subscriber;
std_msgs__msg__Int32             counter_msg;
std_msgs__msg__Float32   battery_voltage_msg;
std_msgs__msg__Int16        sima_command_msg;
std_msgs__msg__Bool            emergency_msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;

extern float Vbattf;
AgentState state = WAITING_AGENT;

#define RCCHECK(fn)        { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn)    { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void error_loop() { while (1) { delay(100); } }

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&counter_publisher, &counter_msg, NULL));
    RCSOFTCHECK(rcl_publish(&battery_voltage_publisher, &battery_voltage_msg, NULL));
    counter_msg.data++;
    battery_voltage_msg.data = Vbattf;
  }
}

void sima_callback(const void* msgin) {
  const std_msgs__msg__Int16* incoming = (const std_msgs__msg__Int16*)msgin;
  int data = incoming->data;
  
  if (data >= 10) {
    // Double-digit: tens digit is address index, ones digit is data
    int addressIndex = data / 10 - 1;
    int dataToSend = data % 10;
    sendESPNow(dataToSend, addressIndex);
  } else {
    // Single-digit: send to all addresses
    for (int i = 0; i < 4; i++) {  // Assuming 4 addresses (SIMA_01 to SIMA_04)
      sendESPNow(data, i);
    }
  }
  
  mode = SIMA_CMD;
  last_override_time = millis();
}

void emergency_callback(const void* msgin) {
  const std_msgs__msg__Bool* incoming = (const std_msgs__msg__Bool*)msgin;
  
  if (incoming->data) { digitalWrite(RELAY_PIN, ENABLE);  mode = EME_ENABLE;  }
  else                { digitalWrite(RELAY_PIN, DISABLE); mode = EME_DISABLE; }
  last_override_time = millis();
}

// Free the resources allocated by micro-ROS
void destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_init_options_fini(&init_options);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

  rcl_publisher_fini(&counter_publisher, &node);
  rcl_publisher_fini(&battery_voltage_publisher, &node);
  rcl_subscription_fini(&sima_command_subscriber, &node);
  rcl_subscription_fini(&emergency_subscriber, &node);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, ROS_NODE_NAME, "", &support);
  
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ROS_TIMER_MS), timer_callback);

  rclc_publisher_init_default(
    &counter_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/esp32_counter");

  rclc_publisher_init_default(
    &battery_voltage_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/robot_status/battery_voltage");


  // Initialize executor with 3 handles (2 subscriptions + 1 timer)
  unsigned int num_handles = 3;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  rclc_subscription_init_default(
    &sima_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/sima/start");
  rclc_executor_add_subscription(&executor, &sima_command_subscriber, &sima_command_msg, &sima_callback, ON_NEW_DATA);

  rclc_subscription_init_default(
    &emergency_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/robot_status/emergency_stop");
  rclc_executor_add_subscription(&executor, &emergency_subscriber, &emergency_msg, &emergency_callback, ON_NEW_DATA);
  
  return true;
}

void initROS() {
  set_microros_serial_transports(Serial);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_INITIAL_STATE);
  
  sima_command_msg.data = 0;
  counter_msg.data = 0;
  battery_voltage_msg.data = 0.0;
  emergency_msg.data = false;
  state = WAITING_AGENT;
}
