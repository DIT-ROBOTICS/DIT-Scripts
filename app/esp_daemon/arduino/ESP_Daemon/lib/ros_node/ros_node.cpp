#include "ros_node.h"
#include "config.h"

#include "espnow_comm.h"
#include "led_control.h"

#include <Arduino.h>
#include <micro_ros_platformio.h> // Must include this to use set_microros_serial_transports()


// micro-ROS essential components
rcl_publisher_t publisher;
rcl_publisher_t float_publisher;
rcl_subscription_t int_subscriber;
rcl_subscription_t emergency_subscriber;
std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 float_msg;
std_msgs__msg__Int32 sima_msg;
std_msgs__msg__Int32 emergency_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

extern float Vbattf;

void error_loop() {
  while (1) {
    delay(100);
  }
}

#define RCCHECK(fn)        { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn)    { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    RCSOFTCHECK(rcl_publish(&float_publisher, &float_msg, NULL));
    msg.data++;
    float_msg.data = Vbattf;
  }
}

void sima_callback(const void* msgin) {
  const std_msgs__msg__Int32* incoming = (const std_msgs__msg__Int32*)msgin;
  sendESPNow(incoming->data);
  mode = incoming->data; // For Testing
}

void emergency_callback(const void* msgin) {
  const std_msgs__msg__Int32* incoming = (const std_msgs__msg__Int32*)msgin;
  if (incoming->data == 1) {
    digitalWrite(RELAY_PIN, HIGH);  // Turn relay on
  } else {
    digitalWrite(RELAY_PIN, LOW);   // Turn relay off
  }
}

void initROS() {
  set_microros_serial_transports(Serial);
  delay(2000);
  allocator = rcl_get_default_allocator();

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Ensure relay is off initially

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, "", &support));
  
  RCCHECK(rclc_publisher_init_default(&publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32_counter"));
  RCCHECK(rclc_publisher_init_default(&float_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/robot_status/battery_voltage"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ROS_TIMER_MS), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_subscription_init_default(&int_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/sima/start"));
  RCCHECK(rclc_executor_add_subscription(&executor, &int_subscriber, &sima_msg, &sima_callback, ON_NEW_DATA));

  RCCHECK(rclc_subscription_init_default(&emergency_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/robot_status/emergency_stop"));
  RCCHECK(rclc_executor_add_subscription(&executor, &emergency_subscriber, &emergency_msg, &emergency_callback, ON_NEW_DATA));

  sima_msg.data = 0;
  msg.data = 0;
  float_msg.data = 0.0;
}
