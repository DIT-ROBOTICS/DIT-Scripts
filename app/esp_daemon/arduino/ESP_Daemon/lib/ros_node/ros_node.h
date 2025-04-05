#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

extern AgentState state;

void initROS();
bool create_entities();
void destroy_entities();
void emergency_callback(const void* msgin);
void sima_callback(const void* msgin);
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);

extern rclc_executor_t executor;

#endif
