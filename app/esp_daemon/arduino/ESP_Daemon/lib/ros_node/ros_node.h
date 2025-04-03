#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

void initROS();
void emergency_callback(const void* msgin);
void sima_callback(const void* msgin);
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);

extern rclc_executor_t executor;

#endif
