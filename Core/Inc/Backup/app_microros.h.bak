#ifndef APP_MICROROS_H
#define APP_MICROROS_H

#include <stdbool.h>

#include "usart.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>

#define APP_MICROROS_DEFAULT_NODE_NAME "cubemx_node"
#define APP_MICROROS_DEFAULT_TWIST_TOPIC "odom_vel"

typedef struct {
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t publisher;
    geometry_msgs__msg__Twist twist_msg;
    bool initialized;
} app_microros_context_t;

bool app_microros_init(
    app_microros_context_t *context,
    UART_HandleTypeDef *uart,
    const char *node_name,
    const char *topic_name);
bool app_microros_publish_twist(
    app_microros_context_t *context,
    float linear_x,
    float angular_z);

#endif
