#ifndef MICRO_ROS_TASK_H
#define MICRO_ROS_TASK_H

#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "microros/esp32_serial_transport.h"

#include "freertos/queue.h"

#include "std_msgs/msg/float32.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Queues
extern QueueHandle_t xQueue_feedback_q1;

// Publicadores y suscriptores.
extern rcl_subscription_t q_des_suscriber;
extern rcl_publisher_t feedback_publisher;

// Definición de interfaces (msg, srv y personalizados)
extern std_msgs__msg__Float32 msg_in;
extern float msg_out;

typedef struct feedback_t
{
  float q;
  float error;
  float signal_p;
  float signal_i;
  float signal_d;
} feedback_t;


// # # # # # Funciones

void timer_callback_feedback_q1(rcl_timer_t * timer, int64_t last_call_time);
void subscription_callback_q1(const void * msgin);

void micro_ros_task(void * arg);


#endif //MICRO_ROS_TASK_H