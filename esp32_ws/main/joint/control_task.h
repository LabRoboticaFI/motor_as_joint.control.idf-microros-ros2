#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "driver/ledc.h"
#include "esp_err.h"
#include "joint/joint_config.h"

// Task
void control_joint_task(void * pvParameters);

// Funciones
float get_pid_signal(joint_t * joint);
esp_err_t set_pwm_duty(ledc_channel_t channel, uint32_t duty);

#endif // CONTROL_TASK_H
