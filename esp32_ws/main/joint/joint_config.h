#ifndef JOINT_CONFIG_H
#define JOINT_CONFIG_H

#include "driver/gpio.h"      // Para importar tipos de datos de las funciones -> gpio_num_t
#include "driver/ledc.h"      // Para importar tipos de datos de las funciones -> ledc_channel_t
#include "driver/pulse_cnt.h" // Para importar tipos de datos de las funciones -> pcnt_unit_handle_t
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>

// COMUNICACIÓN UART0
#define UART0_TX GPIO_NUM_1 // 22. Transmisión  (no van a conectarse)
#define UART0_RX GPIO_NUM_3 // 23. Recepción    (no van a conectarse)
// COMUNICACIÓN UART1
#define UART1_TX GPIO_NUM_17 // 22. Transmisión  (no van a conectarse, solo para debuguear)
#define UART1_RX GPIO_NUM_16 // 23. Recepción    (no van a conectarse, solo para debuguear)

// # # # # # # # # # # # # # # # #    ESTRUCTURA DE DATO: JOINT   # # # # # # # # # # # # # # # # #

// # # # ED que encapsulan información de los ENCODERS # # #

typedef struct
{
    gpio_num_t gpio_signal_A;
    gpio_num_t gpio_signal_B;
    int32_t encoder_ratio; // Relación de ticks por cada revolución de eje post-reducción.
    int16_t redutor_ratio;
} joint_encoder_config_t;

typedef struct
{
    joint_encoder_config_t config;
    pcnt_unit_handle_t pcnt_handle;
    int q_tick_counter;
} joint_encoder_t;

// # # #  ED que encapsula información acerca de los MOTORES # # #

typedef struct
{
    gpio_num_t gpio_direction_1;
    gpio_num_t gpio_direction_2;
    gpio_num_t velocity_pwm_gpio;
    ledc_channel_t velocity_pwm_channel;
    int16_t velocity_pwm_min_value;
    int16_t velocity_pwm_max_value;
    int16_t velocity_pwm_to_start_forward;
} joint_motor_config_t;

typedef struct
{
    joint_motor_config_t config;
} joint_motor_t;

// # # #  ED que encapsula información acerca del CONTROL # # #

// ED que encapsula información del PID
typedef struct
{
    float k_p;
    float k_i;
    float k_d;
    float int_min; // Para anti-windup
    float int_max;
    float out_max;
    float out_min;
    float error_min;
    float error_max;
} joint_control_pid_config_t;

// ED que encapsula información del PID
typedef struct
{
    float integral;
    float prev_error;
    int64_t prev_t;
} joint_control_pid_static_variables_t;

// ED que encapsula información del lazo de control PID
typedef struct
{
    joint_control_pid_config_t config;
    joint_control_pid_static_variables_t static_variables;
    float q_des;
    float pid_signal;
} joint_control_pid_t;

// # # # ED que encapsula información del perfil QUÍNTICO
typedef struct
{
    int64_t periodo_quintico;
} joint_control_quintico_config_t;

typedef struct
{
    joint_control_quintico_config_t config;
    int64_t t_inicial;
    int64_t t_actual;
    float q_inicial;
    float q_des_local;
    float q_final;
    float q_des_master;
    bool perfil_activo;
} joint_control_quintico_t;

typedef struct
{
    float q_min;
    float q_max;
    float delta_q_min;
    float delta_q_max;
    float q_vel_min;
    float q_vel_max;
} joint_control_command_filter_config_t;

typedef struct
{
    joint_control_command_filter_config_t config;
} joint_control_command_filter_t;

// # # # ED que encapsula información del CONTROL de la articulación
typedef struct
{
    joint_control_command_filter_t command_filter;
    joint_control_quintico_t quintico;
    joint_control_pid_t pid;
} joint_control_t;

// # # #  ED que encapsula información de las COMMS QUEUES entre tasks.

typedef struct
{
    QueueHandle_t xQueue_q_des;
    QueueHandle_t xQueue_feedback;
    // QueueHandle_t xQueue_quintico;
    // QueueHandle_t xQueue_q_actual;
    const char *TAG;
} joint_comms_t;

// Estructura principal de datos del JOINT. Se utilizan las estructuras de datos anteriores para encapsular la información de cada área.
typedef struct
{
    joint_motor_t motor;
    joint_encoder_t encoder;
    joint_control_t control;
    joint_comms_t comms;
    float q_angle;
} joint_t;

/* # # # # # # # # # # # #   DECLARACIÓN DE FUNCIONES   # # # # # # # # # # # # # */

extern joint_t joint1; // Estructura de dato para la articulación, se utiliza desde main.c

esp_err_t joint_motor_setup(joint_motor_config_t motor_config);

esp_err_t joint_encoder_setup(joint_encoder_config_t joint_encoder_config,
                              pcnt_unit_handle_t *joint_encoder_pcnt_handler);

#endif // JOINT_CONFIG_H