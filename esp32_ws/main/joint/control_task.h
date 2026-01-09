#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

// Librerías para importar tipos de datos.
#include "driver/gpio.h"        // Para importar tipos de datos -> gpio_num_t
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"     // Para importar tipos de datos -> QueueHandle_t
#include "driver/ledc.h"        // Para importar tipos de datos -> ledc_channel_t
#include "driver/pulse_cnt.h"   // Para importar tipos de datos -> pcnt_unit_handle_t
#include "esp_err.h" // Para importar tipos de datos para funciones -> esp_err_t


// # # # # # # # # # # # # # # # #    ESTRUCTURA DE DATO: JOINT   # # # # # # # # # # # # # # # # #

// ED que encapsula información de los ENCODERS
typedef struct {
    gpio_num_t gpio_encoder_signal_A;
    gpio_num_t gpio_encoder_signal_B;
    pcnt_unit_handle_t pcnt_handle;
    int32_t q_tick_counter;
    int32_t ratio_encoder; // Relación de ticks por cada revolución de eje post-reducción.
} joint_encoder_t;

// ED que encapsula información acerca del PWM para los MOTORES
typedef struct {
    gpio_num_t gpio;
    ledc_channel_t pwm_channel;
} joint_motor_signal_t;

// ED que encapsula información acerca del CONTROL de los MOTORES
typedef struct {
    gpio_num_t control_direction_1;
    gpio_num_t control_direction_2;
    joint_motor_signal_t control_velocity;
} joint_motor_control_t;

// ED que encapsula información de las QUEUES entre tasks.
typedef struct {
    QueueHandle_t xQueue_q_des;
    QueueHandle_t xQueue_feedback;
    QueueHandle_t xQueue_quintico;
    QueueHandle_t xQueue_q_actual;
    const char *TAG;
} joint_comms_t;

// ED que encapsula información del lazo de control PID
typedef struct {
    float integral;
    float prev_error;
    int64_t prev_t;
} pid_state_t;

// ED que encapsula información del lazo de control PID
typedef struct {
    float q_des;
    float k_p;
    float k_i;
    float k_d;
    float int_min; // Para anti-windup
    float int_max;
    float out_max;
    float out_min;
    pid_state_t pid_state;
} joint_pid_control_t;

// Estructura principal de datos del JOINT. Se utilizan las estructuras de datos anteriores para encapsular la información de cada área.
typedef struct {
    joint_encoder_t encoder;
    joint_motor_control_t motor;
    joint_pid_control_t control;
    joint_comms_t comms;
    float q_angle;
} joint_t;

// Hasta aquí termina la estructura de datos de la articulación ---> joint_t




// # # # # # # # # # # # # # # # # # #    DECLARACIONES DE VARIABLES Y FUNCIONES    # # # # # # # # # # # # # # # # #

extern joint_t joint1; // Estructura de dato para la articulación, se utiliza desde main.c

// Tasks principales del núcleo 1 del esp32, encargado del control de las articulaciones.
void control_joint_task(void * pvParameters);
void control_joint_task2(void * pvParameters);
void perfil_quintico_local_task(void * arg);

float get_pid_signal(joint_t * joint); // Función que encapsula el código del PID

esp_err_t set_pwm_duty(ledc_channel_t channel, uint32_t duty); // Función que encapsula la actualización del CT del pwm

#endif //CONTROL_TASK_H