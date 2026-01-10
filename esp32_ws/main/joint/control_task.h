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




// # # # ED que encapsulan información de los ENCODERS # # #

typedef struct {
    gpio_num_t gpio_signal_A;
    gpio_num_t gpio_signal_B;
    int32_t encoder_ratio; // Relación de ticks por cada revolución de eje post-reducción.
    int16_t redutor_ratio;
} joint_encoder_config_t;

typedef struct {
    joint_encoder_config_t config;
    pcnt_unit_handle_t pcnt_handle;
    int32_t q_tick_counter;
} joint_encoder_t;





// # # #  ED que encapsula información acerca de los MOTORES # # #

typedef struct {
    gpio_num_t gpio_direction_1;
    gpio_num_t gpio_direction_2;
    gpio_num_t gpio_velocity_pwm;
    ledc_channel_t velocity_pwm_channel;
} joint_motor_config_t;

typedef struct {
    joint_motor_config_t config;
} joint_motor_t;








// # # #  ED que encapsula información acerca del CONTROL # # #


// ED que encapsula información del PID
typedef struct {
    float k_p;
    float k_i;
    float k_d;
    float int_min; // Para anti-windup
    float int_max;
    float out_max;
    float out_min;
} joint_control_pid_config_t;

// ED que encapsula información del PID
typedef struct {
    float integral;
    float prev_error;
    int64_t prev_t;
} joint_control_pid_static_variables_t;


// ED que encapsula información del lazo de control PID
typedef struct {
    joint_control_pid_config_t config;
    joint_control_pid_static_variables_t static_variables;
    float q_des;
} joint_control_pid_t;






// # # # ED que encapsula información del perfil QUÍNTICO
typedef struct {
    int64_t periodo_quintico;
} joint_control_quintico_config_t;

typedef struct {
    joint_control_quintico_config_t config;
    int64_t t_inicial;
    int64_t t_actual;
    float q_inicial;
    float q_des_local;
    float q_final;
    bool perfil_activo;
} joint_control_quintico_t;




typedef struct {
    float q_min;
    float q_max;
    float delta_q_min;
    float delta_q_max;
    float q_vel_min;
    float q_vel_max;
} joint_control_command_filter_config_t;

typedef struct {
    joint_control_command_filter_config_t config;
} joint_control_command_filter_t;



// # # # ED que encapsula información del CONTROL de la articulación
typedef struct {
    joint_control_command_filter_t command_filter;
    joint_control_quintico_t quintico;
    joint_control_pid_t pid;
} joint_control_t;







// # # #  ED que encapsula información de las COMMS QUEUES entre tasks.

typedef struct {
    QueueHandle_t xQueue_q_des;
    QueueHandle_t xQueue_feedback;
    QueueHandle_t xQueue_quintico;
    QueueHandle_t xQueue_q_actual;
    const char *TAG;
} joint_comms_t;





// Estructura principal de datos del JOINT. Se utilizan las estructuras de datos anteriores para encapsular la información de cada área.
typedef struct {
    joint_motor_t motor;
    joint_encoder_t encoder;
    joint_control_t control;
    joint_comms_t comms;
    float q_angle;
} joint_t;




// # # # # # # # # # # # # # # # # # #    DECLARACIONES DE VARIABLES Y FUNCIONES    # # # # # # # # # # # # # # # # #

extern joint_t joint1; // Estructura de dato para la articulación, se utiliza desde main.c

// Tasks principales del núcleo 1 del esp32, encargado del control de las articulaciones.
void control_joint_task(void * pvParameters);
void control_joint_task2(void * pvParameters);
void perfil_quintico_local_task(void * arg);

float get_pid_signal(joint_t * joint); // Función que encapsula el código del PID

esp_err_t set_pwm_duty(ledc_channel_t channel, uint32_t duty); // Función que encapsula la actualización del CT del pwm

#endif //CONTROL_TASK_H