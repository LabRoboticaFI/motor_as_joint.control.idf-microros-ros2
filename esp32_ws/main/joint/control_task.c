// Librerías de desarrollo
#include "joint/control_task.h"      // Librería de este código
#include "microros/micro_ros_task.h" // Para la comunicación de Queues

// Configuración de gpios
#include "joint/joint_config.h"
#include "driver/gpio.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Log para esta tarea
#include "esp_log.h"
#include "esp_err.h"

// Para trabajar con el tiempo
#include "esp_timer.h"

// Matematica
#include "math.h"
#include <inttypes.h> // Erase later

static const char *TAG_CONTROL_TASK = "CONTROL_TASK"; // El 'TAG' es una etiqueta que se utiliza para identificar la fuente del mensaje de registro.

// # # # # # # # # # # # # # # ENCAPSULAMIENTOS # # # # # # # # # # # # # # # # # # # # #

void get_q_des_master(float *q_des,
                      QueueHandle_t queue_q_des_master)
{
    xQueueReceive(queue_q_des_master,
                  q_des, 0);
}

void joint_command_filter(float *q_des_master,
                          float q_actual,
                          joint_control_command_filter_config_t command_filter_config)
{
    float q_des_filtered = *q_des_master;
    // Condición de límites "mecánicos" de articulaciones
    if (q_des_filtered > command_filter_config.q_max)
        q_des_filtered = command_filter_config.q_max;
    if (q_des_filtered < command_filter_config.q_min)
        q_des_filtered = command_filter_config.q_min;

    // Condición de cambio grande
    float delta_q_des = q_des_filtered - q_actual;
    if (delta_q_des > command_filter_config.delta_q_max ||
        delta_q_des < command_filter_config.delta_q_min)
        q_des_filtered = q_actual;

    *q_des_master = q_des_filtered;
}

void get_encoder_angle(float *q_angle,
                       joint_encoder_t *encoder)
{
    pcnt_unit_get_count(encoder->pcnt_handle,
                        &encoder->q_tick_counter);

    *q_angle = encoder->q_tick_counter * 360 / encoder->config.encoder_ratio;
}

void joint_perfil_quintico(float q_des_master_filtered,
                           float *q_des_pid,
                           float current_angle,
                           joint_control_quintico_t *perfil_quintico)
{
    float q_des_local;

    if (fabsf(perfil_quintico->q_final - q_des_master_filtered) > 0.001)
    {
        perfil_quintico->q_inicial = current_angle;
        perfil_quintico->q_final = q_des_master_filtered;
        perfil_quintico->t_inicial = esp_timer_get_time();
        perfil_quintico->perfil_activo = true;

        ESP_LOGI(TAG_CONTROL_TASK,
                 "Nueva referencia master: %.3f", perfil_quintico->q_final);
    }

    /* 3. Perfil quíntico */
    if (perfil_quintico->perfil_activo)
    {
        int64_t t_actual =
            esp_timer_get_time() - perfil_quintico->t_inicial;

        if (t_actual >= perfil_quintico->config.periodo_quintico)
        {
            t_actual = perfil_quintico->config.periodo_quintico;
            perfil_quintico->perfil_activo = false;
        }

        float tau = (float)t_actual /
                    (float)perfil_quintico->config.periodo_quintico;

        float tau2 = tau * tau;
        float tau3 = tau2 * tau;
        float tau4 = tau3 * tau;
        float tau5 = tau4 * tau;

        float s = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;

        *q_des_pid = perfil_quintico->q_inicial +
                     (perfil_quintico->q_final - perfil_quintico->q_inicial) * s;
    }
    else
    {
        *q_des_pid = perfil_quintico->q_final;
    }
}

void get_control_pid(float q_actual,
                     float q_des,
                     joint_control_pid_t *control_pid)
{
    float pid_signal = 0;

    float error_flag = control_pid->q_des - q_actual;

    if (error_flag > control_pid->config.error_max ||
        error_flag < control_pid->config.error_min)
    {

        float pid_p, pid_d;
        float error, d_error;
        float dt;

        // Tiempo actual
        int64_t t = esp_timer_get_time(); // us

        // dt en segundos
        dt = (float)(t - control_pid->static_variables.prev_t) * 1e-6f;
        if (dt < 1e-6f)
            dt = 1e-6f;

        control_pid->static_variables.prev_t = t;

        // Error
        error = control_pid->q_des - q_actual;
        d_error = (error - control_pid->static_variables.prev_error) / dt;
        control_pid->static_variables.prev_error = error;

        // --- PID ---

        // 1. Proporcional
        pid_p = control_pid->config.k_p * error;

        // 2. Integral (con anti-windup)
        control_pid->static_variables.integral += control_pid->config.k_i * error * dt;

        if (control_pid->static_variables.integral > control_pid->config.int_max)
        {
            control_pid->static_variables.integral = control_pid->config.int_max;
        }
        else if (control_pid->static_variables.integral < control_pid->config.int_min)
        {
            control_pid->static_variables.integral = control_pid->config.int_min;
        }

        // 3. Derivada
        pid_d = control_pid->config.k_d * d_error;

        // 4. Output
        pid_signal = pid_p + control_pid->static_variables.integral + pid_d;

        // Saturación del output (opc)
        if (pid_signal > control_pid->config.out_max)
        {
            pid_signal = control_pid->config.out_max;
        }
        else if (pid_signal < control_pid->config.out_min)
        {
            pid_signal = control_pid->config.out_min;
        }
    }

    control_pid->pid_signal = pid_signal;
}

esp_err_t motor_update_pwm(joint_motor_t joint_motor,
                           float pid_signal)
{

    /* 6. Actuación motor */
    if (pid_signal >= 0)
    {
        gpio_set_level(joint_motor.config.gpio_direction_1, 1);
        gpio_set_level(joint_motor.config.gpio_direction_2, 0);
    }
    else
    {
        pid_signal = -pid_signal;
        gpio_set_level(joint_motor.config.gpio_direction_1, 0);
        gpio_set_level(joint_motor.config.gpio_direction_2, 1);
    }

    if(pid_signal > 10 && pid_signal < 550) pid_signal = 600;

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, joint_motor.config.velocity_pwm_channel, pid_signal); // Actualiza el PARÁMETRO de duty del PWM
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, joint_motor.config.velocity_pwm_channel);   // Actualiza la SALIDA FISICA del periférico
    return ESP_OK;
}

void joint_control(joint_t *joint)
{
    // 1. Se obtiene q_deseada_global.
    get_q_des_master(&joint->control.quintico.q_des_master,
                     joint->comms.xQueue_q_des);

    // 2. Se actualiza posición actual.
    get_encoder_angle(&joint->q_angle,
                      &joint->encoder);

    // 3. Se filtra q_deseada_global (con respecto a parámetros de configuración).
    joint_command_filter(&joint->control.quintico.q_des_master,
                         joint->q_angle,
                         joint->control.command_filter.config);

    // 4. Se obtiene q_deseada_local (perfil quíntico local).
    joint_perfil_quintico(joint->control.quintico.q_des_master,
                          &joint->control.pid.q_des,
                          joint->q_angle,
                          &joint->control.quintico);

    // 5. Se obtiene señal PID (función de PID).
    get_control_pid(joint->q_angle,
                    joint->control.pid.q_des,
                    &joint->control.pid);

    // 6. Se genera la señal de salida hacia el motor.
    motor_update_pwm(joint->motor,
                     joint->control.pid.pid_signal);
}

// # # # # # # # # # # # # # # # # # # # #    TASK PRINCIPAL: PERFIL_QUINTICO_LOCAL_TASK()    # # # # # # # # # # # # # # # # # # # # # # # # #

void control_joint_task(void *pvParameters)
{
    ESP_LOGI(TAG_CONTROL_TASK, "Inicializando control_joint_task()");

    joint_t *joint = (joint_t *)pvParameters;

    int64_t counter = 0;
    int64_t t_current = esp_timer_get_time();
    int64_t periodo_test = 10*1000*1000;

    while (true)
    {
        joint_control(joint);         // Control de joint 1
        //vTaskDelay(pdMS_TO_TICKS(1)); // 1 kHz
        counter++;
        if((esp_timer_get_time() - t_current) >= periodo_test){
            ESP_LOGI(TAG_CONTROL_TASK,"counter -> %" PRId64, counter);
            counter = 0;
            t_current = esp_timer_get_time();
        }
    }
}