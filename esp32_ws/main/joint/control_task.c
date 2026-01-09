// Librerías de desarrollo
#include "joint/control_task.h" // Librería de este código
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

static const char *TAG_CONTROL_TASK = "CONTROL_TASK"; // El 'TAG' es una etiqueta que se utiliza para identificar la fuente del mensaje de registro.



// # # # # # # # # # # # # # # # # # # # #    TASK PRINCIPAL: PERFIL_QUINTICO_LOCAL_TASK()    # # # # # # # # # # # # # # # # # # # # # # # # #

void perfil_quintico_local_task(void * pvParameters){

    ESP_LOGI(TAG_CONTROL_TASK, "Inicializando perfil_quintico_local_task()");

    float q_des_master = 0.0f;
    float q_des_local = 0.0f;
    float q_inicial = 0.0f;
    float q_final = 0.0f;

    int64_t periodo_quintico = 10 * 1000; // us
    int64_t tiempo_inicial = 0;
    int64_t t_actual = 0;

    bool perfil_activo = false;

    ESP_LOGI(TAG_CONTROL_TASK, "debugiiiiiiiiiing()");

    while(true){

        // Nueva referencia desde el master
        if(xQueueReceive(joint1.comms.xQueue_quintico, &q_des_master, 0)){

            ESP_LOGI(TAG_CONTROL_TASK, "Llegó una nueva referencia al perfil quíntico: %.3f", q_des_master);

            // Tomar la última posición conocida
            xQueuePeek(joint1.comms.xQueue_q_actual, &q_inicial, 0);

            q_final = q_des_master;
            tiempo_inicial = esp_timer_get_time();
            perfil_activo = true;
        }

        if(perfil_activo){

            t_actual = esp_timer_get_time() - tiempo_inicial;
            if(t_actual > periodo_quintico){
                t_actual = periodo_quintico;
                perfil_activo = false;
            }

            float tau = (float)t_actual / (float)periodo_quintico;
            float delta_q = q_final - q_inicial;

            float tau2 = tau * tau;
            float tau3 = tau2 * tau;
            float tau4 = tau3 * tau;
            float tau5 = tau4 * tau;

            float s = 10.0f*tau3 - 15.0f*tau4 + 6.0f*tau5;


            q_des_local = q_inicial + delta_q * s;
            ESP_LOGI(TAG_CONTROL_TASK, "debugging: %.3f", q_des_local);

            xQueueOverwrite(joint1.comms.xQueue_q_des, &q_des_local);
        }

        //vTaskDelay(pdMS_TO_TICKS(1)); // MUY recomendable
    }
}




// # # # # # # # # # # # # # # # # # # # #    TASK PRINCIPAL: CONTROL_JOINT_TASK()    # # # # # # # # # # # # # # # # # # # # # # # # #


void control_joint_task(void * pvParameters){
    
    ESP_LOGI(TAG_CONTROL_TASK, "Inicializando control_joint_task()...");

    joint_t * joint = (joint_t *) pvParameters;

    float q_des_msg;
    float q_feedback;
    float pid_signal;

    int64_t counter_loops = 0;
    int64_t t_aux = esp_timer_get_time();
    int64_t time_flag = t_aux;

    ESP_LOGI(TAG_CONTROL_TASK, "debugiiiiiiiiiing()222222");

    // Control PID loop
    while (true)
    {
        // 1. Se obtiene la posición deseada. (q_des)
        if (xQueueReceive(joint->comms.xQueue_q_des, &q_des_msg, 0))
        {
            joint->control.q_des = q_des_msg;
            ESP_LOGI(TAG_CONTROL_TASK, "Llegó una nueva referencia DEL perfil quíntico: %.3f", joint->control.q_des);
        }

        // 2. Se obtiene la posición actual. (q_actual)
        pcnt_unit_get_count(joint->encoder.pcnt_handle, 
                            &joint->encoder.q_tick_counter);
        joint->q_angle = (float) joint->encoder.q_tick_counter * 360.0f / joint->encoder.ratio_encoder;
        xQueueOverwrite(joint->comms.xQueue_q_actual, &joint->q_angle);

        // (2.1 Se manda al publicador de ROS)
        q_feedback = joint->q_angle;
        xQueueOverwrite(joint->comms.xQueue_feedback, &q_feedback);

        // 3. Se computa la señal PID
        pid_signal = get_pid_signal(joint);

        if(pid_signal >= 0){
            gpio_set_level(joint->motor.control_direction_1, 1);
            gpio_set_level(joint->motor.control_direction_2, 0);
            set_pwm_duty(joint->motor.control_velocity.pwm_channel, pid_signal);
        } else if (pid_signal < 0) {
            pid_signal = -pid_signal;
            gpio_set_level(joint->motor.control_direction_1, 0);
            gpio_set_level(joint->motor.control_direction_2, 1);
            set_pwm_duty(joint->motor.control_velocity.pwm_channel, pid_signal);
        }
        //vTaskDelay(1);  // 1 tick (≈1 ms)
    }
    vTaskDelete(NULL);
}


void control_joint_task2(void * pvParameters)
{
    ESP_LOGI(TAG_CONTROL_TASK, "Inicializando control_joint_task2()");

    joint_t * joint = (joint_t *) pvParameters;

    /* -------- PERFIL QUÍNTICO -------- */
    float q_des_master = 0.0f;
    float q_des_local  = 0.0f;
    float q_inicial    = 0.0f;
    float q_final      = 0.0f;

    int64_t periodo_quintico = 5 * 1000 * 1000; // 10 ms (100 Hz)
    int64_t tiempo_inicial  = 0;
    bool perfil_activo = false;

    /* -------- CONTROL -------- */
    float pid_signal;
    float q_feedback;

    ESP_LOGI(TAG_CONTROL_TASK, "Task lista, entrando al loop");

    while (true)
    {
        /* 1. Leer encoder */
        pcnt_unit_get_count(joint->encoder.pcnt_handle,
                            &joint->encoder.q_tick_counter);

        joint->q_angle =
            (float)joint->encoder.q_tick_counter * 360.0f /
            joint->encoder.ratio_encoder;

        /* Publicar feedback para ROS (si hay consumidor) */
        q_feedback = joint->q_angle;
        xQueueOverwrite(joint->comms.xQueue_feedback, &q_feedback);

        /* 2. ¿Nueva referencia desde ROS? */
        if (xQueueReceive(joint->comms.xQueue_quintico,
                          &q_des_master, 0))
        {
            q_inicial = joint->q_angle;
            q_final   = q_des_master;
            tiempo_inicial = esp_timer_get_time();
            perfil_activo = true;

            ESP_LOGI(TAG_CONTROL_TASK,
                     "Nueva referencia master: %.3f", q_final);
        }

        /* 3. Perfil quíntico */
        if (perfil_activo)
        {
            int64_t t_actual =
                esp_timer_get_time() - tiempo_inicial;

            if (t_actual >= periodo_quintico)
            {
                t_actual = periodo_quintico;
                perfil_activo = false;
            }

            float tau = (float)t_actual /
                        (float)periodo_quintico;

            float tau2 = tau * tau;
            float tau3 = tau2 * tau;
            float tau4 = tau3 * tau;
            float tau5 = tau4 * tau;

            float s = 10.0f*tau3
                    - 15.0f*tau4
                    + 6.0f*tau5;

            q_des_local = q_inicial +
                          (q_final - q_inicial) * s;
        }
        else
        {
            q_des_local = q_final;
        }

        /* 4. Actualizar referencia del PID */
        joint->control.q_des = q_des_local;

        /* 5. PID */
        pid_signal = get_pid_signal(joint);

        /* 6. Actuación motor */
        if (pid_signal >= 0)
        {
            gpio_set_level(joint->motor.control_direction_1, 1);
            gpio_set_level(joint->motor.control_direction_2, 0);
        }
        else
        {
            pid_signal = -pid_signal;
            gpio_set_level(joint->motor.control_direction_1, 0);
            gpio_set_level(joint->motor.control_direction_2, 1);
        }

        if (pid_signal > joint->control.out_max)
            pid_signal = joint->control.out_max;

        set_pwm_duty(joint->motor.control_velocity.pwm_channel,
                     (uint32_t)pid_signal);
    }
}


float get_pid_signal(joint_t * joint){

    float pid_signal;
    float pid_p, pid_i_term, pid_d;
    float error, d_error;
    float dt;

    // Tiempo actual
    int64_t t = esp_timer_get_time(); // us

    // dt en segundos
    dt = (float)(t - joint->control.pid_state.prev_t) * 1e-6f;
    if(dt < 1e-6f) dt = 1e-6f;

    joint->control.pid_state.prev_t = t;

    // Error
    error = joint->control.q_des - joint->q_angle;
    d_error = (error - joint->control.pid_state.prev_error) / dt;
    joint->control.pid_state.prev_error = error;

    // --- PID ---

    // 1. Proporcional
    pid_p = joint->control.k_p * error;

    // 2. Integral (con anti-windup)
    joint->control.pid_state.integral += joint->control.k_i * error * dt;

    if(joint->control.pid_state.integral > joint->control.int_max){
        joint->control.pid_state.integral = joint->control.int_max;
    } else if(joint->control.pid_state.integral < joint->control.int_min){
        joint->control.pid_state.integral = joint->control.int_min;
    }

    // 3. Derivada
    pid_d = joint->control.k_d * d_error;

    // 4. Output
    pid_signal = pid_p + joint->control.pid_state.integral + pid_d;

    // Saturación del output (opc)
    if(pid_signal > joint->control.out_max){
        pid_signal = joint->control.out_max;
    } else if(pid_signal < joint->control.out_min){
        pid_signal = joint->control.out_min;
    }

    return pid_signal;
}



// Función que encapsula la actualización del CT del pwm

esp_err_t set_pwm_duty(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty); // Actualiza el PARÁMETRO de duty del PWM
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel); // Actualiza la SALIDA FISICA del periférico
    return ESP_OK;
}
