#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/init_options.h"
#include "microros/esp32_serial_transport.h"

#include "microros/micro_ros_task.h"
#include "joint/control_task.h"
#include "joint/joint_config.h"


static const char *TAG_MAIN = "MAIN_C"; // El 'TAG' es una etiqueta que se utiliza para identificar la fuente del mensaje de registro.

/* # # # # # # # # # #   FUNCIÓN PRINCIPAL (MAIN)   # # # # # # # # # # # */

// Especificación del puerto por donde se comunicará microros con el agente de microros.
static size_t uart_port = UART_NUM_0;

// Estructura de dato para configurar los parámetros de la articulación
joint_t joint1 = {
    .encoder = {
        .gpio_encoder_signal_A = GPIO_NUM_22,
        .gpio_encoder_signal_B = GPIO_NUM_23,
		.pcnt_handle = NULL,
		.q_tick_counter = 0,
		.ratio_encoder = 1120,
    },
	.motor = {
		.control_direction_1 = GPIO_NUM_19,
		.control_direction_2 = GPIO_NUM_21,
		.control_velocity = {
			.gpio = GPIO_NUM_2,
			.pwm_channel = LEDC_CHANNEL_0,
		}
	},
	.control = {
		.k_p = 80,
		.k_i = 5,
		.k_d = 5,
		.int_min = -800,
		.int_max = 800,
		.out_min = -1000,
		.out_max = 1000,
		.pid_state = {
			.integral = 0.0f,
			.prev_error = 0.0f,
			.prev_t = 0,
		},
		.q_des = 0,
	},
	.comms = {
		.TAG = "JOINT1",
		.xQueue_feedback = NULL,
		.xQueue_q_des = NULL,
		.xQueue_q_actual = NULL,
		.xQueue_quintico = NULL,
	},
	.q_angle = 0,
};



void app_main(void)
{

	/* # # # # # # # # # # # # # # # # # #   FASE 1: Configuraciones   # # # # # # # # # # # # # */

	/* # # # # # # # #    JOINT 1    # # # # # # # # # #*/
	
	// Configuración de GPIOs que controlan el movimiento del motor
	joint_gpios_pwm_control_signal_setup(
		joint1.motor.control_direction_1,
		joint1.motor.control_direction_2,
		joint1.motor.control_velocity.gpio,
		joint1.motor.control_velocity.pwm_channel
		);

	// Configuración de periférico PCNT para conteo de encoder.
	joint_encoder_setup(
		joint1.encoder.gpio_encoder_signal_A,
        joint1.encoder.gpio_encoder_signal_B,
        &joint1.encoder.pcnt_handle);

	// Creación de Queue ANTES de lanzar tareas, de acuerdo al diagrama de bloques de task del README.md
    joint1.comms.xQueue_q_des = xQueueCreate(1, sizeof(float));
	joint1.comms.xQueue_q_actual = xQueueCreate(1, sizeof(float));
	joint1.comms.xQueue_feedback = xQueueCreate(1, sizeof(float));
	joint1.comms.xQueue_quintico = xQueueCreate(1, sizeof(float));
	
	// 2.- Configuración de transporte personalizado por UART-USB
	#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
		rmw_uros_set_custom_transport(
			true,
			(void *) &uart_port,
			esp32_serial_open,
			esp32_serial_close,
			esp32_serial_write,
			esp32_serial_read
		);
		ESP_LOGI(TAG_MAIN, "Se configuró correctamente el transporte personalizado.");
	#else
	#error micro-ROS transports misconfigured
	#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

	


	/* # # # # # # # # # # # # # # # #   FASE 2: Lanzamiento de tareas   # # # # # # # # # # # # */

    xTaskCreatePinnedToCore(
			micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            16,
            NULL,
			0
	);

	/*xTaskCreatePinnedToCore(
			perfil_quintico_local_task,
			"perfil_quintico_task",
			4096,          // stack (usa más que control)
			&joint1,       // si luego lo necesitas
			15,            // prioridad (entre ROS y control)
			NULL,
			1              // mismo core que control (recomendado)
	);*/

	/*xTaskCreatePinnedToCore(control_joint_task,
			"control_joint_task",
			4096,
			&joint1,
			14,
			NULL,
			1
	);*/

	xTaskCreatePinnedToCore(control_joint_task2,
			"control_joint_task2",
			4096,
			&joint1,
			14,
			NULL,
			1
	);

}
