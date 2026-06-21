#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/init_options.h"
#include "microros/esp32_serial_transport.h"

#include "microros/micro_ros_task.h"
#include "joint/joint_config.h"
#include "joint/control_task.h"

static const char *TAG_MAIN = "MAIN_C"; // El 'TAG' es una etiqueta que se utiliza para identificar la fuente del mensaje de registro.

/* # # # # # # # # # #   FUNCIÓN PRINCIPAL (MAIN)   # # # # # # # # # # # */

// Especificación del puerto por donde se comunicará microros con el agente de microros.
static size_t uart_port = UART_NUM_0;

// Estructura de dato para configurar los parámetros de la articulación
joint_t joint1 = {

	// Configuración de parámetros del MOTOR
	.motor.config = {
		.gpio_direction_1 = GPIO_NUM_19,
		.gpio_direction_2 = GPIO_NUM_21,
		.velocity_pwm_gpio = GPIO_NUM_2,
		.velocity_pwm_channel = LEDC_CHANNEL_0,
		.velocity_pwm_min_value = 0,
		.velocity_pwm_max_value = 1023, // de acuerdo con la resolución del timer del pwm configurado en joint_motor_setup(), en joint_config.c
		.velocity_pwm_to_start_forward = 557,
	},

	// Configuración de parámetros del ENCODER
	.encoder.config = {
		.gpio_signal_A = GPIO_NUM_22,
		.gpio_signal_B = GPIO_NUM_23,
		//.encoder_ratio = 1135, // Cantidad de ticks por vuelta del motor (PARA MOTOR DC 12V CON ENCODER)
		.encoder_ratio = 1135, // Cantidad de ticks por vuelta del motor
		.redutor_ratio = 1,	   // Cantidad de vueltas que da el motor para que el actuador dé una vuelta.
	},

	// Configuración de parámetros del PID
	.control.pid.config = {
		.k_p = 100,
		.k_i = 500,
		.k_d = 0,
		.int_min = -800,
		.int_max = 800,
		.out_min = -1000,
		.out_max = 1000,
		.error_min = -0.1,
		.error_max = 0.1,
	},

	// Configuración de parámetros del FILTRO de q_des
	.control.command_filter.config = {
		.q_min = -360, // Relacionado al límite mecánico de las articulaciones
		.q_max = 360,
		.delta_q_min = -720, // Relacionado al seguimiento de trayectoria y grandes diferencias de posición. Para evitar golpes.
		.delta_q_max = 720,
	},

	// Configuración de parámetros del perfil QUÍNTICO LOCAL
	.control.quintico.config = {
		.periodo_quintico = 3 * 1000 * 1000, // en micro-segundos (us)
	},

	// Configuración de parámetros del perfil QUÍNTICO LOCAL
	.comms = {
		.TAG = "JOINT1",
		.xQueue_feedback = NULL,
		.xQueue_q_des = NULL,
		//.xQueue_q_actual = NULL,
		//.xQueue_quintico = NULL,
	},
};

void app_main(void)
{

	/* # # # # # # # # # # # # # # # # # #   FASE 1: Configuraciones   # # # # # # # # # # # # # */

	// Creación de Queue ANTES de lanzar tareas, de acuerdo al diagrama de bloques de task del README.md
	joint1.comms.xQueue_q_des = xQueueCreate(1, sizeof(float));
	joint1.comms.xQueue_feedback = xQueueCreate(1, sizeof(float));

	// Configuración de GPIOs que controlan el movimiento del motor
	joint_motor_setup(joint1.motor.config);

	// Configuración de periférico PCNT para conteo de encoder.
	joint_encoder_setup(joint1.encoder.config, &joint1.encoder.pcnt_handle);



// 2.- Configuración de transporte personalizado por UART-USB
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
	rmw_uros_set_custom_transport(
		true,
		(void *)&uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read);
	ESP_LOGI(TAG_MAIN, "Se configuró correctamente el transporte personalizado.");
#else
#error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

	/* # # # # # # # # # # # # # # # #   FASE 2: Lanzamiento de tareas   # # # # # # # # # # # # */

	// NUCLEO 0
	xTaskCreatePinnedToCore(
		micro_ros_task,
		"micro_ros_task",
		CONFIG_MICRO_ROS_APP_STACK,
		NULL,
		16,
		NULL,
		0);

	// NUCLEO 1
	xTaskCreatePinnedToCore(control_joint_task,
							"control_joint_task2",
							4096,
							&joint1,
							14,
							NULL,
							1);
}
