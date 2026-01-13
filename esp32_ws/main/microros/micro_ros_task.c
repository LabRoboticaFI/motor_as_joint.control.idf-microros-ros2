#include "microros/micro_ros_task.h" // Librería de este archivo
#include "joint/control_task.h"		 // Para trabjar con las estructuras de datos de las articulaciones.

#include "std_msgs/msg/float32.h" // Tipo de mensaje que se recibe de los tópicos

#include "rmw_microros/init_options.h" // Para la configuración del cliente en microros.

#include "esp_log.h" // Para debugguear

static const char *TAG_MICRO_ROS_TASK = "MICRO_ROS_TASK_C"; // El 'TAG' es una etiqueta que se utiliza para identificar la fuente del mensaje de registro.

// 1.- Definición de publicadres y suscriptores. (variables, se usan en la parte de configuración del nodo en microros_task().)
rcl_subscription_t q1_des_suscriber;
rcl_publisher_t q1_feedback_publisher;

// 2.- Definición de interfaces y variables de comunicación
std_msgs__msg__Float32 msg_in; // Mensaje que se recibe del tópico en el suscriptor, si se fija se pasa como parámetro en subscription_callback_q1()
float msg_out;				   // Variable para recibir el dato de la Queue de feedback.

// 3.- Definción de funciones callback.

// Función callback del SUSCRIPTOR
void subscription_callback_q1(const void *_msg_in_)
{
	const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)_msg_in_; // Mensaje que recibe del tópico.

	float q1_des = msg->data; // Se obtiene el dato "data" del mensaje

	// ESP_LOGI(TAG_MICRO_ROS_TASK, "Llegó una nueva referencia a microros: %.3f", q1_des);

	xQueueOverwrite(joint1.comms.xQueue_q_des, &q1_des); // Se publica en la Queue hacia el perfil_quintico_local_task()
}

// Función callback del PUBLICADOR
void timer_callback_feedback_q1(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		if (xQueueReceive(joint1.comms.xQueue_feedback, &msg_out, 0))
		{																		 // Recibe la posición actual (feedback) del Queue desde control_task()
			std_msgs__msg__Float32 msg_status;									 // Variable de tipo msg/Float32 para publicar en el tópico de feedback.
			msg_status.data = msg_out;											 // Se asigna el dato de posición actual al msg.
			RCSOFTCHECK(rcl_publish(&q1_feedback_publisher, &msg_status, NULL)); // Se publica en el tópico de feedback hacia el agente de microros.
		}
	}
}

// # # # # # #   FUNCIÓN PRINCIPAL DE MICROROS   # # # # # #

void micro_ros_task(void *arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rcl_init_options_init(&init_options, rcl_get_default_allocator());

	// Take RMW options from RCL options
	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Set RMW client key
	RCCHECK(rmw_uros_options_set_client_key(0x01, rmw_options)); // <----------------------- Configuración de número de cliente de microros.

	// Initialize support with options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	ESP_LOGI(TAG_MICRO_ROS_TASK, "No sé qué es esto, pero ya esta creado");

	// create NODE
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "motor_dc_as_joint", "", &support)); // <---------- Nombre del nodo.

	// Create SUSCRIBER.
	RCCHECK(rclc_subscription_init_default(
		&q1_des_suscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), // <--------------------------- tipo de mensaje del tópico que escucha el suscriber.
		"/motor_as_joint/dc/q_des"));						 // <-------------------------------------------------- nombre del tópico que escucha el suscriber.
	ESP_LOGI(TAG_MICRO_ROS_TASK, "Suscriptor creado...");

	// create PUBLISHER
	RCCHECK(rclc_publisher_init_default(
		&q1_feedback_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), // <--------------------------- tipo de mensaje del tópico en el publica el publisher.
		"/motor_as_joint/dc/joint_state"));					 // <-------------------------------------------- nombre del tópico en el que publica el publisher.
	ESP_LOGI(TAG_MICRO_ROS_TASK, "Publicador creado...");

	// create TIMER,
	rcl_timer_t timer; // <----------------------------------------------------------------- Configuración del timer, para el callback del publisher.
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback_feedback_q1,
		true));
	ESP_LOGI(TAG_MICRO_ROS_TASK, "Timer creado...");

	// create EXECUTOR
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &q1_des_suscriber, &msg_in, &subscription_callback_q1, ON_NEW_DATA));
	ESP_LOGI(TAG_MICRO_ROS_TASK, "Ejecutor creado...");

	// Mantener el nodo activo.
	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	ESP_LOGI(TAG_MICRO_ROS_TASK, "Liberando memoria...");
	// free resources
	RCCHECK(rcl_subscription_fini(&q1_des_suscriber, &node));
	RCCHECK(rcl_publisher_fini(&q1_feedback_publisher, &node));
	RCCHECK(rcl_node_fini(&node)); // <---- Liberación de recursos, agregar libreación de recursos cuando se agreguen nuevos suscriptores, publicardores, etc.

	vTaskDelete(NULL);
}