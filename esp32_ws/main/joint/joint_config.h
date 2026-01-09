#ifndef JOINT_CONFIG_H
#define JOINT_CONFIG_H

#include "driver/gpio.h" // Para importar tipos de datos de las funciones -> gpio_num_t
#include "driver/ledc.h" // Para importar tipos de datos de las funciones -> ledc_channel_t
#include "driver/pulse_cnt.h" // Para importar tipos de datos de las funciones -> pcnt_unit_handle_t

// COMUNICACIÓN UART0
#define UART0_TX GPIO_NUM_1              // 22. Transmisión  (no van a conectarse)
#define UART0_RX GPIO_NUM_3              // 23. Recepción    (no van a conectarse)
// COMUNICACIÓN UART1
#define UART1_TX GPIO_NUM_17             // 22. Transmisión  (no van a conectarse, solo para debuguear)
#define UART1_RX GPIO_NUM_16             // 23. Recepción    (no van a conectarse, solo para debuguear)


/* # # # # # # # # # # # #   DECLARACIÓN DE FUNCIONES   # # # # # # # # # # # # # */

esp_err_t joint_gpios_pwm_control_signal_setup(gpio_num_t gpio_direction_1,
                                               gpio_num_t gpio_direction_2,
                                               gpio_num_t gpio_pwm_velocity,
                                               ledc_channel_t gpio_pwm_velocity_channel);

esp_err_t joint_encoder_setup(gpio_num_t gpio_signal_A,
                        gpio_num_t gpio_signal_B,
                        pcnt_unit_handle_t * joint_encoder_pcnt_handler);

#endif //JOINT_CONFIG_H