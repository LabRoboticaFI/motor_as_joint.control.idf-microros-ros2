#include "joint/joint_config.h" // Librería de este archivo

#include "driver/gpio.h" // Para gpios
#include "driver/ledc.h" // Para pwm

#include "esp_log.h" // Para logs, para debugguear
#include "esp_err.h" // Para detectar errores en las configuraciones

static const char *TAG = "joint_config";



esp_err_t joint_gpios_pwm_control_signal_setup(gpio_num_t gpio_direction_1,
                                               gpio_num_t gpio_direction_2,
                                               gpio_num_t gpio_pwm_velocity,
                                               ledc_channel_t gpio_pwm_velocity_channel){
    
    ESP_LOGI(TAG, "setting joints configurations...");

    // # # # # # # # # # # # #   PWM PARA MOTOR DC   # # # # # # # # # # # # # #
    static bool flag = true;
    if(flag == true){
        static ledc_timer_config_t timerConfig = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, // Aguas con la aplicación de timers, ya utiliza el timer 0 con esta aplicación.
        .freq_hz = (uint32_t) 20 * 1000,             // 20kHz
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));
        flag = false;
    }

    ledc_channel_config_t channelConfig_left = {
        .gpio_num = gpio_pwm_velocity,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = gpio_pwm_velocity_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channelConfig_left));



    // # # # # # # # # # # # #   GPIOs PARA DIRECCIONAMIENTO DE PUENTE H   # # # # # # # # # # # # # 

    gpio_set_direction(gpio_direction_1, GPIO_MODE_OUTPUT); // Se configura la dirección output del pin
    gpio_set_level(gpio_direction_1, 0);

    gpio_set_direction(gpio_direction_2, GPIO_MODE_OUTPUT); // Se configura la dirección output del pin
    gpio_set_level(gpio_direction_2, 0);

    return ESP_OK;
}


esp_err_t joint_encoder_setup(gpio_num_t gpio_signal_A,
                        gpio_num_t gpio_signal_B,
                        pcnt_unit_handle_t * joint_encoder_pcnt_handler){

    // # Configuración de Límites de conteo
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000, // Editar cuando se tengan límites mínimos y máximos de la articulación (traducirlos a ticks)
        .low_limit = -10000, // Editar cuando se tengan límites mínimos y máximos de la articulación (traducirlos a ticks)
    };
    *joint_encoder_pcnt_handler = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, joint_encoder_pcnt_handler));



    // # Configuración del filtro de Glitches
    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000, // Lo dejo como estaba, pero si tenemos un filtrado del Schmitt Trigger antes del GPIO, se podría bajar. Esto es más bien para encoders mecánicos.
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*joint_encoder_pcnt_handler, &filter_config));



    // # Configuración de canales
    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = gpio_signal_A,
        .level_gpio_num = gpio_signal_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*joint_encoder_pcnt_handler, &chan_a_config, &pcnt_chan_a));
    
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = gpio_signal_B,
        .level_gpio_num = gpio_signal_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*joint_encoder_pcnt_handler, &chan_b_config, &pcnt_chan_b));



    // # Configuración de acciones de canales
    //   Define qué pasa cuando llega un flanco y el nivel está alto/bajo:
    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));


    // # Inicialización
    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(*joint_encoder_pcnt_handler));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*joint_encoder_pcnt_handler));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(*joint_encoder_pcnt_handler));

    return ESP_OK;
}