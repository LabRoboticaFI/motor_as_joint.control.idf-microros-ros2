idf_component_register(
    SRCS 
        main.c
        microros/esp32_serial_transport.c
        microros/micro_ros_task.c
        joint/joint_config.c
        joint/control_task.c

    INCLUDE_DIRS .
)
