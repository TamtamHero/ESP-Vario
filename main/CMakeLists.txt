# Embed the server root certificate into the final binary
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS
    "task_vario.c"
    "task_buzzer.c"
    "main.c"
    "task_ota.c"
    "task_user_inputs.c"
    "task_buzzer.c"
    "task_vario.c"
    "adc.c"
    "mpu6050.c"
    "kalmanfilter4d.c"
    "nvd.c"
    "ringbuf.c"
    "imu.c"
    "vario_audio.c"
    "ui.c"
    "task_ble_server.c"
    "leds.c"
    "wifi.c"
    "state.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
add_compile_definitions(LOG_LOCAL_LEVEL=ESP_LOG_DEBUG)