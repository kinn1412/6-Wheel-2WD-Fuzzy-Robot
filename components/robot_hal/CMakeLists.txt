# L2 — Hardware Abstraction. Wraps ESP-IDF drivers; exposes physical units.
# hal_imu.cpp is C++ (wraps the esp32_BNO08x driver) behind the plain-C hal_imu.h.
idf_component_register(
    SRCS "src/hal_motor.c"
         "src/hal_encoder.c"
         "src/hal_imu.cpp"
         "src/hal_uart_link.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp
    PRIV_REQUIRES driver esp_driver_mcpwm esp_driver_pcnt esp_driver_uart esp_driver_gpio esp_timer log esp32_BNO08x
)

# Enable the real BNO085 backend in hal_imu.cpp (else it builds a yaw=0 stub).
target_compile_definitions(${COMPONENT_LIB} PRIVATE HAL_IMU_USE_BNO08X)
