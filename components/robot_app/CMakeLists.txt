# L5 — Orchestration: FreeRTOS tasks, robot FSM, watchdog. Top of the stack.
idf_component_register(
    SRCS "src/app_tasks.c"
         "src/robot_fsm.c"
         "src/task_speed_ctrl.c"
         "src/task_imu.c"
         "src/task_comm.c"
         "src/task_telemetry.c"
         "src/task_motor_bringup.c"
         "src/task_speed_test.c"
         "src/task_imu_test.c"
    INCLUDE_DIRS "include"
    REQUIRES robot_hal control services bsp
    PRIV_REQUIRES freertos esp_timer log
)
