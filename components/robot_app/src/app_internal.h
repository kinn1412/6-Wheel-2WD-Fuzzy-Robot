#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include "robot_state.h"
#include "protocol.h"
#include "controller_if.h"

/* Shared, file-scope objects owned by the app layer. */
extern robot_state_t      g_state;        /* seqlock telemetry, 1 writer/N reader */
extern QueueHandle_t      g_cmd_mailbox;  /* len=1, xQueueOverwrite mailbox       */
extern _Atomic uint64_t   g_last_cmd_us;  /* watchdog timestamp                   */
extern controller_if_t    g_ctrl_l;       /* per-wheel Strategy controllers       */
extern controller_if_t    g_ctrl_r;

void task_speed_ctrl(void *arg);
void task_imu(void *arg);
void task_comm(void *arg);
void task_telemetry(void *arg);
void task_motor_bringup(void *arg);   /* Phase-4 bring-up only (see APP_MODE) */
void task_speed_test(void *arg);      /* Phase-5 closed-loop step test (see APP_MODE) */
void task_imu_test(void *arg);        /* Phase-6 IMU verification (see APP_MODE) */
