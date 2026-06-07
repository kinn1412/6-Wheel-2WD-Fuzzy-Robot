/**
 * @file app_tasks.h
 * @brief L5 — system bring-up entry. Inits HAL, binds controllers, spawns tasks
 *        on the correct cores per the design (control on Core 1; I/O on Core 0).
 */
#pragma once

void app_tasks_start(void);
