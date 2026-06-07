/**
 * @file controller_pid.h
 * @brief Factory: wrap a (caller-owned) pid_t into a controller_if vtable.
 */
#pragma once
#include "controller_if.h"
#include "pid.h"

controller_if_t controller_pid_make(pid_t *pid);
