#include "robot_fsm.h"
#include "safety.h"

robot_state_e robot_fsm_step(robot_state_e cur, unsigned fault_flags, int cmd_mode) {
    if (fault_flags & FAULT_ESTOP) return ROBOT_ESTOP;
    if (fault_flags & ~FAULT_NONE) return ROBOT_FAULT;
    switch (cur) {
        case ROBOT_BOOT:  return ROBOT_IDLE;
        case ROBOT_IDLE:  return (cmd_mode == 1) ? ROBOT_RUN : ROBOT_IDLE;
        case ROBOT_RUN:   return (cmd_mode == 1) ? ROBOT_RUN : ROBOT_IDLE;
        case ROBOT_FAULT: return (fault_flags == FAULT_NONE) ? ROBOT_IDLE : ROBOT_FAULT;
        case ROBOT_ESTOP: return ROBOT_ESTOP;   /* latched until reset */
        default:          return ROBOT_BOOT;
    }
}

const char *robot_fsm_name(robot_state_e s) {
    switch (s) {
        case ROBOT_BOOT:  return "BOOT";
        case ROBOT_IDLE:  return "IDLE";
        case ROBOT_RUN:   return "RUN";
        case ROBOT_FAULT: return "FAULT";
        case ROBOT_ESTOP: return "ESTOP";
        default:          return "?";
    }
}
