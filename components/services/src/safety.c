#include "safety.h"
#include "bsp_params.h"

bool safety_cmd_timed_out(uint64_t last_cmd_us, uint64_t now_us) {
    return (now_us - last_cmd_us) > BSP_CMD_TIMEOUT_US;
}
