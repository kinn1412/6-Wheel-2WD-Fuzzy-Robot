#include <stdio.h>
#include "esp_log.h"
#include "app_tasks.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "MAIN...");
    app_tasks_start();
}
