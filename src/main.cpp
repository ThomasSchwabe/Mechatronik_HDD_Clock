#include "main.h"

#define LOG_LEVEL_LOVAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define LOG_TAG "MAIN"

#define PIN_LED GPIO_NUM_2

extern "C" void app_main(void)
{
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    while (true)
    {
        ESP_LOGI(LOG_TAG, "Turn LED on.");
        gpio_set_level(PIN_LED, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(LOG_TAG, "Turn LED off.");
        gpio_set_level(PIN_LED, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}