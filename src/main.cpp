#include "main.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define LOG_TAG "MAIN"

#define PIN_LED GPIO_NUM_2

// Task Handles
TaskHandle_t taskHandle_bldc = NULL;
TaskHandle_t taskHandle_leds = NULL;

gptimer_handle_t timer0 = NULL;

// Task Functions
void task_bldc(void *pvParameters);
void task_leds(void *pvParameters);
static bool isr_timer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(gpio_set_direction(PIN_LED, GPIO_MODE_INPUT_OUTPUT));

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer0));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 5 * 1000 * 1000,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer0, &alarm_config));

    gptimer_event_callbacks_t callback = {
        .on_alarm = isr_timer};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer0, &callback, NULL));

    ESP_ERROR_CHECK(gptimer_enable(timer0));
    ESP_ERROR_CHECK(gptimer_start(timer0));

    xTaskCreatePinnedToCore(task_bldc, "Task BLDC", 2048, NULL, tskIDLE_PRIORITY, &taskHandle_bldc, 0);
    vTaskSuspend(taskHandle_bldc);
    xTaskCreatePinnedToCore(task_leds, "Task LEDs", 2048, NULL, tskIDLE_PRIORITY, &taskHandle_leds, 1);
}

void task_bldc(void *pvParameters)
{
    while (true)
    {
        ESP_LOGI(LOG_TAG, "Task BLDC running on core %d", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_leds(void *pvParameters)
{
    while (true)
    {
        ESP_LOGI(LOG_TAG, "Level of GPIO LED = %d", gpio_get_level(PIN_LED));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool IRAM_ATTR isr_timer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    gpio_set_level(PIN_LED, !gpio_get_level(PIN_LED));
    return true;
}