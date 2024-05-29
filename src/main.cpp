#include "main.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define LOG_TAG "MAIN"

#define PIN_LED_BLDC GPIO_NUM_2
#define PIN_LED_HALL GPIO_NUM_4

volatile unsigned long t_bldc_delay = 30;

// Task Handles
TaskHandle_t taskHandle_bldc = NULL;
TaskHandle_t taskHandle_leds = NULL;

// Timer Handles
HWTimer timer_bldc;
HWTimer timer_leds_activate;
HWTimer timer_leds_deactivate;
HWTimer timer_hall_simulation;

// Task Functions
void task_bldc(void *pvParameters);
void task_leds(void *pvParameters);

// Interupt Functions
static bool isr_bldc_startup(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
static bool isr_leds_activate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
static bool isr_leds_deactivate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
static bool isr_hall_simulation(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

extern "C" void app_main(void)
{
    xTaskCreatePinnedToCore(task_bldc, "Task BLDC", 4096, NULL, 1, &taskHandle_bldc, 0);
    xTaskCreatePinnedToCore(task_leds, "Task LEDs", 4096, NULL, 1, &taskHandle_leds, 1);
}

void task_bldc(void *pvParameters)
{
    ESP_ERROR_CHECK(gpio_set_direction(PIN_LED_BLDC, GPIO_MODE_INPUT_OUTPUT));
    timer_bldc = HWTimer(500000, true, isr_bldc_startup);
    timer_bldc.start();

    while (true)
    {
        ESP_LOGI(LOG_TAG, "Value of t_bldc_delay %lu", t_bldc_delay);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_leds(void *pvParameters)
{
    ESP_ERROR_CHECK(gpio_set_direction(PIN_LED_HALL, GPIO_MODE_INPUT_OUTPUT));
    timer_hall_simulation = HWTimer(100000, true, isr_hall_simulation);
    vTaskSuspend(NULL);

    vTaskSuspend(taskHandle_bldc);
    timer_bldc.remove();
    timer_hall_simulation.start();
    while (true)
    {
        ESP_LOGI(LOG_TAG, "I am alive!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool IRAM_ATTR isr_bldc_startup(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    ESP_ERROR_CHECK(gpio_set_level(PIN_LED_BLDC, !gpio_get_level(PIN_LED_BLDC))); // toggle led
    t_bldc_delay = t_bldc_delay - 1;
    switch (t_bldc_delay)
    {
    case 20:
        timer_bldc.setAlarmCount(250000);
        break;
    case 10:
        timer_bldc.setAlarmCount(125000);
        break;
    case 0:
        timer_bldc.stop();
        xTaskResumeFromISR(taskHandle_leds);
        break;
    }
    return true;
}

static bool IRAM_ATTR isr_leds_activate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    // do stuff
    return true;
}

static bool IRAM_ATTR isr_leds_deactivate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    // do stuff
    return true;
}

static bool IRAM_ATTR isr_hall_simulation(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    ESP_ERROR_CHECK(gpio_set_level(PIN_LED_HALL, !gpio_get_level(PIN_LED_HALL)));
    return true;
}