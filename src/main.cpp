#include "main.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define LOG_TAG "MAIN"

#define PIN_LED GPIO_NUM_2

volatile unsigned long t_bldc_delay = 30;

// Task Handles
TaskHandle_t taskHandle_bldc = NULL;
TaskHandle_t taskHandle_leds = NULL;

// Timer Handles
gptimer_handle_t timer_bldc = NULL;
gptimer_handle_t timer_leds_activate = NULL;
gptimer_handle_t timer_leds_deactivate = NULL;
gptimer_handle_t timer_hall_simulation = NULL;

// Task Functions
void task_bldc(void *pvParameters);
void task_leds(void *pvParameters);

// Init Functions
void init_gpios();
void init_timers();

// Interupt Functions
static bool isr_bldc_startup(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
static bool isr_leds_activate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
static bool isr_leds_deactivate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
static bool isr_hall_simulation(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

extern "C" void app_main(void)
{
    init_gpios();
    init_timers();

    xTaskCreatePinnedToCore(task_bldc, "Task BLDC", 2048, NULL, tskIDLE_PRIORITY, &taskHandle_bldc, 0);
    vTaskSuspend(taskHandle_bldc);
    xTaskCreatePinnedToCore(task_leds, "Task LEDs", 2048, NULL, tskIDLE_PRIORITY, &taskHandle_leds, 1);
}

void init_gpios()
{
    ESP_ERROR_CHECK(gpio_set_direction(PIN_LED, GPIO_MODE_INPUT_OUTPUT));
}

void init_timers()
{
    // timer startup bldc
    gptimer_config_t timerConfig_bldc = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
        .intr_priority = 1,
    };

    gptimer_alarm_config_t alarmConfig_bldc = {
        .alarm_count = 500 * 1000,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };

    gptimer_event_callbacks_t cbConfig_bldc = {
        .on_alarm = isr_bldc_startup,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_bldc, &timer_bldc));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_bldc, &alarmConfig_bldc));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_bldc, &cbConfig_bldc, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer_bldc));

    // timer leds activate
    gptimer_config_t timerConfig_leds_activate = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };

    gptimer_alarm_config_t alarmConfig_leds_activate = {
        .alarm_count = 5 * 1000 * 1000,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };

    gptimer_event_callbacks_t cbConfig_leds_activate = {
        .on_alarm = isr_leds_activate,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_leds_activate, &timer_leds_activate));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_leds_activate, &alarmConfig_leds_activate));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_leds_activate, &cbConfig_leds_activate, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer_leds_activate));

    // timer leds deactivate
    gptimer_config_t timerConfig_leds_deactivate = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };

    gptimer_alarm_config_t alarmConfig_leds_deactivate = {
        .alarm_count = 5 * 1000 * 1000,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };

    gptimer_event_callbacks_t cbConfig_leds_deactivate = {
        .on_alarm = isr_leds_deactivate,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_leds_deactivate, &timer_leds_deactivate));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_leds_deactivate, &alarmConfig_leds_deactivate));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_leds_deactivate, &cbConfig_leds_deactivate, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer_leds_deactivate));

    // timer hall simulation
    gptimer_config_t timerConfig_hall_simulation = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };

    gptimer_alarm_config_t alarmConfig_hall_simulation = {
        .alarm_count = 5 * 1000 * 1000,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };

    gptimer_event_callbacks_t cbConfig_hall_simulation = {
        .on_alarm = isr_hall_simulation,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_hall_simulation, &timer_hall_simulation));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_hall_simulation, &alarmConfig_hall_simulation));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_hall_simulation, &cbConfig_hall_simulation, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer_hall_simulation));

    // timer startup
    ESP_ERROR_CHECK(gptimer_start(timer_bldc));
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
        ESP_LOGI(LOG_TAG, "Value of t_bldc_delay %lu", t_bldc_delay);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool IRAM_ATTR isr_bldc_startup(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    ESP_ERROR_CHECK(gpio_set_level(PIN_LED, !gpio_get_level(PIN_LED))); // toggle led
    gptimer_alarm_config_t alarm_config;
    t_bldc_delay = t_bldc_delay - 1;
    switch (t_bldc_delay)
    {
    case 20:
        alarm_config.alarm_count = 250000;
        alarm_config.reload_count = 0;
        alarm_config.flags.auto_reload_on_alarm = true;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
        break;
    case 10:
        alarm_config.alarm_count = 125000;
        alarm_config.reload_count = 0;
        alarm_config.flags.auto_reload_on_alarm = true;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
        break;
    case 0:
        ESP_ERROR_CHECK(gptimer_stop(timer));
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
    // do stuff
    return true;
}