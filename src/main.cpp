#include "main.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define TAG_MAIN "MAIN"
#define TAG_BLDC "TASK BLDC"
#define TAG_LEDS "TASK LEDS"

#define RESOLUTION_MICROSECONDS 1000000

/*  TODO:
/   -test task_bldc
    -test task_leds
/   -implement time/date mode switch button
/   -implement symbols update mechanism
/   -implement serial communication mechanism
*/

// structs
typedef struct
{
    gpio_num_t pin;
    char symbol;
    unsigned long time;
} Chamber;

typedef struct
{
    unsigned long time;
    std::vector<gpio_num_t> pins;
} Timeslot;

// Clock/Time Mode
bool show_time = false;

// Sensor PINs
gpio_num_t PIN_HALL = GPIO_NUM_34;

// BLDC
const uint8_t BLDC_DRV_IN1 = 27;
const uint8_t BLDC_DRV_IN2 = 26;
const uint8_t BLDC_DRV_IN3 = 25;
const uint8_t BLDC_DRV_EN1 = 13;
const uint8_t BLDC_DRV_EN2 = 12;
const uint8_t BLDC_DRV_EN3 = 14;
const uint8_t BLDC_MOTOR_POLE = 7;
UcnBrushlessDCMotorPWM bldc;
volatile uint32_t t_bldc_delay = 1500;
uint32_t startValue_bldc = 30;
double exponent = 0.0;

// Rotation time variables
volatile int64_t t_hall_old = esp_timer_get_time();
volatile int64_t t_hall_new = esp_timer_get_time();
int64_t t_led_on;
double t_led_on_faktor;
int64_t t_hall_delta;
int64_t t_chamber_start;
double t_chamber_start_faktor;
int64_t t_chamber_delta;
double t_chamber_delta_faktor;
int64_t t_placement_error = 0;

// chamber variables
const std::string symbols = "0123456789:.";
const std::vector<uint8_t> led_pins = {15, 2, 4, 16, 17, 5, 18, 19, 21, 22};
std::vector<Chamber> chambers;
std::vector<Timeslot> timeslots;
Timeslot *timeslot_ptr;
int64_t t_calculation_error_start;
int64_t t_calculation_error;

// Mutex Handles
SemaphoreHandle_t semaphore_ledLoop = NULL;

// Task Handles
TaskHandle_t taskHandle_bldc = NULL;
TaskHandle_t taskHandle_leds = NULL;
TaskHandle_t taskHandle_cleanup = NULL;

// Timer Handles
gptimer_handle_t timerHandle_leds_activate = NULL;
gptimer_handle_t timerHandle_leds_deactivate = NULL;
gptimer_handle_t timerHandle_bldc = NULL;

// Task Functions
void task_bldc(void *pvParameters);
void task_leds(void *pvParameters);

// Functions
void calculateAbsoluteChamberTimes();
void calculateRelativeTimeslots();
void fitStringSize(std::string &input);
void updateSymbols(std::string &symbols);
void toggleMode();

// ISRs
static bool IRAM_ATTR isr_decrease_power(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    bldc.SetPower(80);
    ESP_ERROR_CHECK(gptimer_disable(timerHandle_bldc));
    xTaskResumeFromISR(taskHandle_cleanup);

    if (xHigherPriorityTaskWoken == pdTRUE) // Prüfe, ob ein höher priorisierter Task geweckt wurde
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Erzwinge einen Kontextwechsel
    }

    return (xHigherPriorityTaskWoken == pdTRUE);
}

static bool IRAM_ATTR isr_bldc_startup(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    gptimer_alarm_config_t timerAlarm_bldc;

    if (t_bldc_delay > 55) // 69 -> 60Hz, 55 -> 72Hz, 42 -> 90Hz
    {
        t_bldc_delay -= 1;
    }
    else
    {
        xTaskResumeFromISR(taskHandle_leds);
    }
    timerAlarm_bldc.alarm_count = exp(exponent) * startValue_bldc;
    timerAlarm_bldc.reload_count = 0;
    timerAlarm_bldc.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_bldc, &timerAlarm_bldc));
    exponent += 0.00495;
    return true;
}

static void IRAM_ATTR isr_hall(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    t_hall_old = t_hall_new;
    t_hall_new = esp_timer_get_time();

    xSemaphoreGiveFromISR(semaphore_ledLoop, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) // Prüfe, ob ein höher priorisierter Task geweckt wurde
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Erzwinge einen Kontextwechsel
    }
}

static bool IRAM_ATTR isr_leds_activate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    for (gpio_num_t pin : timeslot_ptr->pins)
    {
        gpio_set_level(pin, 1);
    }
    // deactivation timer setzen
    gptimer_alarm_config_t alarmConfig;
    alarmConfig.alarm_count = t_led_on;
    alarmConfig.flags.auto_reload_on_alarm = false;
    if (!first_activation)
    {
        ESP_ERROR_CHECK(gptimer_stop(timerHandle_leds_deactivate));
    }
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds_deactivate, &alarmConfig));
    ESP_ERROR_CHECK(gptimer_start(timerHandle_leds_deactivate));

    if (xHigherPriorityTaskWoken == pdTRUE) // Prüfe, ob ein höher priorisierter Task geweckt wurde
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Erzwinge einen Kontextwechsel
    }

    return (xHigherPriorityTaskWoken == pdTRUE);
}

static bool IRAM_ATTR isr_leds_deactivate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    for (gpio_num_t pin : timeslot_ptr->pins)
    {
        gpio_set_level(pin, 0);
    }
    timeslots.erase(timeslots.begin());

    if (!timeslots.empty())
    {
        timeslot_ptr = &timeslots.front();
        // activation timer setzen
        gptimer_alarm_config_t alarmConfig;
        alarmConfig.alarm_count = timeslot_ptr->time;
        alarmConfig.flags.auto_reload_on_alarm = false;
        ESP_ERROR_CHECK(gptimer_stop(timerHandle_leds_deactivate));
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds_activate, &alarmConfig));
        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds_deactivate));
    }

    if (xHigherPriorityTaskWoken == pdTRUE) // Prüfe, ob ein höher priorisierter Task geweckt wurde
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Erzwinge einen Kontextwechsel
    }

    return (xHigherPriorityTaskWoken == pdTRUE);
}

// Functions

void calculateAbsoluteChamberTimes()
{
    int j = 0;
    for (Chamber &chamber : chambers)
    {
        for (; j < 12; j++)
        {
            if (chamber.symbol == 'x')
            {
                chamber.time = -1;
            }
            if (chamber.symbol == symbols[j])
                break;
        }
        chamber.time = (t_chamber_start + t_chamber_delta * j);
    }
}

void calculateRelativeTimeslots() // TODO: laut ChatGPT fehlerhaft. Muss überprüft werden
{
    std::sort(chambers.begin(), chambers.end(), [](const Chamber &a, const Chamber &b)
              { return a.time < b.time; });

    timeslots.clear();
    Timeslot timeslot_buf;
    timeslot_buf.time = chambers[0].time,
    timeslot_buf.pins.push_back(chambers[0].pin);

    for (size_t i = 1; i < chambers.size(); i++)
    {
        if (chambers[i].time == timeslot_buf.time)
        {
            timeslot_buf.pins.push_back(chambers[i].pin);
        }
        else
        {
            timeslots.push_back(timeslot_buf);
            timeslot_buf.pins.clear();
            timeslot_buf.time = chambers[i].time;
            timeslot_buf.pins.push_back(chambers[i].pin);
        }
        timeslots.push_back(timeslot_buf);
    }

    if (show_time)
    {
        timeslots.erase(timeslots.begin(), timeslots.begin() + 2);
    }
    for (size_t i = chambers.size() - 1; i > 0; i--)
    {
        timeslots[i].time = timeslots[i].time - timeslots[i - 1].time;
    }
}

void fitStringSize(std::string &input)
{
    if (input.length() > 10)
    {
        input = input.substr(0, 10);
    }
    else if (input.length() < 10)
    {
        input.append(10 - input.length(), '0');
    }
    if (show_time)
    {
        input.front() = 'x';
        input.back() = 'x';
    }
}

void updateSymbols(std::string &symbols)
{
    fitStringSize(symbols);
    auto symbols_iter = symbols.begin();
    for (Chamber &chamber : chambers)
    {
        if (symbols_iter == symbols.end())
        {
            ESP_LOGE(TAG_MAIN, "Couldn't update symbols properly.");
            break;
        }
        chamber.symbol = *symbols_iter;
    }
}

void toggleMode()
{
    show_time = !show_time;
}

double calculateTimeFactor(uint16_t angle_deg)
{
    return angle_deg / 360;
}

// MAIN
void task_bldc(void *pvParameters)
{
    bldc.begin(BLDC_MOTOR_POLE, BLDC_DRV_IN1, BLDC_DRV_IN2, BLDC_DRV_IN3, BLDC_DRV_EN1, BLDC_DRV_EN2, BLDC_DRV_EN3);
    bldc.SetPower(100);

    gptimer_config_t timerConfig_bldc;
    timerConfig_bldc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig_bldc.direction = GPTIMER_COUNT_UP;
    timerConfig_bldc.resolution_hz = RESOLUTION_MICROSECONDS;
    timerConfig_bldc.intr_priority = 1;
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_bldc, &timerHandle_bldc));

    gptimer_alarm_config_t timerAlarm_bldc;
    timerAlarm_bldc.alarm_count = startValue_bldc;
    timerAlarm_bldc.reload_count = 0;
    timerAlarm_bldc.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_bldc, &timerAlarm_bldc));

    gptimer_event_callbacks_t timerCb_bldc;
    timerCb_bldc.on_alarm = isr_bldc_startup;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle_bldc, &timerCb_bldc, NULL));

    ESP_ERROR_CHECK(gptimer_enable(timerHandle_bldc));
    ESP_ERROR_CHECK(gptimer_start(timerHandle_bldc));

    while (true)
    {
        bldc.DoRotate(4);
        delayMicroseconds(t_bldc_delay);
    }
}

void task_measureSpeed(void *pvParametesr)
{
    t_hall_old = esp_timer_get_time();
    t_hall_new = esp_timer_get_time();

    ESP_ERROR_CHECK(gpio_reset_pin(PIN_HALL));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_HALL, GPIO_MODE_INPUT));
    // TODO: Check if it helps      ESP_ERROR_CHECK(gpio_pullup_en(PIN_HALL));
    ESP_ERROR_CHECK(gpio_pulldown_dis(PIN_HALL));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_HALL, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HALL, isr_hall, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(PIN_HALL));

    double speed = 0;
    while (true)
    {
        if (xSemaphoreTake(semaphore_ledLoop, portMAX_DELAY) == pdFALSE)
        {
            ESP_LOGE(TAG_LEDS, "semaphore_ledLoop could not be obtained!");
            continue;
        }
        speed = 1000000.0 / (double)(t_hall_new - t_hall_old);
        printf("\e[2J\e[H");
        ESP_LOGI(TAG_BLDC, "speed=%f", speed);
    }
}

void task_leds(void *pvParameters)
{
    // initialise chambers
    gpio_config_t conf;
    for (uint8_t pin : led_pins)
    {
        conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
        if (gpio_config(&conf) != ESP_OK)
        {
            ESP_LOGE(TAG_ACL, "GPIO config failed");
            return;
        }
        Chamber chamber = {
            .pin = (gpio_num_t)pin,
            .symbol = symbols[0],
            .time = 0,
        };
        chambers.push_back(chamber);
    }

    // hall interrupt setup and start (to get rotation speed immediately)
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_HALL));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_HALL, GPIO_MODE_INPUT));
    // TODO: Check if it helps      ESP_ERROR_CHECK(gpio_pullup_en(PIN_HALL));
    ESP_ERROR_CHECK(gpio_pulldown_dis(PIN_HALL));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_HALL, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HALL, isr_hall, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(PIN_HALL));

    // leds_activate timer setup
    gptimer_config_t timerConfig_leds_activate;
    timerConfig_leds_activate.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig_leds_activate.direction = GPTIMER_COUNT_UP;
    timerConfig_leds_activate.resolution_hz = RESOLUTION_MICROSECONDS;
    timerConfig_leds_activate.intr_priority = 1;
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_leds_activate, &timerHandle_leds_activate));

    gptimer_event_callbacks_t timerCb_leds_activate;
    timerCb_leds_activate.on_alarm = isr_leds_activate;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle_leds_activate, &timerCb_leds_activate, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timerHandle_leds_activate));

    // leds_deactivate timer setup
    gptimer_config_t timerConfig_leds_deactivate;
    timerConfig_leds_deactivate.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig_leds_deactivate.direction = GPTIMER_COUNT_UP;
    timerConfig_leds_deactivate.resolution_hz = RESOLUTION_MICROSECONDS;
    timerConfig_leds_deactivate.intr_priority = 1;
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_leds_deactivate, &timerHandle_leds_deactivate));

    gptimer_event_callbacks_t timerCb_leds_deactivate;
    timerCb_leds_deactivate.on_alarm = isr_leds_deactivate;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle_leds_deactivate, &timerCb_leds_deactivate, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timerHandle_leds_deactivate));

    // wait for bldc startup to finish
    vTaskSuspend(NULL);

    ESP_ERROR_CHECK(gptimer_stop(timerHandle_bldc));
    ESP_ERROR_CHECK(gptimer_disable(timerHandle_bldc));
    ESP_ERROR_CHECK(gptimer_del_timer(timerHandle_bldc));
    timerHandle_bldc = NULL;

    // LEDs mainloop
    gptimer_alarm_config_t timerAlarm_leds_activate;
    while (true)
    {
        if (xSemaphoreTake(semaphore_ledLoop, portMAX_DELAY) == pdFALSE)
        {
            ESP_LOGE(TAG_LEDS, "semaphore_ledLoop could not be obtained!");
            continue;
        }
        t_calculation_error_start = esp_timer_get_time();
        t_hall_delta = t_hall_new - t_hall_old;
        t_chamber_delta = (int64_t)(t_chamber_delta_faktor * (double)t_hall_delta);
        t_chamber_start = (int64_t)(t_chamber_start_faktor * (double)t_hall_delta);
        t_led_on = (int64_t)(t_led_on_faktor * (double)t_hall_delta);
        calculateAbsoluteChamberTimes();
        calculateRelativeTimeslots();
        timeslot_ptr = &timeslots[0];
        timerAlarm_leds_activate.reload_count = 0;
        timerAlarm_leds_activate.flags.auto_reload_on_alarm = false;
        t_calculation_error = esp_timer_get_time() - t_calculation_error_start;
        timerAlarm_leds_activate.alarm_count = timeslot_ptr->time - t_calculation_error;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds_activate, &timerAlarm_leds_activate));
        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds_activate));
        if (timerAlarm_leds_activate.alarm_count <= 0)
        {
            ESP_LOGE(TAG_MAIN, "LED calculation takes too long! alarm_count = %llu", timerAlarm_leds_activate.alarm_count);
        }
    }
}

extern "C" void app_main(void)
{
    // if (symbols.length() != led_pins.size())
    //{
    //     ESP_LOGE(LOG_TAG, "num of symbols does not match num of led_gpios");
    //     ESP_LOGE(LOG_TAG, "symbols.length()=%d, led_pins.size()=%d", symbols.length(), led_pins.size());
    //     return;
    // }

    semaphore_ledLoop = xSemaphoreCreateBinary();
    if (semaphore_ledLoop == NULL)
    {
        ESP_LOGE(TAG_MAIN, "Failed to create semaphore_ledLoop");
    }

    t_chamber_delta_faktor = calculateTimeFactor(18);
    t_chamber_start_faktor = calculateTimeFactor(162);
    t_led_on_faktor = calculateTimeFactor(8);
    xTaskCreatePinnedToCore(task_bldc, "Task BLDC", 6000, NULL, 2, &taskHandle_bldc, 1); // only task_bldc on core 1 allowed
    // xTaskCreatePinnedToCore(task_measureSpeed, "Task Speed", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_leds, "Task LEDs", 4096, NULL, 2, &taskHandle_leds, 0);
    //  vTaskSuspend(taskHandle_cleanup);
    //  vTaskSuspend(taskHandle_leds);
}