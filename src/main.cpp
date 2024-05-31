#include "main.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define LOG_TAG "MAIN"

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
BLDC bldc;
volatile unsigned long t_bldc_delay = 350;

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
const std::vector<int> led_pins = {15, 2, 4, 16, 17, 5, 18, 19, 21, 22};
std::vector<Chamber> chambers;
std::vector<Timeslot> timeslots;
Timeslot *timeslot_ptr;
int64_t t_calculation_error_start;
int64_t t_calculation_error;

// Mutex Handles
SemaphoreHandle_t mutex_bldc;
SemaphoreHandle_t mutex_leds;

// Task Handles
TaskHandle_t taskHandle_bldc = NULL;
TaskHandle_t taskHandle_leds = NULL;

// Timer Handles
gptimer_handle_t timerHandle_leds_activate = NULL;
gptimer_handle_t timerHandle_leds_deactivate = NULL;
gptimer_handle_t timerHandle_bldc = NULL;
gptimer_handle_t timerHandle_hall = NULL;

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
static bool IRAM_ATTR isr_bldc_startup(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    t_bldc_delay -= 1;
    gptimer_alarm_config_t timerAlarm_bldc;
    switch (t_bldc_delay)
    {
    case 29:
        timerAlarm_bldc.alarm_count = 500000;
        timerAlarm_bldc.reload_count = 0;
        timerAlarm_bldc.flags.auto_reload_on_alarm = true;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_bldc, &timerAlarm_bldc));
        break;
    case 9:
        timerAlarm_bldc.alarm_count = 4500000;
        timerAlarm_bldc.reload_count = 0;
        timerAlarm_bldc.flags.auto_reload_on_alarm = true;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_bldc, &timerAlarm_bldc));
        break;
    case 6:
        gptimer_disable(timerHandle_bldc);
        xTaskResumeFromISR(taskHandle_leds);
        break;
    }
    return true;
}

static bool IRAM_ATTR isr_hall_simulation(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    t_hall_old = t_hall_new;
    t_hall_new = esp_timer_get_time();
    xTaskResumeFromISR(taskHandle_leds);
    return true;
}

static void IRAM_ATTR isr_hall(void *arg)
{
    t_hall_old = t_hall_new;
    t_hall_new = esp_timer_get_time();
    xTaskResumeFromISR(taskHandle_leds);
}

static bool IRAM_ATTR isr_leds_activate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    for (gpio_num_t pin : timeslot_ptr->pins)
    {
        gpio_set_level(pin, 1);
    }
    // deactivation timer setzen
    gptimer_alarm_config_t alarmConfig;
    alarmConfig.alarm_count = t_led_on;
    alarmConfig.flags.auto_reload_on_alarm = false;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds_deactivate, &alarmConfig));
    ESP_ERROR_CHECK(gptimer_start(timerHandle_leds_deactivate));
    return true;
}

static bool IRAM_ATTR isr_leds_deactivate(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
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
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds_activate, &alarmConfig));
        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds_deactivate));
    }
    return true;
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
            ESP_LOGE(LOG_TAG, "Couldn't update symbols properly.");
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
    timerAlarm_bldc.alarm_count = 5000;
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
        bldc.DoRotate();
        delayMicroseconds(t_bldc_delay);
    }
}

void task_leds(void *pvParameters)
{
    // initialise chambers
    for (int pin : led_pins)
    {
        ESP_ERROR_CHECK(gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT));
        Chamber chamber = {
            .pin = (gpio_num_t)pin,
            .symbol = symbols[0],
            .time = 0,
        };
        chambers.push_back(chamber);
    }

    // hall interrupt setup and start (to get rotation speed immediately)
    if (SIMULATION_MODE)
    {
        gptimer_config_t timerConfig_hall;
        timerConfig_hall.clk_src = GPTIMER_CLK_SRC_DEFAULT;
        timerConfig_hall.direction = GPTIMER_COUNT_UP;
        timerConfig_hall.resolution_hz = 60;
        timerConfig_hall.intr_priority = 1;
        ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_hall, &timerHandle_hall));

        gptimer_alarm_config_t timerAlarm_hall;
        timerAlarm_hall.alarm_count = 1;
        timerAlarm_hall.reload_count = 0;
        timerAlarm_hall.flags.auto_reload_on_alarm = true;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_hall, &timerAlarm_hall));

        gptimer_event_callbacks_t timerCb_hall;
        timerCb_hall.on_alarm = isr_hall_simulation;
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle_hall, &timerCb_hall, NULL));
        ESP_ERROR_CHECK(gptimer_enable(timerHandle_hall));
        ESP_ERROR_CHECK(gptimer_start(timerHandle_hall));
    }
    else
    {
        ESP_ERROR_CHECK(gpio_reset_pin(PIN_HALL));
        ESP_ERROR_CHECK(gpio_set_direction(PIN_HALL, GPIO_MODE_INPUT));
        // ESP_ERROR_CHECK(gpio_pullup_en(PIN_HALL)); // TODO: Check if it helps
        ESP_ERROR_CHECK(gpio_pulldown_dis(PIN_HALL));
        ESP_ERROR_CHECK(gpio_set_intr_type(PIN_HALL, GPIO_INTR_NEGEDGE));
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HALL, isr_hall, NULL));
        ESP_ERROR_CHECK(gpio_intr_enable(PIN_HALL));
    }

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
    vTaskSuspend(NULL); // TODO: IMPORTANT! change to mutex or semaphore?
    gptimer_del_timer(timerHandle_bldc);
    timerHandle_bldc = NULL;

    gptimer_alarm_config_t timerAlarm_leds_activate;
    while (true)
    {
        vTaskSuspend(NULL);
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
            ESP_LOGE(LOG_TAG, "LED calculation takes too long. alarm_count = %llu", timerAlarm_leds_activate.alarm_count);
        }
    }
}

extern "C" void app_main(void)
{
    if (symbols.length() != led_pins.size())
    {
        ESP_LOGE(LOG_TAG, "num of symbols does not match num of led_gpios");
    }
    else
    {
        t_chamber_delta_faktor = calculateTimeFactor(18);
        t_chamber_start_faktor = calculateTimeFactor(162);
        t_led_on_faktor = calculateTimeFactor(8);
        xTaskCreatePinnedToCore(task_leds, "Task LEDs", 4096, NULL, 1, &taskHandle_leds, 0);
        xTaskCreatePinnedToCore(task_bldc, "Task BLDC", 4096, NULL, 1, &taskHandle_bldc, 1); // only task_bldc on core 1 allowed
    }
}