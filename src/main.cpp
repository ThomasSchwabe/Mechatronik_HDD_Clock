#include "main.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define TAG_MAIN "MAIN"
#define TAG_BLDC "TASK BLDC"
#define TAG_LEDS "TASK LEDS"

const double angle_chamber = 16;
const double angle_start = 186;
const double angle_on = 1.5;
// TODO: FIX CONSTRUCTION BECAUSE ANGLE CHAMBER VARIES AND ANGLE SYMBOL= const 18° const double angle_symbol =

// variables time measurement/calculation
volatile int64_t t_hall_old;
volatile int64_t t_hall_new;
int64_t t_hall_delta;

int64_t t_led_on;
double t_led_on_faktor;
int64_t t_chamber_start;
double t_chamber_start_faktor;
int64_t t_chamber_delta;
double t_chamber_delta_faktor;
int64_t t_symbol_delta;
double t_symbol_delta_faktor;

int64_t t_calculation_start[2];

volatile ActivationPhase leds_phase;

// chamber variables
Chamber chambers[NUM_CHAMBERS];
Timeslot timeslots[2][NUM_CHAMBERS];
uint8_t num_timeslots[2];
volatile uint8_t showPhase = 0;
uint8_t calculationPhase = 1;
volatile uint8_t active_timeslot;

// bldc variables
UcnBrushlessDCMotorPWM bldc;
volatile uint32_t t_bldc_delay = 1500;
uint32_t startValue_bldc = 50;
double exponent = 0.0;

// ##########FreeRTOS Management Variables//##########//
//  semaphores
SemaphoreHandle_t semaphore_CalculationSync = NULL;
SemaphoreHandle_t semaphore_phaseSync = NULL;
SemaphoreHandle_t semaphore_StartUpFinished = NULL;

// Task Handles
TaskHandle_t taskHandle_bldc = NULL;
TaskHandle_t taskHandle_leds = NULL;

// Task Functions
void task_bldc(void *pvParameters);
void task_leds(void *pvParameters);

// Timer Handles
gptimer_handle_t timerHandle_leds[2] = {NULL, NULL};
gptimer_handle_t timerHandle_bldc = NULL;

// ISRs
static void IRAM_ATTR isr_hall(void *arg)
{
    t_hall_old = t_hall_new;
    t_hall_new = esp_timer_get_time();
    xSemaphoreGiveFromISR(semaphore_CalculationSync, NULL);
}

static bool IRAM_ATTR isr_bldc_startup(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    gptimer_alarm_config_t timerAlarm;

    if (t_bldc_delay > 100) // 69 -> 60Hz, 55 -> 72Hz, stable max =
    {
        t_bldc_delay -= 1;
    }
    else
    {
        xSemaphoreGiveFromISR(semaphore_StartUpFinished, NULL);
    }
    timerAlarm.alarm_count = exp(exponent) * startValue_bldc;
    timerAlarm.reload_count = 0;
    timerAlarm.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_bldc, &timerAlarm));
    exponent += 0.00495;
    return true;
}

static bool IRAM_ATTR isr_leds(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    ESP_ERROR_CHECK(gptimer_stop(timerHandle_leds[showPhase]));

    t_calculation_start[showPhase] = esp_timer_get_time();
    gptimer_alarm_config_t alarmConfig;
    switch (leds_phase)
    {
    case ACTIVATION:
        // activate leds
        for (uint8_t i = 0; i < timeslots[showPhase][active_timeslot].num_pins; i++)
        {
            ESP_ERROR_CHECK(gpio_set_level(timeslots[showPhase][active_timeslot].pins[i], 1));
        }
        // setup deactivation
        alarmConfig.reload_count = 0;
        alarmConfig.flags.auto_reload_on_alarm = true;
        alarmConfig.alarm_count = t_led_on - (esp_timer_get_time() - t_calculation_start[showPhase]);

        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds[showPhase], &alarmConfig));
        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[showPhase]));
        leds_phase = DEACTIVATION;
        break;

    case DEACTIVATION:
        // deactivate leds
        for (uint8_t i = 0; i < timeslots[showPhase][active_timeslot].num_pins; i++)
        {
            ESP_ERROR_CHECK(gpio_set_level(timeslots[showPhase][active_timeslot].pins[i], 0));
        }

        active_timeslot += 1;

        if (active_timeslot < num_timeslots[showPhase])
        {
            // setup activation timer
            alarmConfig.reload_count = 0;
            alarmConfig.flags.auto_reload_on_alarm = true;
            alarmConfig.alarm_count = timeslots[showPhase][active_timeslot].time - (esp_timer_get_time() - t_calculation_start[showPhase]);
            ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds[showPhase], &alarmConfig));
            ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[showPhase]));
        }
        else
        {
            showPhase = !showPhase;
            active_timeslot = 0;
        }
        leds_phase = ACTIVATION;
        break;
    }
    return true;
}

// Functions
double calculateTimeFactor(double angle_deg)
{
    return angle_deg / 360;
}

void init_chambers()
{
    gpio_config_t conf;
    for (uint8_t i = 0; i < NUM_CHAMBERS; i++)
    {
        conf = {
            .pin_bit_mask = (1ULL << led_pins[i]),
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
            .pin = (gpio_num_t)led_pins[i],
            .symbol = symbols[0],
            .time = 0,
        };
        chambers[i] = chamber;
    }
}

void init_hall()
{
    // hall interrupt setup and start (to get rotation speed immediately)
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_HALL));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_HALL, GPIO_MODE_INPUT));
    // TODO: Check if it helps      ESP_ERROR_CHECK(gpio_pullup_en(PIN_HALL));
    ESP_ERROR_CHECK(gpio_pulldown_dis(PIN_HALL));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_HALL, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HALL, isr_hall, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(PIN_HALL));
}

void init_timerBldc()
{
    gptimer_config_t timerConfig;
    timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig.direction = GPTIMER_COUNT_UP;
    timerConfig.resolution_hz = RESOLUTION_MICROSECONDS;
    timerConfig.intr_priority = 1;
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &timerHandle_bldc));

    gptimer_alarm_config_t timerAlarm;
    timerAlarm.alarm_count = startValue_bldc;
    timerAlarm.reload_count = 0;
    timerAlarm.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_bldc, &timerAlarm));

    gptimer_event_callbacks_t timerCb;
    timerCb.on_alarm = isr_bldc_startup;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle_bldc, &timerCb, NULL));

    ESP_ERROR_CHECK(gptimer_enable(timerHandle_bldc));
    ESP_ERROR_CHECK(gptimer_start(timerHandle_bldc));
}

void init_timerLeds()
{
    ESP_LOGI(TAG_LEDS, "register led timers");
    // timer 1
    gptimer_config_t timerConfig_leds_1;
    timerConfig_leds_1.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig_leds_1.direction = GPTIMER_COUNT_UP;
    timerConfig_leds_1.resolution_hz = RESOLUTION_MICROSECONDS;
    timerConfig_leds_1.intr_priority = 1;
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_leds_1, &timerHandle_leds[0]));

    gptimer_event_callbacks_t timerCb_leds_1;
    timerCb_leds_1.on_alarm = isr_leds;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle_leds[0], &timerCb_leds_1, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timerHandle_leds[0]));

    // timer 2
    gptimer_config_t timerConfig_leds_2;
    timerConfig_leds_2.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timerConfig_leds_2.direction = GPTIMER_COUNT_UP;
    timerConfig_leds_2.resolution_hz = RESOLUTION_MICROSECONDS;
    timerConfig_leds_2.intr_priority = 1;
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig_leds_2, &timerHandle_leds[1]));

    gptimer_event_callbacks_t timerCb_leds_2;
    timerCb_leds_2.on_alarm = isr_leds;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle_leds[1], &timerCb_leds_2, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timerHandle_leds[1]));
}

inline void setTimerAlarm(uint8_t phase, uint64_t alarm_count)
{
    gptimer_alarm_config_t timerAlarm_leds;
    timerAlarm_leds.reload_count = 0;
    timerAlarm_leds.flags.auto_reload_on_alarm = true;
    timerAlarm_leds.alarm_count = alarm_count - (esp_timer_get_time() - t_calculation_start[phase]);
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds[phase], &timerAlarm_leds));
}

inline void insertionSort(Chamber chambers[])
{
    Chamber key;
    int i, j;
    for (i = 1; i < NUM_CHAMBERS; i++)
    {
        key = chambers[i];
        j = i - 1;

        // Move elements of arr[0..i-1],
        // that are greater than key,
        // to one position ahead of their
        // current position
        while (j >= 0 && chambers[j].time > key.time)
        {
            chambers[j + 1] = chambers[j];
            j = j - 1;
        }
        chambers[j + 1] = key;
    }
}

inline void calculateAbsoluteChamberTimes()
{
    int j = 0;
    for (uint8_t i = 0; i < NUM_CHAMBERS; i++)
    {
        for (; j < NUM_SYMBOLS; j++)
        {
            if (chambers[i].symbol == 'x')
            {
                chambers[i].time = -1;
            }
            if (chambers[i].symbol == symbols[j])
                break;
        }
        chambers[i].time = t_chamber_start + t_chamber_delta * i + t_symbol_delta * j;
    }
}

inline void calculateRelativeTimeslots(uint8_t phase)
{
    // sortiere chambers nach time
    insertionSort(chambers);

    num_timeslots[phase] = 0;
    Timeslot timeslot_buf;

    timeslot_buf.time = chambers[0].time;
    timeslot_buf.pins[0] = chambers[0].pin;
    timeslot_buf.num_pins = 1;
    for (uint8_t i = 1; i < NUM_CHAMBERS; i++)
    {
        if (chambers[i].time == timeslot_buf.time)
        {
            timeslot_buf.pins[timeslot_buf.num_pins] = chambers[i].pin;
            timeslot_buf.num_pins++;
        }
        else
        {
            timeslots[phase][num_timeslots[phase]] = timeslot_buf;
            num_timeslots[phase]++;

            timeslot_buf.time = chambers[i].time;
            timeslot_buf.pins[0] = chambers[i].pin;
            timeslot_buf.num_pins = 1;
        }
    }
    timeslots[phase][num_timeslots[phase]] = timeslot_buf;
    num_timeslots[phase]++;

    // if (show_time)
    //{
    //     timeslots.erase(timeslots.begin(), timeslots.begin() + 2);       //TODO: implement without std::vector
    // }
    for (uint8_t i = NUM_CHAMBERS - 1; i > 0; i--)
    {
        timeslots[phase][i].time = timeslots[phase][i].time - timeslots[phase][i - 1].time;
    }
}

inline void processTimeslots(uint8_t phase)
{
    t_hall_delta = t_hall_new - t_hall_old;
    t_chamber_delta = (int64_t)(t_chamber_delta_faktor * (double)t_hall_delta);
    t_chamber_start = (int64_t)(t_chamber_start_faktor * (double)t_hall_delta);
    t_symbol_delta = (int64_t)(t_symbol_delta_faktor * (double)t_hall_delta);
    t_led_on = (int64_t)(t_led_on_faktor * (double)t_hall_delta);
    calculateAbsoluteChamberTimes();
    calculateRelativeTimeslots(phase);
}

void remove_timerBldc()
{
    ESP_ERROR_CHECK(gptimer_stop(timerHandle_bldc));
    ESP_ERROR_CHECK(gptimer_disable(timerHandle_bldc));
    ESP_ERROR_CHECK(gptimer_del_timer(timerHandle_bldc));
    timerHandle_bldc = NULL;
}

inline void printTimeslots(uint8_t phase)
{

    ESP_LOGI(TAG_LEDS, "t_hall_delta = %lld ,t_led_on_faktor=%f, t_chamber_delta_faktor=%f, t_chamber_start_faktor=%f", t_hall_delta, t_led_on_faktor, t_chamber_delta_faktor, t_chamber_start_faktor);
    int j = 0;
    for (uint8_t i = 0; i < num_timeslots[phase]; i++)
    {
        ESP_LOGI(TAG_LEDS, "%d Timeslot = %lu", j, timeslots[phase][i].time);
        j++;
    }
}

inline void printChambers()
{
    int j = 0;
    for (uint8_t i = 0; i < NUM_CHAMBERS; i++)
    {
        ESP_LOGI(TAG_LEDS, "%d Chamber = %lu", j, chambers[i].time);
        j++;
    }
}

inline void initial_visualisation()
{
    if (xSemaphoreTake(semaphore_CalculationSync, portMAX_DELAY) == pdFALSE)
        ESP_LOGE(TAG_LEDS, "semaphore_CalculationSync could not be obtained!");

    t_calculation_start[showPhase] = esp_timer_get_time();

    processTimeslots(showPhase);
    // printChambers();
    // printTimeslots(showPhase);

    setTimerAlarm(showPhase, timeslots[showPhase][0].time);
    ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[showPhase]));
}

// MAIN
void task_bldc(void *pvParameters)
{
    bldc.begin(BLDC_MOTOR_POLE, BLDC_DRV_IN1, BLDC_DRV_IN2, BLDC_DRV_IN3, BLDC_DRV_EN1, BLDC_DRV_EN2, BLDC_DRV_EN3);
    bldc.SetPower(100);
    init_hall();
    init_timerBldc(); // timer 1

    while (true)
    {
        bldc.DoRotate(7);
        delayMicroseconds(t_bldc_delay);
    }
}

void task_leds(void *pvParameters)
{
    init_chambers();
    init_timerLeds();

    if (xSemaphoreTake(semaphore_StartUpFinished, portMAX_DELAY) == pdFALSE)
        ESP_LOGE(TAG_LEDS, "semaphore_StartUpFinished could not be obtained!");

    remove_timerBldc();
    int64_t t_debug_start;
    int64_t t_debug_end;
    initial_visualisation();
    while (true)
    {
        if (xSemaphoreTake(semaphore_CalculationSync, portMAX_DELAY) == pdFALSE)
            ESP_LOGE(TAG_LEDS, "semaphore_CalculationSync could not be obtained!");
        t_debug_start = esp_timer_get_time();
        t_calculation_start[calculationPhase] = esp_timer_get_time();

        processTimeslots(calculationPhase);
        // printChambers();
        // printTimeslots(calculationPhase);
        setTimerAlarm(calculationPhase, timeslots[calculationPhase][0].time);

        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[calculationPhase]));
        calculationPhase = !calculationPhase;
        t_debug_end = esp_timer_get_time();
        ESP_LOGI(TAG_LEDS, "delta_time=%lld", t_debug_end - t_debug_start);
    }
}

extern "C" void app_main(void)
{
    semaphore_StartUpFinished = xSemaphoreCreateBinary();
    semaphore_CalculationSync = xSemaphoreCreateBinary();

    if (semaphore_StartUpFinished == NULL)
        ESP_LOGE(TAG_MAIN, "Failed to create semaphore_StartUpFinished");
    if (semaphore_CalculationSync == NULL)
        ESP_LOGE(TAG_MAIN, "Failed to create semaphore_CalculationSync");
    if (semaphore_phaseSync == NULL)
        ESP_LOGE(TAG_MAIN, "Failed to create semaphore_phaseSync");

    t_chamber_delta_faktor = calculateTimeFactor(angle_chamber);
    t_chamber_start_faktor = calculateTimeFactor(angle_start);
    t_led_on_faktor = calculateTimeFactor(angle_on);

    xTaskCreatePinnedToCore(task_bldc, "Task BLDC", 6000, NULL, 2, &taskHandle_bldc, 1);
    xTaskCreatePinnedToCore(task_leds, "Task LEDs", 8000, NULL, 2, &taskHandle_leds, 0);
}