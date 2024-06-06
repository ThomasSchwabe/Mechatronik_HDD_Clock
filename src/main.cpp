#include "main.h"

/* TODO LIST
/  use t_led_on in Timeslot calculation (fix bug)
/  Date - Time switch
/  Serial interface to init time/Date
/  1 timer for date
/  2 timer for time
/
*/

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define TAG_MAIN "MAIN"
#define TAG_BLDC "TASK BLDC"
#define TAG_LEDS "TASK LEDS"

#define DEBUG_MODE 0
uint8_t print_timeslot_num = 100;

const double angle_chamber = 18;
const double angle_start = 76;
const double angle_on = 1.0;
const double angle_symbol = 18;
char initSymbols[NUM_CHAMBERS + 1] = "..........";

// variables time measurement/calculation
volatile int64_t t_hall_old;
volatile int64_t t_hall_new;
int64_t t_hall_delta;

volatile int64_t t_led_on;
double t_led_on_faktor;
int64_t t_chamber_start;
double t_chamber_start_faktor;
int64_t t_symbol_delta;
double t_symbol_delta_faktor;
int64_t t_chamber_delta;
double t_chamber_delta_faktor;

int64_t t_error_calculation_phase[2]; // for calculations
volatile int64_t t_error_show_phase[2];

volatile ActivationPhase leds_phase[2];

// chamber variables
Chamber chambers[NUM_CHAMBERS];
Chamber sorted_chambers[NUM_CHAMBERS];
Timeslot timeslots[2][NUM_CHAMBERS];
uint8_t num_timeslots[2];
volatile uint8_t active_timeslot[2] = {ACTIVATION, ACTIVATION};
uint8_t calculationPhase = 0;

// bldc variables
UcnBrushlessDCMotorPWM bldc;
volatile uint32_t t_bldc_delay = 1500;
uint32_t startValue_timerBldc = 30;
double exponent = 0.0;
const double exp_increase = 0.00495;

// ##########FreeRTOS Management Variables//##########//
//  semaphores
SemaphoreHandle_t semaphore_CalculationSync = NULL;
SemaphoreHandle_t semaphore_StartUpFinished = NULL;

// Task Handles
TaskHandle_t taskHandle_bldc = NULL;
TaskHandle_t taskHandle_leds = NULL;
TaskHandle_t taskHandle_display = NULL;

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
        xTaskResumeFromISR(taskHandle_display);
    }
    timerAlarm.alarm_count = exp(exponent) * startValue_timerBldc;
    timerAlarm.reload_count = 0;
    timerAlarm.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_bldc, &timerAlarm));
    exponent += exp_increase;
    return true;
}

static bool IRAM_ATTR isr_ledProcessor_0(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    const uint8_t ledCore = 0;

    t_error_show_phase[ledCore] = esp_timer_get_time();
    ESP_ERROR_CHECK(gptimer_stop(timerHandle_leds[ledCore]));
    gptimer_alarm_config_t alarmConfig;

    switch (leds_phase[ledCore])
    {
    case ACTIVATION:
        // activate leds
        for (uint8_t i = 0; i < timeslots[ledCore][active_timeslot[ledCore]].num_pins; i++)
        {
            ESP_ERROR_CHECK(gpio_set_level(timeslots[ledCore][active_timeslot[ledCore]].pins[i], 1));
        }
        leds_phase[ledCore] = DEACTIVATION;
        // setup deactivation
        alarmConfig.reload_count = 0;
        alarmConfig.flags.auto_reload_on_alarm = true;
        alarmConfig.alarm_count = t_led_on - (esp_timer_get_time() - t_error_show_phase[ledCore]);

        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds[ledCore], &alarmConfig));
        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[ledCore]));
        return true;

    case DEACTIVATION:
        // deactivate leds
        for (uint8_t i = 0; i < timeslots[ledCore][active_timeslot[ledCore]].num_pins; i++)
        {
            ESP_ERROR_CHECK(gpio_set_level(timeslots[ledCore][active_timeslot[ledCore]].pins[i], 0));
        }
        leds_phase[ledCore] = ACTIVATION;
        active_timeslot[ledCore] += 1;

        if (active_timeslot[ledCore] < num_timeslots[ledCore])
        {
            // setup activation timer
            alarmConfig.reload_count = 0;
            alarmConfig.flags.auto_reload_on_alarm = true;
            alarmConfig.alarm_count = timeslots[ledCore][active_timeslot[ledCore]].time - (esp_timer_get_time() - t_error_show_phase[ledCore]);
            ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds[ledCore], &alarmConfig));
            ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[ledCore]));
        }
        else
        {
            active_timeslot[ledCore] = 0;
        }
        return true;
    }
    return true;
}

static bool IRAM_ATTR isr_ledProcessor_1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    const uint8_t ledCore = 1;

    t_error_show_phase[ledCore] = esp_timer_get_time();
    ESP_ERROR_CHECK(gptimer_stop(timerHandle_leds[ledCore]));
    gptimer_alarm_config_t alarmConfig;

    switch (leds_phase[ledCore])
    {
    case ACTIVATION:
        // activate leds
        for (uint8_t i = 0; i < timeslots[ledCore][active_timeslot[ledCore]].num_pins; i++)
        {
            ESP_ERROR_CHECK(gpio_set_level(timeslots[ledCore][active_timeslot[ledCore]].pins[i], 1));
        }
        leds_phase[ledCore] = DEACTIVATION;
        // setup deactivation
        alarmConfig.reload_count = 0;
        alarmConfig.flags.auto_reload_on_alarm = true;
        alarmConfig.alarm_count = t_led_on - (esp_timer_get_time() - t_error_show_phase[ledCore]);

        ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds[ledCore], &alarmConfig));
        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[ledCore]));
        return true;

    case DEACTIVATION:
        // deactivate leds
        for (uint8_t i = 0; i < timeslots[ledCore][active_timeslot[ledCore]].num_pins; i++)
        {
            ESP_ERROR_CHECK(gpio_set_level(timeslots[ledCore][active_timeslot[ledCore]].pins[i], 0));
        }
        leds_phase[ledCore] = ACTIVATION;
        active_timeslot[ledCore] += 1;

        if (active_timeslot[ledCore] < num_timeslots[ledCore])
        {
            // setup activation timer
            alarmConfig.reload_count = 0;
            alarmConfig.flags.auto_reload_on_alarm = true;
            alarmConfig.alarm_count = timeslots[ledCore][active_timeslot[ledCore]].time - (esp_timer_get_time() - t_error_show_phase[ledCore]);
            ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds[ledCore], &alarmConfig));
            ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[ledCore]));
        }
        else
        {
            active_timeslot[ledCore] = 0;
        }
        return true;
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
            .symbol = initSymbols[i],
            .time = 0,
        };
        chambers[i] = chamber;
        if (DEBUG_MODE)
        {
            ESP_LOGI(TAG_LEDS, "%d chamber.symbol = %c", i, chambers[i].symbol);
        }
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
    timerAlarm.alarm_count = startValue_timerBldc;
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
    timerCb_leds_1.on_alarm = isr_ledProcessor_0;
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
    timerCb_leds_2.on_alarm = isr_ledProcessor_1;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle_leds[1], &timerCb_leds_2, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timerHandle_leds[1]));
}

inline void setTimerAlarm(uint8_t phase, uint64_t alarm_count)
{
    gptimer_alarm_config_t timerAlarm_leds;
    timerAlarm_leds.reload_count = 0;
    timerAlarm_leds.flags.auto_reload_on_alarm = true;
    timerAlarm_leds.alarm_count = alarm_count - (esp_timer_get_time() - t_error_calculation_phase[phase]);
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle_leds[phase], &timerAlarm_leds));
}

inline void sortChambers()
{
    for (uint8_t c = 0; c < NUM_CHAMBERS; c++)
    {
        sorted_chambers[c] = chambers[c];
    }

    Chamber key;
    int i, j;
    for (i = 1; i < NUM_CHAMBERS; i++)
    {
        key = sorted_chambers[i];
        j = i - 1;

        // Move elements of arr[0..i-1],
        // that are greater than key,
        // to one position ahead of their
        // current position
        while (j >= 0 && sorted_chambers[j].time > key.time)
        {
            sorted_chambers[j + 1] = sorted_chambers[j];
            j = j - 1;
        }
        sorted_chambers[j + 1] = key;
    }
}

inline void calculateAbsoluteChamberTimes()
{
    int64_t j = 0;
    for (int64_t i = 0; i < NUM_CHAMBERS; i++)
    {
        for (; j < NUM_SYMBOLS; j++)
        {
            // if (chambers[i].symbol == 'x')
            //{
            //     chambers[i].time = -1; // TODO: FIX for time mode
            // }
            if (chambers[i].symbol == symbols[j])
                break;
        }
        chambers[i].time = t_chamber_start + (t_chamber_delta * i) + (t_symbol_delta * j);
        if (DEBUG_MODE && print_timeslot_num == 0)
        {
            ESP_LOGI(TAG_LEDS, "chambers[i=%lld].symbol=%c -> symbols[j=%lld]=%c", i, chambers[i].symbol, j, symbols[j]);
            ESP_LOGI(TAG_LEDS, "t_chamber_start = %lld, t_chamber_delta = %lld, t_symbol_delta = %lld, ", t_chamber_start, t_chamber_delta, t_symbol_delta);
            ESP_LOGI(TAG_LEDS, "chambers[i=%lld]=%lu", i, chambers[i].time);
        }
        j = 0;
    }
}

inline void calculateRelativeTimeslots(uint8_t phase)
{
    // sortiere chambers nach time
    sortChambers();

    num_timeslots[phase] = 0;
    Timeslot timeslot_buf;

    timeslot_buf.time = sorted_chambers[0].time;
    timeslot_buf.pins[0] = sorted_chambers[0].pin;
    timeslot_buf.num_pins = 1;
    for (uint8_t i = 1; i < NUM_CHAMBERS; i++)
    {
        if (sorted_chambers[i].time == timeslot_buf.time)
        {
            timeslot_buf.pins[timeslot_buf.num_pins] = sorted_chambers[i].pin;
            timeslot_buf.num_pins++;
        }
        else
        {
            timeslots[phase][num_timeslots[phase]] = timeslot_buf;
            num_timeslots[phase]++;

            timeslot_buf.time = sorted_chambers[i].time;
            timeslot_buf.pins[0] = sorted_chambers[i].pin;
            timeslot_buf.num_pins = 1;
        }
    }
    timeslots[phase][num_timeslots[phase]] = timeslot_buf;
    num_timeslots[phase]++;

    // if (show_time)
    //{
    //     timeslots.erase(timeslots.begin(), timeslots.begin() + 2);       //TODO: implement without std::vector
    // }
    for (uint8_t i = num_timeslots[phase] - 1; i > 0; i--)
    {
        timeslots[phase][i].time = timeslots[phase][i].time - timeslots[phase][i - 1].time;
    }

    for (uint8_t i = 0; i < num_timeslots[phase]; i++)
    {
        timeslots[phase][i].time -= (i * t_led_on / 4); // TODO: FIX THIS
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
    for (uint8_t i = 0; i < num_timeslots[phase]; i++)
    {
        ESP_LOGI(TAG_LEDS, "%d Timeslot.time = %lu", i, timeslots[phase][i].time);
        for (int j = 0; j < timeslots[phase][i].num_pins; j++)
        {
            ESP_LOGI(TAG_LEDS, "%d Timeslot.pins = %d", i, timeslots[phase][i].pins[j]);
        }
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

void updateDisplaySymbols(std::string newSymbols)
{
    uint8_t i = 0;
    for (char c : newSymbols)
    {
        chambers[i].symbol = c;
        i++;
    }
}

// MAIN
void task_bldc(void *pvParameters)
{
    bldc.begin(BLDC_MOTOR_POLE, BLDC_DRV_IN1, BLDC_DRV_IN2, BLDC_DRV_IN3, BLDC_DRV_EN1, BLDC_DRV_EN2, BLDC_DRV_EN3);
    bldc.SetPower(70);
    init_hall();
    for (int i = 0; i < 400; i++)
    {
        bldc.DoRotate(1);
        delayMicroseconds(1500);
    }
    init_timerBldc(); // timer 1

    while (true)
    {
        bldc.DoRotate(4);
        delayMicroseconds(t_bldc_delay);
    }
}

void task_measureSpeed(void *pvParameters)
{
    while (true)
    {
        ESP_LOGI(TAG_BLDC, "Speed=%f", 1000000.0 / (t_hall_new - t_hall_old));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_leds(void *pvParameters)
{
    init_chambers();
    init_timerLeds();

    if (xSemaphoreTake(semaphore_StartUpFinished, portMAX_DELAY) == pdFALSE)
        ESP_LOGE(TAG_LEDS, "semaphore_StartUpFinished could not be obtained!");

    remove_timerBldc();
    // initial_visualisation();
    while (true)
    {
        if (xSemaphoreTake(semaphore_CalculationSync, portMAX_DELAY) == pdFALSE) // from hall isr
        {
            ESP_LOGE(TAG_LEDS, "semaphore_CalculationSync could not be obtained!");
        }
        t_error_calculation_phase[calculationPhase] = esp_timer_get_time();

        processTimeslots(calculationPhase);

        if (DEBUG_MODE && print_timeslot_num == 0)
        {
            printChambers();
            printTimeslots(calculationPhase);
        }
        print_timeslot_num--;

        setTimerAlarm(calculationPhase, timeslots[calculationPhase][0].time);

        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[calculationPhase]));
        calculationPhase = !calculationPhase;
    }
}

void task_display(void *pvParameters)
{
    std::vector<std::string> displaySymbols = {
        "0000000000",
        "1111111111",
        "2222222222",
        "3333333333",
        "4444444444",
        "5555555555",
        "6666666666",
        "7777777777",
        "8888888888",
        "9999999999",
        "::::::::::",
        "..........",
    };

    uint8_t i = 0;
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        updateDisplaySymbols(displaySymbols[i]);
        i++;
        if (i == 12)
        {
            i = 0;
        }
    }
}

extern "C" void app_main(void)
{
    ESP_LOGW(TAG_MAIN, "WARNING! DEBUG MODE AKTIVATED!");
    semaphore_StartUpFinished = xSemaphoreCreateBinary();
    semaphore_CalculationSync = xSemaphoreCreateBinary();

    if (semaphore_StartUpFinished == NULL)
        ESP_LOGE(TAG_MAIN, "Failed to create semaphore_StartUpFinished");
    if (semaphore_CalculationSync == NULL)
        ESP_LOGE(TAG_MAIN, "Failed to create semaphore_CalculationSync");

    t_chamber_delta_faktor = calculateTimeFactor(angle_chamber);
    t_chamber_start_faktor = calculateTimeFactor(angle_start);
    t_symbol_delta_faktor = calculateTimeFactor(angle_symbol);
    t_led_on_faktor = calculateTimeFactor(angle_on);

    xTaskCreatePinnedToCore(task_bldc, "Task BLDC", 6000, NULL, 2, &taskHandle_bldc, 1);
    xTaskCreatePinnedToCore(task_leds, "Task LEDs", 8000, NULL, 2, &taskHandle_leds, 0);
    xTaskCreatePinnedToCore(task_display, "Task Display", 2000, NULL, 2, &taskHandle_display, 0);
    vTaskSuspend(taskHandle_display);
}