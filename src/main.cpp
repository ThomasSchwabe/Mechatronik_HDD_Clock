#include "main.h"

/* TODO LIST
/  use t_led_on in Timeslot calculation (fix would be nice, but not neccessary)
*/

#define LOG_LEVEL_LOCAL ESP_LOG_NONE
#include "esp_log.h"
#define TAG_MAIN "MAIN"
#define TAG_BLDC "TASK BLDC"
#define TAG_LEDS "TASK LEDS"

#define PROMPT_STR CONFIG_IDF_TARGET

#define DEBUG_MODE 0

#define OFF 1
#define ON 0

const double angle_chamber = 18;
const double angle_start = 76;
const double angle_on = 0.5;
const double angle_symbol = 18;

// Display variables
DisplayMode displaymode = TIME;
int time_data[3] = {14, 2, 52};
int date_data[3] = {9, 6, 2024};
std::string custom_data = "xxxxxxxxxx";

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
TaskHandle_t taskHandle_test = NULL;
TaskHandle_t taskHandle_speed = NULL;

// Timer Handles
gptimer_handle_t timerHandle_leds[2] = {NULL, NULL};
gptimer_handle_t timerHandle_bldc = NULL;
TimerHandle_t timerHandle_displayTime = NULL;

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

    if (t_bldc_delay > 69) // 69 -> 60Hz, 55 -> 72Hz, stable max =
    {
        t_bldc_delay -= 1;
    }
    else
    {
        xSemaphoreGiveFromISR(semaphore_StartUpFinished, NULL);
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
            ESP_ERROR_CHECK(gpio_set_level(timeslots[ledCore][active_timeslot[ledCore]].pins[i], ON));
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
            ESP_ERROR_CHECK(gpio_set_level(timeslots[ledCore][active_timeslot[ledCore]].pins[i], OFF));
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
            ESP_ERROR_CHECK(gpio_set_level(timeslots[ledCore][active_timeslot[ledCore]].pins[i], ON));
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
            ESP_ERROR_CHECK(gpio_set_level(timeslots[ledCore][active_timeslot[ledCore]].pins[i], OFF));
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
            .mode = GPIO_MODE_OUTPUT_OD,
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
            .symbol = 'x',
            .time = 0,
        };
        chambers[i] = chamber;
        ESP_ERROR_CHECK(gpio_set_level(chambers[i].pin, OFF));
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
    for (int64_t i = 0; i < NUM_CHAMBERS; i++)
    {
        if (chambers[i].symbol == 'x')
        {
            chambers[i].time = 0;
            continue;
        }

        for (int8_t j = 0; j < NUM_SYMBOLS; j++)
        {

            if (chambers[i].symbol == symbols[j])
            {
                chambers[i].time = t_chamber_start + (t_chamber_delta * i) + (t_symbol_delta * j);
                break;
            }
        }
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

    active_timeslot[phase] = (timeslots[phase][0].time == 0) ? 1 : 0;

    for (uint8_t i = num_timeslots[phase] - 1; i > active_timeslot[phase]; i--)
    {
        timeslots[phase][i].time = timeslots[phase][i].time - timeslots[phase][i - 1].time - (i * t_led_on / 4);
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
    uint8_t i = NUM_CHAMBERS - 1;
    for (char c : newSymbols)
    {
        chambers[i].symbol = c;
        i--;
    }
}

bool isValidDate(const std::string &date)
{
    if (date.length() != 10 || date[2] != '.' || date[5] != '.')
    {
        return false;
    }

    for (int i = 0; i < date.size(); ++i)
    {
        if (i == 2 || i == 5)
            continue;
        if (!isdigit(date[i]))
            return false;
    }

    int day = (date[0] - '0') * 10 + (date[1] - '0');
    int month = (date[3] - '0') * 10 + (date[4] - '0');
    int year = (date[6] - '0') * 1000 + (date[7] - '0') * 100 + (date[8] - '0') * 10 + (date[9] - '0');

    if (month < 1 || month > 12)
    {
        return false;
    }

    if (day < 1)
    {
        return false;
    }

    if ((month == 4 || month == 6 || month == 9 || month == 11) && day > 30)
    {
        return false;
    }

    if (month == 2)
    {
        bool isLeapYear = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
        if (isLeapYear && day > 29)
        {
            return false;
        }
        else if (!isLeapYear && day > 28)
        {
            return false;
        }
    }

    return day <= 31;
}

bool isValidTime(const std::string &time)
{
    if (time.length() != 8 || time[2] != ':' || time[5] != ':')
    {
        return false;
    }

    for (int i = 0; i < time.size(); ++i)
    {
        if (i == 2 || i == 5)
            continue;
        if (!isdigit(time[i]))
            return false;
    }

    int hour = (time[0] - '0') * 10 + (time[1] - '0');
    int minute = (time[3] - '0') * 10 + (time[4] - '0');
    int second = (time[6] - '0') * 10 + (time[7] - '0');

    if (hour < 0 || hour >= 24 || minute < 0 || minute >= 60 || second < 0 || second >= 60)
    {
        return false;
    }

    return true;
}

bool validateInput(std::string &input, DisplayMode mode)
{
    std::string validChars = "x:.0123456789";
    const size_t maxLength = 10;

    if (mode == DATE)
    {
        return isValidDate(input);
    }
    else if (mode == TIME)
    {
        if (isValidTime(input))
        {
            input.insert(input.begin(), 'x');
            input.push_back('x');
            return true;
        }
        return false;
    }
    else if (mode == CUSTOM)
    {
        bool hasInvalidChars = false;
        bool isTooLong = input.length() > maxLength;

        // Schritt 1: Ersetze ungültige Zeichen durch 'x'
        for (char &c : input)
        {
            if (validChars.find(c) == std::string::npos)
            {
                c = 'x';
                hasInvalidChars = true;
            }
        }

        // Schritt 2: Reduziere die Länge auf maxLength (10 Zeichen)
        if (isTooLong)
        {
            input = input.substr(0, maxLength);
        }
        // Schritt 3: Fülle den String mit 'x' auf, wenn er kürzer ist
        else if (input.length() < maxLength)
        {
            input.insert(0, maxLength - input.length(), 'x');
        }

        // Ausgabe, falls der String ungültige Zeichen enthielt oder zu lang war
        if (hasInvalidChars)
        {
            printf("Warnung: Der Eingabestring enthielt ungültige Zeichen, die durch 'x' ersetzt wurden.\n");
        }
        if (isTooLong)
        {
            printf("Warnung: Der Eingabestring war zu lang und wurde auf 10 Zeichen gekürzt.\n");
        }
        return true;
    }
    return false;
}

DisplayMode stringToDisplayMode(const std::string &str)
{
    static const std::unordered_map<std::string, DisplayMode> strToEnumMap = {
        {"date", DATE},
        {"time", TIME},
        {"custom", CUSTOM},
        {"test", TEST},
    };

    auto it = strToEnumMap.find(str);
    if (it != strToEnumMap.end())
    {
        return it->second;
    }
    else
    {
        return UNKNOWN; // Rückgabe für unbekannte Werte
    }
}

std::string timeToString()
{
    std::stringstream timeString;
    timeString << "x" << std::setw(2) << std::setfill('0') << time_data[0] << ":";
    timeString << std::setw(2) << std::setfill('0') << time_data[1] << ":";
    timeString << std::setw(2) << std::setfill('0') << time_data[2] << "x";
    return timeString.str();
}

std::string dateToString()
{
    std::stringstream dateString;
    dateString << std::setw(2) << std::setfill('0') << date_data[0] << ".";
    dateString << std::setw(2) << std::setfill('0') << date_data[1] << ".";
    dateString << std::setw(4) << std::setfill('0') << date_data[2];
    return dateString.str();
}

inline bool isLeapYear(int year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

inline void updateDisplayDate()
{
    // Definiere die Anzahl der Tage pro Monat
    std::vector<int> daysInMonth = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    // Erhöhe den Tag
    date_data[0]++;

    // Überprüfe, ob der Monat angepasst werden muss
    if (date_data[0] > daysInMonth[date_data[1] - 1] || (date_data[1] == 2 && date_data[0] > 29 && isLeapYear(date_data[2])))
    {
        date_data[0] = 1; // Setze den Tag auf den ersten Tag des nächsten Monats
        date_data[1]++;   // Erhöhe den Monat

        // Überprüfe, ob das Jahr angepasst werden muss
        if (date_data[1] > 12)
        {
            date_data[1] = 1; // Setze den Monat auf Januar
            date_data[2]++;   // Erhöhe das Jahr
        }
    }
}

void updateDisplayTime(TimerHandle_t xTimer)
{

    time_data[2] += 1;
    if (time_data[2] >= 60)
    {
        time_data[2] = 0;
        time_data[1] += 1;
    }
    if (time_data[1] >= 60)
    {
        time_data[1] = 0;
        time_data[0] += 1;
    }
    if (time_data[0] >= 24)
    {
        time_data[0] = 0;
        updateDisplayDate();
    }

    if (displaymode == TIME)
        updateDisplaySymbols(timeToString());
    if (displaymode == DATE)
        updateDisplaySymbols(dateToString());
}

std::vector<int> extractNumbers(const std::string &str)
{
    std::vector<int> numbers;
    int num = 0;

    for (char ch : str)
    {
        if (isdigit(ch))
        {
            num = num * 10 + (ch - '0'); // Konvertiere das Zeichen in eine Zahl
        }
        else
        {
            if (num != 0)
            {
                numbers.push_back(num); // Füge die extrahierte Zahl zum Vektor hinzu
                num = 0;                // Setze die Nummer zurück
            }
        }
    }

    // Füge die letzte Zahl zum Vektor hinzu, falls vorhanden
    if (num != 0)
    {
        numbers.push_back(num);
    }

    return numbers;
}

// Console Functions
int writeDisplaySymbols(int argc, char **argv)
{
    bool status = 0;
    // Argumentstruktur
    struct arg_str *str = arg_str1(NULL, NULL, "<string>", "String to be processed");
    struct arg_end *end = arg_end(1);
    void *argtable[] = {str, end};

    // Argumente parsen
    int nerrors = arg_parse(argc, argv, argtable);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, end, argv[0]);
        return 1;
    }

    // Verarbeite den eingegebenen String
    std::string value = str->sval[0];

    std::vector<int> inputNumbers;
    switch (displaymode)
    {
    case DATE:
        status = validateInput(value, displaymode);
        if (status)
        {
            inputNumbers = extractNumbers(value);
            date_data[0] = inputNumbers[0];
            date_data[1] = inputNumbers[1];
            date_data[2] = inputNumbers[2];
            updateDisplaySymbols(dateToString());
        }
        break;
    case TIME:
        status = validateInput(value, displaymode);
        if (status)
        {
            inputNumbers = extractNumbers(value);
            time_data[0] = inputNumbers[0];
            time_data[1] = inputNumbers[1];
            time_data[2] = inputNumbers[2];
            updateDisplaySymbols(timeToString());
        }
        break;
    case CUSTOM:
        status = validateInput(value, displaymode);
        if (status)
        {
            custom_data = value;
            updateDisplaySymbols(custom_data);
        }
        break;
    default:
        printf("write not allowed in this mode!");
        break;
    }

    // Speicher freigeben
    arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
    return !status;
}
static const esp_console_cmd_t cmdConfig_updateDisplaySymbols = {
    .command = "write",                                                                          // Name des Befehls
    .help = "Change the displayed symbols of current mode. Symbol pattern must match the mode.", // Hilfe-Text für den Befehl
    .hint = NULL,
    .func = &writeDisplaySymbols, // Funktionszeiger auf die Befehlsfunktion
    .argtable = NULL,
};

int switchMode(int argc, char **argv)
{
    // Argumentstruktur
    struct arg_str *str = arg_str1(NULL, NULL, "<string>", "String to be processed");
    struct arg_end *end = arg_end(1);
    void *argtable[] = {str, end};

    // Argumente parsen
    int nerrors = arg_parse(argc, argv, argtable);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, end, argv[0]);
        return 1;
    }

    // Verarbeite den eingegebenen String
    std::string value = str->sval[0];
    DisplayMode mode = stringToDisplayMode(value);

    switch (mode)
    {
    case TIME:
        displaymode = TIME;
        if (taskHandle_test != NULL)
            vTaskSuspend(taskHandle_test);
        updateDisplaySymbols(timeToString());
        break;
    case DATE:
        displaymode = DATE;
        if (taskHandle_test != NULL)
            vTaskSuspend(taskHandle_test);
        updateDisplaySymbols(dateToString());
        break;
    case CUSTOM:
        displaymode = CUSTOM;
        if (taskHandle_test != NULL)
            vTaskSuspend(taskHandle_test);
        updateDisplaySymbols(custom_data);
        break;
    case TEST:
        displaymode = TEST;
        vTaskResume(taskHandle_test);
        break;
    case UNKNOWN:
        printf("Mode unknown. Valid modes: time, date, test, custom");
        break;
    }

    // Speicher freigeben
    arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));

    return 0; // Rückgabe 0 bedeutet, dass der Befehl erfolgreich war
}
static const esp_console_cmd_t cmdConfig_switchMode = {
    .command = "mode",                                                                     // Name des Befehls
    .help = "Can be used to switch display modes. Valid modes: time, date, test, custom.", // Hilfe-Text für den Befehl
    .hint = NULL,
    .func = &switchMode, // Funktionszeiger auf die Befehlsfunktion
    .argtable = NULL,
};

int printVariables(int argc, char **argv)
{
    bool status;
    // Argumentstruktur
    struct arg_str *str = arg_str1(NULL, NULL, "<string>", "String to be processed");
    struct arg_end *end = arg_end(1);
    void *argtable[] = {str, end};

    // Argumente parsen
    int nerrors = arg_parse(argc, argv, argtable);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, end, argv[0]);
        return 1;
    }

    // Verarbeite den eingegebenen String
    std::string value = str->sval[0];
    if (value == "savedTime")
    {
        printf("savedTime=%s\n", timeToString().c_str());
        status = true;
    }
    else if (value == "savedDate")
    {
        printf("savedDate=%s\n", dateToString().c_str());
        status = true;
    }
    else
    {
        status = false;
    }

    // Speicher freigeben
    arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
    return !status;
}
static const esp_console_cmd_t cmdConfig_printVariables = {
    .command = "print",                            // Name des Befehls
    .help = "Used for debugging. Prints variable", // Hilfe-Text für den Befehl
    .hint = NULL,
    .func = &printVariables, // Funktionszeiger auf die Befehlsfunktion
    .argtable = NULL,
};

int clearDisplay(int argc, char **argv)
{
    if (displaymode == CUSTOM)
    {
        custom_data = "xxxxxxxxxx";
        updateDisplaySymbols(custom_data);
        return 0;
    }
    else
    {
        return 1;
    }
}
static const esp_console_cmd_t cmdConfig_clear = {
    .command = "clear",                                         // Name des Befehls
    .help = "Can be used to clear the display in custom mode.", // Hilfe-Text für den Befehl
    .hint = NULL,
    .func = &clearDisplay, // Funktionszeiger auf die Befehlsfunktion
    .argtable = NULL,
};

int printSpeed(int argc, char **argv)
{
    printf("speed = %fHz\n", 1000000.0 / (t_hall_new - t_hall_old));
    return 0; // Rückgabe 0 bedeutet, dass der Befehl erfolgreich war
}
static const esp_console_cmd_t cmdConfig_printSpeed = {
    .command = "speed",             // Name des Befehls
    .help = "prints current speed", // Hilfe-Text für den Befehl
    .hint = NULL,
    .func = &printSpeed, // Funktionszeiger auf die Befehlsfunktion
    .argtable = NULL,
};

inline void init_console()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_console_register_help_command());

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmdConfig_updateDisplaySymbols));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmdConfig_switchMode));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmdConfig_clear));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmdConfig_printSpeed));
    // ESP_ERROR_CHECK(esp_console_cmd_register(&cmdConfig_printVariables));

    // REPL (Read-Evaluate-Print-Loop) environment
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = PROMPT_STR ">";
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

// MAIN
void task_bldc(void *pvParameters)
{
    bldc.begin(BLDC_MOTOR_POLE, BLDC_DRV_IN1, BLDC_DRV_IN2, BLDC_DRV_IN3, BLDC_DRV_EN1, BLDC_DRV_EN2, BLDC_DRV_EN3);
    bldc.SetPower(80);
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

        if (DEBUG_MODE)
        {
            printChambers();
            printTimeslots(calculationPhase);
        }

        if (num_timeslots[calculationPhase] == 1 && timeslots[calculationPhase][0].time == 0)
        {
            continue;
        }
        setTimerAlarm(calculationPhase, timeslots[calculationPhase][active_timeslot[calculationPhase]].time);
        ESP_ERROR_CHECK(gptimer_start(timerHandle_leds[calculationPhase]));
        calculationPhase = !calculationPhase;
    }
}

void task_measureSpeed(void *pvParameters)
{
    while (true)
    {
        ESP_LOGI("Speed Measurement", "%fHz", 1000000.0 / (t_hall_new - t_hall_old));
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void task_displayTest(void *pvParameters)
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
        "....24....",
        "7991.21.50",
        "4321.:1234",
        "7991021050",
        "xxxxxxxxx1",
        "xxxxxxxxxx",
        "xxxxxxxx12",
        "xxxxxxxxxx",
        "xxxxxxx123",
        "xxxxxxxxxx",
        "xxxxxx1234",
        "xxxxxxxxxx",
        "xxxxx12345",
        "xxxxxxxxxx",
        "xxxx123456",
        "xxxxxxxxxx",
        "xxx1234567",
        "xxxxxxxxxx",
        "xx12345678",
        "xxxxxxxxxx",
        "x123456789",
        "xxxxxxxxxx",
        "123456789x",
        "xxxxxxxxxx",
    };

    uint8_t i = 0;
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        updateDisplaySymbols(displaySymbols[i]);
        i++;
        if (i == displaySymbols.size())
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
    xTaskCreatePinnedToCore(task_displayTest, "Task Display", 2000, NULL, 1, &taskHandle_test, 0);
    vTaskSuspend(taskHandle_test);
    // xTaskCreatePinnedToCore(task_measureSpeed, "Task Speed", 2000, NULL, 1, &taskHandle_speed, 0);
    // vTaskSuspend(taskHandle_speed);

    timerHandle_displayTime = xTimerCreate("Display-Time", pdMS_TO_TICKS(1000), pdTRUE, NULL, updateDisplayTime);
    if (timerHandle_displayTime == NULL)
    {
        ESP_LOGE(TAG_MAIN, "Could not create display timer!");
    }
    else
    {
        if (xTimerStart(timerHandle_displayTime, 0) != pdPASS)
            ESP_LOGE(TAG_MAIN, "Could not start display timer!");
    }

    init_console();
}