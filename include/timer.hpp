#pragma once

#include "driver/gptimer.h"

class Timer
{
private:
    gptimer_config_t config;
    gptimer_alarm_config_t alarm;
    gptimer_event_callbacks_t callback;
    gptimer_handle_t handle;

public:
    // Konstruktor
    Timer(uint64_t alarm_count, bool auto_reload, gptimer_alarm_cb_t cb_function, int intr_priority = 0, uint32_t resolution_hz = 1000000)
    {
        config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
        config.direction = GPTIMER_COUNT_UP;
        config.resolution_hz = resolution_hz; // has default
        config.intr_priority = intr_priority; // has default

        alarm.alarm_count = alarm_count;
        alarm.reload_count = 0;
        alarm.flags.auto_reload_on_alarm = auto_reload;

        callback.on_alarm = cb_function;

        ESP_ERROR_CHECK(gptimer_new_timer(&config, &handle));
        ESP_ERROR_CHECK(gptimer_set_alarm_action(handle, &alarm));
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(handle, &callback, NULL));
        ESP_ERROR_CHECK(gptimer_enable(handle));
    }

    // Methoden
    void start()
    {
        ESP_ERROR_CHECK(gptimer_start(handle));
    }

    void stop()
    {
        ESP_ERROR_CHECK(gptimer_stop(handle));
    }

    void setAlarmCount(uint64_t alarm_count)
    {
        this->alarm.alarm_count = alarm_count;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(handle, &alarm));
    }

    void remove()
    {
        ESP_ERROR_CHECK(gptimer_disable(handle));
        ESP_ERROR_CHECK(gptimer_del_timer(handle));
    }
};