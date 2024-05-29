#pragma once

#include <vector>
#include <iostream>
#include <stdexcept>
#include <algorithm>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "HWTimer.hpp"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define LOG_TAG "HDDClockDisplay"

typedef struct
{
    int pin;
    char symbol;
    unsigned long time;
} Chamber;

typedef struct
{
    unsigned long time;
    std::vector<int> pins;
} Timeslot;

class HDDClockDisplay
{
private:
    // Variabeln
    bool _show_time; // false -> mode=date, true -> mode=time
    const std::string _symbols;
    std::vector<Chamber> _chambers;
    std::vector<Timeslot> _timeslots;
    HWTimer _timer_leds_activate;
    HWTimer _timer_leds_deactivate;

    volatile unsigned long t_hall_old;
    volatile unsigned long t_hall_new;
    volatile unsigned long t_hall_delta;
    volatile unsigned long t_chamber_start;
    volatile unsigned long t_chamber_delta;
    unsigned long t_calculation_error_start;
    unsigned long t_calculation_error;

    // Methoden
    void _fitStringSize(std::string &input)
    {
        if (input.length() > 10)
        {
            input = input.substr(0, 10);
        }
        else if (input.length() < 10)
        {
            input.append(10 - input.length(), '0');
        }
        if (_show_time)
        {
            input.front() = 'x';
            input.back() = 'x';
        }
    }

    void _calculateAbsoluteChamberTimes()
    {
        int j = 0;
        for (Chamber &chamber : _chambers)
        {
            for (; j < 12; j++)
            {
                if (chamber.symbol == 'x')
                {
                    chamber.time = -1;
                }
                if (chamber.symbol == _symbols[j])
                    break;
            }
            chamber.time = (t_chamber_start + t_chamber_delta * j) / 10; // undo float vermeidung / 10
        }
    }

    void _calculateRelativeTimeslots()
    {
        std::sort(_chambers.begin(), _chambers.end(), [](const Chamber &a, const Chamber &b)
                  { return a.time < b.time; });

        _timeslots.clear();
        Timeslot timeslot_buf;
        timeslot_buf.time = _chambers[0].time,
        timeslot_buf.pins.push_back(_chambers[0].pin);

        for (size_t i = 1; i < _chambers.size(); i++)
        {
            if (_chambers[i].time == timeslot_buf.time)
            {
                timeslot_buf.pins.push_back(_chambers[i].pin);
            }
            else
            {
                _timeslots.push_back(timeslot_buf);
                timeslot_buf.pins.clear();
                timeslot_buf.time = _chambers[i].time;
                timeslot_buf.pins.push_back(_chambers[i].pin);
            }
            _timeslots.push_back(timeslot_buf);
        }

        if (_show_time)
        {
            _timeslots.erase(_timeslots.begin(), _timeslots.begin() + 2);
        }
        for (size_t i = _chambers.size() - 1; i > 0; i--)
        {
            _timeslots[i].time = _timeslots[i].time - _timeslots[i - 1].time;
        }
    }

public:
    // Konstruktor
    HDDClockDisplay(const std::string symbols, std::vector<int> led_gpios) : _show_time(false), _symbols(symbols)
    {
        if (symbols.length() != led_gpios.size())
        {
            ESP_LOGE(LOG_TAG, "num of symbols does not match num of led_gpios");
            throw std::invalid_argument("num of symbols does not match num of led_gpios");
        }
        t_hall_old = esp_timer_get_time();
        //_timer_leds_activate = HWTimer(1000000, false, isr_leds_activate);
        //_timer_leds_activate = HWTimer(1000000, false, cb);

        for (int gpio : led_gpios)
        {
            ESP_ERROR_CHECK(gpio_set_direction(static_cast<gpio_num_t>(gpio), GPIO_MODE_OUTPUT));
            Chamber chamber = {
                .pin = gpio,
                .symbol = symbols[0],
                .time = static_cast<unsigned long>(-1),
            };
            _chambers.push_back(chamber);
        }
    }

    // Methoden
    void updateSymbols(std::string &symbols)
    {
        _fitStringSize(symbols);
        auto symbols_iter = symbols.begin();
        for (Chamber &chamber : _chambers)
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
        _show_time != _show_time;
    }

    void showDisplay()
    {
        t_calculation_error_start = esp_timer_get_time();
        _calculateAbsoluteChamberTimes();
        _calculateRelativeTimeslots();
    }
};
