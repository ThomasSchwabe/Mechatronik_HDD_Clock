#pragma once

#include <math.h>
#include "esp_attr.h"
#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#ifdef SOC_LEDC_SUPPORT_HS_MODE
#define LEDC_CHANNELS (SOC_LEDC_CHANNEL_NUM << 1)
#else
#define LEDC_CHANNELS (SOC_LEDC_CHANNEL_NUM)
#endif

#ifdef SOC_LEDC_SUPPORT_XTAL_CLOCK
#define LEDC_DEFAULT_CLK LEDC_USE_XTAL_CLK
#else
#define LEDC_DEFAULT_CLK LEDC_AUTO_CLK
#endif

#define LEDC_MAX_BIT_WIDTH SOC_LEDC_TIMER_BIT_WIDTH

#define LOW 0x0
#define HIGH 0x1

// GPIO FUNCTIONS
#define INPUT 0x01
// Changed OUTPUT from 0x02 to behave the same as Arduino pinMode(pin,OUTPUT)
// where you can read the state of pin even when it is set as OUTPUT
#define OUTPUT 0x03
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09
#define OPEN_DRAIN 0x10
#define OUTPUT_OPEN_DRAIN 0x13
#define ANALOG 0xC0

uint8_t channels_resolution[LEDC_CHANNELS] = {0};

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define TAG_ACL "BLDC"

#define PI M_PI
#define ESP32 1

inline void delayMicroseconds(uint32_t us)
{
    uint64_t m = (uint64_t)esp_timer_get_time();
    if (us)
    {
        uint64_t e = (m + us);
        if (m > e)
        { // overflow
            while ((uint64_t)esp_timer_get_time() > e)
            {
                asm volatile("nop");
            }
        }
        while ((uint64_t)esp_timer_get_time() < e)
        {
            asm volatile("nop");
        }
    }
}

void pinMode(uint8_t pin, uint8_t mode)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pin),         /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        .mode = GPIO_MODE_DISABLE,             /*!< GPIO mode: set input/output mode                     */
        .pull_up_en = GPIO_PULLUP_DISABLE,     /*!< GPIO pull-up                                         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /*!< GPIO pull-down                                       */
        .intr_type = GPIO_INTR_DISABLE         /*!< GPIO interrupt type - previously set                 */
    };
    if (mode < 0x20)
    { // io
        conf.mode = (gpio_mode_t)(mode & (INPUT | OUTPUT));
        if (mode & OPEN_DRAIN)
        {
            conf.mode = (gpio_mode_t)((gpio_mode_t)conf.mode | GPIO_MODE_DEF_OD);
        }
        if (mode & PULLUP)
        {
            conf.pull_up_en = GPIO_PULLUP_ENABLE;
        }
        if (mode & PULLDOWN)
        {
            conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        }
    }
    if (gpio_config(&conf) != ESP_OK)
    {
        ESP_LOGE(TAG_ACL, "GPIO config failed");
        return;
    }
}

void digitalWrite(uint8_t pin, uint8_t val)
{
    gpio_set_level((gpio_num_t)pin, val);
}

uint32_t ledcSetup(uint8_t chan, uint32_t freq, uint8_t bit_num)
{
    if (chan >= LEDC_CHANNELS || bit_num > LEDC_MAX_BIT_WIDTH)
    {
        ESP_LOGE(TAG_ACL, "No more LEDC channels available! (maximum %u) or bit width too big (maximum %u)", LEDC_CHANNELS, LEDC_MAX_BIT_WIDTH);
        return 0;
    }

    uint8_t group = (chan / 8), timer = ((chan / 2) % 4);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = (ledc_mode_t)group,
        .duty_resolution = (ledc_timer_bit_t)bit_num,
        .timer_num = (ledc_timer_t)timer,
        .freq_hz = freq,
        .clk_cfg = LEDC_DEFAULT_CLK,
        .deconfigure = false,
    };
    if (ledc_timer_config(&ledc_timer) != ESP_OK)
    {
        ESP_LOGE(TAG_ACL, "ledc setup failed!");
        return 0;
    }
    channels_resolution[chan] = bit_num;
    return ledc_get_freq((ledc_mode_t)group, (ledc_timer_t)timer);
}

void ledcAttachPin(uint8_t pin, uint8_t chan)
{
    if (chan >= LEDC_CHANNELS)
    {
        return;
    }
    uint8_t group = (chan / 8), channel = (chan % 8), timer = ((chan / 2) % 4);
    uint32_t duty = ledc_get_duty((ledc_mode_t)group, (ledc_channel_t)channel);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = pin,
        .speed_mode = (ledc_mode_t)group,
        .channel = (ledc_channel_t)channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = (ledc_timer_t)timer,
        .duty = duty,
        .hpoint = 0,
        .flags = {
            .output_invert = false,
        },
    };
    ledc_channel_config(&ledc_channel);
}

void ledcWrite(uint8_t chan, uint32_t duty)
{
    if (chan >= LEDC_CHANNELS)
    {
        return;
    }
    uint8_t group = (chan / 8), channel = (chan % 8);

    // Fixing if all bits in resolution is set = LEDC FULL ON
    uint32_t max_duty = (1 << channels_resolution[chan]) - 1;

    if ((duty == max_duty) && (max_duty != 1))
    {
        duty = max_duty + 1;
    }

    ledc_set_duty((ledc_mode_t)group, (ledc_channel_t)channel, duty);
    ledc_update_duty((ledc_mode_t)group, (ledc_channel_t)channel);
}
