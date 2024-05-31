#pragma once

#include <math.h>
#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#include "esp_log.h"
#define LOG_TAG "BLDC"

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

uint8_t channels_resolution[LEDC_CHANNELS] = {0};

void IRAM_ATTR delayMicroseconds(uint32_t us)
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

uint32_t ledcSetup(uint8_t chan, uint32_t freq, uint8_t bit_num)
{
    if (chan >= LEDC_CHANNELS || bit_num > LEDC_MAX_BIT_WIDTH)
    {
        ESP_LOGE(LOG_TAG, "No more LEDC channels available! (maximum %u) or bit width too big (maximum %u)", LEDC_CHANNELS, LEDC_MAX_BIT_WIDTH);
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
        ESP_LOGE(LOG_TAG, "ledc setup failed!");
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

class BLDC
{
protected:
    const uint8_t PWM_MAX = 255;
    const uint8_t PWM_MIN = 0;

    uint16_t _nSinTableSize;

    int16_t _nPhaseIndex1;
    int16_t _nPhaseIndex2;
    int16_t _nPhaseIndex3;

    uint8_t *_pnSinTable;

    uint8_t _nPower = 100;

    volatile bool _bAbortLoop;

    uint8_t _nMotorPoleCount;

    uint8_t _nPinIn1;
    uint8_t _nPinIn2;
    uint8_t _nPinIn3;

    uint8_t _nChannelIn1_eps32 = 0;
    uint8_t _nChannelIn2_eps32 = 1;
    uint8_t _nChannelIn3_eps32 = 2;

    int8_t _nPinEn1;
    int8_t _nPinEn2;
    int8_t _nPinEn3;

    virtual void PrepareSinTable(uint16_t nSinTableSize)
    {
        if (nSinTableSize == 0)
            return;

        _nSinTableSize = nSinTableSize;

        double rad = 0;
        double diff = 2.0 * M_PI / _nSinTableSize / 1;
        int16_t tmp;

        if (_pnSinTable != NULL)
            delete _pnSinTable;

        _pnSinTable = new uint8_t[_nSinTableSize];
        for (int i = 0; i < _nSinTableSize; i++)
        {
            // degree: 0 to 360,  value: PWM_MIN to PWM_MAX
            tmp = (int)((sin(rad) + 1.0) * (PWM_MAX - PWM_MIN) / 2) + PWM_MIN;
            if (tmp < PWM_MIN)
                tmp = PWM_MIN;
            if (tmp > PWM_MAX)
                tmp = PWM_MAX;

            _pnSinTable[i] = tmp;

            rad += diff;
        }

        _nPhaseIndex1 = 0;
        _nPhaseIndex2 = _nSinTableSize / 3;
        _nPhaseIndex3 = _nPhaseIndex2 + _nPhaseIndex2;
    }

    // 0 - (_nSinTableSize-1)
    virtual inline uint8_t GetSinTableValue(int16_t nIndex) const
    {
        if (_nPower >= 100)
            return _pnSinTable[nIndex];

        uint16_t tmp = _nPower;
        tmp *= _pnSinTable[nIndex];
        tmp /= 100;

        if (tmp > PWM_MAX)
            tmp = PWM_MAX;

        return tmp;
    }

    virtual void setupPWM(uint32_t nPwmFreq = 0)
    {
        if (nPwmFreq == 0)
            nPwmFreq = 48000; // default 48KHz PWM

        ledcSetup(_nChannelIn1_eps32, nPwmFreq, 8); // 8bit PWM
        ledcSetup(_nChannelIn2_eps32, nPwmFreq, 8); // 8bit PWM
        ledcSetup(_nChannelIn3_eps32, nPwmFreq, 8); // 8bit PWM

        ledcAttachPin(_nPinIn1, _nChannelIn1_eps32);
        ledcAttachPin(_nPinIn2, _nChannelIn2_eps32);
        ledcAttachPin(_nPinIn3, _nChannelIn3_eps32);
    }

    virtual inline void WritePwmValue(uint8_t nPinOrChannel, uint8_t nValue) const
    {
        ledcWrite(nPinOrChannel, nValue);
    }

    virtual inline void WritePwm(uint8_t nPinOrChannel, uint16_t nPhaseIndex) const
    {
        WritePwmValue(nPinOrChannel, GetSinTableValue(nPhaseIndex));
    }

    virtual inline void WritePwm123(void) const
    {
        WritePwm(_nChannelIn1_eps32, _nPhaseIndex1);
        WritePwm(_nChannelIn2_eps32, _nPhaseIndex2);
        WritePwm(_nChannelIn3_eps32, _nPhaseIndex3);
    }

public:
    virtual void suspend(uint8_t value = 0)
    {
        WritePwmValue(_nChannelIn1_eps32, value);
        WritePwmValue(_nChannelIn2_eps32, value);
        WritePwmValue(_nChannelIn3_eps32, value);
    }

    BLDC(uint8_t nChannelIn1_eps32 = 0, uint8_t nChannelIn2_eps32 = 1, uint8_t nChannelIn3_eps32 = 2)
    {
        _pnSinTable = NULL;
        _bAbortLoop = false;
        _nSinTableSize = 200;

        _nPinEn1 = -1;
        _nPinEn2 = -1;
        _nPinEn3 = -1;

        _nChannelIn1_eps32 = nChannelIn1_eps32;
        _nChannelIn2_eps32 = nChannelIn2_eps32;
        _nChannelIn3_eps32 = nChannelIn3_eps32;
    }

    virtual ~BLDC()
    {
        end();
    }

    virtual void begin(uint8_t nMotorPoleCount, uint8_t nPinIn1, uint8_t nPinIn2, uint8_t nPinIn3, int8_t nPinEn1 = -1, int8_t nPinEn2 = -1, int8_t nPinEn3 = -1, uint16_t nSinTableSize = 200, uint32_t nPwmFreq = 0)
    {
        _nPinIn1 = nPinIn1;
        _nPinIn2 = nPinIn2;
        _nPinIn3 = nPinIn3;

        _nPinEn1 = nPinEn1;
        _nPinEn2 = nPinEn2;
        _nPinEn3 = nPinEn3;

        _nMotorPoleCount = nMotorPoleCount;

        PrepareSinTable(nSinTableSize);

        setupPWM(nPwmFreq);

        // set ENABLE1 pin to HIGH
        if (nPinEn1 >= 0)
        {
            gpio_set_direction((gpio_num_t)nPinEn1, GPIO_MODE_OUTPUT); // pinMode(nPinEn1, OUTPUT);
            gpio_set_level((gpio_num_t)nPinEn1, 1);                    // digitalWrite(nPinEn1, HIGH);
        }

        // set ENABLE2 pin to HIGH
        if (nPinEn2 >= 0)
        {
            gpio_set_direction((gpio_num_t)nPinEn2, GPIO_MODE_OUTPUT); // pinMode(nPinEn2, OUTPUT);
            gpio_set_level((gpio_num_t)nPinEn2, 1);                    // digitalWrite(nPinEn2, HIGH);
        }

        // set ENABLE3 pin to HIGH
        if (nPinEn3 >= 0)
        {
            gpio_set_direction((gpio_num_t)nPinEn3, GPIO_MODE_OUTPUT); // pinMode(nPinEn3, OUTPUT);
            gpio_set_level((gpio_num_t)nPinEn3, 1);                    // digitalWrite(nPinEn3, HIGH);
        }
    }

    virtual void end(void)
    {
        // set ENABLE1 pin to LOW
        if (_nPinEn1 >= 0)
        {
            gpio_set_level((gpio_num_t)_nPinEn1, 0); // digitalWrite(_nPinEn1, LOW);
        }

        // set ENABLE2 pin to LOW
        if (_nPinEn2 >= 0)
        {
            gpio_set_level((gpio_num_t)_nPinEn2, 0); // digitalWrite(_nPinEn2, LOW);
        }

        // set ENABLE3 pin to LOW
        if (_nPinEn3 >= 0)
        {
            gpio_set_level((gpio_num_t)_nPinEn3, 0); // digitalWrite(_nPinEn3, LOW);
        }

        if (_pnSinTable)
            delete _pnSinTable;
        _pnSinTable = NULL;
    }

    // power: 0 to 100 (%)
    void SetPower(uint8_t nPower)
    {
        if (nPower > 100)
            nPower = 100;
        _nPower = nPower;
    }

    virtual void SetAbortFlag(void)
    {
        _bAbortLoop = true;
    }

    virtual void ClearAbortFlag(void)
    {
        _bAbortLoop = false;
    }

    virtual inline void DoRotate(int16_t nStep = 1)
    {
        WritePwm123();

        _nPhaseIndex1 += nStep;
        _nPhaseIndex2 += nStep;
        _nPhaseIndex3 += nStep;

        if (nStep > 0)
        {
            if (_nPhaseIndex1 >= _nSinTableSize)
                _nPhaseIndex1 = 0;
            if (_nPhaseIndex2 >= _nSinTableSize)
                _nPhaseIndex2 = 0;
            if (_nPhaseIndex3 >= _nSinTableSize)
                _nPhaseIndex3 = 0;
        }
        else
        {
            if (_nPhaseIndex1 < 0)
                _nPhaseIndex1 = _nSinTableSize - 1;
            if (_nPhaseIndex2 < 0)
                _nPhaseIndex2 = _nSinTableSize - 1;
            if (_nPhaseIndex3 < 0)
                _nPhaseIndex3 = _nSinTableSize - 1;
        }
    }

    inline uint32_t calcRequireStepCount(int16_t nRotateDegree)
    {
        // 360degree rotate need ((_nMotorPoleCount + 1) / 2.0 * _nSinTableSize) steps
        return (uint32_t)((double)nRotateDegree / 360.0 * (_nMotorPoleCount + 1) / 2.0 * _nSinTableSize);
    }

    void DoRotateLoop(int16_t nRotateDegree, uint16_t nDelayMicroSec, int8_t nStep = 1)
    {
        if (_pnSinTable == NULL)
            return;

        if (nRotateDegree < 0)
        {
            nRotateDegree = -nRotateDegree;
            nStep = -nStep;
        }
        uint8_t nStepAbs = abs(nStep);

        uint32_t nRequireStep = calcRequireStepCount(nRotateDegree);

        while (_bAbortLoop == false)
        {
            DoRotate(nStep);

            nRequireStep -= nStepAbs;
            if (nRequireStep == 0)
                break;

            delayMicroseconds(nDelayMicroSec);
        }
    }
};