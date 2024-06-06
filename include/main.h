#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_system.h"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>

#include "UcnBrushlessDCMotorPWM.h"

#define RESOLUTION_MICROSECONDS 1000000
#define NUM_CHAMBERS 10
#define NUM_SYMBOLS 12

// symbol disc
const char symbols[NUM_SYMBOLS + 1] = "0123456789:.";

// Pins
const uint8_t BLDC_DRV_IN1 = 27;
const uint8_t BLDC_DRV_IN2 = 26;
const uint8_t BLDC_DRV_IN3 = 25;
const uint8_t BLDC_DRV_EN1 = 13;
const uint8_t BLDC_DRV_EN2 = 12;
const uint8_t BLDC_DRV_EN3 = 14;
const uint8_t BLDC_MOTOR_POLE = 7;

gpio_num_t PIN_HALL = GPIO_NUM_34;

const uint8_t led_pins[10] = {15, 2, 4, 16, 17, 5, 18, 19, 21, 22};

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
    gpio_num_t pins[10];
    uint8_t num_pins;
} Timeslot;

typedef enum
{
    ACTIVATION,
    DEACTIVATION,
} ActivationPhase;

typedef enum
{
    DATE,
    TIME,
} DisplayMode;