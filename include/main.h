#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/gptimer.h"

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#include "bldc.h"

#define SIMULATION_MODE true