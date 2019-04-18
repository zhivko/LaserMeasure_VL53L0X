/*
 * Taskmanager.hpp
 *
 *  Created on: Apr 8, 2019
 *      Author: klemen
 */

#pragma once

#include <Preferences.h>
#include "AiEsp32RotaryEncoder.h"

extern AiEsp32RotaryEncoder rotaryEncoder1;
extern AiEsp32RotaryEncoder rotaryEncoder2;
extern int32_t encoder1_value;
extern int32_t encoder2_value;
extern Preferences preferences;
extern void lcd_out(const char *format, ...);

void encoderSaverTask(void * pvParameters);
