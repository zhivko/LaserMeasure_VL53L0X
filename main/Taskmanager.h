/*
 * Taskmanager.hpp
 *
 *  Created on: Apr 8, 2019
 *      Author: klemen
 */

#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>

void taskmanageTask(void * pvParameters);
