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

#define freeheap heap_caps_get_free_size(MALLOC_CAP_8BIT)
extern uint64_t mySecond;
extern float timeH;

void taskmanageTask(void * pvParameters);
