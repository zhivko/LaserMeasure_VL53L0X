/*
 * Server.h
 *
 *  Created on: Apr 24, 2019
 *      Author: klemen
 */

#ifndef MAIN_SERVER_H_
#define MAIN_SERVER_H_

#define freeheap heap_caps_get_free_size(MALLOC_CAP_8BIT)


SemaphoreHandle_t xSemaphore = NULL;

#define enablePwm 0
#define enableLcd 1
#define enableTaskManager 0
bool enableMover = false;
bool enableLed = true;


#endif /* MAIN_SERVER_H_ */
