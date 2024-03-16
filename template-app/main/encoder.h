#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

extern QueueHandle_t encoderQueue1;
extern QueueHandle_t encoderQueue2;
extern QueueHandle_t distanceQueueRight;
extern QueueHandle_t distanceQueueLeft;

void IRAM_ATTR encoderPulseISR1(void *arg);
void IRAM_ATTR encoderPulseISR2(void *arg);
void encoderTask(void *pvParameters);
void initializeEncoder();

#endif