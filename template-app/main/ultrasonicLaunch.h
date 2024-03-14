#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#ifndef ultrasonicLaunch_h
#define ultrasonicLaunch_h


void enqueue(float input);
float dequeue();
void ultrasonic_test(void *pvParameters);

typedef struct USParam
{
    int TRIGGER_GPIO;
    int ECHO_GPIO;
    QueueHandle_t Usqueue;
};

#endif