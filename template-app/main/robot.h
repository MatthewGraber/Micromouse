#ifndef ROBOT
#define ROBOT

#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_timer.h"

#include "maze.h"
#include "motor_control.h"
#include "encoder.h"
#include "ultrasonicLaunch.h"
#include "icm20948.h"

#define DELAY 10

extern SemaphoreHandle_t maze_mutex;
extern struct Maze full_maze;

extern SemaphoreHandle_t scan_semaphore;
extern SemaphoreHandle_t pathfind_semaphore;


extern QueueHandle_t UsQueue1;
extern QueueHandle_t UsQueue2;
extern QueueHandle_t UsQueue3;

void Move();

void TurnLeft();
void TurnRight();
void GoStraight();
void GoBack();
void Stop();
void StopBack();
void SquareUp(float time);

void Calibrate();   // Deprecated

void MoveTask(void *);
void TestTaskGoStraight(void *);
void DemoTaskTurnCorner(void *);

#endif