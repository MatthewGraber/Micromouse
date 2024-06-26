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
void GoForwardsForTime(float time);
void GoBackwardsForTime(float time);
void TurnLeftForTime(float time);
void TurnRightForTime(float time);

// Motor functions
void LeftMotorForward(float pow);
void RightMotorForward(float pow);
void LeftMotorBackward(float pow);
void RightMotorBackward(float pow);

void Calibrate();   // Deprecated

void MoveTask(void *);
void TestTaskGoStraight(void *);
void DemoTaskTurnCorner(void *);

#endif