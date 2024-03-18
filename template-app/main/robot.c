#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "robot.h"
#include "maze.h"
#include "motor_control.h"
#include "encoder.h"
#include "ultrasonicLaunch.h"


extern SemaphoreHandle_t maze_mutex;
extern struct Maze full_maze;

extern SemaphoreHandle_t scan_semaphore;
extern SemaphoreHandle_t pathfind_semaphore;


extern QueueHandle_t UsQueue1;
extern QueueHandle_t UsQueue2;
extern QueueHandle_t UsQueue3;

float left_encoder;
float right_encoder;

void Move() { 

    int targetHeading = North;
    int xDif, yDif;

    // printf("Attempting to move\n");

    xDif = full_maze.currentNode->x - full_maze.nextNode->x;    // -1 == East, 1 == West
    yDif = full_maze.currentNode->y - full_maze.nextNode->y;    // -1 == South, 1 == North

    // Ensure that the direction of the next node is valid
    if ((xDif == 0) == (yDif == 0)) {
        // If it is invalid, exit this task and call the Navigate task
        xSemaphoreGive(maze_mutex);
        xSemaphoreGive(pathfind_semaphore);
    }
    else {
        // Find the correct target heading
        // printf("Finding target heading\n");
        if (xDif == -1) {
            targetHeading = East;
        }
        else if (xDif == 1) {
            targetHeading = West;
        }
        else if (yDif == -1) {
            targetHeading = South;
        }
        else {
            targetHeading = North;
        }


        // Turn until the target heading matches the current heading
        while (targetHeading != full_maze.heading) {
            if ((targetHeading-full_maze.heading + 4) % 4 == 1) {
                TurnRight();
            }
            else {
                TurnLeft();
            }
            vTaskDelay(100);
        }

        GoStraight();
        Stop();

        // If we're in the center and going to the center, turn around
        if (full_maze.goingToCenter) { 
            if ((full_maze.currentNode->x == 4 || full_maze.currentNode->x == 5) &&
                (full_maze.currentNode->y == 4 || full_maze.currentNode->y == 5)) {
                full_maze.goingToCenter = false;
            }
        }
        else if (full_maze.currentNode->x == 0 && full_maze.currentNode->y == 0) {
            full_maze.goingToCenter = true;
        }

        xSemaphoreGive(maze_mutex);
        xSemaphoreGive(scan_semaphore);
    }
}


void TurnRight() {
    const float PI = 3.14159265358979323846;
    const float TURN_RADIUS = 30;  // mm
    // const float TARGET_DIST = PI*TURN_RADIUS/2;  // Quarter-turn in mm
    const float TARGET_DIST = 30;
    // const float GAIN = 0.01;
    const float BASE_POWER = 70;
    const float LOW_POWER = 30;

    float LEFT_START;

    float left_ultra = 0, right_ultra = 100;
    const float ULTRA_STOP = 5;     // Distance at which we will stop turning via ultrasonic sensor

    // Spin motors
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);                    // motor 1 (left)
    Stop();
    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER);     // motor 0 (right)
    xQueueReceive(distanceQueueLeft, &left_encoder, 0);
    LEFT_START = left_encoder;

    printf("Turning right\n");
    

    while ((left_encoder - LEFT_START < TARGET_DIST) || left_ultra < ULTRA_STOP) {
        xQueueReceive(UsQueue1, &left_ultra, 0);
        if(xQueueReceive(UsQueue2, &right_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
            if (right_ultra <= ULTRA_STOP) {
                break;
            }
        }

        if (xQueueReceive(distanceQueueLeft, &left_encoder, 0) == pdTRUE) {
            // printf("Distance turned right: %f\n", left_encoder - LEFT_START);
        }
        // Creates a gain based on the percentage of the distance covered
        float gain = ((TARGET_DIST - (left_encoder - LEFT_START)) / TARGET_DIST) * (100-BASE_POWER);
        // float gain = 0;
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER); // motor 0 (left)
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, LOW_POWER); // motor 1 (right)
        vTaskDelay(1);
    }

    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER);
    // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, LOW_POWER); // motor 0 (left)
    // vTaskDelay(50);
    full_maze.heading = (full_maze.heading + 1) % 4;
    // printMaze();
    Stop();
    vTaskDelay(100);
}


void TurnLeft() {
    const float PI = 3.14159265358979323846;
    const float TURN_RADIUS = 30;  // mm
    // const float TARGET_DIST = PI*TURN_RADIUS/2;  // Quarter-turn in mm
    const float TARGET_DIST = 30;
    // const float GAIN = 0.01;
    const float BASE_POWER = 70;
    const float LOW_POWER = 30;

    float RIGHT_START;

    float left_ultra = 100, right_ultra = 0;
    const float ULTRA_STOP = 5;     // Distance at which we will stop turning via ultrasonic sensor

    // Spin motors
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);                    // motor 1 (left)
    Stop();
    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER);     // motor 0 (right)
    xQueueReceive(distanceQueueRight, &right_encoder, 0);
    RIGHT_START = right_encoder;

    printf("Turning left\n");
    

    while ((right_encoder - RIGHT_START < TARGET_DIST) || right_ultra < ULTRA_STOP) {
        xQueueReceive(UsQueue2, &right_ultra, 0);
        if(xQueueReceive(UsQueue1, &left_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
            if (left_ultra <= ULTRA_STOP) {
                break;
            }
        }

        if (xQueueReceive(distanceQueueRight, &right_encoder, 0) == pdTRUE) {
            // printf("Distance turned left: %f\n", right_encoder - RIGHT_START);
        }
        // Creates a gain based on the percentage of the distance covered
        float gain = ((TARGET_DIST - (right_encoder - RIGHT_START)) / TARGET_DIST) * (100-BASE_POWER);
        // float gain = 0;
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER); // motor 1 (right)
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, LOW_POWER); // motor 0 (left)
        vTaskDelay(1);
    }

    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER);
    // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, LOW_POWER); // motor 0 (left)
    // vTaskDelay(50);
    full_maze.heading = (full_maze.heading + 3) % 4;
    // printMaze();
    Stop();
    vTaskDelay(100);
}


void GoStraight() {
    const float TARGET_DIST = 10*25.4;  // mm
    const float GAIN = 0.1;
    const float BASE_POWER = 50;
    const float STOP_DIST = 10;    // Stop if the ultrasonic sees something this close
    const float WALL_CLOSE_DIST = 5;        // Move away from the wall if it's at least this close
    const float WALL_FAR_DIST = 10;         // Move towards the wall if it's further than this
    const float WALL_VERY_FAR_DIST = 20;    // But not further than this
    const float ULTRA_GAIN = 15;

    float LEFT_START, RIGHT_START;
    // Encoder readings
    // Ultrasonic readings (cm)
    float front_ultra = STOP_DIST + 1, left_ultra = WALL_CLOSE_DIST + 1, right_ultra = WALL_CLOSE_DIST + 1;
    float left_boost = 0, right_boost = 0;  // Used when the ultrasonic sensors say we're close to a wall
    

    printf("Moving straight\n");
    
    // Spin motors
    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER); // motor 1 (left)
    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER); // motor 2 (right)

    // printf("Got left distance\n");
    // printf("Got right distance\n");
    xQueueReceive(distanceQueueLeft, &left_encoder, 0);
    xQueueReceive(distanceQueueRight, &right_encoder, 0);
    LEFT_START = left_encoder;
    RIGHT_START = right_encoder;

    // Exit condition
    // Drive until both distances are greater than the target distance
    while ((left_encoder - LEFT_START < TARGET_DIST) && (right_encoder - RIGHT_START < TARGET_DIST)) {

        // If we're close to the left-facing wall, turn more
        if(xQueueReceive(UsQueue1, &left_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
        }
        // if (left_ultra <= WALL_CLOSE_DIST) {
        //     left_boost = 15;
        // }
        // else {
        //     left_boost = 0;
        // }

        // If we're close to the forward-facing wall, stop moving forwards
        if(xQueueReceive(UsQueue2, &right_ultra, 0) == pdTRUE) {
            // printf("Right distance %f\n", right_ultra);
        }
        
        if (right_ultra <= WALL_CLOSE_DIST || (left_ultra >= WALL_FAR_DIST && left_ultra <= WALL_VERY_FAR_DIST)) {
            right_boost = ULTRA_GAIN;
            left_boost = -ULTRA_GAIN;
        }
        else if (left_ultra <= WALL_CLOSE_DIST || (right_ultra >= WALL_FAR_DIST && right_ultra <= WALL_VERY_FAR_DIST)) {
            right_boost = -ULTRA_GAIN;
            left_boost = ULTRA_GAIN;
        }
        else {
            right_boost = 0;
            left_boost = 0;
        }

        // If we're close to the forward-facing wall, stop moving forwards
        if(xQueueReceive(UsQueue3, &front_ultra, 0) == pdTRUE) {
            // printf("Front distance %f\n", front_ultra);
        }
        if (front_ultra <= STOP_DIST) {
            break;
        }

        // Update the encoder readings
        xQueueReceive(distanceQueueLeft, &left_encoder, 0);
        xQueueReceive(distanceQueueRight, &right_encoder, 0);

        // Diff is positive if the right wheel has moved more and negative if the left wheel has moved more
        // float diff = (right_encoder - RIGHT_START) - (left_encoder - LEFT_START);
        float diff = 0;
        // printf("Diff: %f\n", diff);

        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER + diff*GAIN + left_boost); // motor 0 (left)
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER - diff*GAIN + right_boost); // motor 1 (right)
        vTaskDelay(1);
    }

    // Stop();
    // vTaskDelay(100);
    full_maze.currentNode = full_maze.nextNode;
    // printMaze();
}


// Stops the motors
void Stop() {
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);

    // "Stop" the motors by setting them to move backwards at a low enough power that they won't actually move
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 10); // motor 0 (right)
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, 10); // motor 1 (left)

}


void MoveTask(void * pvParameters) {

    initialize_motor_control();

    while (1) {
        if (xSemaphoreTake(maze_mutex, (TickType_t) portMAX_DELAY)) {
            if (full_maze.nextNode != NULL) {
                Move();
            }
            else {
                // xSemaphoreGive(scan_semaphore);
                xSemaphoreGive(maze_mutex);
                xSemaphoreGive(pathfind_semaphore);
            }

        }
    }
}


void Calibrate() {

    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30); // motor 0 (right)
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, 30); // motor 1 (left)

    // Update the encoder readings
    xQueueReceive(distanceQueueLeft, &left_encoder, portMAX_DELAY);
    xQueueReceive(distanceQueueRight, &right_encoder, portMAX_DELAY);

    Stop();
    vTaskDelay(300);
}


void TestTaskGoStraight(void * pvParameters) {
    initialize_motor_control();
    // Calibrate();

    while (1) {
        if (xSemaphoreTake(maze_mutex, (TickType_t) portMAX_DELAY)) {
            if (full_maze.nextNode != NULL) {
                GoStraight();
            }
            xSemaphoreGive(maze_mutex);

        }
    }
}

void DemoTaskTurnCorner(void * pvParameters) {
    initialize_motor_control();
    // Calibrate();
    float front_ultra;

    while (1) {
        if (xSemaphoreTake(maze_mutex, (TickType_t) portMAX_DELAY)) {
            if (full_maze.nextNode != NULL) {
                xQueueReceive(UsQueue3, &front_ultra, portMAX_DELAY);
                if (front_ultra > 10) {
                    GoStraight();
                    Stop();
                    vTaskDelay(50);
                }
                else {
                    TurnLeft();
                }
            }
            xSemaphoreGive(maze_mutex);
            // xSemaphoreGive(scan_semaphore);

        }
    }
}