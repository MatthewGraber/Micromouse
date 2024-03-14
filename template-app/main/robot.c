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
            if (targetHeading > full_maze.heading) {
                TurnRight();
            }
            else {
                TurnLeft();
            }
        }

        GoStraight();

        xSemaphoreGive(maze_mutex);
        xSemaphoreGive(scan_semaphore);
    }
}


void TurnRight() {
    
    // TODO: spin motors
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, 100); // motor 1 (left)
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0); // motor 2 (right)

    printf("Turning right\n");

    vTaskDelay(100);
    full_maze.heading = (full_maze.heading + 1) % 4;
    printMaze();
    Stop();
}


void TurnLeft() {
    const float PI = 3.14159265358979323846;
    const float TURN_RADIUS = 100;  // mm
    const float TARGET_DIST = PI*TURN_RADIUS/2;  // Quarter-turn in mm
    // const float GAIN = 0.01;
    const float BASE_POWER = 60;

    float RIGHT_START;
    float right_dist;

    xQueueReceive(distanceQueue2, &RIGHT_START, portMAX_DELAY);
    right_dist = RIGHT_START;

    printf("Turning left\n");
    
    // Spin motors
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1); // motor 1 (left)

    while (right_dist - RIGHT_START < TARGET_DIST) {
        xQueueReceive(distanceQueue2, &right_dist, 0);
        // printf("Distance turned left: %f\n", right_dist- RIGHT_START);
        // Creates a gain based on the percentage of the distance covered
        float gain = ((TARGET_DIST - right_dist) / TARGET_DIST) * (100-BASE_POWER);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER + gain); // motor 2 (right)
        vTaskDelay(1);
    }


    // vTaskDelay(100);
    full_maze.heading = (full_maze.heading - 1) % 4;
    printMaze();
    Stop();
}


void GoStraight() {
    const float TARGET_DIST = 10*25.4;  // mm
    const float GAIN = 0.5;
    const float BASE_POWER = 80;
    const float STOP_DIST = 3.0;    // Stop if the ultrasonic sees something this close

    float LEFT_START, RIGHT_START;
    // Encoder readings
    float left_dist, right_dist;
    // Ultrasonic readings (cm)
    float front_ultra = 0, left_ultra = 0, right_ultra = 0;
    

    printf("Moving straight\n");
    
    // Spin motors
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER); // motor 1 (left)
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER); // motor 2 (right)

    xQueueReceive(distanceQueue1, &LEFT_START, portMAX_DELAY);
    printf("Got left distance\n");
    xQueueReceive(distanceQueue2, &RIGHT_START, portMAX_DELAY);
    printf("Got right distance\n");

    left_dist = LEFT_START;
    right_dist = RIGHT_START;



    // Exit condition
    // Drive until both distances are greater than the target distance
    while ((left_dist - LEFT_START < TARGET_DIST) || (right_dist - RIGHT_START < TARGET_DIST)) {
        xQueueReceive(distanceQueue1, &left_dist, 0);
        xQueueReceive(distanceQueue2, &right_dist, 0);
        
        // If we're close to the forward-facing wall, stop moving forwards
        // if(xQueueReceive(UsQueue3, &front_ultra, 0))
        //     printf("Front distance %f\n", front_ultra);
        // if (front_ultra <= STOP_DIST) {
        //     break;
        // }

        // Diff is positive if the right wheel has moved more and negative if the left wheel has moved more
        float diff = (right_dist - RIGHT_START) - (left_dist - LEFT_START);

        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER + diff*GAIN); // motor 1 (left)
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER - diff*GAIN); // motor 2 (right)
        vTaskDelay(1);
    }

    Stop();
    // vTaskDelay(100);
    full_maze.currentNode = full_maze.nextNode;
    printMaze();
}


// Stops the motors
void Stop() {
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
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