#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "robot.h"
#include "maze.h"
#include "motor_control.h"


extern SemaphoreHandle_t maze_mutex;
extern struct Maze full_maze;

extern SemaphoreHandle_t scan_semaphore;
extern SemaphoreHandle_t pathfind_semaphore;


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

        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);

        xSemaphoreGive(maze_mutex);
        xSemaphoreGive(scan_semaphore);
    }
}


void TurnRight() {
    
    // TODO: spin motors
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 100); // motor 1 (left)
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1); // motor 2 (right)

    printf("Turning right\n");

    vTaskDelay(100);
    full_maze.heading = (full_maze.heading + 1) % 4;
    printMaze();
}


void TurnLeft() {
    
    // TODO: spin motors
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0); // motor 1 (left)
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, 100); // motor 2 (right)

    printf("Turning left\n");

    vTaskDelay(100);
    full_maze.heading = (full_maze.heading - 1) % 4;
    printMaze();
}


void GoStraight() {
    // TODO: spin motors
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 100); // motor 1 (left)
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, 100); // motor 2 (right)

    printf("Moving straight\n");

    vTaskDelay(100);
    full_maze.currentNode = full_maze.nextNode;
    printMaze();
}


void MoveTask(void * pvParameters) {

    initialize_motor_control();

    while (1) {
        if (xSemaphoreTake(maze_mutex, (TickType_t) portMAX_DELAY)) {
            if (full_maze.nextNode != NULL) {
                Move();
            }
            else {
                xSemaphoreGive(maze_mutex);
                xSemaphoreGive(pathfind_semaphore);
            }

        }
    }
}