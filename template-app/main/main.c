#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#include "maze.h"
#include "robot.h"
#include <ultrasonic.h>
#include "ultrasonicLaunch.h"
#include "encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "freertos/queue.h"

#define STACK_SIZE 1028*8

// Semaphore that triggers when it's time to scan
SemaphoreHandle_t scan_semaphore;
SemaphoreHandle_t pathfind_semaphore;

// The maze
struct Maze full_maze;
SemaphoreHandle_t maze_mutex;

// Queues
QueueHandle_t UsQueue1;
QueueHandle_t UsQueue2;
QueueHandle_t UsQueue3;

struct Node maze[10][10];

struct Node* currentNode;
struct Node* nextNode;
bool moving = false;

void maze1() {
    update_connection(maze, &(maze[4][4]), North, false);
    update_connection(maze, &(maze[4][4]), West, false);

    update_connection(maze, &(maze[5][4]), North, false);
    update_connection(maze, &(maze[5][4]), East, false);

    update_connection(maze, &(maze[4][5]), South, false);
    update_connection(maze, &(maze[4][5]), West, false);

    update_connection(maze, &(maze[5][5]), South, false);
    update_connection(maze, &(maze[5][5]), East, false);
}

void maze2() {
    update_connection(full_maze.maze, &(full_maze.maze[4][4]), North, false);
    update_connection(full_maze.maze, &(maze[4][4]), West, false);

    update_connection(full_maze.maze, &(full_maze.maze[5][4]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[5][4]), East, false);

    update_connection(full_maze.maze, &(full_maze.maze[4][5]), South, false);
    update_connection(full_maze.maze, &(full_maze.maze[4][5]), West, false);

    update_connection(full_maze.maze, &(full_maze.maze[5][5]), South, false);
    // update_connection(full_maze.maze, &(full_maze.maze[5][5]), East, false);
}


void maze3() {

    update_connection(full_maze.maze, &(full_maze.maze[0][2]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[1][2]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][2]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[3][2]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[5][2]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[6][2]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[7][2]), North, false);

    update_connection(full_maze.maze, &(full_maze.maze[6][4]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[7][4]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[8][4]), North, false);


    update_connection(full_maze.maze, &(full_maze.maze[4][4]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][3]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][4]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][5]), West, false);

    update_connection(full_maze.maze, &(full_maze.maze[5][4]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[5][4]), East, false);

    update_connection(full_maze.maze, &(full_maze.maze[4][5]), South, false);
    update_connection(full_maze.maze, &(full_maze.maze[4][4]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[4][3]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[4][2]), West, false);

    update_connection(full_maze.maze, &(full_maze.maze[7][5]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[7][5]), East, false);
    update_connection(full_maze.maze, &(full_maze.maze[6][5]), East, false);
    update_connection(full_maze.maze, &(full_maze.maze[5][5]), East, false);
}


void maze4() {

    update_connection(full_maze.maze, &(full_maze.maze[0][2]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[1][1]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][1]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[3][1]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[5][1]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[6][1]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[7][1]), North, false);

    update_connection(full_maze.maze, &(full_maze.maze[6][3]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[7][3]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[8][3]), North, false);


    update_connection(full_maze.maze, &(full_maze.maze[4][4]), North, false);

    update_connection(full_maze.maze, &(full_maze.maze[2][1]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][2]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][3]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][4]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][5]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][6]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][7]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[2][8]), West, false);

    update_connection(full_maze.maze, &(full_maze.maze[5][4]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[5][4]), East, false);

    update_connection(full_maze.maze, &(full_maze.maze[4][5]), South, false);
    update_connection(full_maze.maze, &(full_maze.maze[4][4]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[4][3]), West, false);
    update_connection(full_maze.maze, &(full_maze.maze[4][2]), West, false);

    update_connection(full_maze.maze, &(full_maze.maze[7][6]), North, false);
    update_connection(full_maze.maze, &(full_maze.maze[7][6]), East, false);
    update_connection(full_maze.maze, &(full_maze.maze[6][6]), East, false);
    update_connection(full_maze.maze, &(full_maze.maze[5][6]), East, false);
}


void exampleRecieve() {
    float distance1 = 0;
    float distance2 = 0;

    while (1) {
        if (xQueueReceive(distanceQueue1, &distance1, 0)) {
            printf("Distance1: %f, Distance2:%f\n", distance1, distance2);
        }
        if (xQueueReceive(distanceQueue2, &distance2, 0)) {
            printf("Distance1: %f, Distance2:%f\n", distance1, distance2);
        }
        vTaskDelay(1);
    }
}


void SillyOldWay();

void app_main(void) {
    // Initalize semaphores
    scan_semaphore = xSemaphoreCreateBinary();
    pathfind_semaphore = xSemaphoreCreateBinary();
    maze_mutex = xSemaphoreCreateMutex();
    
    // Ultrasonic stuff
    //Sensor 1 (left)
    UsQueue1 = xQueueCreate( 1, sizeof( float));
    struct USParam us1;
    us1.ECHO_GPIO = 17;
    us1.TRIGGER_GPIO = 18;
    us1.Usqueue = UsQueue1;
    float distance1;
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 5,(void*) &us1, 5, NULL);
    
    //Sensor 2 (right)
    UsQueue2 = xQueueCreate( 1, sizeof( float));
    struct USParam us2;
    us2.ECHO_GPIO = 12;
    us2.TRIGGER_GPIO = 11;
    us2.Usqueue = UsQueue2;
    float distance2;
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 5,(void*) &us2, 5, NULL);

    //Sensor 3 (forward)
    UsQueue3 = xQueueCreate( 1, sizeof( float));
    struct USParam us3;
    us3.ECHO_GPIO = 3;
    us3.TRIGGER_GPIO = 8;
    us3.Usqueue = UsQueue3;
    float distance3;
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 5,(void*) &us3, 5, NULL);

    //time_t t;
    //void srand(unsigned int time(&t))

    initalizeMaze(full_maze.maze);
    full_maze.currentNode = &full_maze.maze[0][0];

    // Initialize
    initializeEncoder();


    // maze1();
    // maze2();
    // maze3();
    // maze4();

    printMaze();

    // Pathfind(maze);
    // PrintDistanceToCenter(maze);

    static uint8_t uc_param_pathfind;
    static uint8_t uc_param_scan;
    static uint8_t uc_param_move;

    TaskHandle_t pathfind_task = NULL;
    TaskHandle_t scan_task = NULL;
    TaskHandle_t move_task = NULL;
    
    xSemaphoreGive(scan_semaphore);

    xTaskCreate(PathfindTask, "PATHFIND", STACK_SIZE, &uc_param_pathfind, 1, &pathfind_task );
    // configASSERT(pathfind_task);
    
    xTaskCreate(ScanTask, "SCAN", STACK_SIZE, &uc_param_scan, 1, &scan_task );
    // configASSERT(scan_task);

    xTaskCreate(MoveTask, "MOVE", STACK_SIZE, &uc_param_move, tskIDLE_PRIORITY, &move_task );
    // configASSERT(move_task);



    // Create encoder task
    xTaskCreate(encoderTask, "encoder_task", 4096, NULL, tskIDLE_PRIORITY, NULL);
    // xTaskCreate(exampleRecieve, "example_Recieve", 4096, NULL, tskIDLE_PRIORITY, NULL);


    // xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY, NULL);
    // configASSERT(ultrasonic_test);

}