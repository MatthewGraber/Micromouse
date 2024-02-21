#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#include "maze.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"


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
    update_connection(maze, &(maze[4][4]), North, false);
    update_connection(maze, &(maze[4][4]), West, false);

    update_connection(maze, &(maze[5][4]), North, false);
    update_connection(maze, &(maze[5][4]), East, false);

    update_connection(maze, &(maze[4][5]), South, false);
    update_connection(maze, &(maze[4][5]), West, false);

    update_connection(maze, &(maze[5][5]), South, false);
    // update_connection(maze, &(maze[5][5]), East, false);
}


void maze3() {

    update_connection(maze, &(maze[0][2]), North, false);
    update_connection(maze, &(maze[1][2]), North, false);
    update_connection(maze, &(maze[2][2]), North, false);
    update_connection(maze, &(maze[3][2]), North, false);
    update_connection(maze, &(maze[5][2]), North, false);
    update_connection(maze, &(maze[6][2]), North, false);
    update_connection(maze, &(maze[7][2]), North, false);

    update_connection(maze, &(maze[6][4]), North, false);
    update_connection(maze, &(maze[7][4]), North, false);
    update_connection(maze, &(maze[8][4]), North, false);


    update_connection(maze, &(maze[4][4]), North, false);
    update_connection(maze, &(maze[2][3]), West, false);
    update_connection(maze, &(maze[2][4]), West, false);
    update_connection(maze, &(maze[2][5]), West, false);

    update_connection(maze, &(maze[5][4]), North, false);
    update_connection(maze, &(maze[5][4]), East, false);

    update_connection(maze, &(maze[4][5]), South, false);
    update_connection(maze, &(maze[4][4]), West, false);
    update_connection(maze, &(maze[4][3]), West, false);
    update_connection(maze, &(maze[4][2]), West, false);

    update_connection(maze, &(maze[7][5]), North, false);
    update_connection(maze, &(maze[7][5]), East, false);
    update_connection(maze, &(maze[6][5]), East, false);
    update_connection(maze, &(maze[5][5]), East, false);
}


void maze4() {

    update_connection(maze, &(maze[0][2]), North, false);
    update_connection(maze, &(maze[1][1]), North, false);
    update_connection(maze, &(maze[2][1]), North, false);
    update_connection(maze, &(maze[3][1]), North, false);
    update_connection(maze, &(maze[5][1]), North, false);
    update_connection(maze, &(maze[6][1]), North, false);
    update_connection(maze, &(maze[7][1]), North, false);

    update_connection(maze, &(maze[6][3]), North, false);
    update_connection(maze, &(maze[7][3]), North, false);
    update_connection(maze, &(maze[8][3]), North, false);


    update_connection(maze, &(maze[4][4]), North, false);

    update_connection(maze, &(maze[2][1]), West, false);
    update_connection(maze, &(maze[2][2]), West, false);
    update_connection(maze, &(maze[2][3]), West, false);
    update_connection(maze, &(maze[2][4]), West, false);
    update_connection(maze, &(maze[2][5]), West, false);
    update_connection(maze, &(maze[2][6]), West, false);
    update_connection(maze, &(maze[2][7]), West, false);
    update_connection(maze, &(maze[2][8]), West, false);

    update_connection(maze, &(maze[5][4]), North, false);
    update_connection(maze, &(maze[5][4]), East, false);

    update_connection(maze, &(maze[4][5]), South, false);
    update_connection(maze, &(maze[4][4]), West, false);
    update_connection(maze, &(maze[4][3]), West, false);
    update_connection(maze, &(maze[4][2]), West, false);

    update_connection(maze, &(maze[7][6]), North, false);
    update_connection(maze, &(maze[7][6]), East, false);
    update_connection(maze, &(maze[6][6]), East, false);
    update_connection(maze, &(maze[5][6]), East, false);
}

void app_main(void)
{
    //time_t t;
    //void srand(unsigned int time(&t))

    initalizeMaze(maze);
    currentNode = &maze[0][0];

    maze1();
    // maze2();
    // maze3();
    // maze4();

    printMaze(maze, currentNode);

    // Pathfind(maze);
    // PrintDistanceToCenter(maze);


    bool goingToCenter = true;
    // int present;
    // int past = esp_timer_get_time();

    while (1) {
        // printMaze(maze, currentNode);
        // present = esp_timer_get_time();
        // if (present - past >= 1000000) {
        //     printf("Tick\n");
        //     past = present;
        // }

        vTaskDelay(100);


        // Maybe add something to the maze
        if (rand() % 2 == 0) {
            int x = rand() % 10;
            int y = rand() % 10;
            int head = rand() % 4;

            update_connection(maze, &(maze[x][y]), head, false);            
        }

        Pathfind(maze);
        nextNode = NextNode(maze, currentNode, goingToCenter);
        currentNode = nextNode;
        
        if (goingToCenter && currentNode->dist_to_center == 0) {
            printf("Going to start!\n");
            goingToCenter = false;
            // PrintDistanceToCenter(maze);
        }
        else if (!goingToCenter && currentNode->dist_to_start == 0) {
            printf("Going to center!\n");
            goingToCenter = true;
            // PrintDistanceToCenter(maze);

        }
    }
    

    //printf("hello world");
}