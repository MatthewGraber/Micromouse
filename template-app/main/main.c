#include <stdio.h>
#include <stdbool.h>
#include "maze.h"

struct Node maze[10][10];

struct Node* currentNode;
struct Node* nextNode;
bool moving = false;

void app_main(void)
{
    initalizeMaze(maze);

    // update_connection(&(maze[4][4]), South, false);
    // update_connection(&(maze[4][4]), East, false);
    update_connection(maze, &(maze[4][4]), North, false);
    update_connection(maze, &(maze[4][4]), West, false);

    update_connection(maze, &(maze[5][4]), North, false);
    update_connection(maze, &(maze[5][4]), East, false);

    update_connection(maze, &(maze[4][5]), South, false);
    update_connection(maze, &(maze[4][5]), West, false);

    // update_connection(&(maze[5][5]), South, false);
    // update_connection(&(maze[5][5]), East, false);

    printMaze(maze);

    Pathfind(maze);
    PrintDistanceToCenter(maze);

    //printf("hello world");
}