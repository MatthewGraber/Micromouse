#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <freertos/semphr.h>


#ifndef MAZE
#define MAZE

#define North 0
#define East 1
#define South 2
#define West 3


// Each node represents a space in the maze
struct Node {
    u_int8_t x;
    u_int8_t y;

    // Each connection represents a space without a wall
    bool connection[4];

    int16_t dist_to_center;
    int16_t dist_to_start;
};

struct Maze {
    struct Node maze[10][10];

    struct Node* currentNode;
    struct Node* nextNode;
    bool moving;
    bool goingToCenter;
    int heading;
};

void initalizeMaze(struct Node[10][10]);

// Update the connection of a node
void update_connection(struct Node[10][10], struct Node *node, int heading, bool val);

// Finds the distance of each node from the start and center of the maze
void Pathfind(struct Node[10][10]);
struct Node* NextNode(struct Node maze[10][10], struct Node *currentNode, bool goingToCenter);
void PathfindTask(void *);


void printMaze();
void PrintDistanceToCenter(struct Node[10][10]);

//void ScannerInit(SemaphoreHandle_t scan_handle);

void Scan();
void ScanTask(void *);

// static long get_nanos(void);

#endif