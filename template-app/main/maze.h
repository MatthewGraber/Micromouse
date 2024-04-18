#ifndef MAZE
#define MAZE

#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <freertos/semphr.h>


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

    bool explored;

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

extern struct Maze full_maze;
extern struct Maze backup_maze;

void initalizeMaze(struct Node[10][10]);

// Update the connection of a node
void update_connection(struct Node[10][10], struct Node *node, int heading, bool val);

// Finds the distance of each node from the start and center of the maze
void Pathfind(struct Node[10][10]);
struct Node* NextNode(struct Node maze[10][10], struct Node *currentNode, int heading, bool goingToCenter);
void PathfindTask(void *);  // Deprecated

void CopyNode(struct Node *original, struct Node *copy);
void CopyMaze(struct Maze *original, struct Maze *copy);

// Called when there are discrepancies between our current location and our memory of that location
// Fixes where we are in space
// Returns true if successful
bool AdjustLocation(bool goingToCenter);

void printMaze();
void PrintDistanceToCenter(struct Node[10][10]);


// Returns the number of walls changed if it was already explored
// Otherwise returns 0
int Scan();
void ScanTask(void *);      // Deprecated

struct Node* getNodeAtHeading(struct Node* node, int heading);

#endif