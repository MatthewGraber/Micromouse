#include <stdbool.h>
#include <stdio.h>

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

void initalizeMaze(struct Node[10][10]);

// Update the connection of a node
void update_connection(struct Node[10][10], struct Node *node, int heading, bool val);

// Finds the distance of each node from the start and center of the maze
void Pathfind(struct Node[10][10]);

void printMaze(struct Node[10][10]);

void PrintDistanceToCenter(struct Node[10][10]);


#endif