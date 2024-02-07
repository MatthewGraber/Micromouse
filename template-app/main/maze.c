#include "maze.h"

// Initialize the nodes
void initalizeMaze(struct Node maze[10][10]) {
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            maze[x][y].x = x;
            maze[x][y].y = y;

            // Set the node's default walls
            // Check for a north or south border
            if (y == 0) {
                maze[x][y].connection[North] = false;
                maze[x][y].connection[South] = true;
            }
            else if (y == 9) {
                maze[x][y].connection[North] = true;
                maze[x][y].connection[South] = false;
            }
            else {
                maze[x][y].connection[North] = true;
                maze[x][y].connection[South] = true;
            }

            // Check for an east or west border
            if (x == 0) {
                maze[x][y].connection[East] = true;
                maze[x][y].connection[West] = false;
            }
            else if (x == 9) {
                maze[x][y].connection[East] = false;
                maze[x][y].connection[West] = true;
            }
            else {
                maze[x][y].connection[East] = true;
                maze[x][y].connection[West] = true;
            }

            // Set the initial distance from the center
            // Check to see if we're in the center
            if ((x == 4 || x == 5) && (y == 4 || y == 5)) { 
                maze[x][y].dist_to_center = 0;
            }
            else {
                maze[x][y].dist_to_center = -1;
            }
        }
    }
}


// Update the connection of a node
void update_connection(struct Node maze[10][10], struct Node *node, int heading, bool val) {
    node->connection[heading] = val;

    // Update the corresponding node
    switch (heading)
    {
    case North:
        // Make sure this isn't at the top of the maze
        if (node->y > 0) {
            int x = node->x;
            int y = node->y - 1;
            printf("Updating node X: %d, Y: %d\n", x, y);
            maze[node->x][(node->y)-1].connection[South] = val;
        }
        break;
    
    case East: 
        if (node->x < 9) { 
            maze[(node->x)+1][node->y].connection[West] = val;
        }
        break;

    case South:
        if (node->y < 9) {
            maze[node->x][(node->y)+1].connection[North] = val;
        }
        break;

    case West: 
        if (node->x > 0) { 
            maze[(node->x)-1][node->y].connection[East] = val;
        }
        break;

    default:
        break;
    }
}


// Finds the distance of each node from the start and center of the maze
void Pathfind(struct Node maze[10][10]) {

    // TODO: add a mutex for the maze
    

    // Reset the maze values
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) { 
            maze[i][j].dist_to_center = - 1;
            maze[i][j].dist_to_start = -1;
        }
    }

    maze[4][4].dist_to_center = 0;
    maze[4][5].dist_to_center = 0;
    maze[5][4].dist_to_center = 0;
    maze[5][5].dist_to_center = 0;

    maze[0][0].dist_to_start = 0;

    // Value tracks what distance we currently are from the center
    for (int value = 0; maze[0][0].dist_to_center == -1 || (value > 100); value++) {
        for (int x = 0; x < 10; x++) {
            for (int y = 0; y < 10; y++) {
                if (maze[x][y].dist_to_center == value) {

                    // Check to see if there is a connection North
                    if (maze[x][y].connection[North]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[x][y-1].dist_to_center == -1)
                            maze[x][y-1].dist_to_center = value + 1;
                    }

                    // Check to see if there is a connection East
                    if (maze[x][y].connection[East]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[x+1][y].dist_to_center == -1)
                            maze[x+1][y].dist_to_center = value + 1;
                    }

                    // Check to see if there is a connection North
                    if (maze[x][y].connection[South]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[x][y+1].dist_to_center == -1)
                            maze[x][y+1].dist_to_center = value + 1;
                    }

                    // Check to see if there is a connection North
                    if (maze[x][y].connection[West]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[x-1][y].dist_to_center == -1)
                            maze[x-1][y].dist_to_center = value + 1;
                    }
                }

                // Distance to the start of the maze
                if (maze[x][y].dist_to_start == value) {
                    // Check to see if there is a connection North
                    if (maze[x][y].connection[North]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[x][y-1].dist_to_start == -1)
                            maze[x][y-1].dist_to_start = value + 1;
                    }

                    // Check to see if there is a connection East
                    if (maze[x][y].connection[East]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[x+1][y].dist_to_start == -1)
                            maze[x+1][y].dist_to_start = value + 1;
                    }

                    // Check to see if there is a connection North
                    if (maze[x][y].connection[South]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[x][y+1].dist_to_start == -1)
                            maze[x][y+1].dist_to_start = value + 1;
                    }

                    // Check to see if there is a connection North
                    if (maze[x][y].connection[West]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[x-1][y].dist_to_start == -1)
                            maze[x-1][y].dist_to_start = value + 1;
                    }
                }
            }
        }
    }
}

// Display the maze
void printMaze(struct Node maze[10][10]) {
    printf(" _ _ _ _ _ _ _ _ _ _\n");
    for (int y = 0; y < 10; y++) {
        
            printf("|");
            for (int x = 0; x < 10; x++) {
                if (maze[x][y].connection[South]) {
                    printf(" ");
                }
                else {
                    printf("_");
                }

                if (maze[x][y].connection[East]) { 
                    printf(" ");
                }
                else {
                    printf("|");
                }
            }

        printf("\n");
    }


    for (int y = 0; y < 10; y++) { 
        for (int x = 0; x < 10; x++) {

            int out = 0;
            if (maze[x][y].connection[North]) { out += 1; }
            if (maze[x][y].connection[East]) { out += 2; }
            if (maze[x][y].connection[South]) { out += 4; }
            if (maze[x][y].connection[West]) { out += 8; }
            printf("%x ", out);
        }
        printf("\n");
    }
}

// Displays the distance to the center of the maze
void PrintDistanceToCenter(struct Node maze[10][10]) {
    printf("Distance to the center of the maze\n");
    for (int y = 0; y < 10; y++) { 
        for (int x = 0; x < 10; x++) {
            printf("%02d ", maze[x][y].dist_to_center);
        }
        printf("\n\n");
    }

    printf("Distance to the start of the maze\n");
    for (int y = 0; y < 10; y++) { 
        for (int x = 0; x < 10; x++) {
            printf("%02d ", maze[x][y].dist_to_start);
        }
        printf("\n");
    }
}