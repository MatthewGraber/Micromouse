#include <stdio.h>
#include <stdbool.h>


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
};

struct Node maze[10][10];

struct Node* currentNode;
struct Node* nextNode;
bool moving = false;

// Initialize the nodes
void initalizeMaze(struct Node[10][10]) {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            maze[i][j].x = i;
            maze[i][j].y = j;

            // Set the node's default walls
            // Check for a north or south border
            if (i == 0) {
                maze[i][j].connection[North] = false;
                maze[i][j].connection[South] = true;
            }
            else if (i == 9) {
                maze[i][j].connection[North] = true;
                maze[i][j].connection[South] = false;
            }
            else {
                maze[i][j].connection[North] = true;
                maze[i][j].connection[South] = true;
            }

            // Check for an east or west border
            if (j == 0) {
                maze[i][j].connection[East] = true;
                maze[i][j].connection[West] = false;
            }
            else if (j == 9) {
                maze[i][j].connection[East] = false;
                maze[i][j].connection[West] = true;
            }
            else {
                maze[i][j].connection[East] = true;
                maze[i][j].connection[West] = true;
            }

            // Set the initial distance from the center
            // Check to see if we're in the center
            if ((i == 4 || i == 5) && (j == 4 || j == 5)) { 
                maze[i][j].dist_to_center = 0;
            }
            else {
                maze[i][j].dist_to_center = -1;
            }
        }
    }
}


// Update the connection of a node
void update_connection(struct Node *node, int heading, bool val) {
    node->connection[heading] = val;

    // Update the corresponding node
    switch (heading)
    {
    case North:
        // Make sure this isn't at the top of the maze
        if (node->y > 0) {
            maze[node->x][node->y-1].connection[South] = val;
        }
        break;
    
    case East: 
        if (node->x < 9) { 
            maze[node->x+1][node->y].connection[West] = val;
        }
        break;

    case South:
        if (node->y < 9) {
            maze[node->x][node->y+1].connection[North] = val;
        }
        break;

    case West: 
        if (node->x > 0) { 
            maze[node->x-1][node->y].connection[East] = val;
        }
        break;

    default:
        break;
    }
}



void Pathfind() {

    // TODO: add a mutex for the maze
    

    // Reset the maze values
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) { 
            maze[i][j].dist_to_center = - 1;
        }
    }

    maze[4][4].dist_to_center = 0;
    maze[4][5].dist_to_center = 0;
    maze[5][4].dist_to_center = 0;
    maze[5][5].dist_to_center = 0;

    // Value tracks what distance we currently are from the center
    int value = 0;
    while (true) {
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                if (maze[i][j].dist_to_center == value) {

                    // Check to see if there is a connection North
                    if (maze[i][j].connection[North]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[i][j-1].dist_to_center == -1)
                            maze[i][j-1].dist_to_center = value + 1;
                    }

                    // Check to see if there is a connection East
                    if (maze[i][j].connection[East]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[i+1][j].dist_to_center == -1)
                            maze[i+1][j].dist_to_center = value + 1;
                    }

                    // Check to see if there is a connection North
                    if (maze[i][j].connection[South]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[i][j+1].dist_to_center == -1)
                            maze[i][j+1].dist_to_center = value + 1;
                    }

                    // Check to see if there is a connection North
                    if (maze[i][j].connection[West]) {
                        // If this doesn't already have a distance from the center, give it one
                        if (maze[i-1][j].dist_to_center == -1)
                            maze[i-1][j].dist_to_center = value + 1;
                    }
                }
            }
        }

        if (maze[0][0].dist_to_center != -1)
            break;
    }
}

void printMaze() {
    printf(" _ _ _ _ _ _ _ _ _ _\n");
    for (int x = 0; x < 10; x++) {
        
            printf("|");
            for (int y = 0; y < 10; y++) {
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
        // // If x is odd, print horizontal walls
        // else {
        //     printf(" ");
        //     for (int y = 0; y < 10; y++) {
        //         if (maze[x/2][y].connection[South]) {
        //             printf("  ");
        //         }
        //         else {
        //             printf("- ");
        //         }
        //     }
        // }

        printf("\n");
    }


    for (int x = 0; x < 10; x++) { 
        for (int y = 0; y < 10; y++) {

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

void app_main(void)
{
    initalizeMaze(maze);

    update_connection(&(maze[4][4]), South, false);
    update_connection(&(maze[4][4]), East, false);
    update_connection(&(maze[4][4]), North, false);
    // update_connection(&(maze[4][4]), West, false);

    // update_connection(&(maze[5][4]), North, false);
    // update_connection(&(maze[5][4]), East, false);

    // update_connection(&(maze[4][5]), South, false);
    // update_connection(&(maze[4][5]), West, false);

    // update_connection(&(maze[5][5]), South, false);
    // update_connection(&(maze[5][5]), East, false);

    printMaze();
    //printf("hello world");
}