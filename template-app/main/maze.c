#include "maze.h"
#include <stdlib.h>
#include <time.h>

#include "esp_timer.h"

#define ANSI_RESET_ALL          "\x1b[0m"

#define ANSI_BACKGROUND_BLACK   "\x1b[40m"
#define ANSI_BACKGROUND_RED     "\x1b[41m"
#define ANSI_BACKGROUND_GREEN   "\x1b[42m"
#define ANSI_BACKGROUND_YELLOW  "\x1b[43m"
#define ANSI_BACKGROUND_BLUE    "\x1b[44m"
#define ANSI_BACKGROUND_MAGENTA "\x1b[45m"
#define ANSI_BACKGROUND_CYAN    "\x1b[46m"
#define ANSI_BACKGROUND_WHITE   "\x1b[47m"


// Semaphore that triggers when it's time to scan
extern SemaphoreHandle_t scan_semaphore;
extern SemaphoreHandle_t pathfind_semaphore;

// Task externals
extern TaskHandle_t pathfind_task;
extern TaskHandle_t scan_task;

// The maze
extern struct Maze full_maze;
extern SemaphoreHandle_t maze_mutex;

// Queues
extern QueueHandle_t UsQueue;

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
    // current = &maze[0][0];
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


////////////////////////////////////////////////////////////
///////////// PATHFINDING TASK FUNCTIONS ///////////////////
////////////////////////////////////////////////////////////


// Finds the distance of each node from the start and center of the maze
void Pathfind(struct Node maze[10][10]) {

    // printf("Pathfinding\n");
    
    int start = esp_timer_get_time();

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

    // printf("Reset values\n");

    // Value tracks what distance we currently are from the center
    for (int value = 0; (maze[0][0].dist_to_center == -1) && (value < 100); value++) {
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

    
    //do stuff
    int stop = esp_timer_get_time();
    // printf("took %d us\n", (stop - start)); 
}


// Set the next destination
struct Node* NextNode(struct Node maze[10][10], struct Node *currentNode, bool goingToCenter) {
    // printf("Next node\n");
    
    if (goingToCenter) { 

        struct Node *closestNode = currentNode;
        int x = currentNode->x;
        int y = currentNode->y;

        // Check West
        if (!currentNode->connection[West]) {
            // Pass
        }
        else if (closestNode->dist_to_center > maze[x-1][y].dist_to_center) {
            if (maze[x-1][y].dist_to_center >= 0)
                closestNode = &maze[x-1][y];
        }

        // Check East
        if (!currentNode->connection[East]) {
            // Pass
        }
        else if (closestNode->dist_to_center > maze[x+1][y].dist_to_center) {
            if (maze[x+1][y].dist_to_center >= 0)
                closestNode = &maze[x+1][y];
        }

        // Check North
        if (!currentNode->connection[North]) {
            // Pass
        }
        else if (closestNode->dist_to_center > maze[x][y-1].dist_to_center) {
            if (maze[x][y-1].dist_to_center >= 0)
                closestNode = &maze[x][y-1];
        }

        // Check South
        if (!currentNode->connection[South]) {
            // Pass
        }
        else if (closestNode->dist_to_center > maze[x][y+1].dist_to_center) {
            if (maze[x][y+1].dist_to_center >= 0)
                closestNode = &maze[x][y+1];
        }
        
        // Set the next node to the closest node
        return closestNode;
    }


    // If we're going to the start of the maze
    else { 
        // int dist = currentNode->dist_to_center;
        struct Node *closestNode = currentNode;
        int x = currentNode->x;
        int y = currentNode->y;

        // Check West
        if (!currentNode->connection[West]) {
            // Pass
        }
        else if (closestNode->dist_to_start > maze[x-1][y].dist_to_start) {
            if (maze[x-1][y].dist_to_start >= 0)
                closestNode = &maze[x-1][y];
        }

        // Check East
        if (!currentNode->connection[East]) {
            // Pass
        }
        else if (closestNode->dist_to_start > maze[x+1][y].dist_to_start) {
            if (maze[x+1][y].dist_to_start >= 0)
                closestNode = &maze[x+1][y];
        }

        // Check North
        if (!currentNode->connection[North]) {
            // Pass
        }
        else if (closestNode->dist_to_start > maze[x][y-1].dist_to_start) {
            if (maze[x][y-1].dist_to_start >= 0)
                closestNode = &maze[x][y-1];
        }

        // Check South
        if (!currentNode->connection[South]) {
            // Pass
        }
        else if (closestNode->dist_to_start > maze[x][y+1].dist_to_start) {
            if (maze[x][y+1].dist_to_start >= 0)
                closestNode = &maze[x][y+1];
        }
        
        // Set the next node to the closest node
        return closestNode;
    }


}


void PathfindTask(void * pvParameters) {
    bool goingToCenter = true;
    bool debug = true;

    while (1) {
        if (xSemaphoreTake(pathfind_semaphore, (TickType_t) portMAX_DELAY)) {
            if (xSemaphoreTake(maze_mutex, (TickType_t) portMAX_DELAY)) {

                // Switch direction if necessary
                if ((full_maze.currentNode->x == 4 || full_maze.currentNode->x == 5) &&
                    (full_maze.currentNode->y == 4 || full_maze.currentNode->y == 5)) {

                    goingToCenter = false;
                }
                else if (full_maze.currentNode->x == 0 && full_maze.currentNode->y == 0) {
                    goingToCenter = true;
                }

                // Find the next location to move to
                Pathfind(full_maze.maze);
                full_maze.nextNode = NextNode(full_maze.maze, full_maze.currentNode, goingToCenter);
                xSemaphoreGive(maze_mutex);
            }
        }
        
    }
}


// Display the maze
void printMaze() {

    printf("Heading: ");
    switch (full_maze.heading) {
        case North:
            printf("North\n");
            break;
        case East:
            printf("East\n");
            break;
        case South:
            printf("South\n");
            break;
        case West:
            printf("West\n");
            break;
        default:
            break;
    }

    printf(" _ _ _ _ _ _ _ _ _ _\n");
    for (int y = 0; y < 10; y++) {
        
            printf("|");
            for (int x = 0; x < 10; x++) {

                // If this space is where the robot is, highlight it blue
                if (x == full_maze.currentNode->x && y == full_maze.currentNode->y) {
                    printf(ANSI_BACKGROUND_BLUE);
                }
                if (full_maze.maze[x][y].connection[South]) {
                    printf(" ");
                }
                else {
                    printf("_");
                }

                // Reset background
                printf(ANSI_RESET_ALL);

                if (full_maze.maze[x][y].connection[East]) { 
                    printf(" ");
                }
                else {
                    printf("|");
                }
            }

        printf("\n");
    }


    // for (int y = 0; y < 10; y++) { 
    //     for (int x = 0; x < 10; x++) {

    //         int out = 0;
    //         if (maze[x][y].connection[North]) { out += 1; }
    //         if (maze[x][y].connection[East]) { out += 2; }
    //         if (maze[x][y].connection[South]) { out += 4; }
    //         if (maze[x][y].connection[West]) { out += 8; }
    //         printf("%x ", out);
    //     }
    //     printf("\n");
    // }
}

// Displays the distance to the center of the maze
void PrintDistanceToCenter(struct Node maze[10][10]) {
    printf("Distance to the center of the maze\n");
    for (int y = 0; y < 10; y++) { 
        for (int x = 0; x < 10; x++) {
            printf("%02d ", maze[x][y].dist_to_center);
        }
        printf("\n");
    }

    printf("Distance to the start of the maze\n");
    for (int y = 0; y < 10; y++) { 
        for (int x = 0; x < 10; x++) {
            printf("%02d ", maze[x][y].dist_to_start);
        }
        printf("\n");
    }
}


////////////////////////////////////////////////////////////
/////////////// SCANNING TASK FUNCTIONS ////////////////////
////////////////////////////////////////////////////////////


// void ScannerInit(SemaphoreHandle_t scan_handle) {
//     scan_semaphore = scan_handle;
// }


// Scans the environment and updates the maze accordingly
void Scan() {
    
    // Wall length in cm
    float WALL_LENGTH = 25.4;
    // Furthest distance we can trust to be accurate
    float MAX_DISTANCE = 100;


    // Placeholders for the ultrasonic readings
    // float frontDistance = MAX_DISTANCE;
    float frontDistance;
    float leftDistance = MAX_DISTANCE;
    float rightDistance = MAX_DISTANCE;

    int closestFront = 0;
    int closestLeft = 0;
    int closestRight = 0;

    printf("Scanning\n");

    // Stop the motors
    // TODO: this


    // Wait for a bit
    // TODO: this


    // Front
    if (xQueueReceive(UsQueue, &frontDistance, 0) == pdPASS) {
        printf("Front distance: %f\n", frontDistance);
        if (frontDistance < MAX_DISTANCE) {
            closestFront = (int)(frontDistance / WALL_LENGTH);
            struct Node *updatedNode = full_maze.currentNode;

            // update_connection(maze, currentNode, currentHeading, false);
            switch (full_maze.heading) {
            case North:
                updatedNode = &full_maze.maze[full_maze.currentNode->x][full_maze.currentNode->y-closestFront];
                break;

            case East:
                updatedNode = &full_maze.maze[full_maze.currentNode->x + closestFront][full_maze.currentNode->y];
                break;
            
            case South:
                updatedNode = &full_maze.maze[full_maze.currentNode->x][full_maze.currentNode->y+closestFront];
                break;

            case West:
                updatedNode = &full_maze.maze[full_maze.currentNode->x - closestFront][full_maze.currentNode->y];
                break;

            }

            update_connection(full_maze.maze, updatedNode, full_maze.heading, false);
            
        }
    }


}


// Scanning task
void ScanTask(void * pvParameters)
{
  for( ;; )
  {
    // if (xSemaphoreTake(scan_semaphore, ( TickType_t ) 10) == pdTRUE ) {
        if (xSemaphoreTake(maze_mutex, ( TickType_t ) portMAX_DELAY) == pdTRUE) {
            Scan();
            xSemaphoreGive(maze_mutex);
            xSemaphoreGive(pathfind_semaphore);
        }
    // }
    vTaskDelay(100);
  }
}

// static long get_nanos(void) {
//     struct timespec ts;
//     timespec_get(&ts, TIME_UTC);
//     return (long)ts.tv_sec * 1000000000L + ts.tv_nsec;
// }