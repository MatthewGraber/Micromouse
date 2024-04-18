#include "maze.h"
#include <stdlib.h>
#include <time.h>
#include <robot.h>

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
// extern struct Maze full_maze;
extern SemaphoreHandle_t maze_mutex;

// Queues
extern QueueHandle_t UsQueue1;
extern QueueHandle_t UsQueue2;
extern QueueHandle_t UsQueue3;

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

    printf("Pathfinding\n");
    
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
    for (int value = 0; (full_maze.currentNode->dist_to_center == -1 || full_maze.currentNode->dist_to_start == -1) && (value < 100); value++) {
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
struct Node* NextNode(struct Node maze[10][10], struct Node *currentNode, int heading, bool goingToCenter) {
    printf("Next node\n");
    
    // if (full_maze.goingToCenter) { 

    //     int x = currentNode->x;
    //     int y = currentNode->y;

    //     // Check West
    //     if (!currentNode->connection[West]) {
    //         // Pass
    //     }
    //     else if (closestNode->dist_to_center > maze[x-1][y].dist_to_center) {
    //         if (maze[x-1][y].dist_to_center >= 0)
    //             closestNode = &maze[x-1][y];
    //     }

    //     // Check East
    //     if (!currentNode->connection[East]) {
    //         // Pass
    //     }
    //     else if (closestNode->dist_to_center > maze[x+1][y].dist_to_center) {
    //         if (maze[x+1][y].dist_to_center >= 0)
    //             closestNode = &maze[x+1][y];
    //     }

    //     // Check North
    //     if (!currentNode->connection[North]) {
    //         // Pass
    //     }
    //     else if (closestNode->dist_to_center > maze[x][y-1].dist_to_center) {
    //         if (maze[x][y-1].dist_to_center >= 0)
    //             closestNode = &maze[x][y-1];
    //     }

    //     // Check South
    //     if (!currentNode->connection[South]) {
    //         // Pass
    //     }
    //     else if (closestNode->dist_to_center > maze[x][y+1].dist_to_center) {
    //         if (maze[x][y+1].dist_to_center >= 0)
    //             closestNode = &maze[x][y+1];
    //     }
        
    //     // Set the next node to the closest node
    //     return closestNode;
    // }


    // // If we're going to the start of the maze
    // else { 
    //     // int dist = currentNode->dist_to_center;
    //     struct Node *closestNode = currentNode;
    //     int x = currentNode->x;
    //     int y = currentNode->y;

    //     // Check West
    //     if (!currentNode->connection[West]) {
    //         // Pass
    //     }
    //     else if (closestNode->dist_to_start > maze[x-1][y].dist_to_start) {
    //         if (maze[x-1][y].dist_to_start >= 0)
    //             closestNode = &maze[x-1][y];
    //     }

    //     // Check East
    //     if (!currentNode->connection[East]) {
    //         // Pass
    //     }
    //     else if (closestNode->dist_to_start > maze[x+1][y].dist_to_start) {
    //         if (maze[x+1][y].dist_to_start >= 0)
    //             closestNode = &maze[x+1][y];
    //     }

    //     // Check North
    //     if (!currentNode->connection[North]) {
    //         // Pass
    //     }
    //     else if (closestNode->dist_to_start > maze[x][y-1].dist_to_start) {
    //         if (maze[x][y-1].dist_to_start >= 0)
    //             closestNode = &maze[x][y-1];
    //     }

    //     // Check South
    //     if (!currentNode->connection[South]) {
    //         // Pass
    //     }
    //     else if (closestNode->dist_to_start > maze[x][y+1].dist_to_start) {
    //         if (maze[x][y+1].dist_to_start >= 0)
    //             closestNode = &maze[x][y+1];
    //     }
        
        // Set the next node to the closest node
    // }

    struct Node *closestNode = currentNode;
    int lowest = 100;
    int mod = 0;



    while (mod < 4) {
        int targetHeading = (heading + mod) % 4;
        int nextDist = 1000;

        if (currentNode->connection[targetHeading]) {
            struct Node *node = getNodeAtHeading(currentNode, targetHeading);

            if (node == NULL) {

            }
            else {
                if (goingToCenter) {
                    nextDist = node->dist_to_center;
                }
                else {
                    nextDist = node->dist_to_start;
                }


                if (nextDist < lowest && nextDist >= 0) {
                    lowest = nextDist;
                    closestNode = node;
                    printf("Set closest node to x=%d, y=%d\n", closestNode->x, closestNode->y);
                }

            }
        }
        else {
            printf("No connection at heading: %d\n", targetHeading);
        }
            
        switch (mod) {
            case 0:
                mod = 1;
                break;
            case 1:
                mod = 3;
                break;
            case 2:
                mod = 4;
                break;
            case 3:
                mod = 2;
                break;
            default:
            break;
            
        }
        

    }
    return closestNode;


}


struct Node* getNodeAtHeading(struct Node* node, int heading) {
    heading = (heading + 4) % 4;
    switch (heading) {
        case North: 
            if (node->y > 0)
                return &(full_maze.maze[node->x][node->y-1]);
            break;
        case East: 
            if (node->x < 9)
                return &(full_maze.maze[node->x+1][node->y]);
            break;
        case South: 
            if (node->y < 9)
                return &(full_maze.maze[node->x][node->y+1]);
            break;
        case West: 
            if (node->x > 0)
                return &(full_maze.maze[node->x-1][node->y]);
            break;
    }
    return NULL;
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
                full_maze.nextNode = NextNode(full_maze.maze, full_maze.currentNode, full_maze.heading, goingToCenter);
                xSemaphoreGive(maze_mutex);
            }
        }
        
    }
}


// Copys one node to another
void CopyNode(struct Node *original, struct Node *copy) {
    copy->x = original->x;
    copy->y = original->y;
    
    copy->dist_to_center = original->dist_to_center;
    copy->dist_to_start = original->dist_to_start;

    copy->explored = original->explored;

    for (int i = 0; i < 4; i++) {
        copy->connection[i] = original->connection[i];
    }
}


// Copies the current maze into a backup
void CopyMaze(struct Maze *original, struct Maze *copy) {
    for (int x = 0; x < 10; x++) { 
        for (int y = 0; y < 10; y++) {
            CopyNode(&original->maze[x][y], &copy->maze[x][y]);
        }
    }
    copy->heading = original->heading;
}


// Called when there are discrepancies between our current location and our memory of that location
// Fixes where we are in space
// Returns true if successful
bool AdjustLocation(bool goingToCenter) {

    // The path that is expected to be taken
    const int MAX_PATH = 200;
    struct Node path[MAX_PATH];
    bool match[MAX_PATH];

    int fakeHeading = backup_maze.heading;
    int misses = 0;
    int i;
    for (i = 0; i < MAX_PATH; i++) { 
        struct Node *current;
        current = NextNode(backup_maze.maze, backup_maze.currentNode, fakeHeading, goingToCenter);
        path[i] = *current;

        // Turn
        int targetHeading;
        int xDif = full_maze.currentNode->x - full_maze.nextNode->x;    // -1 == East, 1 == West
        int yDif = full_maze.currentNode->y - full_maze.nextNode->y;    // -1 == South, 1 == North
        if ((xDif == 0) == (yDif == 0)) {
            // printf("Invalid next node\n");
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
            if ((targetHeading-full_maze.heading + 4) % 4 == 1) {
                fakeHeading = (fakeHeading + 1) % 4;
            }
            else if ((targetHeading-full_maze.heading + 4) % 4 == 3) {
                fakeHeading = (fakeHeading + 3) % 4;
            }
        }

        // Make sure we've been there before evaluating it
        if (current->explored) {
            match[i] = true;
            for (int j = 0; j < 4; j++) {
                if (current->connection[j] != full_maze.maze[current->x][current->y].connection[j]) {
                    match[i] = false;
                }
            }
        }
        // If we haven't explored here before, then assume we saw correctly
        else {
            for (int j = 0; j < 4; j++) {
                current->connection[j] = full_maze.maze[current->x][current->y].connection[j];
                // Set these values for the backup maze as well
                // Don't set it to explored, so we can undo it later
                backup_maze.maze[current->x][current->y].connection[j] = full_maze.maze[current->x][current->y].connection[j];
                match[i] = true;
            }
            Pathfind(backup_maze.maze);
        }
        backup_maze.currentNode = current;

        // if (misses  3) {

        // }
        if (current->x != full_maze.currentNode->x) { 

        }
        else if (current->y != full_maze.currentNode->y) { 
        
        }
        else {
            // Get the next space in the path before exiting
            current = NextNode(backup_maze.maze, backup_maze.currentNode, backup_maze.heading, goingToCenter);
            path[i+1] = *current;
            break;
        }

        if (!match[i]) {
            misses++;
        }


    }


    // Check to see if it moved north an extra space
    int certainty[4] = {0, 0, 0, 0};
    for (int j = 0; j <= i; j++) { 
        if (!match[j]) {
            // x and y of the next space
            int x = path[j].x;
            int y = path[j].y-1;

            if (y < 0) {
                // Pass
            }
            else {
                // Check to see if the current space on the path matches the next space in the real maze
                for (int k = 0; k < 4; k++) {
                    if (path[j].connection[k] == full_maze.maze[x][y].connection[k]) {
                        certainty[North]++;
                    }
                } 
            }
        }
    }


    // Check to see if it moved east an extra space
    for (int j = 0; j <= i; j++) { 
        if (!match[j]) {
            // x and y of the next space
            int x = path[j].x+1;
            int y = path[j].y;

            if (x > 9) {
                // Pass
            }
            else {
                // Check to see if the current space on the path matches the next space in the real maze
                for (int k = 0; k < 4; k++) {
                    if (path[j].connection[k] == full_maze.maze[x][y].connection[k]) {
                        certainty[East]++;
                    }
                } 
            }
        }
    }


    // Check to see if it moved south an extra space
    for (int j = 0; j <= i; j++) { 
        if (!match[j]) {
            // x and y of the next space
            int x = path[j].x;
            int y = path[j].y+1;

            if (y > 9) {
                // Pass
            }
            else {
                // Check to see if the current space on the path matches the next space in the real maze
                for (int k = 0; k < 4; k++) {
                    if (path[j].connection[k] == full_maze.maze[x][y].connection[k]) {
                        certainty[South]++;
                    }
                } 
            }
        }
    }


    // Check to see if it moved west an extra space
    for (int j = 0; j <= i; j++) { 
        if (!match[j]) {
            // x and y of the next space
            int x = path[j].x-1;
            int y = path[j].y;

            if (x < 0) {
                // Pass
            }
            else {
                // Check to see if the current space on the path matches the next space in the real maze
                for (int k = 0; k < 4; k++) {
                    if (path[j].connection[k] == full_maze.maze[x][y].connection[k]) {
                        certainty[West]++;
                    }
                } 
            }
        }
    }

    // Find which certainty is the greatest
    int bestMatch = North;
    if (certainty[East] > certainty[bestMatch]) { 
        bestMatch = East;
    }

    if (certainty[West] > certainty[bestMatch]) {
        bestMatch = West;
    }

    if (certainty[South] > certainty[bestMatch]) {
        bestMatch = South;
    }


    // If we have a low confidence, then don't bother
    if (certainty[bestMatch] > misses) {
        switch (bestMatch) { 
            case North:
                full_maze.currentNode = &full_maze.maze[full_maze.currentNode->x][full_maze.currentNode->y+1];
                break;
            case East:
                full_maze.currentNode = &full_maze.maze[full_maze.currentNode->x-1][full_maze.currentNode->y];
                break;
            case South:
                full_maze.currentNode = &full_maze.maze[full_maze.currentNode->x][full_maze.currentNode->y-1];
                break;
            case West:
                full_maze.currentNode = &full_maze.maze[full_maze.currentNode->x+1][full_maze.currentNode->y];
                break;
        }

        // Copy the maze but preserve the heading
        int heading = full_maze.heading;
        CopyMaze(&backup_maze, &full_maze);
        full_maze.heading = heading;
        return true;
    }
    else {
        for (int x = 0; x < 10; x++) {
            for (int y = 0; y < 10; y++) {
                if (!backup_maze.maze[x][y].explored) {
                    for (int a = 0; a < 4; a++) {
                        update_connection(backup_maze.maze, &backup_maze.maze[x][y], a, true);
                    }
                }
            }
        }
        return false;
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
int Scan() {
    
    // Wall length in cm
    float WALL_LENGTH = 25.4;
    // Furthest distance at which we will consider there to be a wall next to us
    float MAX_DISTANCE = 15;     // cm


    // Placeholders for the ultrasonic readings
    // float frontDistance = MAX_DISTANCE;
    float frontDistance;
    float leftDistance;
    float rightDistance;

    printf("Scanning\n");

    // Stop the motors
    Stop();
    
    // Wait for a bit
    vTaskDelay(100);
    
    // We need to be VERY CERTAIN that the ultrasonic readings are recent, so clear them now
    xQueueReceive(UsQueue1, &leftDistance, 0);
    xQueueReceive(UsQueue2, &rightDistance, 0);
    xQueueReceive(UsQueue3, &frontDistance, 0);    

    int walls[4] = {0, 0, 0, 0};


    // Left
    if (xQueueReceive(UsQueue1, &leftDistance, 100) == pdPASS) {
        printf("Left distance: %f\n", leftDistance);
        if (leftDistance < MAX_DISTANCE) {
            walls[(full_maze.heading + 3) % 4] = -1;
            // update_connection(full_maze.maze, full_maze.currentNode, (full_maze.heading + 3) % 4, false);
        }
        else {
            walls[(full_maze.heading + 3) % 4] = 1;
            // update_connection(full_maze.maze, full_maze.currentNode, (full_maze.heading + 3) % 4, true);
        }
    }

    // Right
    if (xQueueReceive(UsQueue2, &rightDistance, 100) == pdPASS) {
        printf("Right distance: %f\n", rightDistance);
        if (rightDistance < MAX_DISTANCE) {
            walls[(full_maze.heading + 1) % 4] = -1;
            // update_connection(full_maze.maze, full_maze.currentNode, (full_maze.heading + 1) % 4, false);
        }
        else {
            walls[(full_maze.heading + 1) % 4] = 1;
            // update_connection(full_maze.maze, full_maze.currentNode, (full_maze.heading + 1) % 4, true);
        }
    }

    // Front
    if (xQueueReceive(UsQueue3, &frontDistance, 100) == pdPASS) {
        printf("Front distance: %f\n", frontDistance);
        if (frontDistance < MAX_DISTANCE) {
            walls[full_maze.heading] = -1;
            // update_connection(full_maze.maze, full_maze.currentNode, full_maze.heading, false);
        }
        else {
            walls[full_maze.heading] = 1;
            // update_connection(full_maze.maze, full_maze.currentNode, full_maze.heading, true);
        }
    }

    // Check to see if what we see lines up with what we've previously seen
    int discrepancy = 0;

    // If we don't see a wall north of us and we think we're at the top, we clearly aren't at the top
    // We can say similarly for the other sides of the maze
    if ((walls[North] == 1) && (full_maze.currentNode->y == 0)) {
        full_maze.currentNode = &full_maze.maze[full_maze.currentNode->x][full_maze.currentNode->y+1];
        discrepancy += 1;
    }
    if ((walls[East] == 1) && (full_maze.currentNode->x == 9)) {
        full_maze.currentNode = &full_maze.maze[full_maze.currentNode->x-1][full_maze.currentNode->y];
        discrepancy += 1;
    }
    if ((walls[South] == 1) && (full_maze.currentNode->y == 9)) {
        full_maze.currentNode = &full_maze.maze[full_maze.currentNode->x][full_maze.currentNode->y-1];
        discrepancy += 1;
    }
    if ((walls[West] == 1) && (full_maze.currentNode->x == 0)) {
        full_maze.currentNode = &full_maze.maze[full_maze.currentNode->x+1][full_maze.currentNode->y];
        discrepancy += 1;
    }


    for (int i = 0; i < 4; i++) {
        if (walls[i] == 1) {
            update_connection(full_maze.maze, full_maze.currentNode, i, true);
        }
        else if (walls[i] == -1) { 
            update_connection(full_maze.maze, full_maze.currentNode, i, false);
        }

        // If we've been here before and it's changed, make a note of it
        if (full_maze.currentNode->explored && (full_maze.currentNode->connection[i] != 
        backup_maze.maze[full_maze.currentNode->x][full_maze.currentNode->y].connection[i])) {
            discrepancy++;
        }
    }


    printMaze();
    return discrepancy;
}


// Scanning task
void ScanTask(void * pvParameters)
{
  for( ;; )
  {
    if (xSemaphoreTake(scan_semaphore, ( TickType_t ) 10) == pdTRUE ) {
        if (xSemaphoreTake(maze_mutex, ( TickType_t ) portMAX_DELAY) == pdTRUE) {
            Scan();
            xSemaphoreGive(maze_mutex);
            xSemaphoreGive(pathfind_semaphore);
        }
    }
    // vTaskDelay(100);
  }
}

// static long get_nanos(void) {
//     struct timespec ts;
//     timespec_get(&ts, TIME_UTC);
//     return (long)ts.tv_sec * 1000000000L + ts.tv_nsec;
// }