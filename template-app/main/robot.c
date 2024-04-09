#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_timer.h"

#include "robot.h"
#include "maze.h"
#include "motor_control.h"
#include "encoder.h"
#include "ultrasonicLaunch.h"
#include "icm20948.h"


extern SemaphoreHandle_t maze_mutex;
extern struct Maze full_maze;

extern SemaphoreHandle_t scan_semaphore;
extern SemaphoreHandle_t pathfind_semaphore;


extern QueueHandle_t UsQueue1;
extern QueueHandle_t UsQueue2;
extern QueueHandle_t UsQueue3;

static float left_encoder = 0;
static float right_encoder = 0;
static float gyroHeading = 0;

void Move() { 

    int targetHeading = North;

    // 0 = scan, 1 = pathfind, 2 = move
    static int state = 0;
    static bool goingToCenter = true;
    static int counter = 0;

    // printf("Attempting to move\n");

    // Scan
    if (state == 0) {
        Scan();
        state = 1;
    }
    // Pathfind
    else if (state == 1) {
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
        if (full_maze.nextNode == full_maze.currentNode) {
            state = 0;
            printf("Couldn't find next node!\n");
            vTaskDelay(10);

            // If we can't find a path, clear the maze
            counter++;
            if (counter >= 10) {
                initalizeMaze(full_maze.maze);
            }
        }
        else {
            state = 2;
            printf("Current node: x = %d, y = %d\n", full_maze.currentNode->x, full_maze.currentNode->y);
            printf("Next node: x = %d, y = %d\n", full_maze.nextNode->x, full_maze.nextNode->y);

        }
    }
    // Move
    else {
        // Ensure that the direction of the next node is valid
        int xDif = full_maze.currentNode->x - full_maze.nextNode->x;    // -1 == East, 1 == West
        int yDif = full_maze.currentNode->y - full_maze.nextNode->y;    // -1 == South, 1 == North
        if ((xDif == 0) == (yDif == 0)) {
            state = 0;
            printf("Invalid next node\n");
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
                TurnRight();
            }
            else if ((targetHeading-full_maze.heading + 4) % 4 == 3) {
                TurnLeft();
            }
            else if ((targetHeading-full_maze.heading + 4) % 4 == 2) {
                GoBack();
                StopBack();
                state = 0;
            }
            else {
                GoStraight();
                Stop();
                state = 0;
            }
            vTaskDelay(100);
            // xSemaphoreGive(maze_mutex);
            // vTaskDelay(100);
        }
    }
}


void TurnRight() {
    const float PI = 3.14159265358979323846;
    const float TURN_RADIUS = 30;  // mm
    // const float TARGET_DIST = PI*TURN_RADIUS/2;  // Quarter-turn in mm
    const float TARGET_DIST = 30;
    // const float GAIN = 0.01;
    const float BASE_POWER = 70;
    const float LOW_POWER = 10;

    float LEFT_START;

    float left_ultra = 0, right_ultra = 100;
    const float ULTRA_STOP = 5;     // Distance at which we will stop turning via ultrasonic sensor

    // Gyro sensor
    float targetHeading = -85;

    // Spin motors
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);                    // motor 1 (left)
    Stop();
    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER);     // motor 0 (right)
    xQueueReceive(distanceQueueLeft, &left_encoder, 0);
    LEFT_START = left_encoder;

    xQueueReceive(heading_queue, &gyroHeading, 0);
    targetHeading += gyroHeading;

    printf("Turning right\n");
    

    int start = esp_timer_get_time();
    int minTime = 0.4*1000000;  // uSecs
    int maxTime = 3*1000000;  // uSecs
    int currentTime = start;
    bool finished = false;

    // Exit condition
    while (((currentTime - start) < minTime || !finished) && ((currentTime - start) < maxTime)) {
        // Get the gyro velocity
        if (xQueueReceive(heading_queue, &gyroHeading, 0) == pdTRUE) {
            printf("Gyro Heading: %f\n", gyroHeading);
        }
        if (gyroHeading <= targetHeading) {
            finished = true;
        }

        // Get the ultrasonic reading
        xQueueReceive(UsQueue1, &left_ultra, 0);
        if(xQueueReceive(UsQueue2, &right_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
            if ((right_ultra <= ULTRA_STOP) && ((currentTime - start) < minTime)) {
                finished = true;
            }
        }

        if (xQueueReceive(distanceQueueLeft, &left_encoder, 0) == pdTRUE) {
            // printf("Distance turned right: %f\n", left_encoder - LEFT_START);
        }
        // Creates a gain based on the percentage of the distance covered
        float gain = ((TARGET_DIST - (left_encoder - LEFT_START)) / TARGET_DIST) * (100-BASE_POWER);
        // float gain = 0;
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER); // motor 0 (left)
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, LOW_POWER); // motor 1 (right)
        currentTime = esp_timer_get_time();
        vTaskDelay(1);
    }

    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER);
    // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, LOW_POWER); // motor 0 (left)
    // vTaskDelay(50);
    full_maze.heading = (full_maze.heading + 1) % 4;
    // printMaze();
    Stop();
    vTaskDelay(100);
}


void TurnLeft() {
    const float PI = 3.14159265358979323846;
    const float TURN_RADIUS = 30;  // mm
    // const float TARGET_DIST = PI*TURN_RADIUS/2;  // Quarter-turn in mm
    const float TARGET_DIST = 30;
    // const float GAIN = 0.01;
    const float BASE_POWER = 70;
    const float LOW_POWER = 10;

    float RIGHT_START;

    float left_ultra = 100, right_ultra = 0;
    const float ULTRA_STOP = 5;     // Distance at which we will stop turning via ultrasonic sensor

    float targetHeading = 85;

    // Spin motors
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);                    // motor 1 (left)
    Stop();
    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER);     // motor 0 (right)
    xQueueReceive(distanceQueueRight, &right_encoder, 0);
    RIGHT_START = right_encoder;

    printf("Turning left\n");
    
    // Get the current heading
    xQueueReceive(heading_queue, &gyroHeading, 0);
    targetHeading += gyroHeading;

    int start = esp_timer_get_time();
    int minTime = 0.4*1000000;  // uSecs
    int maxTime = 3*1000000;  // uSecs
    int currentTime = start;
    bool finished = false;

    // Exit condition
    while (((currentTime - start) < minTime || !finished) && ((currentTime - start) < maxTime)) {
        // Get the gyro heading
        if (xQueueReceive(heading_queue, &gyroHeading, 0) == pdTRUE) {
            printf("Gyro Heading: %f\n", gyroHeading);
        }
        if (gyroHeading >= targetHeading) {
            finished = true;
        }
        
        xQueueReceive(UsQueue2, &right_ultra, 0);
        if(xQueueReceive(UsQueue1, &left_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
            if ((left_ultra <= ULTRA_STOP) && ((currentTime - start) < minTime)) {
                finished = true;
            }
        }

        if (xQueueReceive(distanceQueueRight, &right_encoder, 0) == pdTRUE) {
            // printf("Distance turned left: %f\n", right_encoder - RIGHT_START);
        }
        // Creates a gain based on the percentage of the distance covered
        float gain = ((TARGET_DIST - (right_encoder - RIGHT_START)) / TARGET_DIST) * (100-BASE_POWER);
        // float gain = 0;
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER); // motor 1 (right)
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, LOW_POWER); // motor 0 (left)
        currentTime = esp_timer_get_time();
        vTaskDelay(1);
    }

    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER);
    // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, LOW_POWER); // motor 0 (left)
    // vTaskDelay(50);
    full_maze.heading = (full_maze.heading + 3) % 4;
    // printMaze();
    Stop();
    vTaskDelay(100);
}


void GoStraight() {
    const float TARGET_DIST = 10*25.4;  // mm
    const float GAIN = 0.5;
    const float BASE_POWER = 60;
    const float WALL_TARGET_DIST = 5;       // Ideal distance from the wall
    const float WALL_VERY_FAR_DIST = 20;    // But not further than this
    const float ULTRA_GAIN = 5;
    const float WALL_TOO_CLOSE = 7.5;    // Always stop if the front wall is this close

    const float GAP_STOP_DIST = 10;  // When we see or stop seeing a gap next to us, stop after moving this much further
    
    float targetHeading = 0;
    float frontUltraTarget = WALL_TOO_CLOSE;

    float LEFT_START, RIGHT_START;
    // Encoder readings
    // Ultrasonic readings (cm)
    float front_ultra = 9999, left_ultra = WALL_TARGET_DIST, right_ultra = WALL_TARGET_DIST;
    float left_boost = 0, right_boost = 0;  // Used when the ultrasonic sensors say we're close to a wall
    
    bool wallOnLeft = !(full_maze.currentNode->connection[(full_maze.heading + 3) % 4]);
    bool wallOnRight = !(full_maze.currentNode->connection[(full_maze.heading + 1) % 4]);

    printf("Moving straight\n");

    // Get the distance that the next wall is from us
    xQueueReceive(UsQueue3, &front_ultra, portMAX_DELAY);
    // front_ultra += 10;   // Add a little to the front reading just in case it sees something 23 cm away or something
    // frontUltraTarget += ((int) (front_ultra/25.4) - 1)*25.4;   // Adjust the target distance based on how far the closest wall is
    
    // If we're close to a wall, we can be confident in the exact distance we need
    if (false /*front_ultra < 25*2 + WALL_TOO_CLOSE*/) {
        frontUltraTarget = ((int)((front_ultra - WALL_TOO_CLOSE)/25))*25 + WALL_TOO_CLOSE;
    }
    // If we aren't close, just get something reasonable
    else {
        frontUltraTarget = front_ultra - 25;    // Subtract 22 instead of 25.4 to give it a little time to stop
    }

    if (frontUltraTarget < WALL_TOO_CLOSE) {
        frontUltraTarget = WALL_TOO_CLOSE;
    }
    printf("Front target distance: %f\n", frontUltraTarget);
    
    // Get starting encoder values
    xQueueReceive(distanceQueueLeft, &left_encoder, 0);
    xQueueReceive(distanceQueueRight, &right_encoder, 0);
    LEFT_START = left_encoder;
    RIGHT_START = right_encoder;

    // Get gyro heading
    xQueueReceive(heading_queue, &gyroHeading, 0);
    targetHeading += gyroHeading;

    int start = esp_timer_get_time();
    int minTime = 0.8*1000000;  // uSecs
    int currentTime = start;
    bool finished = false;

    // Exit condition
    // Drive until both distances are greater than the target distance
    while ((currentTime - start) < minTime || !finished) {

        // If we're close to the left-facing wall, turn more
        if(xQueueReceive(UsQueue1, &left_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
        }

        // If we're close to the forward-facing wall, stop moving forwards
        if(xQueueReceive(UsQueue2, &right_ultra, 0) == pdTRUE) {
            // printf("Right distance %f\n", right_ultra);
        }
        
        // If we're close to the forward-facing wall, stop moving forwards
        if(xQueueReceive(UsQueue3, &front_ultra, 0) == pdTRUE) {
            printf("Front distance %f\n", front_ultra);
        }
        // If we're close enough to the wall, stop
        if (frontUltraTarget <= 15) {
            frontUltraTarget = WALL_TOO_CLOSE;
        }
        if (front_ultra <= frontUltraTarget) {
            finished = true;
        }
        if (front_ultra <= WALL_TOO_CLOSE) {
            break;
        }

        // Both sensors see a nearby wall
        if (left_ultra <= WALL_VERY_FAR_DIST && right_ultra <= WALL_VERY_FAR_DIST) {
            left_boost = (WALL_TARGET_DIST - left_ultra)*ULTRA_GAIN;
            right_boost = (WALL_TARGET_DIST - right_ultra)*ULTRA_GAIN;

            // If shouldn't see a wall but do, then only go a little further
            if (wallOnLeft && wallOnRight) { 
            }
            else if (frontUltraTarget < front_ultra - GAP_STOP_DIST) {
                frontUltraTarget = front_ultra - GAP_STOP_DIST;
            }

        }
        // Only the left ultra sees a nearby wall
        else if (left_ultra <= WALL_VERY_FAR_DIST) {
            left_boost = (WALL_TARGET_DIST - left_ultra)*ULTRA_GAIN;
            right_boost = -left_boost;

            // If we shouldn't see a wall but do, then only go a little further
            if (wallOnLeft && !wallOnRight) { 
            }
            else if (frontUltraTarget < front_ultra - GAP_STOP_DIST) {
                frontUltraTarget = front_ultra - GAP_STOP_DIST;
            }
        }
        // Only the right ultra sees a nearby wall
        else if (right_ultra <= WALL_VERY_FAR_DIST) {
            right_boost = (WALL_TARGET_DIST - right_ultra)*ULTRA_GAIN;
            left_boost = -right_boost;

            // If we shouldn't see a wall but do, then only go a little further
            if (!wallOnLeft && wallOnRight) { 
            }
            else if (frontUltraTarget < front_ultra - GAP_STOP_DIST) {
                frontUltraTarget = front_ultra - GAP_STOP_DIST;
            }
        }
        // Neither sensor sees a nearby wall
        else {
            left_boost = 0;
            right_boost = 0;

            // If we should see a wall but don't, then only go a little further
            if (!wallOnLeft && !wallOnRight) { 
            }
            else if (frontUltraTarget < front_ultra - GAP_STOP_DIST) {
                frontUltraTarget = front_ultra - GAP_STOP_DIST;
            }
        }


        // Update the encoder readings
        xQueueReceive(distanceQueueLeft, &left_encoder, 0);
        xQueueReceive(distanceQueueRight, &right_encoder, 0);

        // Get the gyro heading
        xQueueReceive(heading_queue, &gyroHeading, 0);
        float diff = gyroHeading - targetHeading;   // Positive if too far left, negative if too far right
        // printf("Diff: %f\n", diff);

        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER + left_boost + diff*GAIN); // motor 0 (left)
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER + right_boost - diff*GAIN); // motor 1 (right)
        currentTime = esp_timer_get_time();
        vTaskDelay(1);
    }

    // Stop();
    // vTaskDelay(100);
    full_maze.currentNode = full_maze.nextNode;
    // printMaze();
}


void GoBack() {
    const float TARGET_DIST = 10*25.4;  // mm
    const float GAIN = 1;
    const float BASE_POWER = 60;
    const float WALL_TARGET_DIST = 6;       // Ideal distance from the wall
    const float WALL_VERY_FAR_DIST = 20;    // But not further than this
    const float ULTRA_GAIN = 5;
    const float WALL_TOO_CLOSE = 25.4;    // Don't stop if the front wall is this close

    const float GAP_STOP_DIST = 10;  // When we see or stop seeing a gap next to us, stop after moving this much further
    
    float targetHeading = 0;
    float frontUltraTarget = WALL_TOO_CLOSE;

    float LEFT_START, RIGHT_START;
    // Encoder readings
    // Ultrasonic readings (cm)
    float front_ultra = 9999, left_ultra = WALL_TARGET_DIST, right_ultra = WALL_TARGET_DIST;
    float left_boost = 0, right_boost = 0;  // Used when the ultrasonic sensors say we're close to a wall
    
    bool wallOnLeft = !(full_maze.currentNode->connection[(full_maze.heading + 3) % 4]);
    bool wallOnRight = !(full_maze.currentNode->connection[(full_maze.heading + 1) % 4]);

    printf("Moving straight\n");

    // Get the distance that the next wall is from us
    xQueueReceive(UsQueue3, &front_ultra, portMAX_DELAY);
    // front_ultra += 10;   // Add a little to the front reading just in case it sees something 23 cm away or something
    // frontUltraTarget += ((int) (front_ultra/25.4) - 1)*25.4;   // Adjust the target distance based on how far the closest wall is
    frontUltraTarget = front_ultra + 25;
    if (frontUltraTarget < WALL_TOO_CLOSE) {
        frontUltraTarget = WALL_TOO_CLOSE;
    }
    printf("Front target distance: %f\n", frontUltraTarget);
    
    // Get starting encoder values
    xQueueReceive(distanceQueueLeft, &left_encoder, 0);
    xQueueReceive(distanceQueueRight, &right_encoder, 0);
    LEFT_START = left_encoder;
    RIGHT_START = right_encoder;

    // Get gyro heading
    xQueueReceive(heading_queue, &gyroHeading, 0);
    targetHeading += gyroHeading;

    int start = esp_timer_get_time();
    int minTime = 0.8*1000000;  // uSecs
    int currentTime = start;
    bool finished = false;

    // Exit condition
    // Drive until both distances are greater than the target distance
    while ((currentTime - start) < minTime || !finished) {

        // If we're close to the left-facing wall, turn more
        if(xQueueReceive(UsQueue1, &left_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
        }

        // If we're close to the forward-facing wall, stop moving forwards
        if(xQueueReceive(UsQueue2, &right_ultra, 0) == pdTRUE) {
            // printf("Right distance %f\n", right_ultra);
        }
        
        // If we're close to the forward-facing wall, stop moving forwards
        if(xQueueReceive(UsQueue3, &front_ultra, 0) == pdTRUE) {
            printf("Front distance %f\n", front_ultra);
        }
        // If we're far enough from the wall, stop
        if (front_ultra >= frontUltraTarget && front_ultra >= WALL_TOO_CLOSE) {
            finished = true;
        }

        // Both sensors see a nearby wall
        if (left_ultra <= WALL_VERY_FAR_DIST && right_ultra <= WALL_VERY_FAR_DIST) {
            left_boost = (WALL_TARGET_DIST - left_ultra)*ULTRA_GAIN;
            right_boost = (WALL_TARGET_DIST - right_ultra)*ULTRA_GAIN;

            // If shouldn't see a wall but do, then only go a little further
            if (wallOnLeft && wallOnRight) { 
            }
            else if (frontUltraTarget > front_ultra + GAP_STOP_DIST) {
                frontUltraTarget = front_ultra + GAP_STOP_DIST;
            }

        }
        // Only the left ultra sees a nearby wall
        else if (left_ultra <= WALL_VERY_FAR_DIST) {
            left_boost = (WALL_TARGET_DIST - left_ultra)*ULTRA_GAIN;
            right_boost = -left_boost;

            // If we shouldn't see a wall but do, then only go a little further
            if (wallOnLeft && !wallOnRight) { 
            }
            else if (frontUltraTarget > front_ultra + GAP_STOP_DIST) {
                frontUltraTarget = front_ultra + GAP_STOP_DIST;
            }
        }
        // Only the right ultra sees a nearby wall
        else if (right_ultra <= WALL_VERY_FAR_DIST) {
            right_boost = (WALL_TARGET_DIST - right_ultra)*ULTRA_GAIN;
            left_boost = -right_boost;

            // If we shouldn't see a wall but do, then only go a little further
            if (!wallOnLeft && wallOnRight) { 
            }
            else if (frontUltraTarget > front_ultra + GAP_STOP_DIST) {
                frontUltraTarget = front_ultra + GAP_STOP_DIST;
            }
        }
        // Neither sensor sees a nearby wall
        else {
            left_boost = 0;
            right_boost = 0;

            // If we should see a wall but don't, then only go a little further
            if (!wallOnLeft && !wallOnRight) { 
            }
            else if (frontUltraTarget > front_ultra + GAP_STOP_DIST) {
                frontUltraTarget = front_ultra + GAP_STOP_DIST;
            }
        }


        // Update the encoder readings
        xQueueReceive(distanceQueueLeft, &left_encoder, 0);
        xQueueReceive(distanceQueueRight, &right_encoder, 0);

        // Get the gyro heading
        xQueueReceive(heading_queue, &gyroHeading, 0);
        float diff = gyroHeading - targetHeading;   // Positive if too far left, negative if too far right
        // printf("Diff: %f\n", diff);

        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER + left_boost - diff*GAIN); // motor 0 (left)
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER + right_boost + diff*GAIN); // motor 1 (right)
        currentTime = esp_timer_get_time();
        vTaskDelay(1);
    }

    // Stop();
    // vTaskDelay(100);
    full_maze.currentNode = full_maze.nextNode;
    // printMaze();
}

// Stops the motors
void Stop() {
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);

    // "Stop" the motors by setting them to move backwards at a low enough power that they won't actually move
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 10); // motor 0 (right)
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, 10); // motor 1 (left)

}

void StopBack() {
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 10);
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, 10);
}


void MoveTask(void * pvParameters) {

    initialize_motor_control();

    while (1) {
        Move();
    }
}


void Calibrate() {

    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30); // motor 0 (right)
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, 30); // motor 1 (left)

    // Update the encoder readings
    xQueueReceive(distanceQueueLeft, &left_encoder, portMAX_DELAY);
    xQueueReceive(distanceQueueRight, &right_encoder, portMAX_DELAY);

    Stop();
    vTaskDelay(300);
}


void TestTaskGoStraight(void * pvParameters) {
    initialize_motor_control();
    // Calibrate();

    while (1) {
        if (xSemaphoreTake(maze_mutex, (TickType_t) portMAX_DELAY)) {
            if (full_maze.nextNode != NULL) {
                // GoStraight();
                GoBack();
            }
            xSemaphoreGive(maze_mutex);

        }
    }
}

void DemoTaskTurnCorner(void * pvParameters) {
    initialize_motor_control();
    // Calibrate();
    float front_ultra;

    while (1) {
        if (xSemaphoreTake(maze_mutex, (TickType_t) portMAX_DELAY)) {
            if (full_maze.nextNode != NULL) {
                xQueueReceive(UsQueue3, &front_ultra, portMAX_DELAY);
                if (front_ultra > 10) {
                    GoStraight();
                    Stop();
                    vTaskDelay(50);
                }
                else {
                    TurnLeft();
                }
            }
            xSemaphoreGive(maze_mutex);
            // xSemaphoreGive(scan_semaphore);

        }
    }
}