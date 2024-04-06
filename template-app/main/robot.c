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
    int xDif, yDif;

    // printf("Attempting to move\n");

    xDif = full_maze.currentNode->x - full_maze.nextNode->x;    // -1 == East, 1 == West
    yDif = full_maze.currentNode->y - full_maze.nextNode->y;    // -1 == South, 1 == North

    // Ensure that the direction of the next node is valid
    if ((xDif == 0) == (yDif == 0)) {
        // If it is invalid, exit this task and call the Navigate task
        xSemaphoreGive(maze_mutex);
        xSemaphoreGive(pathfind_semaphore);
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
        while (targetHeading != full_maze.heading) {
            if ((targetHeading-full_maze.heading + 4) % 4 == 1) {
                TurnRight();
            }
            else if ((targetHeading-full_maze.heading + 4) % 4 == 3) {
                TurnLeft();
            }
            else {
                GoBack();
                xSemaphoreGive(maze_mutex);
                xSemaphoreGive(scan_semaphore);
                return;
            }
            vTaskDelay(100);
        }

        GoStraight();
        Stop();

        xSemaphoreGive(maze_mutex);
        xSemaphoreGive(scan_semaphore);
        // vTaskDelay(100);
    }
}


void TurnRight() {
    const float PI = 3.14159265358979323846;
    const float TURN_RADIUS = 30;  // mm
    // const float TARGET_DIST = PI*TURN_RADIUS/2;  // Quarter-turn in mm
    const float TARGET_DIST = 30;
    // const float GAIN = 0.01;
    const float BASE_POWER = 70;
    const float LOW_POWER = 40;

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
    

    // while ((left_encoder - LEFT_START < TARGET_DIST) || left_ultra < ULTRA_STOP) {
    while (1) {
        // Get the gyro velocity
        xQueueReceive(heading_queue, &gyroHeading, 0);
        if (gyroHeading <= targetHeading) {
            break;
        }

        // Get the ultrasonic reading
        xQueueReceive(UsQueue1, &left_ultra, 0);
        if(xQueueReceive(UsQueue2, &right_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
            if (right_ultra <= ULTRA_STOP) {
                break;
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
    const float LOW_POWER = 40;

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

    //while ((right_encoder - RIGHT_START < TARGET_DIST) || right_ultra < ULTRA_STOP) {
    while (1) {
        // Get the gyro heading
        xQueueReceive(heading_queue, &gyroHeading, 0);
        if (gyroHeading >= targetHeading) {
            break;
        }
        
        xQueueReceive(UsQueue2, &right_ultra, 0);
        if(xQueueReceive(UsQueue1, &left_ultra, 0) == pdTRUE) {
            // printf("Left distance %f\n", left_ultra);
            if (left_ultra <= ULTRA_STOP) {
                break;
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
    const float BASE_POWER = 50;
    const float WALL_TARGET_DIST = 7;       // Ideal distance from the wall
    const float WALL_VERY_FAR_DIST = 20;    // But not further than this
    const float ULTRA_GAIN = 5;
    
    float targetHeading = 0;
    float frontUltraTarget = 10;

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
    frontUltraTarget = front_ultra - 25.4;
    if (frontUltraTarget < 10) {
        frontUltraTarget = 10;
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

    // Exit condition
    // Drive until both distances are greater than the target distance
    while ((left_encoder - LEFT_START < TARGET_DIST) && (right_encoder - RIGHT_START < TARGET_DIST)) {

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

        // Both sensors see a nearby wall
        if (left_ultra <= WALL_VERY_FAR_DIST && right_ultra <= WALL_VERY_FAR_DIST) {
            left_boost = (WALL_TARGET_DIST - left_ultra)*ULTRA_GAIN;
            right_boost = (WALL_TARGET_DIST - right_ultra)*ULTRA_GAIN;

            // If shouldn't see a wall but do, then only go a little further
            if (wallOnLeft && wallOnRight) { 
            }
            else if (frontUltraTarget > front_ultra - 3) {
                frontUltraTarget = front_ultra - 3;
            }

        }
        // Only the left ultra sees a nearby wall
        else if (left_ultra <= WALL_VERY_FAR_DIST) {
            left_boost = (WALL_TARGET_DIST - left_ultra)*ULTRA_GAIN;
            right_boost = -left_boost;

            // If we shouldn't see a wall but do, then only go a little further
            if (wallOnLeft && !wallOnRight) { 
            }
            else if (frontUltraTarget > front_ultra - 3) {
                frontUltraTarget = front_ultra - 3;
            }
        }
        // Only the right ultra sees a nearby wall
        else if (right_ultra <= WALL_VERY_FAR_DIST) {
            right_boost = (WALL_TARGET_DIST - right_ultra)*ULTRA_GAIN;
            left_boost = -right_boost;

            // If we shouldn't see a wall but do, then only go a little further
            if (!wallOnLeft && wallOnRight) { 
            }
            else if (frontUltraTarget > front_ultra - 3) {
                frontUltraTarget = front_ultra - 3;
            }
        }
        // Neither sensor sees a nearby wall
        else {
            left_boost = 0;
            right_boost = 0;

            // If we should see a wall but don't, then only go a little further
            if (!wallOnLeft && !wallOnRight) { 
            }
            else if (frontUltraTarget > front_ultra - 3) {
                // frontUltraTarget = front_ultra - 3;
            }
        }

        // If we're close enough to the wall, stop
        if (front_ultra <= frontUltraTarget) {
            break;
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
        vTaskDelay(1);
    }

    // Stop();
    // vTaskDelay(100);
    full_maze.currentNode = full_maze.nextNode;
    // printMaze();
}


void GoBack() {
    const float TARGET_DIST = 10*25.4;  // mm
    const float GAIN = 0;
    const float BASE_POWER = 50;
    const float WALL_TARGET_DIST = 7;       // Ideal distance from the wall
    const float WALL_VERY_FAR_DIST = 20;    // But not further than this
    const float ULTRA_GAIN = 5;
    
    float targetHeading = 0;
    float frontUltraTarget = 30;

    float LEFT_START, RIGHT_START;
    // Encoder readings
    // Ultrasonic readings (cm)
    float front_ultra = 9999, left_ultra = WALL_TARGET_DIST, right_ultra = WALL_TARGET_DIST;
    float left_boost = 0, right_boost = 0;  // Used when the ultrasonic sensors say we're close to a wall
    

    printf("Moving backward\n");

    // Get the distance that the next wall is from us
    xQueueReceive(UsQueue3, &front_ultra, portMAX_DELAY);
    front_ultra -= 10;   // Add a little to the front reading just in case it sees something 23 cm away or something
    frontUltraTarget += ((int) (front_ultra/25.4))*25.4;   // Adjust the target distance based on how far the closest wall is
    if (frontUltraTarget < 30) {
        frontUltraTarget = 30;
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

    // Exit condition
    // Drive until both distances are greater than the target distance
    while ((left_encoder - LEFT_START < TARGET_DIST) && (right_encoder - RIGHT_START < TARGET_DIST)) {

        // If we're close to the left-facing wall, turn more
        if(xQueueReceive(UsQueue1, &left_ultra, 0) == pdTRUE) {
            // Calculate the new differential term
            // printf("Left distance %f\n", left_ultra);
        }

        // If we're close to the forward-facing wall, stop moving forwards
        if(xQueueReceive(UsQueue2, &right_ultra, 0) == pdTRUE) {
            // Calculate the new differential term
            // printf("Right distance %f\n", right_ultra);
        }
        

        // Both sensors see a nearby wall
        if (left_ultra <= WALL_VERY_FAR_DIST && right_ultra <= WALL_VERY_FAR_DIST) {
            left_boost = (WALL_TARGET_DIST - left_ultra)*ULTRA_GAIN;
            right_boost = (WALL_TARGET_DIST - right_ultra)*ULTRA_GAIN;
        }
        // Only the left ultra sees a nearby wall
        else if (left_ultra <= WALL_VERY_FAR_DIST) {
            left_boost = (WALL_TARGET_DIST - left_ultra)*ULTRA_GAIN;
            right_boost = -left_boost;
        }
        // Only the right ultra sees a nearby wall
        else if (right_ultra <= WALL_VERY_FAR_DIST) {
            right_boost = (WALL_TARGET_DIST - right_ultra)*ULTRA_GAIN;
            left_boost = -right_boost;
        }
        // Neither sensor sees a nearby wall
        else {
            left_boost = 0;
            right_boost = 0;
        }

        // If we're close to the forward-facing wall, stop moving forwards
        if(xQueueReceive(UsQueue3, &front_ultra, 0) == pdTRUE) {
            // printf("Front distance %f\n", front_ultra);
        }
        if (front_ultra >= frontUltraTarget) {
            break;
        }

        // Update the encoder readings
        xQueueReceive(distanceQueueLeft, &left_encoder, 0);
        xQueueReceive(distanceQueueRight, &right_encoder, 0);


        xQueueReceive(heading_queue, &gyroHeading, 0);
        float diff = gyroHeading - targetHeading;   // Positive if too far left, negative if too far right
        // printf("Diff: %f\n", diff);

        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, BASE_POWER + left_boost + diff*GAIN); // motor 0 (left)
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, BASE_POWER + right_boost - diff*GAIN); // motor 1 (right)
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
        if (xSemaphoreTake(maze_mutex, (TickType_t) portMAX_DELAY)) {
            if (full_maze.nextNode != NULL) {
                Move();
            }
            else {
                // xSemaphoreGive(scan_semaphore);
                xSemaphoreGive(maze_mutex);
                xSemaphoreGive(pathfind_semaphore);
            }

        }
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