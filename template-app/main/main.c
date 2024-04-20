#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#include "maze.h"
#include "robot.h"
#include <ultrasonic.h>
#include "ultrasonicLaunch.h"
#include "encoder.h"
#include "icm20948.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "math.h"


#define STACK_SIZE 1028*8

// Semaphore that triggers when it's time to scan
SemaphoreHandle_t scan_semaphore;
SemaphoreHandle_t pathfind_semaphore;

// The maze
struct Maze full_maze;
SemaphoreHandle_t maze_mutex;

// Queues
QueueHandle_t UsQueue1;
QueueHandle_t UsQueue2;
QueueHandle_t UsQueue3;
QueueHandle_t heading_queue;

struct Node maze[10][10];

struct Node* currentNode;
struct Node* nextNode;
bool moving = false;


#define I2C_MASTER_SCL_IO  4        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  3        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM     I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000    /*!< I2C master clock frequency */

#define GPIO_BIT_MASK (1ULL << GPIO_NUM_0 || 1ULL << GPIO_NUM_1 || 1ULL << GPIO_NUM_15 || 1ULL << GPIO_NUM_16 || 1ULL << GPIO_NUM_17 || 1ULL << GPIO_NUM_18 || 1ULL << GPIO_NUM_5 || 1ULL << GPIO_NUM_6 || 1ULL << GPIO_NUM_7 || 1ULL << GPIO_NUM_8 || 1ULL << GPIO_NUM_9 || 1ULL << GPIO_NUM_10 || 1ULL << GPIO_NUM_11 || 1ULL << GPIO_NUM_12 || 1ULL << GPIO_NUM_13 || 1ULL << GPIO_NUM_14 || 1ULL << GPIO_NUM_38 || 1ULL << GPIO_NUM_39 || 1ULL << GPIO_NUM_40 || 1ULL << GPIO_NUM_41 || 1ULL << GPIO_NUM_42 || 1ULL << GPIO_NUM_45 || 1ULL << GPIO_NUM_47 || 1ULL << GPIO_NUM_48)
// #define GPIO_BIT_MASK GPIO_NUM_0

static const char *TAG = "icm test";
icm20948_handle_t icm20948 = NULL;

bool checkCollision = false;

bool collisionDetected = false;
QueueHandle_t collision_queue = NULL;
QueueHandle_t reset_queue = NULL;
SemaphoreHandle_t check_collision_semaphore = NULL;
bool bumpDetected = false;

/**
 * @brief i2c master initialization
 */
static esp_err_t
i2c_bus_init(void)
{
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

	esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
	if (ret != ESP_OK)
		return ret;

	return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t
icm20948_configure(icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs)
{
	esp_err_t ret;

	/*
	 * One might need to change ICM20948_I2C_ADDRESS to ICM20948_I2C_ADDRESS_1
	 * if address pin pulled low (to GND)
	 */
	
	icm20948 = icm20948_create(I2C_MASTER_NUM, ICM20948_I2C_ADDRESS);
	if (icm20948 == NULL) {
		ESP_LOGE(TAG, "ICM20948 create returned NULL!");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "ICM20948 creation successfull!");

	ret = icm20948_reset(icm20948);
	if (ret != ESP_OK)
		return ret;

	vTaskDelay(10 / portTICK_PERIOD_MS);

	ret = icm20948_wake_up(icm20948);
	if (ret != ESP_OK)
		return ret;

	ret = icm20948_set_bank(icm20948, 0);
	if (ret != ESP_OK)
		return ret;

	uint8_t device_id;
	ret = icm20948_get_deviceid(icm20948, &device_id);
	if (ret != ESP_OK)
		return ret;
	ESP_LOGI(TAG, "0x%02X", device_id);
	if (device_id != ICM20948_WHO_AM_I_VAL)
		return ESP_FAIL;

	ret = icm20948_set_gyro_fs(icm20948, gyro_fs);
	if (ret != ESP_OK)
		return ESP_FAIL;

	ret = icm20948_set_acce_fs(icm20948, acce_fs);
	if (ret != ESP_OK)
		return ESP_FAIL;

	ret = ak09916_init(icm20948);
	ret = ak09916_read_reg(icm20948, 0x11, 8);

	ret = icm20948_set_bank(icm20948, 0);
	return ret;
}


// Collects data from the IMU and sends it to a queue
void icm_read_task(void *args)
{
	esp_err_t ret = icm20948_configure(ACCE_FS_2G, GYRO_FS_1000DPS);
	for (int i = 0; ret != ESP_OK && i < 100; i++) {
		ESP_LOGE(TAG, "ICM configuration failure. Trying again");
        vTaskDelay(10);
        ret = icm20948_configure(ACCE_FS_2G, GYRO_FS_1000DPS);
		// vTaskDelete(NULL);
	}
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ICM configuration failure. We don't know what to do with it");
        vTaskDelete(NULL);
    }
	ESP_LOGI(TAG, "ICM20948 configuration successfull!");


    const float COLLISION_THRESHOLD = 0.28f;
	float headingVectMagn = 0;

	icm20948_acce_value_t acce;
	icm20948_gyro_value_t gyro;
	icm20948_comp_value_t comp;

    int past;
    int present = esp_timer_get_time();
    float heading = 0;
    float previous_gyro = 0;
    int counter = 0;

    while(1) {
		//Generate IMU data, uncomment as needed
		ret = icm20948_get_acce(icm20948, &acce);
		ret = icm20948_get_gyro(icm20948, &gyro);
        past = present;
        present = esp_timer_get_time();

		//checkCollision = gpio_get_level(GPIO_NUM_1);
			
        // Filter any noise
        if (gyro.gyro_z < 1 && gyro.gyro_z > -1) {
            gyro.gyro_z = 0;
        }

        float average_velocity = (gyro.gyro_z + previous_gyro);
        heading += average_velocity*(present-past)/1000000;

        // heading = acce.acce_z*90;

        xQueueOverwrite(heading_queue, &heading);
        if (counter == 5) {
            ESP_LOGI(TAG, "heading: %lf\t\tav_vel: %lf", heading, average_velocity);
            counter = 0;
        }
        // counter++;
        // ESP_LOGI(TAG, "ax: %lf ay: %lf", acce.acce_x, acce.acce_y);
        // ESP_LOGI(TAG, "gx: %lf gy: %lf", gyro.gyro_x, gyro.gyro_y);
        if (gyro.gyro_x > 200 || gyro.gyro_x < -200) {
            bool reset = true;
            xQueueSend(reset_queue, &reset, 0);
        }
        
        // If we've been given an instruction to check for collisions, start doing so
        if (xSemaphoreTake(check_collision_semaphore, 0) == pdTRUE) {
            checkCollision = true;
            collisionDetected = false;
        }

        // Do we need to notify that we want to check for collisions?
        checkCollision = true;

        //What it says. Take the x and y components of the accerometer and find the magnitude
		headingVectMagn = sqrt(acce.acce_x * acce.acce_x + acce.acce_y * acce.acce_y);
        if(headingVectMagn >= COLLISION_THRESHOLD && checkCollision)
        {
            // If we find a collision, raise the flag and stop checking
            ESP_LOGI(TAG, "ax: %lf ay: %lf Magn: %lf", acce.acce_x, acce.acce_y,headingVectMagn);
            collisionDetected = 1;
            checkCollision = false;
            if (collision_queue!= NULL) {
                xQueueSend(collision_queue, &collisionDetected, 0);
            }
        }

        // // Continue sending the collision detection flag until we're told to check for another collision detection
        // if (collisionDetected) {
        //     xSemaphoreGive(&collision_semaphore);
        // }
        /*
        if(headingVectMagn > 0.25 && checkCollision)
        {
            ESP_LOGI(TAG, "Bump Detected :|, ax: %lf ay: %lf Magn: %lf", acce.acce_x, acce.acce_y,headingVectMagn);
            bumpDetected = 1;
        }
        */
			
		vTaskDelay(1);
	}

	vTaskDelete(NULL);
}

void exampleRecieve() {
    float distance1 = 0;
    float distance2 = 0;

    while (1) {
        if (xQueueReceive(distanceQueueRight, &distance1, 0)) {
            printf("Distance1: %f, Distance2:%f\n", distance1, distance2);
        }
        if (xQueueReceive(distanceQueueLeft, &distance2, 0)) {
            printf("Distance1: %f, Distance2:%f\n", distance1, distance2);
        }
        vTaskDelay(1);
    }
}


void app_main(void) {

    // Delay start
    Stop();
    vTaskDelay(200);

    // Initalize semaphores
    scan_semaphore = xSemaphoreCreateBinary();
    pathfind_semaphore = xSemaphoreCreateBinary();
    check_collision_semaphore = xSemaphoreCreateBinary();
    maze_mutex = xSemaphoreCreateMutex();
    
    
    // Ultrasonic stuff
    //Sensor 1 (left)
    UsQueue1 = xQueueCreate( 1, sizeof( float));
    struct USParam us1;
    us1.ECHO_GPIO = 12;
    us1.TRIGGER_GPIO = 11;
    us1.Usqueue = UsQueue1;
    float distance1;
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 5,(void*) &us1, 5, NULL);
    
    //Sensor 2 (right)
    UsQueue2 = xQueueCreate( 1, sizeof( float));
    struct USParam us2;
    us2.ECHO_GPIO = 8;
    us2.TRIGGER_GPIO = 7;
    us2.Usqueue = UsQueue2;
    float distance2;
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 5,(void*) &us2, 5, NULL);

    //Sensor 3 (forward)
    UsQueue3 = xQueueCreate( 1, sizeof( float));
    struct USParam us3;
    us3.ECHO_GPIO = 10;
    us3.TRIGGER_GPIO = 9;
    us3.Usqueue = UsQueue3;
    float distance3;
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 5,(void*) &us3, 5, NULL);


    // IMU initialization stuff
    heading_queue = xQueueCreate(1, sizeof(float));
    collision_queue = xQueueCreate(1, sizeof(bool));
    reset_queue = xQueueCreate(1, sizeof(bool));
    gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_BIT_MASK;
	io_conf.pull_down_en = 1;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);



	//I2C Init
	ESP_LOGI(TAG, "Starting ICM test");
	esp_err_t ret = i2c_bus_init();
	ESP_LOGI(TAG, "I2C bus initialization: %s", esp_err_to_name(ret));

    xTaskCreate(icm_read_task, "icm read task", 1024 * 100, NULL, 10, NULL);


    // Maze initialization
    initalizeMaze(full_maze.maze);
    full_maze.currentNode = &full_maze.maze[0][0];
    full_maze.heading = South;
    full_maze.goingToCenter = true;

    // Initialize
    initializeEncoder();


    // maze1();
    // maze2();
    // maze3();
    // maze4();



    printMaze();

    // Pathfind(maze);
    // PrintDistanceToCenter(maze);

    static uint8_t uc_param_pathfind;
    static uint8_t uc_param_scan;
    static uint8_t uc_param_move;

    TaskHandle_t pathfind_task = NULL;
    TaskHandle_t scan_task = NULL;
    TaskHandle_t move_task = NULL;

    TaskHandle_t test_task = NULL;
    
    xSemaphoreGive(scan_semaphore);
    xSemaphoreGive(maze_mutex);

    // xTaskCreate(PathfindTask, "PATHFIND", STACK_SIZE, &uc_param_pathfind, 1, &pathfind_task );
    // configASSERT(pathfind_task);
    
    // xTaskCreate(ScanTask, "SCAN", STACK_SIZE, &uc_param_scan, 1, &scan_task );
    // configASSERT(scan_task);

    xTaskCreate(MoveTask, "MOVE", STACK_SIZE, &uc_param_move, tskIDLE_PRIORITY, &move_task );
    // configASSERT(move_task);


    // xTaskCreate(TestTaskGoStraight, "GOStraight", STACK_SIZE, NULL, 10, test_task);
    // xTaskCreate(DemoTaskTurnCorner, "DemoTaskTurnRight", STACK_SIZE, NULL, 10, test_task);
    // Create encoder task
    // xTaskCreate(encoderTask, "encoder_task", 4096, NULL, 5, NULL);
    // xTaskCreate(exampleRecieve, "example_Recieve", 4096, NULL, tskIDLE_PRIORITY, NULL);


}