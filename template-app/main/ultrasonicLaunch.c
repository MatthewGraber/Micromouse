#include "ultrasonicLaunch.h"
#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "ultrasonic.h"
#include <esp_err.h>
//#include <time.h>
#include "esp_timer.h"

#define MAX_DISTANCE_CM 500 // 5m max

#define TRIGGER_GPIO 19
#define ECHO_GPIO 18

#define MAX_SIZE 1

extern QueueHandle_t UsQueue;






void ultrasonic_test(void *pvParameters)
{   
    float distance2 = 0;
    float distance3 = 0;
    float avDis = 0;
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };
    
    ultrasonic_init(&sensor);

    while (true)
    {   
        //int start = esp_timer_get_time();
        //clock_t begin = clock();
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else 
        {
            // printf("avDis = %0.04f cm\n", avDis*100);
            avDis = (distance+distance2+distance3)/3.0;
            if ((avDis - distance)*100 > 2 || (avDis - distance)*100 <-2)
            {
                printf("Average Sink Error\n");
                //printf("Distance: %0.04f cm\n",distance*100);
                //printf("Distance2: %0.04f cm\n",distance2*100);
                //printf("Distance3: %0.04f cm\n",distance3*100);
                //printf("Ave_Distance: %0.04f cm\n",avDis*100);
            }
            else
            {
                if ( (distance2 - distance)*100 > 5 || (distance2 - distance)*100 <-5)
                {
                    printf("Data Error\n");
                }
                else
                {
                    // printf("avDis = %0.04f cm\n", avDis*100);
                    printf("Distance: %0.04f cm\n", distance*100);
                    float distanceSend = distance*100;
                    xQueueOverwrite(UsQueue,&distanceSend);
                    //xQueueOverwriteFromISR(UsQueue, &distance, pdTRUE);
                }
            }
            distance3 = distance2;
            distance2 = distance;
        }
        //int stop = esp_timer_get_time();
        //int time_spent = (stop - start);
        //printf("Response Time: %d us\n", time_spent);

        //clock_t end = clock();
        //double time_spent =  (double)((end-begin)/_CLOCKS_PER_SEC_)*1000.0;
        //printf("Response Time: %f us\n",time_spent);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}