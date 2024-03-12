#include "encoder.h"

#define ENCODER_1_CHANNEL_A_PIN GPIO_NUM_40
#define ENCODER_1_CHANNEL_B_PIN GPIO_NUM_39
#define ENCODER_2_CHANNEL_A_PIN GPIO_NUM_42
#define ENCODER_2_CHANNEL_B_PIN GPIO_NUM_41

volatile int32_t encoderCount1 = 0;
volatile int32_t encoderCount2 = 0;

QueueHandle_t encoderQueue1;
QueueHandle_t encoderQueue2;
QueueHandle_t distanceQueue1;
QueueHandle_t distanceQueue2;

void IRAM_ATTR encoderPulseISR1(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int8_t direction = 0;

    // Determine the direction based on the state of Channel B
    if (gpio_get_level(ENCODER_1_CHANNEL_B_PIN) == 1)
    {
        direction = (gpio_get_level(ENCODER_1_CHANNEL_A_PIN) == 1) ? -1 : 1;
    }
    else
    {
        direction = (gpio_get_level(ENCODER_1_CHANNEL_A_PIN) == 0) ? -1 : 1;
    }

    encoderCount1 += direction;

    // Send the updated count to the queue
    xQueueSendFromISR(encoderQueue1, &encoderCount1, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}
void IRAM_ATTR encoderPulseISR2(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int8_t direction = 0;

    // Determine the direction based on the state of Channel B
    if (gpio_get_level(ENCODER_2_CHANNEL_B_PIN) == 1)
    {
        direction = (gpio_get_level(ENCODER_2_CHANNEL_A_PIN) == 1) ? -1 : 1;
    }
    else
    {
        direction = (gpio_get_level(ENCODER_2_CHANNEL_A_PIN) == 0) ? -1 : 1;
    }

    encoderCount2 += direction;

    // Send the updated count to the queue
    xQueueSendFromISR(encoderQueue2, &encoderCount2, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void encoderTask(void *pvParameters)
{
    int32_t receivedCount1;
    int32_t receivedCount2;

    // variables from datasheet
    uint32_t counts_per_rev = 12;
    float gear_ratio = 29.86;
    uint32_t distance_per_rev = 32; // mm
    // float distance_per_rev = 1.25; //in

    float PI = 3.14159265358979323846;

    while (1)
    {
        if (xQueueReceive(encoderQueue1, &receivedCount1, 0))
        {
            float distance1 = ((float)distance_per_rev * PI / ((float)counts_per_rev * gear_ratio / 2) * (float)receivedCount1);
            // printf("Position: %ld   Distance: %f\n", receivedCount1, distance1);

            // Send the updated distance to the queue
            xQueueOverwrite(distanceQueue1, &distance1);
            // printf("Updated distance of encoder 1: %f\n", distance1);
        }
        if (xQueueReceive(encoderQueue2, &receivedCount2, 0))
        {
            float distance2 = ((float)distance_per_rev * PI / ((float)counts_per_rev * gear_ratio / 2) * (float)receivedCount2);
            // printf("Position: %ld   Distance: %f\n", receivedCount2, distance2);

            // Send the updated distance to the queue
            xQueueOverwrite(distanceQueue2, &distance2);
            // printf("Updated distance of encoder 2: %f\n", distance2);
        }
        vTaskDelay(1);
    }
}

void initializeEncoder()
{
    // confiqure input pins from encoder
    gpio_config_t io_conf_1 = {
        // encoder 1
        .pin_bit_mask = (1ULL << ENCODER_1_CHANNEL_A_PIN) | (1ULL << ENCODER_1_CHANNEL_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf_1);

    gpio_config_t io_conf_2 = {
        /// encoder 2
        .pin_bit_mask = (1ULL << ENCODER_2_CHANNEL_A_PIN) | (1ULL << ENCODER_2_CHANNEL_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf_2);

    // Install ISR service
    gpio_install_isr_service(0);

    // Hook ISR handler for encoder pulses
    gpio_isr_handler_add(ENCODER_1_CHANNEL_A_PIN, encoderPulseISR1, (void *)ENCODER_1_CHANNEL_A_PIN);
    gpio_isr_handler_add(ENCODER_2_CHANNEL_A_PIN, encoderPulseISR2, (void *)ENCODER_2_CHANNEL_A_PIN);

    encoderQueue1 = xQueueCreate(10, sizeof(int32_t));
    encoderQueue2 = xQueueCreate(10, sizeof(int32_t));

    distanceQueue1 = xQueueCreate(1, sizeof(float));
    distanceQueue2 = xQueueCreate(1, sizeof(float));
}