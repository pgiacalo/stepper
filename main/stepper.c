#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
#define LIMIT_SWITCH_PIN 15

#define STEPS_PER_REVOLUTION 2048

static const char *TAG = "stepper";

static const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

typedef struct {
    int32_t currentPos;
    int32_t targetPos;
    float maxSpeed;        
    float acceleration;    
    float currentSpeed;
    uint8_t currentStep;
    int64_t lastStepTime;
} stepper_state_t;

static stepper_state_t stepper = {
    .currentPos = 0,
    .targetPos = STEPS_PER_REVOLUTION,
    .maxSpeed = 1000.0,
    .acceleration = 500.0,
    .currentSpeed = 0.0,
    .currentStep = 0,
    .lastStepTime = 0
};

static void set_output_pins(uint8_t step_index) {
    gpio_set_level(IN1, step_sequence[step_index][0]);
    gpio_set_level(IN2, step_sequence[step_index][1]);
    gpio_set_level(IN3, step_sequence[step_index][2]);
    gpio_set_level(IN4, step_sequence[step_index][3]);
}

static void init_gpio(void) {
    // Configure stepper motor pins
    gpio_config_t motor_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | (1ULL << IN3) | (1ULL << IN4),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&motor_conf);

    // Configure limit switch pin with pull-up
    gpio_config_t switch_conf = {
        .intr_type = GPIO_INTR_DISABLE,  // No interrupts
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LIMIT_SWITCH_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1  // Enable internal pull-up
    };
    gpio_config(&switch_conf);
}

static void stepper_task(void *pvParameters) {
   int64_t current_time;
   int32_t distance_to_go;
   int last_logged_pos = 0;
   int steps_since_yield = 0;
   int last_switch_state = 1;
   int rotation_direction = 1;
   int64_t last_trigger_time = 0;
   const int64_t DEBOUNCE_TIME_US = 200000;  // 200ms debounce

   // Set initial direction of continuous rotation
   stepper.targetPos = INT32_MAX * rotation_direction;

   while (1) {
       current_time = esp_timer_get_time();
       distance_to_go = stepper.targetPos - stepper.currentPos;

       // Check if we need to update target position to keep moving
       if (abs(distance_to_go) < STEPS_PER_REVOLUTION) {
           stepper.targetPos = rotation_direction * INT32_MAX;
       }

       // Check for falling edge on limit switch with debounce
       int current_switch_state = gpio_get_level(LIMIT_SWITCH_PIN);
       if (current_switch_state == 0 && last_switch_state == 1) {
           if ((current_time - last_trigger_time) > DEBOUNCE_TIME_US) {
               ESP_LOGI(TAG, "Limit switch triggered - setting zero position");
               stepper.currentPos = 0;
               rotation_direction = -rotation_direction;  // Reverse direction
               stepper.targetPos = rotation_direction * INT32_MAX;
               last_trigger_time = current_time;
               
               vTaskDelay(pdMS_TO_TICKS(50));
           }
       }
       last_switch_state = current_switch_state;

       // Log every 256 steps
       if (abs(stepper.currentPos - last_logged_pos) >= 256) {
           ESP_LOGD(TAG, "Position: %d", stepper.currentPos);
           last_logged_pos = stepper.currentPos;
       }

       // Check if 2000 microseconds (2ms) have passed
       if ((current_time - stepper.lastStepTime) >= 2000) {
           // Move one step
           if (distance_to_go > 0) {
               stepper.currentPos++;
               stepper.currentStep = (stepper.currentStep + 1) % 8;
           } else {
               stepper.currentPos--;
               stepper.currentStep = (stepper.currentStep + 7) % 8;
           }

           set_output_pins(stepper.currentStep);
           stepper.lastStepTime = current_time;
           
           // Yield every 32 steps
           steps_since_yield++;
           if (steps_since_yield >= 32) {
               vTaskDelay(1);
               steps_since_yield = 0;
           }
       }
   }
}

void app_main(void) {
    init_gpio();
    
    // set logging levels in the menuconfig under "Component config → Log output → Default log verbosity"
    #if CONFIG_LOG_DEFAULT_LEVEL > ESP_LOG_INFO
        esp_log_level_set(TAG, ESP_LOG_INFO);    // Only show INFO and above
    #endif

    // Create task pinned to core 1
    xTaskCreatePinnedToCore(
        stepper_task,
        "stepper_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,  // Highest priority
        NULL,
        1  // Run on core 1
    );
    
    ESP_LOGI(TAG, "Stepper motor control started");
}