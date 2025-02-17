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
#define LIMIT_SWITCH_PIN_1 15
#define LIMIT_SWITCH_PIN_2 16 

#define STEPS_PER_REVOLUTION 4096
#define START_FREQ 100   // Hz - Base frequency for starting/direction changes
#define MAX_FREQ 600    // Hz - Maximum frequency under load (increased from 600)
#define ACCEL_RATE 600 // Hz/s - How fast to increase frequency (much faster acceleration)

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
    uint8_t currentStep;
    int64_t lastStepTime;
    float current_freq;   // Current operating frequency
    float target_freq;    // Target frequency to reach
    int64_t last_freq_update; // Time of last frequency update
    bool ramping_up;      // Whether we're currently accelerating
    bool direction_changed; // Flag for direction changes
} stepper_state_t;

static stepper_state_t stepper = {
    .currentPos = 0,
    .targetPos = INT32_MAX,
    .currentStep = 0,
    .lastStepTime = 0,
    .current_freq = START_FREQ,
    .target_freq = MAX_FREQ,
    .last_freq_update = 0,
    .ramping_up = true,
    .direction_changed = false
};

static void update_frequency(int64_t current_time) {
    if (!stepper.ramping_up && !stepper.direction_changed) {
        return;
    }

    float elapsed_sec = (current_time - stepper.last_freq_update) / 1000000.0f;
    if (elapsed_sec < 0.001f) { // Update every 1ms
        return;
    }

    if (stepper.direction_changed) {
        stepper.current_freq = START_FREQ;
        stepper.direction_changed = false;
        stepper.ramping_up = true;
    } else if (stepper.ramping_up) {
        stepper.current_freq += ACCEL_RATE * elapsed_sec;
        if (stepper.current_freq >= stepper.target_freq) {
            stepper.current_freq = stepper.target_freq;
            stepper.ramping_up = false;
        }
    }

    stepper.last_freq_update = current_time;
}

static void set_output_pins(uint8_t step_index) {
    gpio_set_level(IN1, step_sequence[step_index][0]);
    gpio_set_level(IN2, step_sequence[step_index][1]);
    gpio_set_level(IN3, step_sequence[step_index][2]);
    gpio_set_level(IN4, step_sequence[step_index][3]);
}

static void init_gpio(void) {
    gpio_config_t motor_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | (1ULL << IN3) | (1ULL << IN4),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&motor_conf);

    gpio_config_t switch_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LIMIT_SWITCH_PIN_1) | (1ULL << LIMIT_SWITCH_PIN_2),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&switch_conf);
}

static void stepper_task(void *pvParameters) {
    int64_t current_time;
    int32_t distance_to_go;
    int last_logged_pos = 0;
    int steps_since_yield = 0;
    int last_switch1_state = 1;
    int last_switch2_state = 1;
    int rotation_direction = 1;
    int64_t last_trigger_time = 0;
    const int64_t DEBOUNCE_TIME_US = 200000;  // 200ms debounce

    stepper.targetPos = INT32_MAX * rotation_direction;
    stepper.last_freq_update = esp_timer_get_time();

    while (1) {
        current_time = esp_timer_get_time();
        distance_to_go = stepper.targetPos - stepper.currentPos;

        // Update frequency based on ramping
        update_frequency(current_time);
        
        // Calculate step timing based on current frequency
        uint32_t micros_per_step = (uint32_t)(1000000.0f / stepper.current_freq);

        if (abs(distance_to_go) < STEPS_PER_REVOLUTION) {
            stepper.targetPos = rotation_direction * INT32_MAX;
        }

        // Check both switches for falling edge with debounce
        int current_switch1_state = gpio_get_level(LIMIT_SWITCH_PIN_1);
        int current_switch2_state = gpio_get_level(LIMIT_SWITCH_PIN_2);
        
        if (((current_switch1_state == 0 && last_switch1_state == 1) ||
             (current_switch2_state == 0 && last_switch2_state == 1)) &&
            ((current_time - last_trigger_time) > DEBOUNCE_TIME_US)) {
            
            ESP_LOGI(TAG, "Limit switch triggered - reversing direction");
            stepper.currentPos = 0;
            rotation_direction = -rotation_direction;
            stepper.targetPos = rotation_direction * INT32_MAX;
            stepper.direction_changed = true;  // Signal for frequency reset
            last_trigger_time = current_time;
            
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        last_switch1_state = current_switch1_state;
        last_switch2_state = current_switch2_state;

        // Log position periodically
        if (abs(stepper.currentPos - last_logged_pos) >= 256) {
            ESP_LOGD(TAG, "Position: %d, Frequency: %.1f Hz", 
                     stepper.currentPos, stepper.current_freq);
            last_logged_pos = stepper.currentPos;
        }

        // Step the motor if enough time has passed
        if ((current_time - stepper.lastStepTime) >= micros_per_step) {
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
            
            // Yield periodically
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
    
    // Create stepper motor task
    xTaskCreatePinnedToCore(
        stepper_task,
        "stepper_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        1
    );
    
    ESP_LOGI(TAG, "Stepper motor control started");
}