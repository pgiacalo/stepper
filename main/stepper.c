/*
 * Stepper Motor Control System
 * 
 * This module implements a stepper motor controller for a bidirectional system with limit switches.
 * The motor accelerates smoothly from a start frequency to a maximum frequency and automatically
 * reverses direction when limit switches are triggered. The system uses an 8-step sequence for
 * smooth motor operation and implements acceleration ramping for smooth speed transitions.
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

// GPIO pin definitions for motor control and limit switches
#define IN1_MOTOR1 4
#define IN2_MOTOR1 5
#define IN3_MOTOR1 6
#define IN4_MOTOR1 7
#define IN1_MOTOR2 8   // New pins for second motor
#define IN2_MOTOR2 9
#define IN3_MOTOR2 10
#define IN4_MOTOR2 11
#define LIMIT_SWITCH_MOTOR1 15
#define LIMIT_SWITCH_MOTOR2 16

// Motor control parameters
#define STEPS_PER_REVOLUTION 4096
#define START_FREQ 100   // Hz - Base frequency for starting/direction changes
#define MAX_FREQ 600    // Hz - Maximum frequency under load
#define ACCEL_RATE 600 // Hz/s - How fast to increase frequency

static const char *TAG = "stepper";

/* 
 * 8-step sequence for stepper motor control
 * Each row represents one step in the sequence, controlling four coils (IN1-IN4)
 * This sequence provides smoother operation compared to simpler 4-step sequences
 */
static const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},  // Step 1
    {1, 1, 0, 0},  // Step 2
    {0, 1, 0, 0},  // Step 3
    {0, 1, 1, 0},  // Step 4
    {0, 0, 1, 0},  // Step 5
    {0, 0, 1, 1},  // Step 6
    {0, 0, 0, 1},  // Step 7
    {1, 0, 0, 1}   // Step 8
};

/*
 * Structure to maintain the stepper motor's state
 * Tracks position, movement parameters, and acceleration state
 */
typedef struct {
    int32_t currentPos;      // Current position in steps
    int32_t targetPos;       // Target position to move to
    uint8_t currentStep;     // Current step in the sequence (0-7)
    int64_t lastStepTime;    // Timestamp of last step
    float current_freq;      // Current operating frequency
    float target_freq;       // Target frequency to reach
    int64_t last_freq_update;// Time of last frequency update
    bool ramping_up;         // Whether we're currently accelerating
    bool direction_changed;   // Flag for direction changes
} stepper_state_t;

typedef struct {
    uint32_t steps_per_rev;
    int gpio_limit1;
    int gpio_limit2;
} motor_config_t;

typedef struct {
    void (*limit_switch_callback)(int motor_id);
    void (*position_callback)(int32_t position);
    // other callback functions
} motor_callbacks_t;

typedef struct {
    int motor_id;
    stepper_state_t *stepper;
    uint32_t steps_per_rev;
    int gpio_limit1;
    int gpio_limit2;
    motor_callbacks_t *callbacks;
} motor_handle_t;

static stepper_state_t stepper1 = {
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

static stepper_state_t stepper2 = {
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

// Add these as static globals at the top of the file with other static variables
static motor_callbacks_t callbacks1;
static motor_callbacks_t callbacks2;
static motor_handle_t handle1;
static motor_handle_t handle2;

// Forward declarations
static void handle_limit_switch_trigger(motor_handle_t *handle, int *rotation_direction, int64_t current_time);

/*
 * Updates the motor's operating frequency based on acceleration parameters
 * Handles both acceleration ramping and direction changes
 * 
 * @param current_time: Current system time in microseconds
 */
static void update_frequency(int64_t current_time, stepper_state_t *stepper) {
    if (!stepper->ramping_up && !stepper->direction_changed) {
        return;
    }

    float elapsed_sec = (current_time - stepper->last_freq_update) / 1000000.0f;
    if (elapsed_sec < 0.001f) { // Update every 1ms
        return;
    }

    if (stepper->direction_changed) {
        stepper->current_freq = START_FREQ;
        stepper->direction_changed = false;
        stepper->ramping_up = true;
    } else if (stepper->ramping_up) {
        stepper->current_freq += ACCEL_RATE * elapsed_sec;
        if (stepper->current_freq >= stepper->target_freq) {
            stepper->current_freq = stepper->target_freq;
            stepper->ramping_up = false;
        }
    }

    stepper->last_freq_update = current_time;
}

/*
 * Sets the output pins according to the current step in the sequence
 * 
 * @param step_index: Index into the step_sequence array (0-7)
 */
static void set_output_pins(uint8_t step_index, bool is_motor1) {
    if (is_motor1) {
        gpio_set_level(IN1_MOTOR1, step_sequence[step_index][0]);
        gpio_set_level(IN2_MOTOR1, step_sequence[step_index][1]);
        gpio_set_level(IN3_MOTOR1, step_sequence[step_index][2]);
        gpio_set_level(IN4_MOTOR1, step_sequence[step_index][3]);
    } else {
        gpio_set_level(IN1_MOTOR2, step_sequence[step_index][0]);
        gpio_set_level(IN2_MOTOR2, step_sequence[step_index][1]);
        gpio_set_level(IN3_MOTOR2, step_sequence[step_index][2]);
        gpio_set_level(IN4_MOTOR2, step_sequence[step_index][3]);
    }
}

/*
 * Initializes GPIO pins for motor control and limit switches
 * Configures motor pins as outputs and limit switch pins as inputs with pull-up
 */
static void init_gpio(void) {
    gpio_config_t motor1_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << IN1_MOTOR1) | (1ULL << IN2_MOTOR1) | 
                       (1ULL << IN3_MOTOR1) | (1ULL << IN4_MOTOR1),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&motor1_conf);

    gpio_config_t motor2_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << IN1_MOTOR2) | (1ULL << IN2_MOTOR2) | 
                       (1ULL << IN3_MOTOR2) | (1ULL << IN4_MOTOR2),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&motor2_conf);

    gpio_config_t switch_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LIMIT_SWITCH_MOTOR1) | (1ULL << LIMIT_SWITCH_MOTOR2),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&switch_conf);
}

/*
 * Main stepper motor control task
 * Handles motor movement, limit switch monitoring, and speed control
 * Implements smooth acceleration and direction changes
 * 
 * @param pvParameters: Task parameters (unused)
 */
static void stepper_task(void *pvParameters) {
    (void)pvParameters;  // Add this since we use handle directly from global
    motor_handle_t *handle = (motor_handle_t*)pvParameters;
    int64_t current_time;
    int32_t distance_to_go;
    int last_logged_pos = 0;
    int steps_since_yield = 0;
    int rotation_direction = 1;
    int64_t last_trigger_time = 0;
    bool switch1_triggered = false;  // Track if switch1 is currently triggered
    bool switch2_triggered = false;  // Track if switch2 is currently triggered
    const int64_t DEBOUNCE_TIME_US = 200000;  // 200ms debounce

    handle->stepper->targetPos = INT32_MAX * rotation_direction;
    handle->stepper->last_freq_update = esp_timer_get_time();

    while (1) {
        current_time = esp_timer_get_time();
        distance_to_go = handle->stepper->targetPos - handle->stepper->currentPos;

        // Update frequency based on ramping
        update_frequency(current_time, handle->stepper);
        
        // Calculate step timing based on current frequency
        uint32_t micros_per_step = (uint32_t)(1000000.0f / handle->stepper->current_freq);

        if (abs(distance_to_go) < handle->steps_per_rev) {
            handle->stepper->targetPos = rotation_direction * INT32_MAX;
        }

        // Check both switches for falling edge with debounce
        int current_switch1_state = gpio_get_level(handle->gpio_limit1);
        int current_switch2_state = gpio_get_level(handle->gpio_limit2);
        
        if ((current_time - last_trigger_time) > DEBOUNCE_TIME_US) {
            // Check switch 1
            if (current_switch1_state == 0 && !switch1_triggered) {
                switch1_triggered = true;
                handle_limit_switch_trigger(handle, &rotation_direction, current_time);
                last_trigger_time = current_time;
            } else if (current_switch1_state == 1) {
                switch1_triggered = false;
            }

            // Check switch 2
            if (current_switch2_state == 0 && !switch2_triggered) {
                switch2_triggered = true;
                handle_limit_switch_trigger(handle, &rotation_direction, current_time);
                last_trigger_time = current_time;
            } else if (current_switch2_state == 1) {
                switch2_triggered = false;
            }
        }

        // Log position periodically
        if (abs(handle->stepper->currentPos - last_logged_pos) >= 256) {
            ESP_LOGD(TAG, "Position: %d, Frequency: %.1f Hz", 
                     handle->stepper->currentPos, handle->stepper->current_freq);
            last_logged_pos = handle->stepper->currentPos;
            
            if (handle->callbacks && handle->callbacks->position_callback) {
                handle->callbacks->position_callback(handle->stepper->currentPos);
            }
        }

        // Step the motor if enough time has passed
        if ((current_time - handle->stepper->lastStepTime) >= micros_per_step) {
            // Move one step
            if (distance_to_go > 0) {
                handle->stepper->currentPos++;
                handle->stepper->currentStep = (handle->stepper->currentStep + 1) % 8;
            } else {
                handle->stepper->currentPos--;
                handle->stepper->currentStep = (handle->stepper->currentStep + 7) % 8;
            }

            set_output_pins(handle->stepper->currentStep, handle->motor_id == 1);
            handle->stepper->lastStepTime = current_time;
            
            // Yield periodically
            steps_since_yield++;
            if (steps_since_yield >= 32) {
                vTaskDelay(1);
                steps_since_yield = 0;
            }
        }
    }
}

/*
 * Application entry point
 * Initializes the GPIO and creates the stepper motor control task
 */
void app_main(void) {
    motor_config_t config1 = {
        .steps_per_rev = 4096,
        .gpio_limit1 = LIMIT_SWITCH_MOTOR1,
        .gpio_limit2 = 0  // Not used
    };

    motor_config_t config2 = {
        .steps_per_rev = 4096,
        .gpio_limit1 = LIMIT_SWITCH_MOTOR2,
        .gpio_limit2 = 0  // Not used
    };

    init_gpio();
    
    // Initialize callbacks
    callbacks1.limit_switch_callback = NULL;
    callbacks1.position_callback = NULL;
    callbacks2.limit_switch_callback = NULL;
    callbacks2.position_callback = NULL;

    // Initialize handles
    handle1.motor_id = 1;
    handle1.stepper = &stepper1;
    handle1.steps_per_rev = config1.steps_per_rev;
    handle1.gpio_limit1 = config1.gpio_limit1;
    handle1.gpio_limit2 = 0;
    handle1.callbacks = &callbacks1;

    handle2.motor_id = 2;
    handle2.stepper = &stepper2;
    handle2.steps_per_rev = config2.steps_per_rev;
    handle2.gpio_limit1 = config2.gpio_limit1;
    handle2.gpio_limit2 = 0;
    handle2.callbacks = &callbacks2;

    // Create tasks for both motors on different cores
    xTaskCreatePinnedToCore(
        stepper_task,
        "stepper1_task",
        4096,
        &handle1,
        5,  // Lower priority than the other task
        NULL,
        0   // Run on core 0
    );

    xTaskCreatePinnedToCore(
        stepper_task,
        "stepper2_task",
        4096,
        &handle2,
        configMAX_PRIORITIES - 1,  // Higher priority
        NULL,
        1   // Run on core 1
    );
    
    ESP_LOGI(TAG, "Dual stepper motor control started");
}

static void handle_limit_switch_trigger(motor_handle_t *handle, int *rotation_direction, int64_t current_time) {
    (void)current_time;
    
    ESP_LOGI(TAG, "Limit switch triggered - reversing direction for motor %d", handle->motor_id);
    handle->stepper->currentPos = 0;
    *rotation_direction = -(*rotation_direction);
    handle->stepper->targetPos = *rotation_direction * INT32_MAX;
    handle->stepper->direction_changed = true;  // Signal for frequency reset
    
    if (handle->callbacks && handle->callbacks->limit_switch_callback) {
        handle->callbacks->limit_switch_callback(handle->motor_id);
    }
}