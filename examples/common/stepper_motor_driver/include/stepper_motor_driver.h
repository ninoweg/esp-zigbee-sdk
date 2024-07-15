#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*esp_step_callback_t)(uint16_t *step, uint16_t *min, uint16_t *max);

// Define the StepperDriver structure
typedef struct {
    uint8_t pin_step;      // GPIO pins for stepper motor control
    uint8_t pin_dir;       // GPIO pins for stepper motor direction
    uint8_t pin_sleep;     // GPIO pins for setting driver to sleep mode
    uint16_t step_n;        // Step count for stepper motor position
    uint16_t step_min;
    uint16_t step_max;
    uint16_t step_goal;     // Movement direction (clockwise or counter-clockwise)
    bool toggle_step;
    bool pos_dir;
    bool stop_task;
    float delay_us;        // Delay between steps in microseconds
    TaskHandle_t task_handle; // Task handle for the movement task
} stepper_driver;

// Function prototypes
void init_stepper_driver(stepper_driver *driver, uint8_t pin_step, uint8_t pin_dir, uint8_t pin_sleep, esp_step_callback_t cb);
void set_rpm(stepper_driver *driver, uint16_t rpm);
void start_move_task(stepper_driver *driver, uint8_t lift_percentage);
void stop_move_task(stepper_driver *driver);
void step(stepper_driver *driver);
bool set_direction(stepper_driver *driver);
void IRAM_ATTR timer_isr(void *param);
void deinit_timer();
void init_timer(stepper_driver *driver);

#ifdef __cplusplus
} // extern "C"
#endif
