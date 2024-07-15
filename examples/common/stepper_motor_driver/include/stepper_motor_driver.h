#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*esp_step_callback_t)(int16_t *step, int16_t *min, int16_t *max);

// Define the StepperDriver structure
typedef struct {
    int16_t pin_step;      // GPIO pins for stepper motor control
    int16_t pin_dir;       // GPIO pins for stepper motor direction
    int16_t pin_sleep;     // GPIO pins for setting driver to sleep mode
    int16_t step_n;        // Step count for stepper motor position
    int16_t step_min;
    int16_t step_max;
    int16_t step_goal;     // Movement direction (clockwise or counter-clockwise)
    bool toggle_step;
    bool clockwise;
    float delay_us;        // Delay between steps in microseconds
    bool running;
    TaskHandle_t task_handle; // Task handle for the movement task
} stepper_driver;

// Function prototypes
void init_stepper_driver(stepper_driver *driver, int16_t pin_step, int16_t pin_dir, int16_t pin_sleep, esp_step_callback_t cb);
void set_rpm(stepper_driver *driver, int16_t rpm);
void start_move_task(stepper_driver *driver, int8_t lift_percentage);
void stop_move_task(stepper_driver *driver);
void move_task(void *param);
void step(stepper_driver *driver);
bool set_direction(stepper_driver *driver);
bool check_limits(stepper_driver *driver);
void IRAM_ATTR timer_isr(void *param);
void deinit_timer();
void init_timer(stepper_driver *driver);

#ifdef __cplusplus
} // extern "C"
#endif
