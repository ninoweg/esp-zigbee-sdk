#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

// Define the total steps for your stepper motor
#define TOTAL_STEPS 4096 

// Define the StepperDriver structure
typedef struct {
    int16_t pins[4];        // GPIO pins for stepper motor control
    int16_t seq_n;          // Sequence number for stepper motor control
    int16_t step_n;         // Step count for stepper motor position
    bool clockwise;         // Movement direction (clockwise or counter-clockwise)
    float delay_us;         // Delay between steps in microseconds
    bool running;
    TaskHandle_t task_handle; // Task handle for the movement task
} stepper_driver;

// Function prototypes
void init_stepper_driver(stepper_driver *driver, int16_t in1, int16_t in2, int16_t in3, int16_t in4);
void set_rpm(stepper_driver *driver, int16_t rpm);
void start_move_task(stepper_driver *driver, bool clockwise);
void stop_move_task(stepper_driver *driver);
void move_task(void *param);
void step(stepper_driver *driver);
void IRAM_ATTR timer_isr(void *param);
void deinit_timer();
void init_timer(stepper_driver *driver);

#ifdef __cplusplus
} // extern "C"
#endif
