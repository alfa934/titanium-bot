#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>

// Define your pins for ultrasonic sensors and motor control
#define LEFT_TRIG_PIN GPIO_PIN_0
#define LEFT_TRIG_PORT GPIOA
#define LEFT_ECHO_PIN GPIO_PIN_1
#define LEFT_ECHO_PORT GPIOA

#define RIGHT_TRIG_PIN GPIO_PIN_2
#define RIGHT_TRIG_PORT GPIOA
#define RIGHT_ECHO_PIN GPIO_PIN_3
#define RIGHT_ECHO_PORT GPIOA

// Motor control pins (example for L298N driver)
#define LEFT_MOTOR_PWM_PIN GPIO_PIN_4
#define LEFT_MOTOR_PWM_PORT GPIOA
#define LEFT_MOTOR_DIR1_PIN GPIO_PIN_5
#define LEFT_MOTOR_DIR1_PORT GPIOA
#define LEFT_MOTOR_DIR2_PIN GPIO_PIN_6
#define LEFT_MOTOR_DIR2_PORT GPIOA

#define RIGHT_MOTOR_PWM_PIN GPIO_PIN_7
#define RIGHT_MOTOR_PWM_PORT GPIOA
#define RIGHT_MOTOR_DIR1_PIN GPIO_PIN_8
#define RIGHT_MOTOR_DIR1_PORT GPIOA
#define RIGHT_MOTOR_DIR2_PIN GPIO_PIN_9
#define RIGHT_MOTOR_DIR2_PORT GPIOA

// Constants
#define DESIRED_DISTANCE 10.0f  // 10 cm
#define MAX_DISTANCE 200.0f     // Maximum measurable distance
#define SENSOR_TIMEOUT 1000     // Timeout for sensor reading (ms)
#define PID_UPDATE_RATE 100     // PID update rate in ms
#define WALL_LOST_THRESHOLD 15.0f // Distance threshold to consider wall as lost

// PID parameters
#define KP_DIST 1.0f
#define KI_DIST 0.0f
#define KD_DIST 0.0f

// Global variables
volatile uint32_t left_echo_start = 0;
volatile uint32_t left_echo_end = 0;
volatile uint32_t right_echo_start = 0;
volatile uint32_t right_echo_end = 0;
volatile uint8_t left_echo_received = 0;
volatile uint8_t right_echo_received = 0;

float left_distance = 0;
float right_distance = 0;

// PID variables
float dist_integral = 0;
float prev_dist_error = 0;

// Movement direction
typedef enum {
  MOVING_LEFT,
  MOVING_RIGHT
} MovementDirection;

MovementDirection current_direction = MOVING_RIGHT;

// Function prototypes
void trigger_ultrasonic(GPIO_TypeDef* trig_port, uint16_t trig_pin);
float calculate_distance(uint32_t echo_start, uint32_t echo_end);
void read_sensors(void);
void control_loop(void);
void set_motor_speeds(int left_speed, int right_speed);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

int main(void) {
  // HAL initialization and peripheral setup
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM_Init();  // For PWM and timing
  
  // Enable interrupts for echo pins
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  
  uint32_t last_pid_time = HAL_GetTick();
  
  while (1) {
    // Read sensors periodically
    read_sensors();
    
    // Run control loop at fixed interval
    if (HAL_GetTick() - last_pid_time >= PID_UPDATE_RATE) {
      control_loop();
      last_pid_time = HAL_GetTick();
    }
    
    HAL_Delay(10);
  }
}

void trigger_ultrasonic(GPIO_TypeDef* trig_port, uint16_t trig_pin) {
  // Send 10us pulse to trigger pin
  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
  HAL_Delay(1);  // 10us delay (approximate)
  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
}

float calculate_distance(uint32_t echo_start, uint32_t echo_end) {
  if (echo_end < echo_start) return MAX_DISTANCE;  // Overflow protection
  
  uint32_t pulse_duration = echo_end - echo_start;
  // Calculate distance in cm (sound speed = 343 m/s = 0.0343 cm/μs)
  return (pulse_duration * 0.0343f) / 2.0f;
}

void read_sensors(void) {
  // Reset flags
  left_echo_received = 0;
  right_echo_received = 0;
  
  // Trigger left sensor
  trigger_ultrasonic(LEFT_TRIG_PORT, LEFT_TRIG_PIN);
  
  // Wait for echo or timeout
  uint32_t start_time = HAL_GetTick();
  while (!left_echo_received && (HAL_GetTick() - start_time < SENSOR_TIMEOUT)) {
    // Wait for echo
  }
  
  if (left_echo_received) {
    left_distance = calculate_distance(left_echo_start, left_echo_end);
  } else {
    left_distance = MAX_DISTANCE;
  }
  
  // Small delay between sensors
  HAL_Delay(10);
  
  // Trigger right sensor
  trigger_ultrasonic(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN);
  
  // Wait for echo or timeout
  start_time = HAL_GetTick();
  while (!right_echo_received && (HAL_GetTick() - start_time < SENSOR_TIMEOUT)) {
    // Wait for echo
  }
  
  if (right_echo_received) {
    right_distance = calculate_distance(right_echo_start, right_echo_end);
  } else {
    right_distance = MAX_DISTANCE;
  }
}

void control_loop(void) {
  static uint32_t last_time = 0;
  uint32_t current_time = HAL_GetTick();
  float dt = (current_time - last_time) / 1000.0f;  // Convert to seconds
  if (dt <= 0) dt = 0.01f;  // Prevent division by zero
  last_time = current_time;
  
  // Check if either sensor has lost the wall
  int left_lost = (left_distance >= WALL_LOST_THRESHOLD);
  int right_lost = (right_distance >= WALL_LOST_THRESHOLD);
  
  // Determine which sensor to use for distance control
  float control_distance;
  if (!left_lost && !right_lost) {
    // Both sensors see the wall, use the average
    control_distance = (left_distance + right_distance) / 2.0f;
  } else if (!left_lost && right_lost) {
    // Only left sensor sees the wall
    control_distance = left_distance;
    
    // If we're moving right and the right sensor is lost, change direction to left
    if (current_direction == MOVING_RIGHT) {
      current_direction = MOVING_LEFT;
    }
  } else if (left_lost && !right_lost) {
    // Only right sensor sees the wall
    control_distance = right_distance;
    
    // If we're moving left and the left sensor is lost, change direction to right
    if (current_direction == MOVING_LEFT) {
      current_direction = MOVING_RIGHT;
    }
  } else {
    // Both sensors lost, move forward slowly to find the wall
    set_motor_speeds(30, 30);
    return;
  }
  
  // Distance PID control
  float dist_error = control_distance - DESIRED_DISTANCE;
  dist_integral += dist_error * dt;
  float dist_derivative = (dist_error - prev_dist_error) / dt;
  prev_dist_error = dist_error;
  
  float dist_correction = KP_DIST * dist_error + KI_DIST * dist_integral + KD_DIST * dist_derivative;
  
  // Base movement speed
  int base_speed = 50;
  
  // Apply movement direction
  int left_speed, right_speed;
  if (current_direction == MOVING_LEFT) {
    left_speed = base_speed - dist_correction;
    right_speed = base_speed + dist_correction;
  } else { // MOVING_RIGHT
    left_speed = base_speed + dist_correction;
    right_speed = base_speed - dist_correction;
  }
  
  // Constrain speeds to valid range
  left_speed = (left_speed < 0) ? 0 : (left_speed > 100) ? 100 : left_speed;
  right_speed = (right_speed < 0) ? 0 : (right_speed > 100) ? 100 : right_speed;
  
  set_motor_speeds(left_speed, right_speed);
}

void set_motor_speeds(int left_speed, int right_speed) {
  // Set left motor direction and speed
  if (current_direction == MOVING_LEFT) {
    // Moving left: right motor forward, left motor backward
    HAL_GPIO_WritePin(LEFT_MOTOR_DIR1_PORT, LEFT_MOTOR_DIR1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEFT_MOTOR_DIR2_PORT, LEFT_MOTOR_DIR2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RIGHT_MOTOR_DIR1_PORT, RIGHT_MOTOR_DIR1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RIGHT_MOTOR_DIR2_PORT, RIGHT_MOTOR_DIR2_PIN, GPIO_PIN_RESET);
  } else {
    // Moving right: left motor forward, right motor backward
    HAL_GPIO_WritePin(LEFT_MOTOR_DIR1_PORT, LEFT_MOTOR_DIR1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEFT_MOTOR_DIR2_PORT, LEFT_MOTOR_DIR2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_MOTOR_DIR1_PORT, RIGHT_MOTOR_DIR1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_MOTOR_DIR2_PORT, RIGHT_MOTOR_DIR2_PIN, GPIO_PIN_SET);
  }
  
  // Set PWM duty cycle for motors (assuming you have PWM setup)
  // __HAL_TIM_SET_COMPARE(&htimx, LEFT_MOTOR_PWM_CHANNEL, left_speed);
  // __HAL_TIM_SET_COMPARE(&htimx, RIGHT_MOTOR_PWM_CHANNEL, right_speed);
}

// Echo pin interrupt callbacks
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == LEFT_ECHO_PIN) {
    if (HAL_GPIO_ReadPin(LEFT_ECHO_PORT, LEFT_ECHO_PIN) == GPIO_PIN_SET) {
      // Rising edge - start timing
      left_echo_start = HAL_GetTick();
    } else {
      // Falling edge - stop timing
      left_echo_end = HAL_GetTick();
      left_echo_received = 1;
    }
  }
  
  if (GPIO_Pin == RIGHT_ECHO_PIN) {
    if (HAL_GPIO_ReadPin(RIGHT_ECHO_PORT, RIGHT_ECHO_PIN) == GPIO_PIN_SET) {
      // Rising edge - start timing
      right_echo_start = HAL_GetTick();
    } else {
      // Falling edge - stop timing
      right_echo_end = HAL_GetTick();
      right_echo_received = 1;
    }
  }
}