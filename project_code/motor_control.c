// motor_control.c
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

void initMotors(void) {
    // Configure motor pins as outputs
    GPIO_PORTB_DIR_R |= (1<<MOTOR1_PIN1) | (1<<MOTOR1_PIN2) |
                        (1<<MOTOR2_PIN1) | (1<<MOTOR2_PIN2) |
                        (1<<MOTOR3_PIN1) | (1<<MOTOR3_PIN2) |
                        (1<<MOTOR4_PIN1) | (1<<MOTOR4_PIN2);
    
    // Enable digital functions
    GPIO_PORTB_DEN_R |= (1<<MOTOR1_PIN1) | (1<<MOTOR1_PIN2) |
                        (1<<MOTOR2_PIN1) | (1<<MOTOR2_PIN2) |
                        (1<<MOTOR3_PIN1) | (1<<MOTOR3_PIN2) |
                        (1<<MOTOR4_PIN1) | (1<<MOTOR4_PIN2);
    
    // Initially stop all motors
    stopMotors();
}

void moveForward(void) {
    // Set all motors forward
    GPIO_PORTB_DATA_R |= (1<<MOTOR1_PIN1) | (1<<MOTOR2_PIN1) |
                         (1<<MOTOR3_PIN1) | (1<<MOTOR4_PIN1);
    GPIO_PORTB_DATA_R &= ~((1<<MOTOR1_PIN2) | (1<<MOTOR2_PIN2) |
                          (1<<MOTOR3_PIN2) | (1<<MOTOR4_PIN2));
}

void moveBackward(void) {
    // Set all motors backward
    GPIO_PORTB_DATA_R &= ~((1<<MOTOR1_PIN1) | (1<<MOTOR2_PIN1) |
                          (1<<MOTOR3_PIN1) | (1<<MOTOR4_PIN1));
    GPIO_PORTB_DATA_R |= (1<<MOTOR1_PIN2) | (1<<MOTOR2_PIN2) |
                         (1<<MOTOR3_PIN2) | (1<<MOTOR4_PIN2);
}

void turnLeft(void) {
    // Right side forward, left side backward
    GPIO_PORTB_DATA_R |= (1<<MOTOR2_PIN1) | (1<<MOTOR4_PIN1) |
                         (1<<MOTOR1_PIN2) | (1<<MOTOR3_PIN2);
    GPIO_PORTB_DATA_R &= ~((1<<MOTOR2_PIN2) | (1<<MOTOR4_PIN2) |
                          (1<<MOTOR1_PIN1) | (1<<MOTOR3_PIN1));
}

void turnRight(void) {
    // Left side forward, right side backward
    GPIO_PORTB_DATA_R |= (1<<MOTOR1_PIN1) | (1<<MOTOR3_PIN1) |
                         (1<<MOTOR2_PIN2) | (1<<MOTOR4_PIN2);
    GPIO_PORTB_DATA_R &= ~((1<<MOTOR1_PIN2) | (1<<MOTOR3_PIN2) |
                          (1<<MOTOR2_PIN1) | (1<<MOTOR4_PIN1));
}

void stopMotors(void) {
    // Stop all motors
    GPIO_PORTB_DATA_R &= ~((1<<MOTOR1_PIN1) | (1<<MOTOR1_PIN2) |
                          (1<<MOTOR2_PIN1) | (1<<MOTOR2_PIN2) |
                          (1<<MOTOR3_PIN1) | (1<<MOTOR3_PIN2) |
                          (1<<MOTOR4_PIN1) | (1<<MOTOR4_PIN2));
}