// motor_control.c
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

void initSystem(void) {
    // Initialize Bluetooth module
    HC05_init();
    
    // Initialize motor control
    initMotors();
    
    // Initialize state machine
    initStateMachine();
}

int main(void) {
    initSystem();
    
    while(1) {
        processBluetoothCommand();
        Delay(1000);  // Small delay to prevent system overload
    }
}