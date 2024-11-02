// File: main.c
#include "main.h"
#include "C:\Keil_v5\Lab 2\inc/tm4c123gh6pm.h"
// #include "motor_control.h"
// #include "bluetooth_comm.h"
// #include "state_machine.h"

void Init_IO(void) {
    // Activate clocks for Port B and Port E
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE;
    
    // Wait for clock to stabilize
    while((SYSCTL_PRGPIO_R & (SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE)) == 0){}
    
    // Configure Port B pins for motor control
    GPIO_PORTB_DIR_R |= (MASK(IN_13R)|MASK(IN_24R)|MASK(EN_R)|
                        MASK(IN_13L)|MASK(IN_24L)|MASK(EN_L));  // Set as outputs
    GPIO_PORTB_DEN_R |= (MASK(IN_13R)|MASK(IN_24R)|MASK(EN_R)|
                        MASK(IN_13L)|MASK(IN_24L)|MASK(EN_L));  // Enable digital I/O
    
    // Configure Port E pins for UART
    GPIO_PORTE_DEN_R |= (MASK(TX)|MASK(RX));  // Enable digital I/O for UART pins
}

void Init_PWM(void) {
    // Enable PWM0 module
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;
    
    // Wait for clock to stabilize
    while((SYSCTL_RCGC0_R & SYSCTL_RCGC0_PWM0) == 0){}
    
    // Configure PWM pins
    GPIO_PORTB_AFSEL_R |= (MASK(EN_R)|MASK(EN_L));  // Enable alternate function
    GPIO_PORTB_PCTL_R &= ~(0xFF << (EN_R*4));       // Clear PCTL bits
    GPIO_PORTB_PCTL_R |= (4 << (EN_R*4));           // Set PCTL for PWM
    GPIO_PORTB_PCTL_R &= ~(0xFF << (EN_L*4));       // Clear PCTL bits
    GPIO_PORTB_PCTL_R |= (4 << (EN_L*4));           // Set PCTL for PWM
    
    // Configure PWM generators
    PWM0_0_CTL_R = 0;                    // Disable PWM while configuring
    PWM0_0_GENA_R = 0x0000008C;          // Set PWM output when counter=LOAD, clear when matches CMP
    PWM0_0_LOAD_R = 16000;               // Set load value for 1kHz (16MHz/16000)
    PWM0_0_CMPA_R = 8000;                // Set duty cycle to 50% initially
    PWM0_0_CTL_R = 1;                    // Enable PWM0
    PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;  // Enable PWM0 outputs
}

void Init_BT(void) {
    // Configure UART for Bluetooth
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R5;  // Enable UART5 clock
    
    // Configure UART pins
    GPIO_PORTE_AFSEL_R |= (MASK(TX)|MASK(RX));  // Enable alternate function
    GPIO_PORTE_PCTL_R &= ~(0xFF << (TX*4));     // Clear PCTL bits
    GPIO_PORTE_PCTL_R |= (1 << (TX*4));         // Set PCTL for UART
    GPIO_PORTE_PCTL_R &= ~(0xFF << (RX*4));     // Clear PCTL bits
    GPIO_PORTE_PCTL_R |= (1 << (RX*4));         // Set PCTL for UART
    
    // Configure UART5
    UART5_CTL_R &= ~UART_CTL_UARTEN;    // Disable UART during setup
    UART5_IBRD_R = 104;                 // 9600 baud rate (16MHz/16/9600)
    UART5_FBRD_R = 11;                  // Fractional part
    UART5_LCRH_R = (UART_LCRH_WLEN_8|   // 8-bit word length
                    UART_LCRH_FEN);      // Enable FIFOs
    UART5_CTL_R |= UART_CTL_UARTEN;     // Enable UART
}

int main(void) {
    // Initialize all subsystems
    Init_IO();
    Init_PWM();
    Init_BT();
    
    // Initialize state machine
    initStateMachine();
    
    while(1) {
        // Process bluetooth commands
        processBluetoothCommand();
        delay(1000);  // Small delay to prevent system overload
    }
}

void delay(uint32_t count) {
    volatile uint32_t i;
    for(i = 0; i < count; i++) {}
}