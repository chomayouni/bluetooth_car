#include <stdint.h>
#include "bluetooth_comm.h"
#include "C:\Keil_v5\Lab 2\inc/tm4c123gh6pm.h"

//This is the weird spooky shit that I don't understand
//It might work, It might not, but that will be for testing later

void HC05_init(void) {
    // Enable clocks
    SYSCTL->RCGCUART |= 0x20;  // Enable clock to UART5
    SYSCTL->RCGCGPIO |= 0x10;  
    Delay(1);
    
    // UART5 initialization
    UART5->CTL = 0;            // Disable UART5 module during setup
    UART5->IBRD = 104;         // 9600 baud rate, integer = 104
    UART5->FBRD = 11;          // 9600 baud rate, fractional = 11
    UART5->CC = 0;             // Select system clock
    UART5->LCRH = 0x60;        // 8-bit data, no parity, no FIFO
    UART5->CTL = 0x301;        // Enable UART5 module, Rx and Tx
    
    // Configure UART5 pins (PE4 and PE5)
    GPIOE->DEN = 0x30;         // Set PE4 and PE5 as digital
    GPIOE->AFSEL = 0x30;       // Use PE4,PE5 alternate function
    GPIOE->AMSEL = 0;          // Turn off analog function
    GPIOE->PCTL = 0x00110000;  // Configure PE4 and PE5 for UART
}

char Bluetooth_Read(void) {
    char data;
    while((UART5->FR & (1<<4)) != 0);  // Wait until Rx buffer is not full
    data = UART5->DR;                   // Read data
    return (unsigned char) data;
}

void Bluetooth_Write(unsigned char data) {
    while((UART5->FR & (1<<5)) != 0);  // Wait until Tx buffer not full
    UART5->DR = data;                   // Send data
}

void Bluetooth_Write_String(char *str) {
    while(*str) {
        Bluetooth_Write(*(str++));
    }
}

void Delay(unsigned long counter) {
    unsigned long i = 0;
    for(i=0; i< counter; i++);
}

// functions for RC car control
void sendCarStatus(char command) {
    switch(command) {
        case CMD_FORWARD:
            Bluetooth_Write_String("Moving Forward\n");
            break;
        case CMD_BACK:
            Bluetooth_Write_String("Moving Backward\n");
            break;
        case CMD_LEFT:
            Bluetooth_Write_String("Turning Left\n");
            break;
        case CMD_RIGHT:
            Bluetooth_Write_String("Turning Right\n");
            break;
        case CMD_STOP:
            Bluetooth_Write_String("Stopped\n");
            break;
        default:
            Bluetooth_Write_String("Invalid Command\n");
            break;
    }
}

// control loop function
void processBluetoothCommand(void) {
    char command = Bluetooth_Read();
    
    switch(command) {
        case CMD_FORWARD:
            moveForward();
            sendCarStatus(command);
            break;
            
        case CMD_BACK:
            moveBackward();
            sendCarStatus(command);
            break;
            
        case CMD_LEFT:
            turnLeft();
            sendCarStatus(command);
            break;
            
        case CMD_RIGHT:
            turnRight();
            sendCarStatus(command);
            break;
            
        case CMD_STOP:
            stopMotors();
            sendCarStatus(command);
            break;
            
        default:
            stopMotors();
            sendCarStatus(CMD_INVALID);
            break;
    }
}