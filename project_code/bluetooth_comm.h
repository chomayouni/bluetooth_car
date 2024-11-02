#ifndef BLUETOOTH_COMM_H
#define BLUETOOTH_COMM_H

#include "C:\Keil_v5\Lab 2\inc/tm4c123gh6pm.h" // TODO swtich this to the correct header file
#include <stdint.h>
#include <stdlib.h>

// Command definitions for RC car
#define CMD_FORWARD     'A'    
#define CMD_BACK        'B'    
#define CMD_LEFT        'C'    
#define CMD_RIGHT       'D'    
#define CMD_STOP        'E'    
#define CMD_INVALID     'X'    

// Function prototypes
void HC05_init(void);                       // Initialize UART5 for HC-05
char Bluetooth_Read(void);                  
void Bluetooth_Write(unsigned char data);    
void Bluetooth_Write_String(char *str);      
void Delay(unsigned long counter);           

#endif 