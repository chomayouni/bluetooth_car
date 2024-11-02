// File: main.h
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "C:\Keil_v5\Lab 2\inc/tm4c123gh6pm.h" // need to switch this to the correct header file

// Utility macros
#define MASK(x) (1UL << (x))

// Motor control pins - Right motor
#define IN_13R (0)    // Port B
#define IN_24R (1)    // Port B
#define EN_R  (5)     // Port B - PWM

// Motor control pins - Left motor
#define IN_13L (2)    // Port B
#define IN_24L (3)    // Port B
#define EN_L  (7)     // Port B - PWM

// Bluetooth module pins
#define TX    (4)     // Port E
#define RX    (5)     // Port E

// Function prototypes
void Init_IO(void);
void Init_PWM(void);
void Init_BT(void);
void delay(uint32_t count);

#endif // MAIN_H