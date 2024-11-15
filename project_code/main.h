// File: main.h
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "TM4C123.h"
#include "tm4c123gh6pm.h" // need to switch this to the correct header file
// Utility macros
#define MASK(x) (1UL << (x))

// Motor control pins - Right motor
#define IN_13R (0)    // Port B
#define IN_24R (1)    // Port B
#define EN_R  (6)     // Port B - PWM

// Motor control pins - Left motor
#define IN_13L (2)    // Port B
#define IN_24L (3)    // Port B
#define EN_L  (7)     // Port B - PWM

// duty cycles for the left and right motors, duty cycle = (CMP_Max - R_CMP)/CMP_Max
#define R_CMP (PWM0_0_CMPA_R)
#define L_CMP (PWM0_0_CMPB_R)
#define CMP_Max (16667 - 1)
#define spinK (0.000045)

// Bluetooth module pins
#define TX    (4)     // Port E
#define RX    (5)     // Port E


#endif // MAIN_H