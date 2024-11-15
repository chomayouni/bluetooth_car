
#include "main.h"


void GPIO_Init(void);
void Init_PWM(void);
void HC05_init(void);

void Init_BT(void);
char Bluetooth_Read(void);
void Bluetooth_Write(unsigned char data);
void Bluetooth_Write_String(char *str);

void LED_Control(char direction);

void moveForward(long direction, unsigned long speed, unsigned int duration);
void moveBackward(long direction, unsigned long speed, unsigned int duration);
void tankTurnLeft(long angle, unsigned int speed);
void tankTurnRight(long angle, unsigned int speed);
void stopMotors(void);

void delay(uint32_t count);
void delayms(uint32_t count);

int main(void) {
		// default speed and direction
		long cur_direction = 0;
		unsigned long cur_speed = 8000;
		unsigned int cur_duration = 1000;

	
    // Initialize all subsystems
	  HC05_init();
    GPIO_Init();
    Init_PWM();
	
    // Send welcome message and controls
    Bluetooth_Write_String("Connection initialized. Car is under your Control!\n");
    Bluetooth_Write_String("Controls:\n");
    Bluetooth_Write_String("W - Forward\n");
    Bluetooth_Write_String("S - Backward\n");
    Bluetooth_Write_String("A - Left\n");
    Bluetooth_Write_String("D - Right\n");
    
    while(1)
    {
        char c = Bluetooth_Read();
        
        switch(c) {
            case 'W':
            case 'w':
                LED_Control('W');  // Set LED to indicate forward (Blue)
                Bluetooth_Write_String("Moving Forward\n");
								moveForward(cur_direction,cur_speed,cur_duration);
                break;
                
            case 'S':
            case 's':
                LED_Control('S');  // Set LED to indicate backward (Red)
                Bluetooth_Write_String("Moving Backward\n");
								moveBackward(cur_direction,cur_speed,cur_duration);
                break;
                
            case 'A':
            case 'a':
                LED_Control('A');  // Set LED to indicate left (Green)
                Bluetooth_Write_String("Moving Left\n");
								tankTurnLeft(90,cur_speed);
                break;
                
            case 'D':
            case 'd':
                LED_Control('D');  // Set LED to indicate right (All LEDs)
                Bluetooth_Write_String("Moving Right\n");
								tankTurnRight(90,cur_speed);
                break;
        }
    }
}
void GPIO_Init(void) {
	  SYSCTL->RCGCGPIO |= 0x20;   // Enable clock to GPIOF
    delay(1);
    
    // Unlock PF0 (SW2)
    GPIOF->LOCK = 0x4C4F434B;
    GPIOF->CR = 0x1F;
    
    GPIOF->DIR |= 0x0E;         // Set PF1, PF2, PF3 as outputs (LEDs)
    GPIOF->DEN |= 0x0E;         // Enable digital functions for PF1, PF2, PF3
    GPIOF->DATA &= ~0x0E;       // Turn off all LEDs initially
 
}

void Init_PWM(void) {
		// designed around 16 MHz default clock
	
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB; // Active clock for port B
		// wait for clock to stabilize
		while((SYSCTL_PRGPIO_R & SYSCTL_RCGC2_GPIOB) == 0){};
			
		// Configure Port B pins for motor control
    GPIO_PORTB_DIR_R |= (MASK(IN_13R)|MASK(IN_24R)|MASK(IN_13L)|
                        MASK(IN_24L));  // Set as outputs
    GPIO_PORTB_DEN_R |= (MASK(IN_13R)|MASK(IN_24R)|MASK(IN_13L)|
                        MASK(IN_24L));  // Enable digital I/O
			
    // Enable PWM0 module
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;
 
    // Wait for clock to stabilize
    while((SYSCTL_RCGC0_R & SYSCTL_RCGC0_PWM0) == 0){};
    
		// change PWM CLK to 1 MHz, 16MHz/16 = 1 MHz
		SYSCTL_RCC_R|=	SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32;
		
    // Wait for clock to stabilize
    while((SYSCTL_RCC_R & (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32)) == 0){};			
			
    // Configure PWM pins
    GPIO_PORTB_AFSEL_R |= (MASK(EN_R)|MASK(EN_L));  // Enable alternate function
    GPIO_PORTB_PCTL_R &= ~(0xF << (EN_R*4));       // Clear PCTL bits
    GPIO_PORTB_PCTL_R |= (4 << (EN_R*4));           // Set PCTL for PWM
    GPIO_PORTB_PCTL_R &= ~(0xF << (EN_L*4));       // Clear PCTL bits
    GPIO_PORTB_PCTL_R |= (4 << (EN_L*4));           // Set PCTL for PWM
			
    GPIO_PORTB_AMSEL_R &= ~(MASK(EN_R)|MASK(EN_L)) ; // disable analog function for pins
		GPIO_PORTB_DEN_R |= (MASK(EN_R)|MASK(EN_L)); // Enable digital output
			
    // Configure PWM generators
    PWM0_0_CTL_R = 0;                    // Disable PWM while configuring
    PWM0_0_GENA_R = PWM_0_GENA_ACTLOAD_ONE|PWM_0_GENA_ACTCMPAD_ZERO;          // Set PWM output when counter=LOAD, clear when matches CMPA
		PWM0_0_GENB_R = PWM_0_GENB_ACTLOAD_ONE|PWM_0_GENB_ACTCMPBD_ZERO;          // Set PWM output when counter=LOAD, clear when matches CMPB
    PWM0_0_LOAD_R = 16667;               // Set load value for 60Hz (1MHz/16667)
    PWM0_0_CMPA_R = CMP_Max;           // Set R duty cycle to ~0% initially
		PWM0_0_CMPB_R = CMP_Max;           // Set L duty cycle to ~0% initially
    PWM0_0_CTL_R = 1;                    // Enable PWM0
    PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN;  // Enable PWM0 outputs
		PWM0_ENABLE_R &= ~(PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Disable PWM0 outputs
}

void HC05_init(void)
{
    SYSCTL->RCGCUART |= 0x20;  /* enable clock to UART5 */
    SYSCTL->RCGCGPIO |= 0x10;  /* enable clock to PORTE for PE4/Rx and RE5/Tx */
    delay(1);
    /* UART0 initialization */
    UART5->CTL = 0;         /* UART5 module disbable */
    UART5->IBRD = 104;      /* for 9600 baud rate, integer = 104 */
    UART5->FBRD = 11;       /* for 9600 baud rate, fractional = 11*/
    UART5->CC = 0;          /*select system clock*/
    UART5->LCRH = 0x60;     /* data lenght 8-bit, not parity bit, no FIFO */
    UART5->CTL = 0x301;     /* Enable UART5 module, Rx and Tx */

    /* UART5 TX5 and RX5 use PE4 and PE5. Configure them digital and enable alternate function */
    GPIOE->DEN = 0x30;      /* set PE4 and PE5 as digital */
    GPIOE->AFSEL = 0x30;    /* Use PE4,PE5 alternate function */
    GPIOE->AMSEL = 0;    /* Turn off analg function*/
    GPIOE->PCTL = 0x00110000;     /* configure PE4 and PE5 for UART */
}

void Init_BT(void) {
    // Configure UART for Bluetooth
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R5;  // Enable UART5 clock
    
    // Configure UART pins
    GPIO_PORTE_AFSEL_R |= (MASK(TX)|MASK(RX));  // Enable alternate function
    GPIO_PORTE_PCTL_R &= ~(0xF << (TX*4));     // Clear PCTL bits
    GPIO_PORTE_PCTL_R |= (1 << (TX*4));         // Set PCTL for UART
    GPIO_PORTE_PCTL_R &= ~(0xF << (RX*4));     // Clear PCTL bits
    GPIO_PORTE_PCTL_R |= (1 << (RX*4));         // Set PCTL for UART
    
    // Configure UART5
    UART5_CTL_R &= ~UART_CTL_UARTEN;    // Disable UART during setup
    UART5_IBRD_R = 104;                 // 9600 baud rate (16MHz/16/9600)
    UART5_FBRD_R = 11;                  // Fractional part
    UART5_LCRH_R = (UART_LCRH_WLEN_8|   // 8-bit word length
                    UART_LCRH_FEN);      // Enable FIFOs
    UART5_CTL_R |= UART_CTL_UARTEN;     // Enable UART
}

char Bluetooth_Read(void)  
{
    char data;
    while((UART5->FR & (1<<4)) != 0); /* wait until Rx buffer is not full */
    data = UART5->DR;  	/* before giving it another byte */
    return (unsigned char) data; 
}

void Bluetooth_Write(unsigned char data)  
{
    while((UART5->FR & (1<<5)) != 0); /* wait until Tx buffer not full */
    UART5->DR = data;                  /* before giving it another byte */
}

void Bluetooth_Write_String(char *str)
{
    while(*str)
    {
        Bluetooth_Write(*(str++));
    }
}

void LED_Control(char direction)
{
    // Turn off all LEDs first
    GPIOF->DATA &= ~0x0E;
    
    // Set LED based on direction
    switch(direction) {
        case 'W':  // Forward - Blue LED
            GPIOF->DATA |= (1<<2);  // PF2 (Blue LED)
            break;
            
        case 'S':  // Backward - Red LED
            GPIOF->DATA |= (1<<1);  // PF1 (Red LED)
            break;
            
        case 'A':  // Left - Green LED
            GPIOF->DATA |= (1<<3);  // PF3 (Green LED)
            break;
            
        case 'D':  // Right - All LEDs
            GPIOF->DATA |= 0x0E;    // All LEDs
            break;
    }
}

void moveForward(long direction, unsigned long speed, unsigned int duration) {
		// Direction range [-1000,+1000] -1000 corresponds to full left, +1000 full right, speed range [0,16k] effectively duty cycle if straight, duration in ms
	
    // Set all motors forward using H bridge while retaining enable settings
		GPIO_PORTB_DATA_R &= (MASK(IN_24R)|MASK(IN_24L)|MASK(EN_R)|MASK(EN_L));
		GPIO_PORTB_DATA_R |= (MASK(IN_24R)|MASK(IN_24L));
	
		if (direction >= 0){
			// Right turn
			R_CMP = CMP_Max-speed;
			L_CMP = (unsigned short)( CMP_Max - (CMP_Max - pow(direction*CMP_Max/1000,2)/CMP_Max)*speed/16000);
		}
		else{
			// Left turn
			R_CMP = (unsigned short)( CMP_Max - (CMP_Max - pow(direction*CMP_Max/1000,2)/CMP_Max)*speed/16000);
			L_CMP = CMP_Max-speed;
		}
		PWM0_ENABLE_R |= (PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Enable PWM0 outputs
		delayms(duration);
		PWM0_ENABLE_R &= ~(PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Disable PWM0 outputs
}

void moveBackward(long direction, unsigned long speed, unsigned int duration) {
	  // inputs same as forward
    // Set all motors backward using H bridge while retaining enable settings
		GPIO_PORTB_DATA_R &= (MASK(IN_13R)|MASK(IN_13L)|MASK(EN_R)|MASK(EN_L));
		GPIO_PORTB_DATA_R |= (MASK(IN_13R)|MASK(IN_13L));
	
		if (direction >= 0){
			// Right turn
			R_CMP = CMP_Max-speed;
			L_CMP = (unsigned short)( CMP_Max - (CMP_Max - pow(direction*CMP_Max/1000,2)/CMP_Max)*speed/16000);
		}
		else{
			// Left turn
			R_CMP = (unsigned short)( CMP_Max - (CMP_Max - pow(direction*CMP_Max/1000,2)/CMP_Max)*speed/16000);
			L_CMP = CMP_Max-speed;
		}
		PWM0_ENABLE_R |= (PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Enable PWM0 outputs
		delayms(duration);
		PWM0_ENABLE_R &= ~(PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Disable PWM0 outputs
}

void tankTurnLeft(long angle, unsigned int speed) {
    // Right side forward, left side backward
		GPIO_PORTB_DATA_R &= (MASK(IN_24R)|MASK(IN_13L)|MASK(EN_R)|MASK(EN_L));
		GPIO_PORTB_DATA_R |= (MASK(IN_24R)|MASK(IN_13L));
	
		// set to given speed
		L_CMP = CMP_Max-speed;
		R_CMP = CMP_Max-speed;
	
		// calculate approx time to turn that far using an empircal constant
		unsigned long angleDelay = labs(angle)/( spinK * (float)speed) ; 
	
		PWM0_ENABLE_R |= (PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Enable PWM0 outputs
		delayms(angleDelay);
		PWM0_ENABLE_R &= ~(PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Disable PWM0 outputs
}

void tankTurnRight(long angle, unsigned int speed) {
    // Left side forward, right side backward
		GPIO_PORTB_DATA_R &= (MASK(IN_13R)|MASK(IN_24L)|MASK(EN_R)|MASK(EN_L));
		GPIO_PORTB_DATA_R |= (MASK(IN_13R)|MASK(IN_24L));
	
		// set to given speed
		L_CMP = CMP_Max-speed;
		R_CMP = CMP_Max-speed;
	
		// calculate approx time to turn that far using an empircal constant
		unsigned long angleDelay = labs(angle)/( spinK * (float)speed) ; 
	
		PWM0_ENABLE_R |= (PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Enable PWM0 outputs
		delayms(angleDelay);
		PWM0_ENABLE_R &= ~(PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Disable PWM0 outputs
} 

void stopMotors(void) {
    // Stop all motors, 
		PWM0_ENABLE_R &= ~(PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);  // Disable PWM0 outputs
}


void delayms(uint32_t count){
    volatile uint32_t i;
    for(i = 0; i < count; i++) {
			delay(1058); // empirical constant for 1ms delay
		}
}
void delay(uint32_t count) {
    volatile uint32_t i;
    for(i = 0; i < count; i++) {}
}



