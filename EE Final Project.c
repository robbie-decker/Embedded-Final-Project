#define RS 1 /* BIT0 mask */
#define RW 2 /* BIT1 mask */
#define EN 4 /* BIT2 mask */
#include "MKL25Z4.h"
#include "board.h"
#include <stdio.h>
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
void delayMs(int n);
void Trigger_Timer0_init(void);
void Capture_Timer2_init(void);
void ADC0_IRQHandler(void);
void ADC0_init(void);
void PWM_init(void);
void LED_init(void);
void LED_set(int s);
void UART0_init(void);
void UART0Tx(char c);
void UART0_puts(char* s);

void keypad_init(void);
unsigned char keypad_getkey(void);

unsigned char LCDdecode(int);
void LCD_nibble_write(unsigned char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void UART0_Transmit_Poll(uint8_t);
uint8_t UART0_Receive_Poll(void);

unsigned char LCDpattern[] = {0x0, 'A', '3', '2', '1', 'B', '6', '5', '4', 'C', '9', '8', '7', 'D', 'E', '0', 'F'};
unsigned char keypress;
volatile uint32_t pulse_width;
volatile float distance;
// https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
int tmpInt1;
float tmpFrac;
int tmpInt2;
volatile uint32_t cont[2];
short int result;
char buffer[30];
int mode;
int i = 0;
#define mod 44999  //14999


int main(void) {

	keypad_init();
	LCD_init();
	LCD_command(1); /* clear display */
	delayMs(1);
	LCD_command(0x80); /* sets first value at the beginning of the LCD */
	delayMs(1);

	__disable_irq();
    BOARD_InitBootClocks();

    LED_init();                     /* Configure LEDs */
    ADC0_init();                    /* Configure ADC0 */
    PWM_init();						/* Configure PWM */
    UART0_init();
    sprintf(buffer, "\r\nUltrasonic sensor Testing"); 	/* convert to string */
	UART0_puts(buffer);

	        					/* global disable IRQs */
	Trigger_Timer0_init();                     	/* Configure PWM */
	Capture_Timer2_init();
	__enable_irq();

		ADC0->SC1[0] &= ~0x1F;
		unsigned char key;
    while (1) {
    	key = keypad_getkey();
		delayMs(5);

		if(key!= 0) /* checks if a value has been entered */
		{
			LCD_command(1); /* clear display */
			delayMs(1);
			LCD_command(0x80);
			delayMs(1);
			keypress = LCDdecode(key);
			LCD_data(keypress);
			delayMs(10);
			UART0_Transmit_Poll(keypress);

			if(keypress == '1')
			{
				mode = 1;
			}

			if(keypress == '2')
			{
				mode = 2;
			}
			if(keypress == '3')
			{
				mode = 3;
			}
			if(keypress == '4')
			{
				mode = 4;
			}
			if(keypress == '5')
			{
				mode = 5;
			}



//        ADC0->SC1[0] = 0;     /* start conversion on channel 0 */
//
//        while(!(ADC0->SC1[0] & 0x80)) { } /* wait for conversion complete */
//        result = ADC0->R[0];
//        /* read conversion result and clear COCO flag */
//
////      For LED
//        //LED_set(result >> 7);       /* display result on LED with bits 2, 1, & 0 after shift */
//
////      For Servo
//        //TPM0->CONTROLS[1].CnV =result;  /* Set up channel value between 2.5% and 12.5%*/
//
////      For the Buzzer
//        //TPM0->MOD = 6000 - 5*(result/4);      /* Frequencies from 500 Hz to 3000 Hz*/
//        //TPM0->CONTROLS[1].CnV = TPM0->MOD/2;  /* Set up channel value between 50%*/
//
////      For DC motor
//        TPM0->CONTROLS[1].CnV = result*14;  /* Set up channel value between 0% - 93%*/
		}
    }
}

void ADC0_IRQHandler(void){
	int voltage;
	result = ADC0->R[0];
	if(mode == 1)
	{
		TPM0->CONTROLS[0].CnV = result;
	}

	if(mode == 2)
	{
		TPM0->CONTROLS[1].CnV = result*14;
	}
	voltage = result * 3300/4096;
	sprintf(buffer, "\r\n voltage = %d mV", voltage);
	UART0_puts(buffer);
	delayMs(10);

	ADC0->SC1[0] &= ~0x17;

}


void PWM_init(void)
{
    SIM->SCGC5 |= 0x800;       		   	   /* enable clock to Port C*/
    PORTC->PCR[2] = 0x0400;     		   /* PTC2 used by TPM0 channel 1 (DC Motor)*/
    PORTC->PCR[1] = 0x0400;					// PTC1 used by TPm0 channel 0 (Servo motor)
    SIM->SCGC6 |= 0x01000000;   		   /* enable clock to TPM0 */
    SIM->SOPT2 |= 0x01000000;   		   /* use MCGFLLCLK as timer counter clock */

    TPM0->SC = 0;               		  /* disable timer */
//    TPM0->CONTROLS[1].CnSC = 0x20|0x08;   /* edge-aligned PWM, pulse high MSB:MSA=10, ELSB:ELSA=10*/
    TPM0->CONTROLS[1].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH1 as edge-aligned PWM
    TPM0->CONTROLS[0].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH0 as edge-aligned PWM
    TPM0->MOD = 60000;          		  /* Set up modulo register for 50 Hz - 48.00 MHz */
    TPM0->CONTROLS[1].CnV = 1500;  		  /* Set up channel value for 2.5% duty-cycle */
    TPM0->CONTROLS[0].CnV = 1500;
    TPM0->SC |= 0x0C;            		  /* enable TPM0 with pre-scaler /16 */
}

void ADC0_init(void)
{
	uint16_t calibration;

    SIM->SCGC5 |= 0x2000;       /* clock to PORTE */
    PORTE->PCR[20] = 0;         /* PTE20 analog input */

    SIM->SCGC6 |= 0x8000000;    /* clock to ADC0 */
    ADC0->SC2 &= ~0x40;         /* software trigger */
    /* clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
    ADC0->SC1[0] |= 0x40;
    NVIC->ISER[0] |= 0x8000;					//enables ADC0 interrupt
    //Start Calibration
    ADC0->SC3 |= ADC_SC3_CAL_MASK;
	while (ADC0->SC3 & ADC_SC3_CAL_MASK) {
	// Wait for calibration to complete
	}
	// Finish off the calibration
	// Initialize a 16-bit variable in RAM
	calibration = 0x0;
	// Add the plus-side calibration results to the variable
	calibration += ADC0->CLP0;
	calibration += ADC0->CLP1;
	calibration += ADC0->CLP2;
	calibration += ADC0->CLP3;
	calibration += ADC0->CLP4;
	calibration += ADC0->CLPS;
	// Divide by two
	calibration /= 2;
	// Set the MSB of the variable
	calibration |= 0x8000;
	// Store the value in the plus-side gain calibration register
	ADC0->PG = calibration;
	// Repeat the procedure for the minus-side calibration value
	calibration = 0x0000;
	calibration += ADC0->CLM0;
	calibration += ADC0->CLM1;
	calibration += ADC0->CLM2;
	calibration += ADC0->CLM3;
	calibration += ADC0->CLM4;
	calibration += ADC0->CLMS;
	calibration /= 2;
	calibration |= 0x8000;
	ADC0->MG = calibration;
    //Done Calibration

	/* Reconfigure ADC0*/
    /* clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
}

void LED_init(void) {
    SIM->SCGC5 |= 0x400;        /* enable clock to Port B */
    SIM->SCGC5 |= 0x1000;       /* enable clock to Port D */
    PORTB->PCR[18] = 0x100;     /* make PTB18 pin as GPIO */
    PTB->PSOR = 0x40000;    	/* turn off red LED */
    PTB->PDDR |= 0x40000;       /* make PTB18 as output pin */
    PORTB->PCR[19] = 0x100;     /* make PTB19 pin as GPIO */
    PTB->PSOR = 0x80000;    	/* turn off green LED */
    PTB->PDDR |= 0x80000;       /* make PTB19 as output pin */
    PORTD->PCR[1] = 0x100;      /* make PTD1 pin as GPIO */
    PTD->PSOR = 0x02;			// turn off Blue LED
    PTD->PDDR |= 0x02;  		/* make PTD1 as output pin */



}


void LED_set(int s) {
    if (s & 1)    /* use bit 0 of s to control red LED */
        PTB->PCOR = 0x40000;    /* turn on red LED */
    else
        PTB->PSOR = 0x40000;    /* turn off red LED */

    if (s & 2)    /* use bit 1 of s to control green LED */
        PTB->PCOR = 0x80000;    /* turn on green LED */
    else
        PTB->PSOR = 0x80000;    /* turn off green LED */

    if (s & 4)    /* use bit 2 of s to control blue LED */
        PTD->PCOR = 0x02;       /* turn on blue LED */
    else
        PTD->PSOR = 0x02;       /* turn off blue LED */
}

void Trigger_Timer0_init(void)
{
    SIM->SCGC5 |= 0x0800;       /* enable clock to Port C*/
    SIM->SCGC6 |= 0x01000000;   /* enable clock to TPM0 */
    SIM->SOPT2 |= 0x01000000;   /* use MCGFLLCLK as timer counter clock */
    PORTC->PCR[2] = 0x0400;     /* PTC2 used by TPM0_CH1 */

    TPM0->SC = 0;               /* disable timer */
    TPM0->CONTROLS[1].CnSC  = 0x80;   	  /* clear CHF  for Channel 1*/
    TPM0->CONTROLS[1].CnSC |= 0x20|0x08;  /* edge-aligned, pulse high MSB:MSA=10, ELSB:ELSA=10*/
    TPM0->CONTROLS[1].CnV   = 8;  		  /* Set up channel value for >10 us*/
	TPM0->SC |= 0x06;           		  /* set timer with prescaler /64 */
    TPM0->MOD = mod;            		  /* Set up modulo register = 44999 */
//*************************PRE-Scaler settings **************************
	TPM0->SC |= 0x08;           	      /* enable timer */
//***********************************************************************
}


void Capture_Timer2_init(void)  // Also enables the TPM2_CH1 interrupt
{
    SIM->SCGC5 |= 0x2000;       /* enable clock to Port E*/
    SIM->SCGC6 |= 0x04000000;   /* enable clock to TPM2 */
    PORTE->PCR[23] = 0x0300;    /* PTE23 used by TPM2_CH1 */
    SIM->SOPT2 |= 0x01000000;   /* use MCGFLLCLK as timer counter clock */
    TPM2->SC = 0;               /* disable timer */

    TPM2->CONTROLS[1].CnSC = 0x80;   	/* clear CHF  for Channel 1*/
//  MSB:MSA=00, ELSB:ELSA=11 and set interrupt*/
/*  capture on both edges, MSB:MSA=00, ELSB:ELSA=11 and set interrupt*/
    TPM2->CONTROLS[1].CnSC |= TPM_CnSC_CHIE_MASK|TPM_CnSC_ELSB_MASK|TPM_CnSC_ELSA_MASK;
    TPM2->MOD = mod;            		  /* Set up modulo register = 44999*/
    TPM2->CONTROLS[1].CnV = (mod+1)/2 -1; /* Set up 50% dutycycle */

	TPM2->SC |= 0x80;           /* clear TOF */
	TPM2->SC |= 0x06;           /* enable timer with prescaler /2^6 = 64 */
//*************************PRE-Scaler settings *********************************************
	TPM2->SC |= 0x08;           /* enable timer             */
//******************************************************************************************
    NVIC_EnableIRQ(TPM2_IRQn);  /* enable Timer2 interrupt in NVIC */
}


void UART0_init(void) {
    SIM->SCGC4 |= 0x0400;    /* enable clock for UART0 */
    SIM->SOPT2 |= 0x04000000;    /* use FLL output for UART Baud rate generator */
    UART0->C2   = 0;          /* turn off UART0 while changing configurations */
    UART0->BDH  = 0x00;
    UART0->BDL  = 0x1A;      /* 115200 Baud with 48 MHz*/
    UART0->C4   = 0x0F;       /* Over Sampling Ratio 16 */
    UART0->C1   = 0x00;       /* 8-bit data */
    UART0->C2   = 0x08;       /* enable transmit */

    SIM->SCGC5   |= 0x0200;    /* enable clock for PORTA */
    PORTA->PCR[2] = 0x0200; /* make PTA2 UART0_Tx pin */
}


void UART0Tx(char c) {
    while(!(UART0->S1 & 0x80)) {
    }   /* wait for transmit buffer empty */
    UART0->D = c; /* send a char */
}

void UART0_puts(char* s) {
    while (*s != 0)         /* if not end of string */
        UART0Tx(*s++);      /* send the character through UART0 */
}

void TPM2_IRQHandler(void) {
	while(!(TPM2->CONTROLS[1].CnSC & 0x80)) { } /* wait until the CHF is set */
	cont[i%2] = TPM2->CONTROLS[1].CnV;
	if(i%2 == 1){

		if(cont[1] > cont[0] ){
			pulse_width = cont[1] - cont[0];
		}
		    else {
		    pulse_width = cont[1] - cont[0] + mod + 1;
		}

		sprintf(buffer, "Pulse width %d \r\n", pulse_width); /* convert to string */
		UART0_puts(buffer);
		distance = (float)(pulse_width*2/3)*0.0343;

		tmpInt1 = distance;
		tmpFrac = distance - tmpInt1;
		tmpInt2 = (tmpFrac * 10000);

	    sprintf(buffer, "Distance %d.%04d cm\r\n", tmpInt1, tmpInt2); /* convert to string */
	    UART0_puts(buffer);

		distance = (float)(distance/2.54);

		tmpInt1 = distance;
		tmpFrac = distance - tmpInt1;
		tmpInt2 = (tmpFrac * 10000);

	    sprintf(buffer, "Distance %d.%04d inches\r\n", tmpInt1, tmpInt2); /* convert to string */
	    UART0_puts(buffer);

    	if(distance < (float)12.0){
    		PTD->PSOR = 0x02;			//turn off blue LED
    	    PTB->PCOR |= 0x40000;       /* Clear PTB18 to turn on red LED */
    	    TPM0->CONTROLS[1].CnV = 0;
    	}
    	else if(distance < (float)24. && distance > (float)12.0){
    		PTB->PSOR |= 0x40000;		//turn off red LED
    		PTD->PCOR = 0x02;       /* turn on blue LED */
    	}
    	else
    	    PTB->PSOR |= 0x40000;	/* Set PTB18 to turn off red LED */
	}
	i++;
/*---------------------------------------------------------------------*/
    TPM2->CONTROLS[1].CnSC |= 0x80;    /* clear CHF */
}

void delayMs(int n) {
	int i;
	int j;
	for(i = 0 ; i < n; i++)
    	for (j = 0; j < 3500; j++) {}
}

void LCD_init(void)
{
	SIM->SCGC5 |= 0x400; /* enable clock to Port B */
	PORTB->PCR[0] = 0x100; /* make PTB0 pin as GPIO */
	PORTB->PCR[1] = 0x100; /* make PTB1 pin as GPIO */
	PORTB->PCR[2] = 0x100; /* make PTB2 pin as GPIO */
	PORTB->PCR[8] = 0x100; /* make PTB8 pin as GPIO */
	PORTB->PCR[9] = 0x100; /* make PTB9 pin as GPIO */
	PORTB->PCR[10] = 0x100; /* make PTB10 pin as GPIO */
	PORTB->PCR[11] = 0x100; /* make PTB11 pin as GPIO */
	PTB->PDDR |= 0xF07;
	delayMs(30); /* initialization sequence */
	LCD_nibble_write(0x30, 0);
	delayMs(10);
	LCD_nibble_write(0x30, 0);
	delayMs(1);
	LCD_nibble_write(0x30, 0);
	delayMs(1);
	LCD_nibble_write(0x20, 0); /* use 4-bit data mode */
	delayMs(1);
	LCD_command(0x28); /* set 4-bit data, 2-line, 5x7 font */
	LCD_command(0x06); /* move cursor right */
	LCD_command(0x01); /* clear screen, move cursor to home */
	LCD_command(0x0F); /* turn on display, cursor blinking */
}
void LCD_nibble_write(unsigned char data, unsigned char control)
{
	unsigned char data1;
	unsigned char data2;
	unsigned char data3;
	data &= 0xF0; /* clear lower nibble for control */
	control &= 0x0F; /* clear upper nibble for data */
	data1 = data |control;
	data2 = data | control |EN;
	data3 = data;
	PTB->PDOR = (data1 & 0xF0) << 4 | (data1 & 0x0F);
	PTB->PDOR = (data2 & 0xF0) << 4 | (data2 & 0x0F);
	delayMs(2);
	PTB->PDOR = (data3 & 0xF0) << 4 | (data3 & 0x0F);
	PTB->PDOR = 0;
}
void LCD_command(unsigned char command)
{
	LCD_nibble_write(command & 0xF0, 0); /* upper nibble first */
	LCD_nibble_write(command << 4, 0); /* then lower nibble */
	if (command < 4)
	delayMs(4); /* commands 1 and 2 need up to 1.64ms */
	else
	delayMs(1); /* all others 40 us */
}
void LCD_data(unsigned char data)
{
	LCD_nibble_write(data & 0xF0, RS); /* upper nibble first */
	LCD_nibble_write(data << 4, RS); /* then lower nibble */
	delayMs(1);
}
unsigned char LCDdecode(int n)
{
	return LCDpattern[n];
}

unsigned char keypad_getkey(void)
{
	int row, col;
	const int row_select[] = {0x01, 0x800, 0x400, 0x08}; /* one row is active */
	/* check to see any key pressed */
	PTC->PDDR |= 0xC09; /* enable all rows */
	PTC->PCOR = 0xC09;
	delayMs(2); /* wait for signal return */
	col = PTC->PDIR & 0xF0; /* read all columns */
	PTC->PDDR = 000; /* disable all rows */
	if (col == 0xF0)
		return 0; /* no key pressed */
	/* If a key is pressed, it gets here to find out which key.
	* It activates one row at a time and read the input to see
	* which column is active. */
	for (row = 0; row < 4; row++)
	{
		PTC->PDDR = 000; /* disable all rows */
		PTC->PDDR |= row_select[row]; /* enable one row */
		PTC->PCOR = row_select[row]; /* drive the active row low */
		delayMs(2); /* wait for signal to settle */
		col = PTC->PDIR & 0xF0; /* read all columns */
		if (col != 0xF0) break; /* if one of the input is low, some key is pressed. */
	}
	PTC->PDDR = 0; /* disable all rows */
	if (row == 4)
		return 0; /* if we get here, no key is pressed */
	/* gets here when one of the rows has key pressed, check which column it is */
	if (col == 0xE0) return row * 4 + 1; /* key in column 0 */
	if (col == 0xD0) return row * 4 + 2; /* key in column 1 */
	if (col == 0xB0) return row * 4 + 3; /* key in column 2 */
	if (col == 0x70) return row * 4 + 4; /* key in column 3 */
	return 0; /* just to be safe */
}

void keypad_init(void)
{
	SIM->SCGC5 |= 0x0800; /* enable clock to Port C */
	PORTC->PCR[0] = 0x103; /* make PTC0 pin as GPIO and enable pullup*/
	PORTC->PCR[11] = 0x103; /* make PTC1 pin as GPIO and enable pullup*/
	PORTC->PCR[10] = 0x103; /* make PTC2 pin as GPIO and enable pullup*/
	PORTC->PCR[3] = 0x103; /* make PTC3 pin as GPIO and enable pullup*/
	PORTC->PCR[4] = 0x103; /* make PTC4 pin as GPIO and enable pullup*/
	PORTC->PCR[5] = 0x103; /* make PTC5 pin as GPIO and enable pullup*/
	PORTC->PCR[6] = 0x103; /* make PTC6 pin as GPIO and enable pullup*/
	PORTC->PCR[7] = 0x103; /* make PTC7 pin as GPIO and enable pullup*/
	PTC->PDDR = 0xC09; /* make PTC7-4 as input pins, PTC3-0 as outputs */
}

uint8_t UART0_Receive_Poll(void) {
	while (!(UART0->S1 & UART_S1_RDRF_MASK)); // wait until receive data register is full
	return UART0->D;
	}

void UART0_Transmit_Poll(uint8_t data) {
		while (!(UART0->S1 & UART_S1_TDRE_MASK)); // wait until transmit data register is empty
		UART0->D = data;
	}
