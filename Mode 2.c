
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
void Buzzer_Timer_Init(void);
void ADC0_IRQHandler(void);
void ADC0_init(void);
void PWM_init(void);
void LED_init(void);
void LED_set(int s);
void UART0_init(void);
void UART0Tx(char c);
void UART0_puts(char* s);
volatile uint32_t pulse_width;
volatile float distance;
// https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
char mode;
int tmpInt1;
float tmpFrac;
int tmpInt2;
volatile uint32_t cont[2];
short int result;
char buffer[30];

int i = 0;
#define mod 44999  //14999


int main(void) {
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
    while (1) {
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

void ADC0_IRQHandler(void){
	int voltage;
	result = ADC0->R[0];
	TPM0->CONTROLS[1].CnV = result*14;
	voltage = result * 3300/4096;
	sprintf(buffer, "\r\n voltage = %d mV", voltage);
	UART0_puts(buffer);
	delayMs(10);

	ADC0->SC1[0] &= ~0x17;

}

void Buzzer_Timer_Init(void){
	SIM->SCGC5 |= 0x200;		//enable clock to port A
	PORTA->PCR[12] = 0x0300; 	//PTA12 used by TPM1
	SIM->SCGC6 |= 0x02000000;	//enable clock to TPM1
	SIM->SOPT2 |= 0x01000000;
	TPM1->SC = 0;               		  /* disable timer */
	//    TPM0->CONTROLS[1].CnSC = 0x20|0x08;   /* edge-aligned PWM, pulse high MSB:MSA=10, ELSB:ELSA=10*/
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH1 as edge-aligned PWM
	//TPM0->CONTROLS[3].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH1 as edge-aligned PWM
	TPM1->MOD = 60000;          		  /* Set up modulo register for 50 Hz - 48.00 MHz */
	TPM1->CONTROLS[0].CnV = TPM1->MOD/2;  		  /* Set up channel value for 2.5% duty-cycle */
	TPM1->SC |= 0x0C;
}


void PWM_init(void)
{
    SIM->SCGC5 |= 0x800;       		   	   /* enable clock to Port C*/
    //SIM->SCGC5 |= 0x1000;					//enable clock to Port D
    PORTC->PCR[2] = 0x0400;     		   /* PTC2 used by TPM0 */
    PORTD->PCR[3] = 0x0400; 				//PTD3 used by TPMO channel 3
    SIM->SCGC6 |= 0x01000000;   		   /* enable clock to TPM0 */
    SIM->SOPT2 |= 0x01000000;   		   /* use MCGFLLCLK as timer counter clock */

    TPM0->SC = 0;               		  /* disable timer */
//    TPM0->CONTROLS[1].CnSC = 0x20|0x08;   /* edge-aligned PWM, pulse high MSB:MSA=10, ELSB:ELSA=10*/
    TPM0->CONTROLS[1].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH1 as edge-aligned PWM
    //TPM0->CONTROLS[3].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH1 as edge-aligned PWM
    TPM0->MOD = 60000;          		  /* Set up modulo register for 50 Hz - 48.00 MHz */
    TPM0->CONTROLS[1].CnV = 1500;  		  /* Set up channel value for 2.5% duty-cycle */
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
    NVIC->ISER[0] |= 0x8000;
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
    	    TPM1->CONTROLS[0].CnV = 0;
    	}
    	else if(distance < (float)24. && distance > (float)12.0){
    		PTB->PSOR |= 0x40000;		//turn off red LED
    		PTD->PCOR = 0x02;
    		TPM1->MOD = 6000 - (420* distance);
			TPM1->CONTROLS[0].CnV = TPM1->MOD/2;/* turn on blue LED */

    	}
    	else
    	    PTB->PSOR |= 0x40000;	/* Set PTB18 to turn off red LED */
    		TPM1->CONTROLS[0].CnV = 0;
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

