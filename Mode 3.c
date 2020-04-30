
#include "MKL25Z4.h"
#include "board.h"
void ADC0_init(void);
void PWM_init(void);
void LED_init(void);
void LED_set(int s);
void UART0_puts(char* s);
void UART0Tx(char c);
uint8_t UART0_Receive_Poll(void);
void UART0_Transmit_Poll(uint8_t);
void UART0_init(void);
void delayMs(int);
int result;

int main(void) {

	BOARD_BootClockRUN();
    //BOARD_InitBootClocks();

    LED_init();                     /* Configure LEDs */
    ADC0_init();                    /* Configure ADC0 */
    PWM_init();                     /* Configure PWM */
    result = 0;
    UART0_init();
    int scale;
    char a;
    while (1) {
        ADC0->SC1[0] = 0;     /* start conversion on channel 0 */
        UART0_puts(" Enter a scan speed between 1-9: ");
        a = UART0_Receive_Poll();
        delayMs(2);
        UART0_Transmit_Poll(a);
        scale = a-48;

        //while(!(ADC0->SC1[0] & 0x80)) { } /* wait for conversion complete */
        //result = ADC0->R[0];
        while(result < 7500){

        	result += 50;

        	TPM0->CONTROLS[1].CnV = result * scale;
        }
        while(result > 1500){
        	result -= 50;

        	TPM0->CONTROLS[1].CnV = result *scale;
        }
        /* read conversion result and clear COCO flag */

//      For LED
        //LED_set(result >> 7);       /* display result on LED with bits 2, 1, & 0 after shift */

//      For Servo
        //TPM0->CONTROLS[1].CnV =result;  /* Set up channel value between 2.5% and 12.5%*/

//      For the Buzzer
        //TPM0->MOD = 6000 - 5*(result/4);      /* Frequencies from 500 Hz to 3000 Hz*/
       // TPM0->CONTROLS[1].CnV = TPM0->MOD/2;  /* Set up channel value between 50%*/

//      For DC motor
        //TPM0->CONTROLS[1].CnV = result*14;  /* Set up channel value between 0% - 93%*/
    }
}

void PWM_init(void)
{
    SIM->SCGC5 |= 0x800;       		   	   /* enable clock to Port C*/
    PORTC->PCR[2] = 0x0400;     		   /* PTC2 used by TPM0 */
    SIM->SCGC6 |= 0x01000000;   		   /* enable clock to TPM0 */
    SIM->SOPT2 |= 0x01000000;   		   /* use MCGFLLCLK as timer counter clock */

    TPM0->SC = 0;               		  /* disable timer */
//    TPM0->CONTROLS[1].CnSC = 0x20|0x08;   /* edge-aligned PWM, pulse high MSB:MSA=10, ELSB:ELSA=10*/
    TPM0->CONTROLS[1].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH1 as edge-aligned PWM
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
    PTB->PDDR |= 0x40000;       /* make PTB18 as output pin */
    PORTB->PCR[19] = 0x100;     /* make PTB19 pin as GPIO */
    PTB->PDDR |= 0x80000;       /* make PTB19 as output pin */
    PORTD->PCR[1] = 0x100;      /* make PTD1 pin as GPIO */
    PTD->PDDR |= 0x02;          /* make PTD1 as output pin */
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

void UART0_puts(char* s) {
	while (*s != 0)         /* if not end of string */
	UART0Tx(*s++);      /* send the character through UART0 */
}

void UART0Tx(char c) {
	    while(!(UART0->S1 & 0x80)) {
	    }   /* wait for transmit buffer empty */
	    UART0->D = c; /* send a char */
}

void UART0_Transmit_Poll(uint8_t data) {
		while (!(UART0->S1 & UART_S1_TDRE_MASK)); // wait until transmit data register is empty
		UART0->D = data;
	}

uint8_t UART0_Receive_Poll(void) {
	while (!(UART0->S1 & UART_S1_RDRF_MASK)); // wait until receive data register is full
	return UART0->D;
	}

void UART0_init(void) {
    SIM->SCGC4 |= SIM_SCGC4_UART0(1);            //0x0400;    /* enable clock for UART0 */
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);         //0x04000000 /* use FLL output for UART Baud rate generator */
    UART0->C2   = 0;                             //0x00       /* turn off UART0 while changing configurations */
    UART0->BDH  = UART0_BDH_SBR(0);              //0x00;      /* SBR12:SBR8 = 0b00000    - 115200 Baud
    UART0->C4   = UART0_C4_OSR(15);              //0x0F;      /* Over Sampling Ratio (15+1) */
//	    UART0->BDL  = UART0_BDL_SBR(137);            //0x89;      /* SBR7:SBR0  = 0b10001001 -   9600 Baud - Clock = 20.97 MHz*/
//	    UART0->BDL  = UART0_BDL_SBR(12);             //0x0C;      /* SBR7:SBR0  = 0b00001100 - 115200 Baud - Clock = 20.97 MHz*/
    UART0->BDL  = UART0_BDL_SBR(26); 			 //0x1A;      /* SBR7:SBR0  = 0b00011010 - 115200 Baud - Clock = 48.00 MHz */
    UART0->C1   = UART0_C1_M(0);                 //0x00;      /* 8-bit data */
    UART0->C2   = UART0_C2_TE(1)|UART0_C2_RE(1); //|=0x0C;    /* enable transmit & Receive*/

    SIM->SCGC5     = SIM_SCGC5_PORTA(1);         //|= 0x0200; /* enable clock for PORTA */
    PORTA->PCR[2]  = PORT_PCR_MUX(2);            //0x0200;    /* make PTA2 UART0_Tx pin */
    PORTA->PCR[1]  = PORT_PCR_MUX(2);            //0x0200;    /* make PTA1 UART0_Rx pin */
}

void delayMs(int n) {
	    int i, j;
	    for(i = 0 ; i < n; i++)
	        for (j = 0; j < 3500; j++) {}
	}
