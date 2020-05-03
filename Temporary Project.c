#include <MKL25Z4.H>
#define RS 1 /* BIT0 mask */
#define RW 2 /* BIT1 mask */
#define EN 4 /* BIT2 mask */
void delayMs(int n);
void keypad_init(void);
unsigned char keypad_getkey(void);
unsigned char keypress;
unsigned char LCDdecode(int);
void LCD_nibble_write(unsigned char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);

unsigned char LCDpattern[] = {0x0, 'A', '3', '2', '1', 'B', '6', '5', '4', 'C', '9', '8', '7', 'D', 'E', '0', 'F'};

int main(void)
{
	keypad_init();
	LCD_init();



	LCD_command(1); /* clear display */
	delayMs(1);
	LCD_command(0x80); /* sets first value at the beginning of the LCD */
	delayMs(1);
	unsigned char key;
	for(;;)
	{
		key = keypad_getkey();
		//PTD->PDOR = decode(0); /* changed to 0 as test, it was originally store[0] which should be null */
		//PTE->PSOR = 0x01; /* turn of 1st digit */
		//PTE->PCOR = 0x0E;
		delayMs(5);
		//PTD->PDOR = decode(store[1]);
		//PTE->PSOR = 0x02; /* turn on 2nd digit */
		//PTE->PCOR = 0x0D;
		//delayMs(5);
		//PTD->PDOR = decode(store[2]);
		//PTE->PSOR = 0x04; /* turn off 3rd digit */
		//PTE->PCOR = 0x0B;
		//delayMs(5);
		//PTD->PDOR = decode(store[3]);
		//PTE->PSOR = 0x08;
		//PTE->PCOR = 0x07;
		//delayMs(5);
		if(key!= 0) /* checks if a value has been entered */
		{
			LCD_command(1); /* clear display */
			delayMs(1);
			LCD_command(0x80);
			delayMs(1);
			//store[3] = store[2];
			//store[2] = store[1];
			//store[1] = store[0];
			//store[0] = key;
			//for(int n = 0; n < 4; n++)
			//{
				//PTD->PDOR = decode(key); /* drive pattern of 2 on the segments

				//PTE->PSOR = 0x01; /* turn of 1st
				//digit */
				//PTE->PCOR = 0x0E;
				//delayMs(5);
				//PTD->PDOR = decode(store[1]);
				//PTE->PSOR = 0x02; /* turn on 2nd digit */
				//PTE->PCOR = 0x0D;
				//delayMs(5);
				//PTD->PDOR = decode(store[2]);
				//PTE->PSOR = 0x04; /* turn off 3rd digit */
				//PTE->PCOR = 0x0B;
				//delayMs(5);
				//PTD->PDOR = decode(store[3]);
				//PTE->PSOR = 0x08;
				//PTE->PCOR = 0x07;
				//delayMs(5);
			//}
			//for(int j = 0 ; j<4; j++)
			//{
				LCD_data(LCDdecode(key));
				delayMs(10);
			//}
			//keypress = LCDdecode(key);
			//UART0_Transmit_Poll(keypress);
		}
	}
}

void delayMs(int n) {
	int i;
	int j;
	for(i = 0 ; i < n; i++)
		for (j = 0; j < 7000; j++) {}
}

void keypad_init(void)
{
	SIM->SCGC5 |= 0x0800; /* enable clock to Port C */
	PORTC->PCR[0] = 0x103; /* make PTC0 pin as GPIO and enable pullup*/
	PORTC->PCR[1] = 0x103; /* make PTC1 pin as GPIO and enable pullup*/
	PORTC->PCR[2] = 0x103; /* make PTC2 pin as GPIO and enable pullup*/
	PORTC->PCR[3] = 0x103; /* make PTC3 pin as GPIO and enable pullup*/
	PORTC->PCR[4] = 0x103; /* make PTC4 pin as GPIO and enable pullup*/
	PORTC->PCR[5] = 0x103; /* make PTC5 pin as GPIO and enable pullup*/
	PORTC->PCR[6] = 0x103; /* make PTC6 pin as GPIO and enable pullup*/
	PORTC->PCR[7] = 0x103; /* make PTC7 pin as GPIO and enable pullup*/
	PTC->PDDR = 0x0F; /* make PTC7-4 as input pins, PTC3-0 as outputs */
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
//unsigned char decode(int n)
//{/
//	return digitPattern[n];
//}
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
