Project Goals

Using the FRDM board and Rex Qualis Project Starter Kit, create a project that can cycle between 5 modes that are as follows:

Mode 1: ​Control the position of the servo using the potentiometer
Mode 2: ​Control the speed of the DC motor using the potentiometer.
Mode 3: ​Control the speed of the servo while it scans 180 degrees using Putty
Mode 4: ​Control the position of the servo using Putty
Mode 5: ​Control the speed of the DC motor using Putty
For both Mode 2 and Mode 5, an ultrasonic sensor should be used to detect whether an object is within certain distances of the motor, and activate a corresponding LED and siren with respect to the object’s distance. The current mode should be chosen using the matrix keyboard, and then displayed simultaneously on both the LCD display and Putty. Also most modes here used variations of the ADC0_PWM.zip file from the course documents.

![Diagram of board](photos/diagram%20of%20board.png)

Project Description

Peripheral: ​The most important thing we needed to get working for the project is the UART communication to the PC, as the code for both the LCD display and the matrix keyboard could be modified versions of what we had written for project 4. To begin, we initialized UART0 at a clock rate of 48 MHz using the two following lines:

SIM -> SCGC4

SIM -> SOPT2

To make UART work at this clock rate, we set the baud rate to 115200. Using this established UART, we created three functions. The first is UART0_Receive_Poll, which takes in a single character by waiting until the UART receive data register is full, and then sends the character it gets. The next two are UART0_puts and UART0Tx. The two work in tandem to pass full strings through UART, which happens by breaking up the string in UART0_puts into single characters, and then sending them through UART0Tx by waiting until the transmit data register is empty and then sending the character. The advantage to coding it in this way is that UART stays the same throughout the many modes.


Mode 1: To activate the servo properly, we need to configure PWM and ADC to run at the boot
clock rate of 48 MHz. For the PWM, we started by initializing the clock to both port C and TMP0,
since Port C2 is used by TPM0. We additionally set TPM0 to use a timer at 48 MHz, before
enabling TPM0 channel 1 as an edge aligned PWM. The mod value of the timer needed to be
usable for 50 Hz, so we did the following calculation to figure out what value needed to be used:
48 MHz / 16 = 3 MHz
3,000,000 Hz / 50 Hz = 60,000.
So the mod value was set to 60,000. The CNV needed to be set to 1,500, as this will give a duty
cycle of 2.5%. To finish the PWM, we enabled the timer to have a prescaler of 16. For our ADC,
we began by initializing ADC0 by enabling the clock to Port E (our analog input). A few steps
were then needed to properly enable ADC0, and were as follows: use the bus clock as an input
clock, which was the boot clock divided by 2, or 24 MHz, run at 6 MHz (see calculation below),
have a long sample time, and use single ended 12-bit conversion.
Bus clock / 4 = (24 MHz) / 4 = 6 MHz.
We then had the ADC conversion work off of interrupts by using the ADC complete interrupt,
which first required the ADC interrupt to be enabled. This was done through the ADC0->SC1
register. We initialized the ADC to enable the ADC0 interrupt through the NVIC->ISER register
which is interrupt number 15. In addition we had an ADC0 interrupt handler which would then
set the CnV value for the servo motor. It is important to note that we needed to start the
conversion at the end of this handler again to get it working right. Once the interrupt was
enabled, the corresponding interrupt register for ADC0 needed to be enabled as well. We then
had an ADC0 interrupt handler function which would take in the result from the ADC conversion
and then change the value of the PWM’s timer CNV corresponding to the given voltage coming
in. At the end of this function we would give the ADC0 a software trigger so that it would start
the conversion again.



figure 7 and 8


Mode 2: This mode uses the same code for the PWM and ADC0 as mode 1. This code uses an
ADC complete interrupt, since interrupts were needed for the ultrasonic sensor used. First, we
needed to configure the DC motor. All that was needed to change from mode 1 was that we
changed the CnV value for the timer used with the PWM to the result of the ADC conversion *
14, since this will appropriately scale the CnV between a 0% and 93% duty cycle. This mode
once again used the ADC0 complete interrupt. With the DC motor configured properly, we then
moved on to the ultrasonic sensor. This needed its own timer, since it needed to run with a
modular register of 45,000. To get this working, we used TPM2 channel 1, which ran through
pin PTE23. This timer still used the 48 MHz clock, but the prescale value was adjusted to 64 so
that the PWM had a 10 microsecond pulse width. The ultrasonic sensor also ran off of interrupts
so we had to enable interrupt 19 since that enabled the TPM2 interrupt. Much like the complete
interrupt we had we also have a TPM2 interrupt handler. This estimates the echo pulse after 2
interrupts. By using this echo pulse, we can calculate the distance between some object and the
ultrasonic sensor. The equation used to find the distance is:
distance = (pulsewidth * 4/3)/2 * (.0343)
0.0343 was used since that is the speed of sound in cm/μs. This would give us the distance in
cm, and we would just need to convert to inches by doing:
distance = distance/2.54
We used if statements to see if this distance was less than 12 inches and between 12 and 24
inches. When the distance was between 24 and 12, we turned on a blue LED (PTD1) and
activated a buzzer. When the distance was less than 12 inches, the blue LED was turned off
and a red LED was turned on (PTB18). We also turned off the DC motor by setting the
corresponding timer and channel CnV value = 0. When the distance was not in this range the
motor would operate normally without any LEDs turning on. The last thing we needed to
configure was the buzzer. To get this to work, we had to activate a separate timer and channel.
The only timer that was left was TPM1 and we used channel 0, since it would correspond with
PTA12. We initialized our timer in the Buzzer_Timer_Init function so that it would have a 48 MHz
clock, be edge aligned, have a modular register of 60,000 and a duty cycle of 50%. We wanted
this buzzer to range between 500 Hz and 3,000 Hz when the distance was between 12 inches
and 24 inches. To do this, we set the timer’s mod value to the following equation:
Mod = 6000 - (420 * (distance-12))
Since the timer needed to go high and low we had to have the mod value go between 1000 and
6000 since that is the double of 500 and 3000. The above equation would range between those
values, and then set the corresponding timers duty cycle to 50% by dividing the mod value in
half.


figures 9 and 10


Figures 9 and 10: Operation of Mode 2
Mode 3: A majority of this mode’s function was already programmed in mode 1, and therefore is
reincorporated here. The main difference is the ability for the user to enter a scan speed using
UART. The function UART0_Recieve_Poll was used once again, however this time the received
value was subtracted by 48 to make a “scale” and to get around the function’s limitation of only
being able to accept chars. The value 48 was picked as it would change the received speed’s
ASCII value to its corresponding numerical value. The two while loops in the code increment the
variable result by 50 times the scale value that the user input. This resulting number is set
equal to the CnV of the timer and channel being used for the PWM. One while loop runs while
result is less than 7500 (12.5% duty cycle) and the other runs while result is greater than 1500
(2.5% duty cycle).

Mode 4: As in mode 3, a lot of code from mode 1 can be reincorporated to make the servo
motor function. An additional function needs to be added called getInput, which asks a user to
enter an integer from 0 to 180 through UART which is the angle to which the servo will move to.
The input has to be a 3 digit number, meaning if the user wishes to make the angle 45, they
would need to type 045. This is because UART0_Recieve_Poll can only take in a single char,
so we once again had to adjust based on the ASCII value of the char before adding it to an
array of length 3. Once all three digits of the array are filled, we have the int value that the user
requested. To translate this number to the required angle, we multiply the first value of the array
by 100, add it to the second value times 10, then add the final element. This value is then
multiplied by 33 and added to 1500 to get a CnV value that represents the angle. The last step
is to set this CnV value to the corresponding register for the servo’s CnV value.

Mode 3 and Mode 4 running together: We worked on mode 3 and 4 working together to
showcase the ability of our matrix keyboard and LCD screen. Our keyboard would take an input
from the user to draw a distinction between modes 3 and 4. We first had to initialize the
keyboard since it had to use ports C0, C11, C10 and C3 as output pins and C4, C5, C6 and C7
as input pins. We then figured out what key was pressed by using the function keypad_getkey,
which would scan through the rows and columns of the keyboard to see which key was pressed.
In our main while loop, we would call the function keypad_getkey to see if the user input a
value, and if the key press was 3, it would go into the function mode3, which would then perform
mode 3. If the key press was 4, then it would go into the function mode4, and execute mode 4.
All the while, whatever input the user gives would appear on the LCD screen. We initialized the
pins B0, B1, B2, B8, B9, B10 and B11 as GPIO output pins. We used the function LCD_data to
send the input from the keypad to the LCD screen.


figure 11 - 17

Mode 5: Mode 5 reincorporates a majority of the code utilized in mode 2, as both serve similar
functions. The main difference between the two is that mode 5 uses UART to ask the user to
input a motor speed. This follows a similar process as mode 3, but without any while loops
incrementing the CnV value. All we had to do was use UART0_Receive_Poll to get a char,
subtract 48 from that character to get the numerical value (scale) and do the following equation
6,200 * scale
This equation works since the highest number a user can input is 9 and since 6,200 * 9 =
55,800 (which is close to the 93% duty cycle for the motor)




Mode Switching: ​To get all modes working together to showcase the ability of our matrix keyboard and LCD screen. Our keyboard would take an input from the user to draw a distinction between modes. We first had to initialize the keyboard since it had to use ports C0, C11, C10 and C3 as output pins and C4, C5, C6 and C7 as input pins. We then figured out what key was pressed by using the function keypad_getkey, which would scan through the rows and columns of the keyboard to see which key was pressed. In our main while loop, we would call the function keypad_getkey to see if the user input a value, and if the key press was 3, 4 or 5 it would go into the corresponding function (mode3, mode4 or mode5, respectively), which would then perform the mode’s function. Since modes 1 and 2 relied on ADC0 conversion, we had to use a different strategy. Because they operated on an ADC0 complete interrupt, we instead had an int variable called mode which would be set to either 1 or 2 depending on the keypress. These modes would instead be carried out in the ADC0 interrupt handler function. To do this we had if statements that would activate the right timer channel for the servo if the mode was 1, and a similar if statement that would activate the channel for the DC motor if the mode was 2. In this handler function we also had to add another key = keypad_getkey() to see if a new input was made by the user on the matrix keyboard. All the while, whatever input the user gives would appear on the LCD screen. We initialized the pins B0, B1, B2, B8, B9, B10 and B11 as GPIO output pins. We used the function LCD_data to send the input from the keypad to the LCD screen.


figures 18 - 21

