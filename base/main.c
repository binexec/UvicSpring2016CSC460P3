#include "shared.h"
#include "base_declarations.h"

// read ch and filter it into 7 levels
// within low and high threshold is 0
// higher than 1000, lower than 24 is high speed 2/-2
// in between the threshold and high speed is low speed, 1/-1
char readAndFilter(uint8_t ch)
{
	uint16_t num = readadc(ch);
	if (num < THRESHOLD_1)
		return (NEGATIVE_HIGH);
	else if (num < THRESHOLD_2)
		return (NEGATIVE_LOW);
	else if (num < THRESHOLD_3)
		return (NOT_MOVING);
	else if (num < THRESHOLD_4)
		return (POSITIVE_LOW);
	else
		return (POSITIVE_HIGH);	
}

void readAndSend()
{
	char direction = readAndFilter(0);   // read ch 0 which is to Y of the joystick
	char speed = readAndFilter(1);		// read ch 1 which is the X of the joystick
	char fire;
	
	if (PINB & (1<<PB1))
		fire = HOLD;
		//PORTB &= ~(1<<PB2);	//pin 51 off	
	else
		fire = FIRE;
		//PORTB |= (1<<PB2);	//pin 51 on
		
	uart0_sendbyte(CMD_PREAMBLE);
	uart0_sendbyte(direction);
	uart0_sendbyte(speed);
	uart0_sendbyte(fire);
	uart0_sendbyte(CMD_POSTAMBLE);
}

int main(void)
{
	DDRB &= ~(1<<PB1);  // set pin 52 to input
	PORTB |= (1<<PB1);  // enable pull up
	DDRB |= (1<<PB2);	// pin 51 as output
	
	uart0_init();
	InitADC();

	while (1)
	{
		readAndSend();
		_delay_ms(100);
		
		// result: x mid position is 504
		//         y mid position is 518
		// range from 0 - 1024
	}
}