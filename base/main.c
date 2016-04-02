#include <util/delay.h>
#include <avr/io.h>
#include "uart/uart.h"
#include "adc/adc.h"

// use Y on ch 0 as left and right
// use X on ch 1 as forward and backward
// cords are facing you

//Special case values for speed, used in reading joystick
#define NEGATIVE_HIGH			0x0		//reverse in high speed/or turn left high speed
#define NEGATIVE_LOW			0x1		//reverse in low speed
#define NOT_MOVING				0x2
#define POSITIVE_LOW			0x3
#define POSITIVE_HIGH			0x4

#define THRESHOLD_1				50		//lower than threshold 1 is NEGATIVE_HIGH
#define THRESHOLD_2				482		//between threshold 1 is NEGATIVE
#define THRESHOLD_3				542		//not moving
#define THRESHOLD_4				974		// lower then P_low higher then P_high



void switch_uart_19200()
{
	DDRB |= (1<<PB0);		//Use ping 53 for BRC
	PORTB |= (1<<PB0);		//Initialize BRC as high
	_delay_ms(2000);
	
	//Pulse BRC three times
	PORTB &= ~(1<<PB0);
	_delay_ms(100);
	PORTB |= (1<<PB0);
	_delay_ms(100);
	
	PORTB &= ~(1<<PB0);
	_delay_ms(100);
	PORTB |= (1<<PB0);
	_delay_ms(100);
	
	PORTB &= ~(1<<PB0);
	_delay_ms(100);
	PORTB |= (1<<PB0);
}
/*
void InitADC(void)
{
	ADMUX|=(1<<REFS0);
	ADCSRA|=(1<<ADEN)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); //ENABLE ADC, PRESCALER 128
}
uint16_t readadc(uint8_t ch)
{
	ch&=0b00000111;         //ANDing to limit input to 7
	ADMUX = (ADMUX & 0xf8)|ch;  //Clear last 3 bits of ADMUX, OR with ch
	ADCSRA|=(1<<ADSC);        //START CONVERSION
	while((ADCSRA)&(1<<ADSC));    //WAIT UNTIL CONVERSION IS COMPLETE
	return(ADC);        //RETURN ADC VALUE
}
*/
// example from: http://www.embedds.com/interfacing-analog-joystick-with-avr/

// read ch and filter it into 7 levels
// within low and high threshold is 0
// higher than 1000, lower than 24 is high speed 2/-2
// in between the threshold and high speed is low speed, 1/-1
uint8_t readAndFilter(uint8_t ch)
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
	uint8_t direction = readAndFilter(0);   // read ch 0 which is to Y of the joystick
	uint8_t speed = readAndFilter(1);		// read ch 1 which is the X of the joystick
	//char a,b;
	//a = direction  + '0';
	//b = speed + '0';
	uart0_sendbyte('$');
	uart0_sendbyte(direction);
	uart0_sendbyte(speed);
	uart0_sendbyte('#');
}

int main(void)
{
	uart0_init();
	InitADC();

	uint16_t num;
	while (1)
	{
		/*num = readadc(0);
		itoa(num, a, 10);
		printf("adc0: %s\n", a);
		printf("%d\n",num);
		num = readadc(1);
		itoa(num, a, 10);
		printf("adc1: %s\n", a);
		*/
		readAndSend();
		_delay_ms(100);

		// result: x mid position is 504
		//         y mid position is 518
		// range from 0 - 1024
	}
}