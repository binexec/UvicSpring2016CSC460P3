#include <util/delay.h>
#include <avr/io.h>
#include "uart/uart.h"

//Special case values for radius, used for drive
#define DRIVE_STRAIGHT			0x7FFF			//32767
#define CLOCKWISE_TURN			0xFFFF			//-1
#define COUNTER_CLOCKWISE_TURN	0x1				//1

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

void start_robot_safe()
{
	uart_sendbyte(128);
	uart_sendbyte(131);
}

void drive(int16_t vel, int16_t rad)
{
	//Making sure velocity is within valid range
	if(vel < -500) 
		vel = -500;
	else if (vel > 500)
		vel = 500;
	
	//Making sure radius is within valid range
	if(rad < -2000)
		rad = -2000;
	else if(rad > 2000 && rad != DRIVE_STRAIGHT)	//32767 and 32768 are special cases to drive straight
		rad = 2000;
	
	uart_sendbyte(137);				//Opcode for drive
	uart_sendbyte(vel >> 8);		//Velocity high byte
	uart_sendbyte(vel);				//velocity low byte
	uart_sendbyte(rad >> 8);		//Radius high byte
	uart_sendbyte(rad);				//Radius low byte
	
}

void beep()
{		
	/*Example from:
	http://www.robotappstore.com/Knowledge-Base/4-How-to-Send-Commands-to-Roomba/18.html */
		
	//Write a "song" for the beep
	uart_sendbyte(140);
	uart_sendbyte(0);
	uart_sendbyte(1);
	uart_sendbyte(62);
	uart_sendbyte(32);
	
	//Play the beep "song"
	uart_sendbyte(141);
	uart_sendbyte(0);
}

int main()
{
	uart_init();
	switch_uart_19200();
	start_robot_safe();
	
	while(1)
	{
		beep();
		drive(100,DRIVE_STRAIGHT);
		_delay_ms(2000);
		beep();
		drive(100,CLOCKWISE_TURN);
		_delay_ms(2000);
	}
	
	return 0;
}


