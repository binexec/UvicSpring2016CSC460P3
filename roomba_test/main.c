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
	uart_sendbyte(128);		//Send START command
	uart_sendbyte(131);		//Switch to SAFE mode
}

void roomba_init();
{
	switch_uart_19200();
	start_robot_safe();
	
	//Write a "song" for the beep into slot 0
	//Example from: http://www.robotappstore.com/Knowledge-Base/4-How-to-Send-Commands-to-Roomba/18.html
	uart_sendbyte(140);
	uart_sendbyte(0);
	uart_sendbyte(1);
	uart_sendbyte(62);
	uart_sendbyte(32);
}

void beep()
{		
	//Play the beep "song" created in roomba_init
	uart_sendbyte(141);
	uart_sendbyte(0);
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

void query_sensors()
{
	uart_sendbyte(149);				//Opcode for Query List
	uart_sendbyte(4);				//Query will send three sensors packets
	uart_sendbyte(7);				//Packet 7: Bump/Wheeldrop detection
	uart_sendbyte(8);				//Packet 8: Wall seen?
	uart_sendbyte(27);				//Packet 27: Strength of wall signal
	uart_sendbyte(13);				//Packet 13: Virtual wall seen?
}

void send_query_list()
{
	
	
}

int main()
{
	uart_init();
	roomba_init();
	
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


