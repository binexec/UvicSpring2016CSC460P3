#include <util/delay.h>
#include <avr/io.h>
#include "uart/uart.h"

#define SENSORS_TO_QUERY 2		//How many sensors are we querying from the robot?
#define TWO_BYTE_SENSORS 1		//How many of the sensors above returns 2 bytes of data instead of 1?

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
	uart1_sendbyte(128);		//Send START command
	uart1_sendbyte(131);		//Switch to SAFE mode
}

void roomba_init()
{
	switch_uart_19200();
	start_robot_safe();
	
	//Write a "song" for the beep into slot 0
	//Example from: http://www.robotappstore.com/Knowledge-Base/4-How-to-Send-Commands-to-Roomba/18.html
	uart1_sendbyte(140);
	uart1_sendbyte(0);
	uart1_sendbyte(1);
	uart1_sendbyte(62);
	uart1_sendbyte(32);
}

void beep()
{		
	//Play the beep "song" created in roomba_init
	uart1_sendbyte(141);
	uart1_sendbyte(0);
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
	
	uart1_sendbyte(137);				//Opcode for drive
	uart1_sendbyte(vel >> 8);			//Velocity high byte
	uart1_sendbyte(vel);				//velocity low byte
	uart1_sendbyte(rad >> 8);			//Radius high byte
	uart1_sendbyte(rad);				//Radius low byte
}

void query_sensors()
{
	uart1_sendbyte(149);				//Opcode for Query List
	uart1_sendbyte(SENSORS_TO_QUERY);	//Query will send three sensors packets
	//uart1_sendbyte(7);					//Packet 7: Bump/Wheeldrop detection
	uart1_sendbyte(8);					//Packet 8: Wall seen?
	uart1_sendbyte(27);					//Packet 27: Strength of wall signal (Two bytes)
	//uart1_sendbyte(13);					//Packet 13: Virtual wall seen?
}

void send_query_list()
{
	uint8_t i;
	uint8_t curbyte;
	
	for(i=0; i<SENSORS_TO_QUERY + TWO_BYTE_SENSORS; i++)
	{
		curbyte = uart1_recvbyte();		//Read a byte returned by the robot
		uart0_sendbyte(curbyte);		//Forward the byte back to the base station via BT 
	}
}

int main()
{
	uart0_init();		//UART0 is used for BT
	uart1_init();		//UART1 is used to communicate with the robot
	roomba_init();
	
	uart0_sendstr("Robot initialized!\n");
	
	while(1)
	{
		//beep();
		drive(100,DRIVE_STRAIGHT);
		_delay_ms(2000);
		//beep();
		drive(100,CLOCKWISE_TURN);
		_delay_ms(2000);
		query_sensors();
		_delay_ms(1);
		send_query_list();
	}
	
	return 0;
}


