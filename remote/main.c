#include <util/delay.h>
#include <avr/io.h>
#include "uart/uart.h"
#include "adc/adc.h"
#include "rtos/os.h"
#include "rtos/kernel.h"

#define NEGATIVE_HIGH			'N'		//reverse in high speed/or turn left high speed
#define NEGATIVE_LOW			'n'		//reverse in low speed
#define NOT_MOVING				'z'
#define POSITIVE_LOW			'p'
#define POSITIVE_HIGH			'P'

//pins
#define PHOTORESIS_PIN   0		//photosensor pin on A0

#define SENSORS_TO_QUERY 2		//How many sensors are we querying from the robot?
#define TWO_BYTE_SENSORS 0		//How many of the sensors above returns 2 bytes of data instead of 1?

//Special case values for radius, used for drive
#define DRIVE_STRAIGHT			0x7FFF			//32767
#define CLOCKWISE_TURN			0xFFFF			//-1
#define COUNTER_CLOCKWISE_TURN	0x1				//1


#define MAX_CMD_LENG 4

//Command/OPcodes base station sends to remote
#define MOVE_CMD ((const char *) "!MV\n")
#define LASER_CMD ((const char *) "!TL\n")
#define HIT_CMD ((const char *) "!HT\n")

//Command/OPcodes remote station sends to base
#define SENSOR_DATA ((const char *) "!SD\n")

// gloable variable
volatile char direction;
volatile char speed;
volatile uint16_t photores_neutral;

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

void beep()
{
	//Play the beep "song" created in roomba_init
	uart1_sendbyte(141);
	uart1_sendbyte(0);
}

void roomba_init()
{
	//Default values
	direction = NOT_MOVING;
	speed = NOT_MOVING;
	
	switch_uart_19200();
	start_robot_safe();
	
	//Write a "song" for the beep into slot 0
	//Example from: http://www.robotappstore.com/Knowledge-Base/4-How-to-Send-Commands-to-Roomba/18.html
	uart1_sendbyte(140);
	uart1_sendbyte(0);
	uart1_sendbyte(1);
	uart1_sendbyte(62);
	uart1_sendbyte(32);
	beep();
}

void drive(int16_t vel, int16_t rad)
{
	//printf("vel: %d, rad: %d\n", vel, rad);
	
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

void move_as_global()
{
	int16_t vel;
	int16_t rad;

	while (1)
	{
		switch(direction)
		{
			case NEGATIVE_HIGH:
				rad = -1;
				break;
			case NEGATIVE_LOW:
				rad = 500;
				break;
			case POSITIVE_LOW:
				rad = 500;
				break;
			case POSITIVE_HIGH:
				rad = 1;
				break;
			case NOT_MOVING:
			default:
				rad = 0;
				break;
		}
	
		switch(speed)
		{
			case NEGATIVE_HIGH:
				vel = -500;
				break;
			case NEGATIVE_LOW:
				vel = -250;
				break;
			case POSITIVE_LOW:
				vel = 250;
				break;
			case POSITIVE_HIGH:
				vel = 500;
				break;
			case NOT_MOVING:
			default:
				vel = 0;
				break;
		}
	
		if (rad == 1 || rad == -1) {
			vel = 250;
		}
	
		drive(vel, rad);
		Task_Sleep(3);
	}
}


void handle_sensors()
{
	uint8_t i;
	uint8_t bytes[SENSORS_TO_QUERY + TWO_BYTE_SENSORS];
	
	//Roomba sends 8 bytes of unnecessary data, so we have to ignore them
	//for(i=0; i<7; i++) uart1_recvbyte();
	
	
	while(1)
	{
		/*Queries the sensors*/
		
		uart1_sendbyte(149);				//Opcode for Query List
		uart1_sendbyte(SENSORS_TO_QUERY);	//Query will send three sensors packets
		uart1_sendbyte(7);					//Packet 7: Bump/Wheeldrop detection
		uart1_sendbyte(13);					//Packet 13: Virtual wall seen?
	
		Task_Sleep(2);
	
		//Read sensor data returned by the roomba
		for(i=0; i<SENSORS_TO_QUERY + TWO_BYTE_SENSORS; i++)
			bytes[i] = uart1_recvbyte();		//Read a byte returned by the robot
	
		/*
			Currently, the data bytes for the following sensors will be returned in order:
				1: Bump/Wheeldrop
				2: Virtual wall detection 
			Note that after robot initialization, the robot will send 8 bytes of unneeded data back to the base station.
			Be sure to ignore these 8 bytes.
		*/
	
		/*BELOW SENSOR ARE INTENDED FOR MANUAL CONTROLS*/
	
		//If the left bumper has been hit, back up a bit and then rotate 90 degrees to the right
		if(bytes[0] == 1)
		{
			beep();
			drive(-200, DRIVE_STRAIGHT);
			_delay_ms(200);
			drive(200,COUNTER_CLOCKWISE_TURN);
			_delay_ms(1000);
			drive(0,0);
		}
		//If the right bumper has been hit, back up a bit and then rotate 90 degrees  to the right
		else if(bytes[0] == 2)
		{
			beep();
			drive(-200, DRIVE_STRAIGHT);
			_delay_ms(200);
			drive(200,CLOCKWISE_TURN);
			_delay_ms(1000);
			drive(0,0);
		}
		//If the middle has been hit or virtual wall has been detected, back up a bit and then rotate 180 degrees 
		//else if (bytes[0] == 3 || bytes[1] == 1)
		else if (bytes[0] == 3)
		{
			beep();
			drive(-200, DRIVE_STRAIGHT);
			_delay_ms(200);
			drive(200,COUNTER_CLOCKWISE_TURN);
			_delay_ms(2000);
			drive(0,0);
		}
	
		Task_Sleep(3);
	}
}

int receive_and_update()
{
	uint8_t curbyte;
	uint8_t count;
	while(1)
	{
		curbyte = uart0_recvbyte();
		count = 0;
		while (curbyte != '$')
		{
			if (++count > 6)
				goto receive_and_update_continue;
				
			curbyte = uart0_recvbyte();
		}
	
		direction = uart0_recvbyte();
		speed = uart0_recvbyte();
		
receive_and_update_continue:
		Task_Sleep(8);	
	}
}
void calibratePhotores()
{
	int i;

	//Sample the ambient lighting 10 times
	for(i=0; i<10; i++)
	{
		photores_neutral += readadc(PHOTORESIS_PIN);
		_delay_ms(200);
	}
	photores_neutral /= i;     //Use the average as neutral value

}

int isHit()
{
	uint16_t photores = readadc(PHOTORESIS_PIN);

	//Determine laser hits based on the brightness of the ambient lightint
	if(photores_neutral < 150)
		return photores > 3*photores_neutral;
	else if (photores_neutral < 250)
		return photores > 2*photores_neutral;
	else
		return photores > 1.2*photores_neutral;
}

void a_main()
{
	OS_Init();
	
	uart0_init();		//UART0 is used for BT
	uart1_init();		//UART1 is used to communicate with the robot
	roomba_init();
	beep();
	
	Task_Create(receive_and_update, 4, 0);
	Task_Create(move_as_global, 5, 0);
	Task_Create(handle_sensors, 3, 0);
	
	OS_Start();
}


