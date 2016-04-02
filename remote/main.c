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

#define SENSORS_TO_QUERY 4		//How many sensors are we querying from the robot?
#define TWO_BYTE_SENSORS 1		//How many of the sensors above returns 2 bytes of data instead of 1?

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
volatile photores_neutral;

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
	
	direction = NOT_MOVING;
	speed = NOT_MOVING;
}

void beep()
{		
	//Play the beep "song" created in roomba_init
	uart1_sendbyte(141);
	uart1_sendbyte(0);
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
		Task_Sleep(2);
	}
}

void query_sensors()
{
	uart1_sendbyte(149);				//Opcode for Query List
	uart1_sendbyte(SENSORS_TO_QUERY);	//Query will send three sensors packets
	uart1_sendbyte(7);					//Packet 7: Bump/Wheeldrop detection
	uart1_sendbyte(8);					//Packet 8: Wall seen?
	uart1_sendbyte(27);					//Packet 27: Strength of wall signal (Two bytes)
	uart1_sendbyte(13);					//Packet 13: Virtual wall seen?
}

void send_query_list()
{
	uint8_t i;
	uint8_t curbyte;
	
	//Sends preamble to base
	uart0_sendstr(SENSOR_DATA);
	
	for(i=0; i<SENSORS_TO_QUERY + TWO_BYTE_SENSORS; i++)
	{
		curbyte = uart1_recvbyte();		//Read a byte returned by the robot
		uart0_sendbyte(curbyte);		//Forward the byte back to the base station via BT 
	}
	
	/*
		Currently, the data bytes for the following sensors will be returned in order:
			1: Bump/Wheeldrop
			2: Wall detection
			3: Upper byte of wall strength
			4: Lower byte of wall strength
			5: Virtual wall detection 
			
		Note that after robot initialization, the robot will send 8 bytes of unneeded data back to the base station.
		Be sure to ignore these 8 bytes.
	*/
}

// return 0 if success, 1 otherwise
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
			if (++count > 6) {
				return (1);		// failed
			}
			curbyte = uart0_recvbyte();
		}
	
		direction = uart0_recvbyte();
		speed = uart0_recvbyte();
		Task_Sleep(3);	
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

int a_main()
{
	OS_Init();
	
	uart0_init();		//UART0 is used for BT
	uart1_init();		//UART1 is used to communicate with the robot
	roomba_init();
	beep();
	
	//uart0_sendstr("Robot initialized!\n");
	//uart_setredir();
	Task_Create(receive_and_update, 5, 0);
	Task_Create(move_as_global, 4, 0);
	/*
	while(1)
	{
		receive_and_update();
		move_as_global();
		_delay_ms(30);
	}
	*/
	OS_Start();
}


