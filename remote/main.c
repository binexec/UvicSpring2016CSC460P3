#include "shared.h"
#include "remote_declarations.h"

#define SEMIAUTO						//If uncommented, the robot will always drive forward if joystick is neutral. 

//Global variables used for updating movement information 
char direction = NOT_MOVING;
char last_direction = NOT_MOVING;
char speed = NOT_MOVING;
char last_speed = NOT_MOVING;
char fire = HOLD;

//Global variables used for photoresistors and storing dead state
uint16_t photores_thres;
uint16_t photores_neutral;
uint8_t isDead = 0;

void calibratePhotores()
{
	int i;

	//Sample the ambient lighting 10 times
	for(i=0; i<10; i++)
	{
		photores_neutral += readadc(PHOTORESIS_PIN);
		_delay_ms(100);
	}
	photores_neutral /= i;     //Use the average as neutral value
	photores_thres = 1.4*photores_neutral;
}

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

void movement_controller()
{
	int16_t vel;
	int16_t rad;
	
	while (1)
	{
		//If the base station hasn't issued a new direction or speed, skip updating
		if (direction == last_direction && speed == last_speed)
			goto move_as_global_continue;
		
		//Decode the direction sent by the base station
		switch(direction)
		{
			case NEGATIVE_HIGH:
				rad = BACKWARDS_FAST_RAD;
				break;
			case NEGATIVE_LOW:
				rad = BACKWARDS_SLOW_RAD;
				break;
			case POSITIVE_LOW:
				rad = FORWARD_SLOW_RAD;
				break;
			case POSITIVE_HIGH:
				rad = FORWARD_FAST_RAD;
				break;
			case NOT_MOVING:
			default:
				rad = STAY_STATIONARY;
				break;
		}
	
		//Decode the speed sent by the base station
		switch(speed)
		{
			case NEGATIVE_HIGH:
				vel = BACKWARDS_FAST;
				break;
			case NEGATIVE_LOW:
				vel = BACKWARDS_SLOW;
				break;
			case POSITIVE_LOW:
				vel = FORWARD_SLOW;
				break;
			case POSITIVE_HIGH:
				vel = FORWARD_FAST;
				break;
			case NOT_MOVING:
			default:
				vel = STAY_STATIONARY;
				break;
		}
		
		//Special case 1: Give the robot some velocity when making stationary turns
		if (rad == 1 || rad == -1) 
		{
			vel = 250;
		} 
		
		//If robot is in semi-autonomous mode, keep the robot driving instead of stopping
		#ifdef SEMIAUTO
		else if (rad == 0 && vel == 0) {
			vel = 250;
		}
		#endif
	
		drive(vel, rad);

move_as_global_continue:
		if (fire == HOLD)
			PORTB &= ~(1<<PB2);	//pin 51 off
		else if (fire == FIRE)
			PORTB |= (1<<PB2);	//pin 51 on
		Task_Sleep(3);
	}
}

int isHit()
{
	uint16_t val = readadc(PHOTORESIS_PIN); 
	return val > photores_thres;
}

void handle_sensors()
{
	uint8_t i;
	uint8_t bytes[SENSORS_TO_QUERY + TWO_BYTE_SENSORS];	
	
	while(1)
	{
		/*Queries the sensors*/
		
		uart1_sendbyte(149);				//Opcode for Query List
		uart1_sendbyte(SENSORS_TO_QUERY);	//Query will send three sensors packets
		uart1_sendbyte(7);					//Packet 7: Bump/Wheeldrop detection
		uart1_sendbyte(13);					//Packet 13: Virtual wall seen?
	
		Task_Sleep(2);						//End of upper half of handle_sensor and wait for Roomba to reply with sensor data
		
		//Check if laser has hit our photosensor
		if (isHit()) 
		{
			isDead = 1;
			drive(0,0);
			beep();
			
			//Write "DEAD" on the front LEDs
			uart1_sendbyte(164);
			uart1_sendbyte(68);
			uart1_sendbyte(69);
			uart1_sendbyte(65);
			uart1_sendbyte(68);
			
			//Send the start command to go to passive mode and act dead
			uart1_sendbyte(128);	
			//Task_Terminate();
		}
	
		/*
		Read sensor data returned by the roomba
		Currently, the data bytes for the following sensors will be returned in order:
			1: Bump/Wheeldrop
			2: Virtual wall detection 
		*/
		for(i=0; i<SENSORS_TO_QUERY + TWO_BYTE_SENSORS; i++)
			bytes[i] = uart1_recvbyte();
	
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
		else if (bytes[0] == 3 || bytes[1] == 1)
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
		count = 0;
		curbyte = uart0_recvbyte();
		
		while (curbyte != CMD_PREAMBLE)
		{
			//If more than MAX characters has been received and none is the preamble, skip it.
			if (++count > MAX_CMD_LENG)
				goto receive_and_update_continue;
				
			curbyte = uart0_recvbyte();
		}
		
		//Save current speed and direction
		last_direction = direction;
		last_speed = speed;
		
		//When preamble is received, receive the subsequent bytes containing new data for control
		direction = uart0_recvbyte();
		speed = uart0_recvbyte();
		fire = uart0_recvbyte();
		
receive_and_update_continue:
		Task_Sleep(8);	
	}
}

void a_main()
{
	DDRB |= (1<<PB2);	//pin 51 set as output for laser
	
	OS_Init();
	
	InitADC();
	uart0_init();		//UART0 is used for BT
	uart1_init();		//UART1 is used to communicate with the robot
	roomba_init();
	calibratePhotores();
	beep();
	
	Task_Create(receive_and_update, 4, 0);
	Task_Create(movement_controller, 5, 0);
	Task_Create(handle_sensors, 3, 0);
	
	OS_Start();
}


