#ifndef SHARED_H_
#define SHARED_H_

#include <util/delay.h>
#include <avr/io.h>
#include "uart/uart.h"
#include "adc/adc.h"
#include "rtos/os.h"
#include "rtos/kernel.h"

//Preamble and postamble used by basestation to wrap data
#define CMD_PREAMBLE			'$'
#define CMD_POSTAMBLE			'#'

//Encoded values used by base station to transmit movements and commands
#define NEGATIVE_HIGH			'N'		//reverse in high speed/or turn left high speed
#define NEGATIVE_LOW			'n'		//reverse in low speed
#define NOT_MOVING				'z'
#define POSITIVE_LOW			'p'
#define POSITIVE_HIGH			'P'
#define FIRE					'F'
#define HOLD					'H'

//Special case values for radius, used for drive
#define DRIVE_STRAIGHT			0x7FFF			//32767
#define CLOCKWISE_TURN			0xFFFF			//-1
#define COUNTER_CLOCKWISE_TURN	0x1				//1


#define SENSORS_TO_QUERY 2		//How many sensors are we querying from the robot?
#define TWO_BYTE_SENSORS 0		//How many of the sensors above returns 2 bytes of data instead of 1?

#endif /* SHARED_H_ */