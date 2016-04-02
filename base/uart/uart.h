#ifndef MY_UART_H
#define MY_UART_H

/*Sources used:
	http://www.appelsiini.net/2011/simple-usart-with-avr-libc
	https://hekilledmywire.wordpress.com/2011/01/05/using-the-usartserial-tutorial-part-2/
*/

#include <avr/io.h>
#include <stdio.h>
#include <util/setbaud.h>
#include <avr/sfr_defs.h>

#ifndef F_CPU
	#define F_CPU 16000000UL
#endif

#ifndef BAUD
	#define BAUD 19200
#endif

void uart_putchar(char c, FILE *stream);
char uart_getchar(FILE *stream);
void uart_init(void);
void uart_setredir(void);

void uart_sendbyte(uint8_t data);
uint8_t uart_recvbyte(void);
void uart_sendstr(char* input);

#endif