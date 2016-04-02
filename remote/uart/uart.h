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

void uart0_init(void);
void uart1_init(void);

void uart_putchar(char c, FILE *stream);
char uart_getchar(FILE *stream);
void uart_setredir(void);

void uart0_sendbyte(uint8_t data);
uint8_t uart0_recvbyte(void);
void uart0_sendstr(char* input);

void uart1_sendbyte(uint8_t data);
uint8_t uart1_recvbyte(void);
void uart1_sendstr(char* input);

#endif