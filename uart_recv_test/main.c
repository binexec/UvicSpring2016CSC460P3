#include <stdio.h>
#include <avr/io.h>
#include "uart/uart.h"


int main(void)
{
    uart0_init();
	uart1_init();
	
	uart_setredir();
	
	while (1) 
    {
		printf("%u\n", uart1_recvbyte()); 
    }
}

