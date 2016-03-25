#include "uart/uart.h"

int main()
{
	uart_init();
	
	while(1)
	{
		uart_sendstr("i'm gay lol \n");
	}
	
	return 0;
}


