#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

void InitADC(void);

uint16_t readadc(uint8_t ch);




#endif /* ADC_H_ */