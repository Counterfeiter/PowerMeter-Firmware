/*
 * adc.h
 *
 * Created: 25.01.2015 14:35:07
 *  Author: Basti
 */ 


#ifndef ADCCC_H_
#define ADCCC_H_

#include <asf.h>


uint8_t init_adc(void);

volatile int16_t adc_value;


#endif /* ADC_H_ */