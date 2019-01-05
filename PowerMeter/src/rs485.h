/*
 * rs485.h
 *
 * Created: 22.03.2015 11:28:14
 *  Author: Basti
 */ 


#ifndef RS485_H_
#define RS485_H_


#include <asf.h>


extern void rs485_init(void);
extern void rs485_deinit(void);
extern uint8_t rs485_add_data(uint8_t *data, uint8_t size);
extern void rs485_init_dma(void);

#endif /* RS485_H_ */