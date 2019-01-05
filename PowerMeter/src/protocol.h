/*
 * protocol.h
 *
 * Created: 12.01.2015 18:10:01
 *  Author: Basti
 */ 


#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "main.h"
#include <asf.h>
#include "rs485.h"

extern void proto_send_id(void);
extern void proto_send_data(void);
extern void proto_send_scale(void);
extern void proto_send(void);

extern void proto_receiver(void);


#endif /* PROTOCOL_H_ */