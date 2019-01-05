#ifndef _INA219_driver_h_
#define _INA219_driver_h_

#include <asf.h>
#include "powermeter_board.h"

//fifo driver dependings
#define FIFO_BUFFER_LENGTH  8
fifo_desc_t fifo_desc;

#define INA219_1					0b01000000
#define INA219_2					0b01000001
#define INA219_3					0b01000100
#define INA219_4					0b01000101

#define INA_CONF_REG				0x00
#define INA_SHUNT_VOLT				0x01
#define INA_BUS_VOLT				0x02
#define INA_POWER					0x03
#define INA_CURRENT					0x04
#define INA_CALIBRATION				0x05

#define INA219_PGA_COUNTER			255

struct INA219_Data {
	uint16_t	Spannung_ADC;
	int16_t		Strom_ADC;
	float		Scale_U;
	float		Scale_I;
	uint8_t		conf_msb;
	uint8_t		conf_lsb;
	uint8_t		addr;
	int32_t		filter_reg_u;
	int32_t		filter_reg_i;
	uint8_t		pga_current;
	uint8_t		pga_current_counter;
	uint8_t		pga_voltage;
	uint8_t		pga_voltage_counter;
};


extern uint8_t INA219_init(struct INA219_Data *ina_daten);
extern uint8_t INA219_read(struct INA219_Data *ina);


#endif
