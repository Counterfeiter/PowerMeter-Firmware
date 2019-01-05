#include "INA219_driver.h"
#include "main.h"
#include "adc_hall.h"


#define SWITCH_SCALE_VALUE						4000


// about 40 mV offset because there is a ~40 uA parasitic current through the 1k2 ground protection resistor -> 1k2 * 40 uA = 48 mV
#define VOLTAGE_OFFSET_SERIES_GND_RESISTOR		10

uint32_t fifo_buffer[FIFO_BUFFER_LENGTH];

uint8_t INA219_init(struct INA219_Data *ina_daten)
{
	//status_code_t master_status;
	
	uint8_t buffer[3];
	// Package to send
	twi_package_t packet = {
		// No address or command
		.addr_length = 0,
		// issue to slave
		.chip        = INA219_1,
		.buffer      = (void *)buffer,
		.length      = 3,
		// Wait if bus is busy
		.no_wait     = false
	};

	// TWI master options
	twi_options_t m_options = {
		.speed = TWI_SPEED,
		.chip  = 0x50,
		.speed_reg = TWI_BAUD(sysclk_get_cpu_hz(), TWI_SPEED)
	};


	// Initialize TWI_MASTER
	sysclk_enable_peripheral_clock(&TWI_MASTER);
	twi_master_init(&TWI_MASTER, &m_options);
	twi_master_enable(&TWI_MASTER);

	buffer[0] = INA_CONF_REG;
	buffer[1] = ina_daten->conf_msb = 0b00000100;
	buffer[2] = ina_daten->conf_lsb = 0b01000111;
	
	//set current mode to 1 and counter time to max
	ina_daten->pga_current = 1;
	ina_daten->pga_current_counter = INA219_PGA_COUNTER;
	
	//set voltage scale to 16 Volt and PGA Counter to max
	ina_daten->pga_voltage = 16;
	ina_daten->pga_voltage_counter = INA219_PGA_COUNTER;

	if(twi_master_write(&TWI_MASTER, &packet) != STATUS_OK) {
		return false;
	}
	
	//Write calibration
	buffer[0] = INA_CALIBRATION;
	buffer[1] = 0x0F;
	buffer[2] = 0x00;
	
	if(twi_master_write(&TWI_MASTER, &packet) != STATUS_OK) {
		return false;
	}
	
	
	if(fifo_init(&fifo_desc, fifo_buffer, FIFO_BUFFER_LENGTH) != FIFO_OK) {
		return false;
	}
	
	return true;

}

uint8_t INA219_read(struct INA219_Data *ina_daten)
{

	uint8_t buffer[3]; //keep it higher to write the config data if necessary 
		
	// Package to send
	twi_package_t packet = {
		//address or command
		.addr_length =	1,
		.addr[0] =		INA_BUS_VOLT,
		// issue to slave
		.chip        = INA219_1,
		.buffer      = (void *)buffer,
		.length      = 2,
		// Wait if bus is busy
		.no_wait     = false
	};
		
		
	if(twi_master_read(&TWI_MASTER, &packet)) {
		return false;
	}
	
	ina_daten->Spannung_ADC = (uint16_t)buffer[0]<<8;
	ina_daten->Spannung_ADC |= (uint16_t)buffer[1];
	
	
	//neue Daten im Buffer
	if((ina_daten->Spannung_ADC & (1<<1))) {

		//if((msb_ina_temp & (1<<0))) overflow = 1;
		
		ina_daten->Spannung_ADC >>= 3;
			
		packet.addr[0] = INA_SHUNT_VOLT;
			
		if(twi_master_read(&TWI_MASTER, &packet)) {
			return false;
		}
		
		ina_daten->Strom_ADC = (uint16_t)buffer[0]<<8;
		ina_daten->Strom_ADC |= (uint16_t)buffer[1];
		
		int8_t negativ_flag;

		if(ina_daten->Strom_ADC < 0) {
			negativ_flag = 1;
			ina_daten->Strom_ADC *= (-1);
		}
		else {
			negativ_flag = (-1);
		}
		
		uint8_t conf_update = false;

		//scale up if necessary
		if((ina_daten->pga_current*SWITCH_SCALE_VALUE) <= ina_daten->Strom_ADC) {
			if(ina_daten->pga_current < 8) {
				ina_daten->pga_current <<= 1;
				conf_update = true;
				switch(ina_daten->pga_current)
				{
					case 2:
						ina_daten->conf_msb |= (1<<3);
						ina_daten->conf_msb &= ~(1<<4);
					break;
					case 4:
						ina_daten->conf_msb |= (1<<4);
						ina_daten->conf_msb &= ~(1<<3);
					break;
					case 8:
						ina_daten->conf_msb |= (1<<3) | (1<<4);
					break;
				}
			}
		}

		//scale down!!
		if(SWITCH_SCALE_VALUE > ina_daten->Strom_ADC) {
			if(ina_daten->pga_current_counter > 0) {
				ina_daten->pga_current_counter--;
				if(ina_daten->pga_current_counter == 0) {
					ina_daten->conf_msb &= ~(1<<3);
					ina_daten->conf_msb &= ~(1<<4);
					ina_daten->pga_current = 1;
					conf_update = true;
				}
			}
					
		} else
			ina_daten->pga_current_counter=INA219_PGA_COUNTER;


		ina_daten->Strom_ADC *= negativ_flag;

		if(4000 > ina_daten->Spannung_ADC) {
			if(ina_daten->pga_voltage_counter > 0) {
				ina_daten->pga_voltage_counter--;
				if(ina_daten->pga_voltage_counter == 0) {
					ina_daten->conf_msb &= ~(1<<5);
					ina_daten->pga_voltage = 16;
					conf_update = true;
				}
			}
		} else {
			ina_daten->pga_voltage_counter = INA219_PGA_COUNTER;

			if(ina_daten->pga_voltage == 16) {
				ina_daten->pga_voltage = 32;
				conf_update = true;
				ina_daten->conf_msb |= (1<<5);
			}
		}
		
		if(use_hall_sensor) {
			cli();
			ina_daten->Strom_ADC = adc_value;
			sei();
		}
		
		if(ina_daten->Spannung_ADC > VOLTAGE_OFFSET_SERIES_GND_RESISTOR) {
			ina_daten->Spannung_ADC -= VOLTAGE_OFFSET_SERIES_GND_RESISTOR;
		} else {
			ina_daten->Spannung_ADC = 0;
		}
		
		if(fifo_push_uint32(&fifo_desc,((uint32_t)ina_daten->Strom_ADC << 16) | ina_daten->Spannung_ADC) != FIFO_OK) {
			
			return false;
		}
		
		if(conf_update == true) {

			packet.addr_length = 0;
			packet.length      = 3;
			
			buffer[0] = INA_CONF_REG;
			buffer[1] = ina_daten->conf_msb;
			buffer[2] = ina_daten->conf_lsb;
			
			if(twi_master_write(&TWI_MASTER, &packet)) {
				return false;
			}
		}
		
	}


	return true;
}
