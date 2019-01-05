/*
 * protocol.c
 *
 * Created: 12.01.2015 18:09:39
 *  Author: Basti
 */ 

#include "protocol.h"
#include "INA219_driver.h"
#include "adc_hall.h"
#include <string.h>

#define TPM2_BLOCK_START			0xC9
#define TPM2_BLOCK_DATAFRAME		0xDA
#define TPM2_BLOCK_END				0x36

#define TPM2_ACK					0xAC

#define TPM2_NET_BLOCK_START		0x9C

#define PM_DATAFRAME				0xDA
#define PM_SCALEVALUES				0x5C
#define PM_SENDSCALE				0x5D
#define PM_HELLO					0x11
#define PM_START					0x5A
#define PM_STOP						0x50
#define PM_BOOTLOADER				0xB0

#define SEND_BUFFER_NUM				3

uint8_t send_buf[65];
uint8_t send_buf_rdy = false;
uint8_t send_buf_pointer = 0;

void proto_gen_tpm2_frame(uint8_t *buf,uint8_t len);

void proto_send_data()
{
	
	uint8_t buf[50];
	buf[0] = PM_DATAFRAME;
	uint8_t cnt=1;
	uint32_t tempi = 0;
	
	while(FIFO_OK == fifo_pull_uint32(&fifo_desc,&tempi)) {
		memcpy(&buf[cnt],&tempi,4);
		cnt += 4;
	}
	
	proto_gen_tpm2_frame(buf,cnt);
}

void proto_send_id() 
{
	uint8_t buf[] = {PM_HELLO,(product_bytes.coordy0 ^ product_bytes.coordy1),(product_bytes.coordx0 ^ product_bytes.coordx1),product_bytes.wafnum,product_bytes.lotnum0,POWER_METER_VER1,POWER_METER_VER2};
	proto_gen_tpm2_frame(buf,sizeof(buf));
}

void proto_send_scale()
{
	uint8_t buf[16];
	
	buf[0] = PM_SCALEVALUES;
	buf[1] = ee_scale_settings.opt_v & 0x01;
	buf[6] = ee_scale_settings.opt_cc & 0x01;
	buf[11] = ee_scale_settings.opt_ch & 0x01;
	
	if(use_hall_sensor) {
		buf[11] |= 0x04;
	} else {
		buf[6] |= 0x04;
	}
	
	memcpy(&buf[2],&ee_scale_settings.scale_v,4);
	memcpy(&buf[7],&ee_scale_settings.scale_cc,4);
	memcpy(&buf[12],&ee_scale_settings.scale_ch,4);
	
	proto_gen_tpm2_frame(buf,sizeof(buf));
}


void proto_gen_tpm2_frame(uint8_t *buf,uint8_t len)
{
	if(len > (64 - send_buf_pointer - 5)) return;
	
	volatile uint16_t frame_len = (uint16_t)len;
		
	send_buf[send_buf_pointer] = TPM2_BLOCK_START;
	send_buf_pointer++;
	send_buf[send_buf_pointer] = TPM2_BLOCK_DATAFRAME;
	send_buf_pointer++;
	send_buf[send_buf_pointer] = (uint8_t)(frame_len>>8);
	send_buf_pointer++;
	send_buf[send_buf_pointer] = (uint8_t)(frame_len);
	send_buf_pointer++;
	
	for(uint8_t i = 0;i<len;i++) {
		send_buf[send_buf_pointer] = buf[i];
		send_buf_pointer++;
	}
	
	send_buf[send_buf_pointer] = TPM2_BLOCK_END;
	send_buf_pointer++;
	
	/*if(send_buf_rdy==true) {
		frame_len = 0;
	}*/
	
	if(sysclk_module_is_enabled(SYSCLK_PORT_C,SYSCLK_USART0)) {
		//send rs485 data
		rs485_add_data(send_buf,send_buf_pointer);
		
		//test send system
		//uint8_t da[] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
		//rs485_add_data(da,17);
	}
	
	//send usb data?
	if(main_b_generic_enable) {
		if(udi_hid_generic_send_report_in(send_buf))
		{
			//gpio_toggle_pin(LED_GREEN_O);
			send_buf_rdy = false;
			send_buf_pointer = 0;
		} else {
			send_buf_rdy = true;	
		}
	} else {
		send_buf_rdy = false;
		send_buf_pointer = 0;
	}
}

void proto_send()
{
	if(send_buf_rdy == true) {
		//send usb data?
		if(main_b_generic_enable) {
			if(udi_hid_generic_send_report_in(send_buf))
			{
				//gpio_toggle_pin(LED_GREEN_O);
				send_buf_rdy = false;
				send_buf_pointer = 0;
			}
		} else {
			send_buf_rdy = false;
			send_buf_pointer = 0;
		}
	}
}

enum {IDLE, STARTB, STARTB2,STARTB3,STARTB4,STARTB5,STARTB6,ENDB};			//tpm2 states

static uint8_t tpm2state = IDLE;

//handle tpm2 protocol
static uint16_t count = 0;
static uint16_t Framesize = 0;

void proto_receiver()
{
	uint8_t *msg_buf;
	//every new package -> reset!
	tpm2state = IDLE;
	
	for(uint8_t i = 0;i<64;i++) {
		if(tpm2state == STARTB4) {
			if(count < Framesize) {
				if(count == 0) {
					msg_buf = &hid_revc_buf[i];
				}
				count++;
				continue;
			}
			else
			tpm2state = ENDB;
		}

		//check for start- and sizebyte
		if(tpm2state == IDLE && hid_revc_buf[i] == TPM2_BLOCK_START) {
			tpm2state = STARTB;
			continue;
		}
		
		if(tpm2state == STARTB && hid_revc_buf[i] == TPM2_BLOCK_DATAFRAME) {
			tpm2state = STARTB2;
			continue;
		}
		
		if(tpm2state == STARTB2)
		{
			Framesize = (uint16_t)hid_revc_buf[i]<<8;
			tpm2state = STARTB3;
			continue;
		}
		
		if(tpm2state == STARTB3)
		{
			Framesize |= (uint16_t)hid_revc_buf[i];
			if(Framesize <= 60) {
				count = 0;
				tpm2state = STARTB4;
			} else tpm2state = IDLE;
			
			continue;
			
		}

		//check end byte
		if(tpm2state == ENDB) {
			if(hid_revc_buf[i] == TPM2_BLOCK_END) {
				switch(msg_buf[0]) {
					case PM_HELLO:
						if(count > 0)
							proto_send_id();
					break;
					case PM_BOOTLOADER:
					{
						udc_stop();
						uint8_t temp_buf[EEPROM_PAGE_SIZE];
						temp_buf[0]=0xB0;
						nvm_eeprom_flush_buffer();
						nvm_eeprom_load_page_to_buffer(temp_buf);
						nvm_eeprom_atomic_write_page(EEPROM_PAGE_BOOT);
						delay_ms(500);
						wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_8CLK);
						wdt_enable();
						break;
					}
					case PM_SCALEVALUES:
						//recv new scale values
						if(count > 15) {
							if(msg_buf[1] & 0x02) {
								
								memcpy(&ee_scale_settings.scale_v,&msg_buf[2],4);
							}
							if(msg_buf[6] & 0x02) {
								
								memcpy(&ee_scale_settings.scale_cc,&msg_buf[7],4);
							}
							if(msg_buf[11] & 0x02) {
								
								memcpy(&ee_scale_settings.scale_ch,&msg_buf[12],4);
							}
							
							ee_scale_settings.opt_v = msg_buf[1];
							ee_scale_settings.opt_cc = msg_buf[6];
							ee_scale_settings.opt_ch = msg_buf[11];
							
							use_hall_sensor = false;
							
							if(init_adc()) {
								if(ee_scale_settings.opt_ch & 0x01) {
									use_hall_sensor = true;
								}
							}
							//if not auto mode and hall should be used
							if(ee_scale_settings.opt_ch & 0x01 && !(ee_scale_settings.opt_cc & 0x01)) {
								use_hall_sensor = true;
							}
							
							nvm_eeprom_flush_buffer();
							nvm_eeprom_load_page_to_buffer((const uint8_t*)&ee_scale_settings);
							nvm_eeprom_atomic_write_page(EEPROM_PAGE_SCALE);
						}
					break;
				}
			}

			tpm2state = IDLE;
		}
	}
}