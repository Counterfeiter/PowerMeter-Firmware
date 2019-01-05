/**
 * \file
 *
 * \brief Main functions for Generic example
 *
 * Copyright (c) 2011-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <asf.h>
#include "main.h"
#include "conf_usb.h"
#include "powermeter_board.h"
#include "protocol.h"
#include "INA219_driver.h"
#include "adc_hall.h"
#include "rs485.h"

void (*start_bootloader) (void) = (void (*)(void))(BOOT_SECTION_START/2+0x1FC/2);

struct INA219_Data ina_data;
struct eeprom_settings ee_scale_settings;
 
volatile uint8_t timer_cnt=0;

uint8_t	use_hall_sensor = false;

struct nvm_device_serial product_bytes;

void TimerCallback(void);
void init_timer(void);

void TimerCallback(void)
{
	timer_cnt++;
	//gpio_toggle_pin(LED_GREEN_O);
}

void init_timer(void)
{
	sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_TC0);
	
	tc_enable(&TCC0);

	tc_write_period(&TCC0,375*4);
	
	/*
	* Enable TC interrupts overflow
	*/
	tc_set_overflow_interrupt_callback(&TCC0,TimerCallback);
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
	
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV8_gc);
}
int main(void)
{
	
	//random uid auslesen
	nvm_read_device_serial(&product_bytes);
	
	irq_initialize_vectors();
	cpu_irq_enable();

	// Initialize the sleep manager
	sleepmgr_init();

	sysclk_init();
	
	
	//should we start the bootloader???
	if(nvm_eeprom_read_byte(EEPROM_PAGE_BOOT * EEPROM_PAGE_SIZE)==0xB0) {
		uint8_t temp_buf[EEPROM_PAGE_SIZE];
		temp_buf[0]=0xFF;
		nvm_eeprom_flush_buffer();
		nvm_eeprom_load_page_to_buffer(temp_buf);
		nvm_eeprom_atomic_write_page(EEPROM_PAGE_BOOT);
		start_bootloader();
	}
		
	board_init();	
	
	init_timer();


	// Start USB stack to authorize VBus monitoring
	udc_start();
	
	nvm_eeprom_read_buffer(EEPROM_PAGE_SCALE * EEPROM_PAGE_SIZE,&ee_scale_settings,sizeof(ee_scale_settings));
	
	//check if eeprom erased
	if(ee_scale_settings.opt_v == 0xFF) {
		
		ee_scale_settings.scale_v = 0.0040206;
		ee_scale_settings.scale_cc = 0.001;
		ee_scale_settings.scale_ch = 0.002;
		
		ee_scale_settings.opt_v = 0x05;
		ee_scale_settings.opt_cc = 0x05;
		ee_scale_settings.opt_ch = 0x01;

		nvm_eeprom_flush_buffer();
		nvm_eeprom_load_page_to_buffer((const uint8_t*)&ee_scale_settings);
		nvm_eeprom_atomic_write_page(EEPROM_PAGE_SCALE);
	}
	

	
	
	while (true) {
		
		//wenn usb und ina aktiviert
		if((main_b_attached & 0x03) == 0x03) {
			//send id and scale values sometimes... 
			if(timer_cnt > 250) {
				timer_cnt = 0;
				proto_send_id();
				proto_send_scale();
			}
		
			//versuche Daten vom INA IC blockierend zu lesen
			if(!INA219_read(&ina_data)) {
				msg_ready = false;
			}
			
		
			//wenn fifo etwas angestiegen, versuche Strom und Spannungswerte zu senden...
			if(fifo_get_used_size(&fifo_desc) >= 6) {
				proto_send_data();
			}
		
			//wenn etwas im Sendebuffer vorhanden ist, versuchen mit der nächsten HID Msg zu senden...
			proto_send();
		
			//empfangene HID-Nachricht auswerten
			if(msg_ready) {
				proto_receiver();
				msg_ready = false;
			}
		}
		
		//wenn sich attached status geändert hat
		if((main_b_attached & 0x01) && !(main_b_attached & 0x02)) {
			//entprellen
			delay_ms(1000);
			//ina aktivieren
			if(main_b_attached & 0x01) {
				main_b_attached = 0x03;
				
				gpio_set_pin_high(LED_GREEN_O);
				gpio_set_pin_high(EN_5V_O);
				
				delay_ms(500);
				
				rs485_init();
				
				INA219_init(&ina_data);
				
				
				if(init_adc()) {
					if(ee_scale_settings.opt_ch & 0x01) {
						use_hall_sensor = true;
					}
				}
				
				//if not auto mode and hall should be used
				if(ee_scale_settings.opt_ch & 0x01 && !(ee_scale_settings.opt_cc & 0x01)) {
					use_hall_sensor = true;
				}
			}
		//wenn sich status zu dettached geändert hat
		} else if(!(main_b_attached & 0x01) && (main_b_attached & 0x02)) {
			//entprellen
			delay_ms(1000);
			//rs485 disable
			rs485_deinit();
			//5v abschalten
			if(!(main_b_attached & 0x01)) {
				main_b_attached = 0;
				gpio_set_pin_low(LED_GREEN_O);
				gpio_set_pin_low(EN_5V_O);
				//rs485_deinit();
				sleepmgr_enter_sleep();
			}
		}

	}
}