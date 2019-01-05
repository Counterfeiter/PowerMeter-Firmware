/**
 * \file
 *
 * \brief Declaration of main function used by HID generic example
 *
 * Copyright (c) 2011 Atmel Corporation. All rights reserved.
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

#ifndef _MAIN_H_
#define _MAIN_H_

#include <asf.h>

////versions history
// 0.1 -> inital (adc + ina could send data over hid)
// 0.2 -> adding rs485 support with dma fifo sendbuffers
// 0.3 -> fix rs485 startup bug
//	   -> correct the voltage reading

#define POWER_METER_VER1		0x00
#define POWER_METER_VER2		0x03

#define EEPROM_PAGE_SCALE		1
#define EEPROM_PAGE_BOOT		63

//32 Byte to fit in one eeprom section!
struct eeprom_settings
{
	float		scale_v;
	float		scale_cc;
	float		scale_ch;
	uint8_t		opt_v;
	uint8_t		opt_cc;
	uint8_t		opt_ch;
	uint8_t		temp[(EEPROM_PAGE_SIZE-sizeof(float)*3-sizeof(uint8_t)*3)];
	
} __attribute__((packed));

extern struct eeprom_settings ee_scale_settings;

extern struct nvm_device_serial product_bytes;

uint8_t	use_hall_sensor;


#endif // _MAIN_H_
