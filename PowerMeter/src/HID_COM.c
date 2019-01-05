/*
 * HID_COM.c
 *
 * Created: 11.12.2014 18:14:21
 *  Author: Basti
 */ 

#include "HID_COM.h"
#include <string.h>

volatile bool		main_b_generic_enable = false;
volatile uint8_t	main_b_attached = 0;

uint8_t hid_revc_buf[65];
volatile bool msg_ready = false;

void hid_message(uint8_t *message)
{
	if(msg_ready == false)
		memcpy(hid_revc_buf,message,64);
	msg_ready = true;
}

void main_suspend_action(void)
{
	main_b_attached &= ~0x01;
}

void main_resume_action(void)
{
	main_b_attached |= 0x01;
}

void main_sof_action(void)
{
	if (!main_b_generic_enable)
	return;
}

bool main_generic_enable(void)
{
	main_b_generic_enable = true;
	return true;
}

void main_generic_disable(void)
{
	main_b_generic_enable = false;
}

void main_hid_set_feature(uint8_t* report)
{

}
