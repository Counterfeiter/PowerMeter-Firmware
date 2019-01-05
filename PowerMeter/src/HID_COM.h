/*
 * HID_COM.h
 *
 * Created: 11.12.2014 18:14:37
 *  Author: Basti
 */ 


#ifndef HID_COM_H_
#define HID_COM_H_

#include <asf.h>
#include "powermeter_board.h"


extern volatile bool		main_b_generic_enable;
extern volatile uint8_t		main_b_attached;
extern volatile bool		msg_ready;
extern			uint8_t		hid_revc_buf[65];

//extern void hid_receiver(uint8_t *buffer, uint8_t len);
extern void hid_receiver(void);

/*! \brief Called by HID interface
 * Callback running when USB Host enable generic interface
 *
 * \retval true if generic startup is ok
 */

void hid_message(uint8_t *msg);


bool main_generic_enable(void);

/*! \brief Called by HID interface
 * Callback running when USB Host disable generic interface
 */
void main_generic_disable(void);

/*! \brief Called when a start of frame is received on USB line
 */
void main_sof_action(void);

/*! \brief Enters the application in low power mode
 * Callback called when USB host sets USB line in suspend state
 */
void main_suspend_action(void);

/*! \brief Called by UDD when the USB line exit of suspend state
 */
void main_resume_action(void);

/*! \brief Called by UDI HID generic when USB Host send a feature request
 */
void main_hid_set_feature(uint8_t* report);



#endif /* HID_COM_H_ */