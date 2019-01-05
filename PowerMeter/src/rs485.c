/*
 * rs485.c
 *
 * Created: 22.03.2015 11:27:58
 *  Author: Basti
 */ 


#include "rs485.h"

#define DMA_DATA_MSG_SIZE		64
#define DMA_DATA_BUFFERS		5

#define DMA_DATA_STATE_FULL		1
#define DMA_DATA_STATE_NEW		2
#define DMA_DATA_STATE_READY	3
#define DMA_DATA_STATE_TRANS	4

struct DMA_DATA_FIFO {
	uint8_t		msg[DMA_DATA_MSG_SIZE];
	uint8_t		state;
	uint8_t		level;
};

//used by add data function -> write pointer
uint8_t dma_fifo_writepointer = 0;
//used by DMA -> sendpointer
uint8_t dma_fifo_readpointer = 0;

volatile uint8_t temp = 0;

struct DMA_DATA_FIFO dma_fifo[DMA_DATA_BUFFERS];

void dma1_transfer_done(enum dma_channel_status status);
void rs485_trigger_dma_transfer(void);
void rs485_trigger_dma_transfer_intr(void);

void dma1_transfer_done(enum dma_channel_status status)
{
	if (status == DMA_CH_TRANSFER_COMPLETED) {
		dma_channel_disable(RS485_DMA_CHANNEL);
		rs485_trigger_dma_transfer_intr();
	}
}

void rs485_trigger_dma_transfer(void)
{
	cli();
	if(dma_get_channel_status(RS485_DMA_CHANNEL) != DMA_CH_BUSY) {
		
		if(dma_fifo[dma_fifo_readpointer].state == DMA_DATA_STATE_FULL) {
			dma_fifo[dma_fifo_readpointer].state = DMA_DATA_STATE_TRANS;
			dma_channel_write_transfer_count(RS485_DMA_CHANNEL,dma_fifo[dma_fifo_readpointer].level);
			dma_channel_write_source(RS485_DMA_CHANNEL,(uint16_t)(uintptr_t)dma_fifo[dma_fifo_readpointer].msg);
			
			dma_channel_enable(RS485_DMA_CHANNEL);
		}
	}
	sei();
}

void rs485_trigger_dma_transfer_intr(void)
{

	if(dma_fifo[dma_fifo_readpointer].state != DMA_DATA_STATE_TRANS) {
		//error
		temp++;
	}
	dma_fifo[dma_fifo_readpointer].state = DMA_DATA_STATE_NEW;
	dma_fifo[dma_fifo_readpointer].level = 0;
	
	//set the next element
	if(dma_fifo_readpointer+1>=DMA_DATA_BUFFERS) {
		dma_fifo_readpointer=0;
	} else {
		dma_fifo_readpointer++;
	}
	
	if(dma_fifo[dma_fifo_readpointer].state == DMA_DATA_STATE_FULL) {
		dma_fifo[dma_fifo_readpointer].state = DMA_DATA_STATE_TRANS;
		dma_channel_write_transfer_count(RS485_DMA_CHANNEL,dma_fifo[dma_fifo_readpointer].level);
		dma_channel_write_source(RS485_DMA_CHANNEL,(uint16_t)(uintptr_t)dma_fifo[dma_fifo_readpointer].msg);
			
		dma_channel_enable(RS485_DMA_CHANNEL);
	}
}

void rs485_init_dma(void)
{

	struct dma_channel_config       config;
		
	memset(&config, 0, sizeof(config));
		
	dma_enable();
	
	//wait for possible ongoing dma transfers
	while(dma_get_channel_status(RS485_DMA_CHANNEL) == DMA_CH_BUSY);
		
	//interface port with DMA
	dma_set_callback(RS485_DMA_CHANNEL, dma1_transfer_done);
	dma_channel_set_interrupt_level(&config, DMA_INT_LVL_LO);
	dma_channel_set_burst_length(&config, DMA_CH_BURSTLEN_1BYTE_gc);

	dma_channel_set_trigger_source(&config,DMA_CH_TRIGSRC_USARTC0_DRE_gc);
	dma_channel_set_src_reload_mode(&config,DMA_CH_SRCRELOAD_TRANSACTION_gc);
	dma_channel_set_dest_reload_mode(&config, DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_src_dir_mode(&config, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_dest_dir_mode(&config, DMA_CH_DESTDIR_FIXED_gc);//don't count the dest. addr.
	dma_channel_set_single_shot(&config);

	dma_channel_set_destination_address(&config,(uint16_t)(uintptr_t)&USARTC0.DATA);
	dma_channel_write_config(RS485_DMA_CHANNEL, &config);
	
	
	for(uint8_t i=0;i<DMA_DATA_BUFFERS;i++) {
		dma_fifo[i].state = DMA_DATA_STATE_NEW;
		dma_fifo[i].level = 0;
	}
	
	dma_fifo_writepointer = 0;
	dma_fifo_readpointer = 0;
}

uint8_t rs485_add_data(uint8_t *data, uint8_t size)
{
	uint8_t data_in_buffer = false;
	if(dma_fifo[dma_fifo_writepointer].state == DMA_DATA_STATE_READY || dma_fifo[dma_fifo_writepointer].state == DMA_DATA_STATE_NEW) {
		dma_fifo[dma_fifo_writepointer].state = DMA_DATA_STATE_READY;
		
		rs485_trigger_dma_transfer();
		//try to push the new data
		if(DMA_DATA_MSG_SIZE - dma_fifo[dma_fifo_writepointer].level > size) {
			// data could be send with this buffer
			memcpy(&dma_fifo[dma_fifo_writepointer].msg[dma_fifo[dma_fifo_writepointer].level],data,size);
			dma_fifo[dma_fifo_writepointer].level += size;
			data_in_buffer = true;
		} else {
			//lets send this buffer
			dma_fifo[dma_fifo_writepointer].state = DMA_DATA_STATE_FULL;
				
			//set the next element
			if(dma_fifo_writepointer+1>=DMA_DATA_BUFFERS) {
				dma_fifo_writepointer=0;
			} else {
				dma_fifo_writepointer++;
			}
				
			if(dma_fifo[dma_fifo_writepointer].state == DMA_DATA_STATE_NEW) {
				//try to push the new data
				if(DMA_DATA_MSG_SIZE - dma_fifo[dma_fifo_writepointer].level > size) {
					// data could be send with this buffer
					memcpy(&dma_fifo[dma_fifo_writepointer].msg[dma_fifo[dma_fifo_writepointer].level],data,size);
					dma_fifo[dma_fifo_writepointer].level += size;
					data_in_buffer = true;
					dma_fifo[dma_fifo_writepointer].state = DMA_DATA_STATE_READY;
				}
			}
		}
	}
		
	if(!data_in_buffer) {
		//data loss
		temp++;
	}
	
	return data_in_buffer;
}

void rs485_init()
{
	#ifdef CONF_BOARD_ENABLE_USARTC0
	
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTC, 3), IOPORT_DIR_OUTPUT	| IOPORT_INIT_HIGH);
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTC, 2), IOPORT_DIR_INPUT);
		
	//rs485 enable
	usart_rs232_options_t usart_opt;
	
	usart_opt.baudrate = 115200;
	usart_opt.charlength = USART_CHSIZE_8BIT_gc;
	usart_opt.paritytype = USART_PMODE_DISABLED_gc;
	usart_opt.stopbits = true;
	
	usart_init_rs232(&RS485_UART,&usart_opt);
	
	rs485_init_dma();
	
	gpio_set_pin_high(DIR_RS485_O);
	
	#endif
}

void rs485_deinit(void)
{
	#ifdef CONF_BOARD_ENABLE_USARTC0
	
	//clear fifo buffer is need for stopping dma transfer really at the next command
	for(uint8_t i=0;i<DMA_DATA_BUFFERS;i++) {
		dma_fifo[i].state = DMA_DATA_STATE_NEW;
		dma_fifo[i].level = 0;
	}
	
	while(dma_channel_is_busy(RS485_DMA_CHANNEL));
	
	dma_disable();
	
	gpio_set_pin_low(DIR_RS485_O);
	
	usart_tx_disable(&RS485_UART);
	usart_rx_disable(&RS485_UART);
	
	sysclk_disable_peripheral_clock(&RS485_UART);	
	
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTC, 3), IOPORT_DIR_INPUT);
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTC, 2), IOPORT_DIR_INPUT);
	
	#endif
}