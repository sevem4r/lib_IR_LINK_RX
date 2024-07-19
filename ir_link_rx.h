/*
 * ir_link_rx.h
 *
 *  Created on: 10 mar 2022
 *      Author: Mariusz
 */

#ifndef IR_LINK_RX_H_
#define IR_LINK_RX_H_

#include <inttypes.h>

typedef struct{
	uint8_t address;
	uint8_t data0to2bit;
	uint8_t data0to8bit1;
	uint8_t data0to8bit2;
}ir_link_rx_payload;

typedef void (*callback_ir_link_tx_on_rx)(ir_link_rx_payload* payload);

void ir_link_rx_event(void);
void ir_link_rx_set_on_rx_callback(callback_ir_link_tx_on_rx callback);
void ir_link_rx_init();

#define IR_LINK_RX_DBG	0

#if IR_LINK_RX_DBG
#define IR_LINK_RX_DBG_SIZE		200
extern volatile uint16_t dbg_time[IR_LINK_RX_DBG_SIZE];
extern volatile uint8_t dbg_edge[IR_LINK_RX_DBG_SIZE];
extern volatile uint8_t dbg_ready;
#endif

#endif /* IR_LINK_RX_H_ */
