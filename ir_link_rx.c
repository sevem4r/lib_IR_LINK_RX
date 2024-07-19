/*
 * ir_link_rx.c
 *
 *  Created on: 10 mar 2022
 *      Author: Mariusz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "ir_link_rx.h"

#define IR_LINK_RX_BYTES 	4
#define IR_LINK_RX_BITS		32
#define IR_LINK_RX_SAMPLES	(IR_LINK_RX_BITS * 2 + 1)//58

#if IR_LINK_RX_DBG
volatile uint16_t dbg_time[IR_LINK_RX_DBG_SIZE];
volatile uint8_t dbg_edge[IR_LINK_RX_DBG_SIZE];
volatile uint16_t dbg_cnt;
volatile uint8_t dbg_ready;
#endif

enum IR_LINK_RX_DATA{
	IR_LINK_RX_DATA_EMPTY,
	IR_LINK_RX_DATA_PENDING
};

static volatile uint16_t ir_link_rx_time[IR_LINK_RX_SAMPLES];
static volatile uint8_t ir_link_rx_cnt;
static volatile uint8_t ir_link_rx_status;
static callback_ir_link_tx_on_rx ir_link_rx_on_rx_callback;

void ir_link_rx_set_on_rx_callback(callback_ir_link_tx_on_rx callback){
	ir_link_rx_on_rx_callback = callback;
}

static uint8_t ir_link_rx_CRC8(uint8_t* bytes, uint8_t size){
    const uint8_t generator = 0x1D;
    uint8_t crc = 0; // start with 0 so first byte can be 'xored' in

    for(uint8_t i = 0; i < size; i++){
        crc ^= bytes[i]; // XOR-in the next input byte

        for(uint8_t j = 0; j < 8; j++){
            if((crc & 0x80) != 0){
                crc = (uint8_t)((crc << 1) ^ generator);
            }
            else{
                crc <<= 1;
            }
        }
    }

    return crc;
}

void ir_link_rx_event(void){
	if(ir_link_rx_on_rx_callback){
		if(ir_link_rx_status == IR_LINK_RX_DATA_PENDING){
			uint16_t time_bit_zero;
			uint16_t time_bit_one;

			// 2 start bits
			// calculate bit time
			// time[0] - time between frames - invalid
			time_bit_zero = (ir_link_rx_time[1] +
							ir_link_rx_time[2] +
							ir_link_rx_time[3] +
							ir_link_rx_time[4]) / 4;
			// calculate full bit time
			time_bit_one = time_bit_zero * 2;
			// restore last bit time
			ir_link_rx_time[IR_LINK_RX_SAMPLES - 1] = time_bit_zero;

			// bits decode
			uint16_t delta1, delta2;
			uint32_t ir_data = 0;

			for(uint8_t i = 0; i < 32; i++){
				delta1 = abs(ir_link_rx_time[i * 2 + 1] - time_bit_one);
				delta2 = abs(ir_link_rx_time[i * 2 + 1] - time_bit_zero);

				// one
				if(delta1 < delta2){
					ir_data = (ir_data << 1) | 0x01;
				}
				// zero
				else{
					ir_data = (ir_data << 1);
				}
			}

			ir_link_rx_payload payload;
			uint8_t crc;
			uint8_t data[IR_LINK_RX_BYTES] = {
				(ir_data >> 26) & 0x0F,
				(ir_data >> 18) & 0xFF,
				(ir_data >> 10) & 0xFF,
				(ir_data >> 2) & 0xFF
			};

			crc = ir_link_rx_CRC8( (uint8_t*)data, IR_LINK_RX_BYTES - 1);

			if(crc == data[IR_LINK_RX_BYTES - 1]){

				payload.address 		= (data[0] >> 2) & 0x03;
				payload.data0to2bit 	= data[0] & 0x03;
				payload.data0to8bit1 	= data[1];
				payload.data0to8bit2 	= data[2];

				ir_link_rx_on_rx_callback(&payload);
			}

			ir_link_rx_status = IR_LINK_RX_DATA_EMPTY;
		}
	}
}

void ir_link_rx_init(void){
	TCCR1B |= (1<<CS10); // Normal, 0,125us
	TIMSK |= (1<<TICIE1);

	TCCR1B &= ~(1<<ICES1);
}

ISR(TIMER1_CAPT_vect){
	uint16_t tcnt;

	tcnt = TCNT1;
	TCNT1 = 0;

	TCCR1B ^= (1<<ICES1);

	if(TIFR & (1<<TOV1)){
		ir_link_rx_cnt = 0;

		TIFR |= (1<<TOV1);
	}

	if(ir_link_rx_cnt < (IR_LINK_RX_SAMPLES - 1) && ir_link_rx_status == IR_LINK_RX_DATA_EMPTY){
		ir_link_rx_time[ir_link_rx_cnt] = tcnt;

		ir_link_rx_cnt++;

		if(ir_link_rx_cnt == (IR_LINK_RX_SAMPLES - 1)){
			PORTD ^= (1<<PD5);

			ir_link_rx_status = IR_LINK_RX_DATA_PENDING;
		}
	}
}
