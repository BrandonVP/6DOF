/*
 * buffer.h
 *
 *  Created on: Sep 3, 2021
 *      Author: bvanpelt
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifndef CAN_BUFFER_H_
#define CAN_BUFFER_H_

#define BUFFER_SIZE 32

struct CAN_Buffer
{
	uint8_t data[8];
	uint8_t length;
    uint16_t id;
};

#ifdef	CAN_BUFFER_C_

bool push(struct CAN_Buffer);
void pop(struct CAN_Buffer *bufOut);
uint8_t stack_size(void);
void peek(struct *CAN_Buffer)
void clear_buffer(void);

#else


extern bool push(struct CAN_Buffer);
extern void pop(struct CAN_Buffer*);
extern uint8_t stack_size(void);
void peek(struct CAN_Buffer*);
extern void clear_buffer(void);
#endif


#endif // BUFFER_H_
