/*
 * buffer.c
 *
 *  Created on: Sep 3, 2021
 *      Author: bvanpelt
 */
#include "can_buffer.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define CAN_BUFFER_C_

uint8_t bufferOutPtr = 0;
uint8_t bufferInPtr = 0;

struct CAN_Buffer rxBuffer[BUFFER_SIZE];

bool push(struct CAN_Buffer addFrame)
{
	// Copy message
	rxBuffer[bufferInPtr].id = addFrame.id;
	rxBuffer[bufferInPtr].length = addFrame.length;
	for(uint8_t i = 0; i < 8; i++)
	{
		rxBuffer[bufferInPtr].data[i] = addFrame.data[i];
	}

	bufferInPtr++;
	// End of circular buffer
	if (bufferInPtr == (BUFFER_SIZE - 1))
	{
		bufferInPtr = 0;
	}
	// Overflow case
	if (bufferInPtr == bufferOutPtr)
	{
		bufferOutPtr++;
		// Let user know an overwrite occurred
		return false;
	}
	return true;
}

void pop(struct CAN_Buffer *bufOut)
{
	// Copy message
	bufOut->id = rxBuffer[bufferOutPtr].id;
	bufOut->length = rxBuffer[bufferInPtr].length;
	for(uint8_t i = 0; i < 8; i++)
	{
		bufOut->data[i] = rxBuffer[bufferInPtr].data[i];
	}

	// Check if empty
	if (bufferOutPtr != bufferInPtr)
	{
		bufferOutPtr++;

		// End of circular buffer
		if (bufferOutPtr > BUFFER_SIZE - 1)
		{
			bufferOutPtr = 0;
		}
	}
}

uint8_t stack_size(void)
{
	uint8_t size = (bufferInPtr - bufferOutPtr) & (BUFFER_SIZE - 1);
	return size;
}


void peek(struct CAN_Buffer *peek)
{
	peek->id = rxBuffer[bufferOutPtr].id;
	peek->length = rxBuffer[bufferInPtr].length;
	for(uint8_t i = 0; i < 8; i++)
	{
		peek->data[i] = rxBuffer[bufferInPtr].data[i];
	}
}

void clear_buffer()
{
	bufferOutPtr = 0;
	bufferInPtr = 0;
}
