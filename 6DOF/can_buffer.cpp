/*
 Name:		buffer.c
 Created:	9/3/2021 9:36:16 PM
 Author:	Brandon Van Pelt
*/

#include "can_buffer.h"

#define CAN_BUFFER_C_

//#define DEBUG_PUSH
// Copies provided structure into the buffer
void can_buffer::push(struct CAN_Frame addFrame)
{
	// Copy message
	rxBuffer[bufferInPtr].id = addFrame.id;
	memcpy((void*)rxBuffer[bufferInPtr].data, (const void*)addFrame.data, 8);

	// Increment bufferInPtr
	bufferInPtr = ((bufferInPtr + 1) & BUFFER_SIZE);

	// Overflow case
	if (bufferInPtr == bufferOutPtr)
	{
		bufferOutPtr = ((bufferOutPtr + 1) & BUFFER_SIZE);
	}

#if defined DEBUG_PUSH
	Serial.println("push");
	Serial.print("New InPtr Position: ");
	Serial.println(bufferInPtr);
	Serial.print("New OutPtr Position: ");
	Serial.println(bufferOutPtr);
#endif
}

//#define DEBUG_POP
// Assign next structure in buffer to provided structure
void can_buffer::pop(struct CAN_Frame* bufOut)
{
	// Copy message
	bufOut->id = rxBuffer[bufferOutPtr].id;
	memcpy((void*)bufOut->data, (const void*)rxBuffer[bufferOutPtr].data, 8);
	bufferOutPtr = ((bufferOutPtr + 1) & BUFFER_SIZE);
#if defined DEBUG_PUSH
	Serial.println("pop");
	Serial.print("New OutPtr Position: ");
	Serial.println(bufferOutPtr);
#endif
}

// Calculates current structures in buffer by subtracting points then anding with max buffer size value
uint8_t can_buffer::stack_size(void)
{
	uint8_t size = (bufferInPtr - bufferOutPtr) & BUFFER_SIZE;
	return size;
}

// Check the next structure int he buffer without incrementing
void can_buffer::peek(struct CAN_Frame* peek)
{
	peek->id = rxBuffer[bufferOutPtr].id;
	memcpy((void*)peek->data, (const void*)rxBuffer[bufferInPtr].data, 8);
}

// Reset both points back to zero
void can_buffer::clear_buffer()
{
	bufferOutPtr = 0;
	bufferInPtr = 0;
}
