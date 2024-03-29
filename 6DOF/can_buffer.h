/*
 ===========================================================================
 Name        : buffer.h
 Author      : Brandon Van Pelt
 Created	 : 9/3/2021
 Description : Ring buffer for CAN Bus messages
 ===========================================================================
 */

#ifndef CAN_BUFFER_H_
#define CAN_BUFFER_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#define BUFFER_SIZE 0x1F

struct CAN_Frame
{
	volatile uint16_t id;
	volatile uint8_t data[8];
};

class can_buffer
{
private:
	uint8_t bufferOutPtr = 0;
	uint8_t bufferInPtr = 0;
	struct CAN_Frame rxBuffer[BUFFER_SIZE + 1];
public:
	void push(long unsigned int, byte*);
	void pop(struct CAN_Frame* bufOut);
	void peek(struct CAN_Frame*);
	void clear_buffer(void);
	uint8_t stack_size(void);
};

#endif // BUFFER_H_
