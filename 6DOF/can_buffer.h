/*
 Name:		buffer.h
 Created:	9/3/2021 9:36:16 PM
 Author:	Brandon Van Pelt
*/

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifndef CAN_BUFFER_H_
#define CAN_BUFFER_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#define BUFFER_SIZE 32

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
	struct CAN_Frame rxBuffer[BUFFER_SIZE];
public:
	bool push(struct CAN_Frame);
	void pop(struct CAN_Frame* bufOut);
	void peek(struct CAN_Frame*);
	void clear_buffer(void);
	uint8_t stack_size(void);
};

#endif // BUFFER_H_
