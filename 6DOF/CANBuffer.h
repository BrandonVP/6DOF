// CANBuffer.h

#ifndef _CANBuffer_h
#define _CANBuffer_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class CANBuffer
{
 protected:
	 uint16_t ID;
	 uint8_t CANFrame[8];

 public:
	 CANBuffer(uint16_t, uint8_t*);
	 void setID(uint16_t);
	 uint16_t getID();
	 void setMessage(uint8_t*);
	 uint8_t* getMessage();

};


#endif

