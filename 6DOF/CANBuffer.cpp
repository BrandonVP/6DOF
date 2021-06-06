// 
// 
// 
#include "CANBuffer.h"

//
CANBuffer::CANBuffer(uint16_t incomingID, uint8_t* incomingMSG)
{
	ID = incomingID;
	for (int i = 0; i < 8; i++)
	{
		CANFrame[i] = incomingMSG[i];
	}
}

//
void CANBuffer::setID(uint16_t incomingID)
{
	ID = incomingID;
}

//
uint16_t CANBuffer::getID()
{
	return ID;
}

//
void CANBuffer::setMessage(uint8_t * incomingMSG)
{
	for (int i = 0; i < 8; i++)
	{
		CANFrame[i] = incomingMSG[i];
	}
}

//
uint8_t* CANBuffer::getMessage()
{
	return CANFrame;
}



