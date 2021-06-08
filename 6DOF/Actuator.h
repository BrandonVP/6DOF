// Actuator.h

#ifndef _Actuator_h
#define _Actuator_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class Actuator
{
protected:
	uint16_t currentAngle = 0;       
	uint16_t nextAngle = 0;			 
	uint16_t maxAngle;
	uint16_t minAngle;
	uint32_t stepsToMove = 0;   
	bool actuatorDirection = true;		
	bool enableActuator = true;    
	bool readyToMove = false;

public:
	Actuator(uint16_t, uint16_t, uint16_t);
	void move();
	void reduceSteps();
	void set_actuator(uint16_t new_pos_x1);
	void set_current_angle(uint16_t current_angle);
	void set_steps_to_move(uint32_t steps_to_move);
	void set_actuator_direction(bool actuator_direction);
	void set_enable_actuator(bool enable_actuator);
	int get_current_angle();
	int long get_steps_to_move();
	bool increment_current_angle();
	bool get_actuator_direction();
	bool get_enable_actuator();
};

#endif

