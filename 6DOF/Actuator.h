/*
 ===========================================================================
 Name        : Actuator.h
 Author      : Brandon Van Pelt
 Created	 :
 Description : Create objects to control each stepper motor axis
 ===========================================================================
 */

#ifndef _Actuator_h
#define _Actuator_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//#define STEPS_PER_ROTATION 200		  // Amount of steps needed for one NEMA stepper motor rotation
//#define ACTUATOR_GEAR_RATIO 38		  // Gear reduction for lower actuators
//#define ACTUATOR_GEAR_RATIO_UPPER 4  // Gear reduction for upper actuators - Needs to be determined
//#define DRIVER_STEPS 16			  // Current driver hardware pin settings from 1 to 32. (fractions of 1/1 - 1/32)
//#define DEGREES_IN_CIRCLE 360		  //
#define STEPS_PER_DEGREE 337.777      //(DRIVER_STEPS * STEPS_PER_ROTATION * ACTUATOR_GEAR_RATIO) / DEGREES_IN_CIRCLE;
										  // 1/32 steps = 675.555~
										  // 1/16 steps = 337.777~
#define DEGREE_STEPS 337

class Actuator
{
protected:
	uint8_t pulsePin;
	uint8_t directionPin;
	uint8_t enablePin;
	uint16_t maxAngle;
	uint16_t minAngle;
	float currentAngle;     

	float nextAngle = 0;	
	uint32_t stepsToMove = 0;   
	bool actuatorDirection = true;		
	bool enableActuator = true;    
	bool readyToMove = false;

public:
	Actuator(uint8_t, uint8_t, uint8_t, uint16_t, uint16_t, uint16_t);
	void move();
	void reduceSteps();
	void set_actuator(float);
	bool set_deg(uint16_t);
	void set_steps(uint32_t);
	void set_direction(bool);
	void set_enable(bool);
	uint16_t get_deg();
	uint32_t get_steps();
	bool increment_deg();
	bool get_direction();
	bool get_enable();
};

#endif

