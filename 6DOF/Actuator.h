// Actuator.h

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

class Actuator
{
protected:
	float currentAngle = 0;       
	float nextAngle = 0;			 
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
	void set_actuator(float new_pos_x1);
	void set_current_angle(uint16_t current_angle);
	void set_steps_to_move(uint32_t steps_to_move);
	void set_actuator_direction(bool actuator_direction);
	void set_enable_actuator(bool enable_actuator);
	uint16_t get_current_angle();
	uint32_t get_steps_to_move();
	bool increment_current_angle();
	bool get_actuator_direction();
	bool get_enable_actuator();
};

#endif

