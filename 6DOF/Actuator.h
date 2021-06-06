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
	int currentAngle;        // Angle of actuators position in degrees
	int nextAngle;			 // Angle moved to current once sent to G1-3
	int maxAngle = 360;
	int minAngle = 0;
	unsigned int long stepsToMove;   // Set quantity of steps to move in next cycle
	double actuatorSpeed;    // Set individual actuator speed
	bool actuatorDirection;  // Current direction of actuator
	bool enableActuator;     // Enable actuator to accept pulse
	bool readyToMove = false;

public:
	void move();
	Actuator(uint16_t, uint16_t, uint16_t);
	void set_actuator(uint16_t new_pos_x1);
	void set_current_angle(uint16_t current_angle);
	int get_current_angle();
	void set_steps_to_move(uint32_t steps_to_move);
	int long get_steps_to_move();
	bool increment_current_angle();
	void reduceSteps();
	void set_actuator_direction(bool actuator_direction);
	bool get_actuator_direction();
	void set_enable_actuator(bool enable_actuator);
	bool get_enable_actuator();
};

#endif

