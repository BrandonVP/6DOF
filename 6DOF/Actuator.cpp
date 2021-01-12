// 
// 
// 

#include "Actuator.h"

#define STEPS_PER_ROTATION 200		  // Amount of steps needed for one NEMA stepper motor rotation
#define ACTUATOR_GEAR_RATIO 38		  // Gear reduction for lower actuators
#define ACTUATOR_GEAR_RATIO_UPPER 4  // Gear reduction for upper actuators - Needs to be determined
#define DRIVER_STEPS 16			  // Current driver hardware pin settings from 1 to 32. (fractions of 1/1 - 1/32)
#define DEGREES_IN_CIRCLE 360		  //
#define STEPS_PER_DEGREE 337.777      //(DRIVER_STEPS * STEPS_PER_ROTATION * ACTUATOR_GEAR_RATIO) / DEGREES_IN_CIRCLE;
										  // 1/32 steps = 675.555~
										  // 1/16 steps = 337.777~

// new_pos_x1 is an angle
void Actuator::set_actuator(int new_pos_x1) {
	if (readyToMove == true)
	{
		return;
	}
	// Set direction
	if (new_pos_x1 < currentAngle) {
		actuatorDirection = false;
	}
	else {
		actuatorDirection = true;
	}

	// Calculate distance to move
	int dis_to_move = new_pos_x1 - currentAngle;
	if (dis_to_move < 0) {
		//dis_to_move = dis_to_move - (2 * dis_to_move);
		dis_to_move = dis_to_move * ((dis_to_move > 0) - (dis_to_move < 0));
	}

	// Convert degrees to steps
	stepsToMove = STEPS_PER_DEGREE * dis_to_move;

	// Enable if steps are greater than 0
	if (dis_to_move == 0) {
		enableActuator = false;
	}
	else {
		enableActuator = true;
	}

	// Set actuator to new currentAngle
	nextAngle = new_pos_x1;

	readyToMove = true;
}

void Actuator::move()
{
	currentAngle = nextAngle;
	readyToMove = false;
}

void Actuator::set_current_angle(int currentAngle) {
	this->currentAngle = currentAngle;
}

int Actuator::get_current_angle() {
	return currentAngle;
}

void Actuator::set_steps_to_move(int long stepsToMove) {
	this->stepsToMove = stepsToMove;
}

int long Actuator::get_steps_to_move() {
	return stepsToMove;
}

void Actuator::set_actuator_direction(bool actuatorDirection) {
	this->actuatorDirection = actuatorDirection;
}

bool Actuator::get_actuator_direction() {
	return actuatorDirection;
}

void Actuator::set_enable_actuator(bool enableActuator) {
	this->enableActuator = enableActuator;
}

bool Actuator::get_enable_actuator() {
	return enableActuator;
}
