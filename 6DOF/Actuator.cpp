// 
// 
// 

#include "Actuator.h"

//#define STEPS_PER_ROTATION 200		  // Amount of steps needed for one NEMA stepper motor rotation
//#define ACTUATOR_GEAR_RATIO 38		  // Gear reduction for lower actuators
//#define ACTUATOR_GEAR_RATIO_UPPER 4  // Gear reduction for upper actuators - Needs to be determined
//#define DRIVER_STEPS 16			  // Current driver hardware pin settings from 1 to 32. (fractions of 1/1 - 1/32)
//#define DEGREES_IN_CIRCLE 360		  //
#define STEPS_PER_DEGREE 337.777      //(DRIVER_STEPS * STEPS_PER_ROTATION * ACTUATOR_GEAR_RATIO) / DEGREES_IN_CIRCLE;
										  // 1/32 steps = 675.555~
										  // 1/16 steps = 337.777~

Actuator::Actuator(uint16_t min, uint16_t max, uint16_t startingAngle)
{
	minAngle = min;
	maxAngle = max;
	currentAngle = startingAngle;
}

// new_pos_x1 is an angle
void Actuator::set_actuator(uint16_t new_pos_x1)
{

	// Is there already a move quened?
	if (readyToMove == true) {return;}

	// Set direction
	(new_pos_x1 < currentAngle) ? actuatorDirection = false : actuatorDirection = true;

	// Check min and max. If reached the movement value will be changes to either min or max 
	// that way the program will still run but not exceed the max or min angle.
	if (actuatorDirection && currentAngle - new_pos_x1 != 0 && new_pos_x1 > this->maxAngle)
	{
		new_pos_x1 = this->maxAngle;
	}
	else if (currentAngle - new_pos_x1 != 0 && new_pos_x1 < this->minAngle)
	{
		new_pos_x1 = this->minAngle;
	}

	// Calculate distance to move
	int dis_to_move = new_pos_x1 - currentAngle;

	// Already assigned direction, find ABS
	if (dis_to_move < 0) 
	{
		dis_to_move = dis_to_move * ((dis_to_move > 0) - (dis_to_move < 0));
	}

	// Convert degrees to steps
	stepsToMove = STEPS_PER_DEGREE * dis_to_move;

	// Enable if steps are greater than 0
	(dis_to_move == 0) ? enableActuator = false : enableActuator = true;

	// Set actuator to new currentAngle
	nextAngle = new_pos_x1;

	// Disables set_actuator until run is called
	readyToMove = true;
}

void Actuator::move()
{
	currentAngle = nextAngle;
	readyToMove = false;
}

void Actuator::set_current_angle(uint16_t currentAngle) {
	this->currentAngle = currentAngle;
}

bool Actuator::increment_current_angle() 
{
	(this->get_actuator_direction()) ? this->currentAngle++ : this->currentAngle--;
	return true;
}

int Actuator::get_current_angle() {
	return currentAngle;
}

void Actuator::set_steps_to_move(uint32_t stepsToMove) {
	this->stepsToMove = stepsToMove;
}

int long Actuator::get_steps_to_move() {
	return stepsToMove;
}

void Actuator::reduceSteps()
{
	stepsToMove--;
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
