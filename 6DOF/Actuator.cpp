// 
// 
// 

#include "Actuator.h"

Actuator::Actuator(uint8_t pPin, uint8_t dPin, uint8_t ePin, uint16_t min, uint16_t max, uint16_t startingAngle)
{
	pulsePin = pPin;
	directionPin = dPin;
	enablePin = ePin;
	minAngle = min;
	maxAngle = max;
	currentAngle = startingAngle;
}

// new_pos_x1 is an angle
void Actuator::set_actuator(float new_pos_x1)
{
	// Is there already a move quened?
	if (readyToMove == true) {return;}

	// Set direction
	(new_pos_x1 < currentAngle) ? actuatorDirection = false : actuatorDirection = true;

	// Check min and max. If reached the movement value will be changes to either min or max 
	// that way the program will still run but not exceed the max or min angle.
	if ((actuatorDirection) && (currentAngle - new_pos_x1 != 0) && (new_pos_x1 > this->maxAngle))
	{
		new_pos_x1 = this->maxAngle;
	}
	else if ((currentAngle - new_pos_x1 != 0) && (new_pos_x1 < this->minAngle))
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

void Actuator::set_deg(uint16_t currentAngle) {
	this->currentAngle = currentAngle;
}

bool Actuator::increment_deg() 
{
	(this->get_direction()) ? this->currentAngle++ : this->currentAngle--;
	return true;
}

uint16_t Actuator::get_deg() {
	return currentAngle;
}

void Actuator::set_steps(uint32_t stepsToMove) {
	this->stepsToMove = stepsToMove;
}

uint32_t Actuator::get_steps() {
	return stepsToMove;
}

void Actuator::reduceSteps()
{
	stepsToMove--;
}

void Actuator::set_direction(bool actuatorDirection) {
	this->actuatorDirection = actuatorDirection;
}

bool Actuator::get_direction() {
	return actuatorDirection;
}

void Actuator::set_enable(bool enableActuator) {
	this->enableActuator = enableActuator;
}

bool Actuator::get_enable() {
	return enableActuator;
}
