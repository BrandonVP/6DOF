/*
 ===========================================================================
 Name        : Actuator.cpp
 Author      : Brandon Van Pelt
 Created	 : 
 Description : Create objects to control each stepper motor axis
 ===========================================================================
 */

#include "Actuator.h"

// Default constructor
Actuator::Actuator(uint8_t pPin, uint8_t dPin, uint8_t ePin, uint16_t min, uint16_t max, uint16_t startingAngle)
{
	pulsePin = pPin;
	directionPin = dPin;
	enablePin = ePin;
	minAngle = min;
	maxAngle = max;
	currentAngle = startingAngle;
	nextAngle = currentAngle;
	currentSteps = startingAngle * STEPS_PER_DEGREE;
	nextSteps = currentSteps;
}

// Calculate steps needed to reach next angle command
void Actuator::set_actuator(uint32_t newAngle)
{
	// Block a set if run is already active to prevent changing steps mid run and range check incoming value
	if (readyToMove == true || newAngle > 360 || newAngle < 1) { return; }

	// Convert degrees to steps
	uint32_t newSteps = newAngle * STEPS_PER_DEGREE;

	// Set actuator to new currentAngle once move is complete
	nextSteps = newSteps;
	nextAngle = newAngle;

	// Set direction
	(newSteps < currentSteps) ? actuatorDirection = false : actuatorDirection = true;

	// Check min and max. If reached the movement value will be changes to either min or max 
	// that way the program will still run but not exceed the max or min angle.
	/*
	if ((actuatorDirection) && (newAngle > MAX_DEGREES_IN_STEPS))
	{
		newSteps = MAX_DEGREES_IN_STEPS;
	}
	else if ((!actuatorDirection) && (actuatorDirection < MIN_DEGREES_IN_STEPS))
	{
		newSteps = MIN_DEGREES_IN_STEPS;
	}
	*/

	// Calculate distance to move
	if (newSteps >= currentSteps)
	{
		newSteps = newSteps - currentSteps;
	}
	else if (newSteps < currentSteps)
	{
		newSteps = currentSteps - newSteps;
	}

	// Assign newly calculated value
	stepsToMove = newSteps;

	if (stepsToMove > 0)
	{
		// Disables set_actuator until run is called
		readyToMove = true;
	}
}

// Set actuator to new position once move is complete
void Actuator::move()
{
	currentAngle = nextAngle;
	currentSteps = nextSteps;
	readyToMove = false;
}

bool Actuator::addSteps(uint32_t newSteps)
{
	if (actuatorDirection)
	{
		currentSteps += newSteps;
	}
	else
	{
		currentSteps -= newSteps;
	}
}

bool Actuator::set_deg(uint16_t newAngle) 
{
	this->currentAngle = newAngle;
	return true;
}

bool Actuator::increment_deg() 
{
	(this->get_direction()) ? this->currentAngle++ : this->currentAngle--;
	return true;
}

uint16_t Actuator::get_deg() 
{
	return ((currentSteps / 337.7777) + 1);
}

void Actuator::set_steps(uint32_t newSteps) 
{
	this->stepsToMove = newSteps;
}

uint32_t Actuator::get_steps() 
{
	return stepsToMove;
}

void Actuator::reduceSteps()
{
	if (stepsToMove > 0)
	{
		stepsToMove--;
	}
}

void Actuator::set_direction(bool actuatorDirection) 
{
	this->actuatorDirection = actuatorDirection;
}

bool Actuator::get_direction() 
{
	return actuatorDirection;
}

void Actuator::set_enable(bool enableActuator) 
{
	this->enableActuator = enableActuator;
}

bool Actuator::get_enable() 
{
	return enableActuator;
}
