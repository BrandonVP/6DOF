//#include "Actuator.h"

using namespace std;

class Actuator
{
private:
	uint8_t g_code;           // Set g_code mode for next cycle  
	int current_angle;        // Angle of actuators position in degrees
	int long steps_to_move;   // Set quantity of steps to move in next cycle
	bool actuator_direction;  // Current direction of actuator
	bool enable_actuator;     // Enable actuator to accept pulse
	double actuator_speed;    // Set individual actuator speed
public:
	void actuator(int actuator_value) {
	// Use value in switch statement to decide which stored position to retrieve from txt file
	}

	void set_g_gode(uint8_t g_code) {
		this->g_code = g_code;
	}
	uint8_t get_g_gode() {
		return g_code;
	}
	void set_current_angle(int current_angle) {
		this->current_angle = current_angle;
	}
	int get_current_angle() {
		if (current_angle) {
			return current_angle;
		}
		else if (current_angle) { // If current angle is in txt file
			//get current_angle from txt file
			return current_angle;
		}
		else {
			return;
		}
	}
	void set_steps_to_move(int long steps_to_move) {
		this->steps_to_move = steps_to_move;
	}
	int long get_steps_to_move() {
		return steps_to_move;
	}
	void set_actuator_direction(bool actuator_direction) {
		this->actuator_direction = actuator_direction;
	}
	bool get_actuator_direction() {
		return actuator_direction;
	}
	void set_enable_actuator(bool enable_actuator) {
		this->enable_actuator = enable_actuator;
	}
	bool get_enable_actuator() {
		return enable_actuator;
	}
	void set_actuator_speed(double actuator_speed) {
		this->actuator_speed = actuator_speed;
	}
	double get_actuator_speed() {
		return actuator_speed;
	}
};