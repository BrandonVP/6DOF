#include <math.h>

const int DRIVER_STEPS;
const int STEPS_PER_ROTATION;
const int ACTUATOR_GEAR_RATIO;
const int DEGREES_IN_CIRCLE = 360;
const double  STEPS_PER_DEGREE = 675.555; // (DRIVER_STEPS * STEPS_PER_ROTATION * ACTUATOR_GEAR_RATIO) / DEGREES_IN_CIRCLE = 675.555~

using namespace std;

class Actuator
{
private:
	int g_code;           // Set g_code mode for next cycle  
	int current_angle;        // Angle of actuators position in degrees
	int long steps_to_move;   // Set quantity of steps to move in next cycle
	bool actuator_direction;  // Current direction of actuator
	bool enable_actuator;     // Enable actuator to accept pulse
	double actuator_speed;    // Set individual actuator speed
public:
	void actuator(int actuator_value) {
	// Use value in switch statement to decide which stored position to retrieve from txt file
	}

	void set_actuator(int new_pos_x1) {
		// Set direction
		if (new_pos_x1 < current_angle) {
			actuator_direction = false;
		}
		else {
			actuator_direction = true;
		}

		// Calculate distance to move
		int dis_to_move = new_pos_x1 - current_angle;
		if (dis_to_move < 0) {
			dis_to_move = dis_to_move - (2 * dis_to_move);
		}

		// Convert degrees to steps
		steps_to_move = STEPS_PER_DEGREE * dis_to_move;

		// Enable is steps are greater than 0
		if (dis_to_move == 0) {
			enable_actuator = false;
		}
		else {
			enable_actuator = true;
		}

		// Set actuator to new current_angle
		current_angle = new_pos_x1;
	}

	void set_g_gode(int g_code) {
		this->g_code = g_code;
	}

	int get_g_gode() {
		return g_code;
	}

	void set_current_angle(int current_angle) {
		this->current_angle = current_angle;
	}

	int get_current_angle() {

			return current_angle;
	
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