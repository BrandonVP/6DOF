#pragma once
#ifndef ACTUATOR_H
#define ACTUATOR_H
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
	void set_actuator(int new_pos_x1);
	void actuator(int new_pos);
	void set_g_gode(int g_code);
	uint8_t get_g_gode();
	void set_current_angle(int current_angle);
	int get_current_angle();
	void set_steps_to_move(int long steps_to_move);
	int long get_steps_to_move();
	void set_actuator_direction(bool actuator_direction);
	bool get_actuator_direction();
	void set_enable_actuator(bool enable_actuator);
	bool get_enable_actuator();
	void set_actuator_speed();
	double get_actuator_speed();
};
#endif

