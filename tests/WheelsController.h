#ifndef wheels_controllerH
#define wheels_controllerH

#include "jenny5_command_module.h"
#include "jenny5_events.h"
#include <iostream>

//----------------------------------------------------------------
#define NO_WAIT -1
#define MOTOR_FOOT_LEFT 0
#define MOTOR_FOOT_RIGHT 1
#define WHEELS_MOVING 0
#define WHEELS_ALIGNING 1
#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3
//----------------------------------------------------------------

class t_wheels_controller {
private:
	t_jenny5_command_module wheels_motors_controller;
	double distance_to_person = 150; //cm
	double modify_distance_by = 50;  //cm
	double minimun_distance_allowed = 20;  //cm
	double maximum_distance_allowed = 300;  //cm

	int compute_steps_from_distance(int current_distance);

public:
	t_wheels_controller(void);
	~t_wheels_controller(void);

	bool connect();
	bool setup();
	void wait_for_action(int wait_for);
	
	void turn_right(int displacement, int wait_for);
	void turn_left(int displacement, int wait_for);
	
	void move_forward(int current_distance, int wait_for);
	void move_backwards(int current_distance, int wait_for);
	
	double get_minimum_distance_allowed();
	double get_maximum_distance_allowed();
	double get_distance_to_person();
	void set_minimum_distance_allowed(double new_distance);
	void set_maximum_distance_allowed(double new_distance);
	void set_distance_to_person(double new_distance);
	double get_modify_distance_by();
	void set_modify_distance_by(double new_distance);
};

#endif