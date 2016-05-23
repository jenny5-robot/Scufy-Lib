#include "../include/WheelsController.h"
#include <time.h>

//----------------------------------------------------------------

t_wheels_controller::t_wheels_controller(void) {
	if (connect()) {
		printf("Connected to wheels...\n");
	}
	if (setup()) {
		printf("Wheels setup complete...\n");
	}
}
//----------------------------------------------------------------

bool t_wheels_controller::connect() {
	//connect to serial ports
	if (!wheels_motors_controller.connect(2, 115200)) {
		printf("Error attaching to Jenny 5' wheels!\n");
	}

	//jenny5_event connect_to_wheels_motors_event(IS_ALIVE_EVENT);
	//return wheels_motors_controller.wait_for_command_completion(connect_to_wheels_motors_event);

	// now wait to see if I have been connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();
	bool foot_responded = false;

	while (1) {
		if (!wheels_motors_controller.update_commands_from_serial())
			Sleep(5);
		if (!foot_responded)
			if (wheels_motors_controller.query_for_event(IS_ALIVE_EVENT, 0))  // have we received the event from Serial ?
				foot_responded = true;

		if (foot_responded)
			break;
		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!foot_responded)
				printf("Foot does not respond! Game over!\n");
			return false;
		}
	}
	return true;
}
//----------------------------------------------------------------

bool t_wheels_controller::setup() {
	//create the motor controllers
	int wheels_motors_dir_pins[2] = { 2, 7 };
	int wheels_motors_step_pins[2] = { 3, 8 };
	int wheels_motors_enable_pins[2] = { 6, 9 };

	wheels_motors_controller.send_create_stepper_motors(2, wheels_motors_dir_pins, wheels_motors_step_pins, wheels_motors_enable_pins);

	//jenny5_event create_wheels_motors_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT);
	//return wheels_motors_controller.wait_for_command_completion(create_wheels_motors_event);

	clock_t start_time = clock();
	bool foot_motors_controler_created = false;

	while (1) {
		if (!wheels_motors_controller.update_commands_from_serial())
			Sleep(5);
		if (!foot_motors_controler_created)
			if (wheels_motors_controller.query_for_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
				foot_motors_controler_created = true;

		if (foot_motors_controler_created)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!foot_motors_controler_created)
				printf("Cannot create foot's motor controller! Game over!\n");
			return false;
		}
	}

	wheels_motors_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_FOOT_LEFT, 300, 100);
	wheels_motors_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_FOOT_RIGHT, 300, 100);
	return true;
}
//----------------------------------------------------------------

void t_wheels_controller::wait_for_action(int wait_for) {
	if (wait_for != NO_WAIT) {
		jenny5_event motor_done_event(STEPPER_MOTOR_MOVE_DONE_EVENT, wait_for);

		if (wait_for == WHEELS_MOVING) {
			wheels_motors_controller.wait_for_command_completion(motor_done_event, EVENT_INFO_TYPE | EVENT_INFO_PARAM1);
		}
		if (wait_for == WHEELS_ALIGNING) {
			wheels_motors_controller.wait_for_command_completion(motor_done_event, EVENT_INFO_TYPE | EVENT_INFO_PARAM1);
		}
	}
}
//----------------------------------------------------------------
double t_wheels_controller::get_distance_to_person() {
	return distance_to_person;
}
//----------------------------------------------------------------
double t_wheels_controller::get_maximum_distance_allowed() {
	return maximum_distance_allowed;
}
//----------------------------------------------------------------
double t_wheels_controller::get_minimum_distance_allowed() {
	return minimun_distance_allowed;
}
//----------------------------------------------------------------
void t_wheels_controller::set_distance_to_person(double new_distance) {
	distance_to_person = new_distance;
}
//----------------------------------------------------------------
void t_wheels_controller::set_maximum_distance_allowed(double new_distance) {
	maximum_distance_allowed = new_distance;
}
//----------------------------------------------------------------
void t_wheels_controller::set_minimum_distance_allowed(double new_distance) {
	minimun_distance_allowed = new_distance;
}
//----------------------------------------------------------------
double t_wheels_controller::get_modify_distance_by() {
	return modify_distance_by;
}
//----------------------------------------------------------------
void t_wheels_controller::set_modify_distance_by(double new_distance) {
	modify_distance_by = new_distance;
}
//----------------------------------------------------------------

int t_wheels_controller::compute_steps_from_distance(int current_distance) {
	int y = (int)(((current_distance / (1.8)) / 0.2) * (0.4));
	return y;
}
//----------------------------------------------------------------
void t_wheels_controller::move_backwards(int current_distance, int wait_for) {
	int num_steps = compute_steps_from_distance(current_distance);

	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_RIGHT, num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_RIGHT, COMMAND_SENT);
	printf("foot: MS%d %d# - sent\n", MOTOR_FOOT_RIGHT, num_steps);

	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_LEFT, -num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_LEFT, COMMAND_SENT);
	printf("foot: MS%d %d# - sent\n", MOTOR_FOOT_LEFT, num_steps);
}
//----------------------------------------------------------------
void t_wheels_controller::move_forward(int current_distance, int wait_for) {
	int num_steps = compute_steps_from_distance(current_distance);

	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_LEFT, num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_LEFT, COMMAND_SENT);
	printf("foot: MS%d %d# - sent\n", MOTOR_FOOT_LEFT, num_steps);

	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_RIGHT, -num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_RIGHT, COMMAND_SENT);
	printf("foot: MS%d %d# - sent\n", MOTOR_FOOT_RIGHT, num_steps);
}
//----------------------------------------------------------------
void t_wheels_controller::turn_left(int displacement, int wait_for) {
	int num_steps = compute_steps_from_distance(displacement);
	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_RIGHT, num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_RIGHT, COMMAND_SENT);
	printf("foot: MS%d %d# - sent\n", MOTOR_FOOT_RIGHT, num_steps);
}
//----------------------------------------------------------------
void t_wheels_controller::turn_right(int displacement, int wait_for) {
	int num_steps = compute_steps_from_distance(displacement);
	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_LEFT, num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_LEFT, COMMAND_SENT);
	printf("foot: MS%d %d# - sent\n", MOTOR_FOOT_LEFT, num_steps);
}
//----------------------------------------------------------------

t_wheels_controller::~t_wheels_controller(void) {
	wheels_motors_controller.close_connection();
}
//----------------------------------------------------------------
