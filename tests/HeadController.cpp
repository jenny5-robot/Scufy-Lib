#include "../include/HeadController.h"


#define MOTOR_HEAD_HORIZONTAL 0
#define MOTOR_HEAD_VERTICAL 1

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3

//----------------------------------------------------------------
t_head_controller::t_head_controller(void) {
	if (connect()) {

		printf("Connected to head...\n");
	}
	if (setup()) {
		printf("Head setup complete...\n");
	}
}
//----------------------------------------------------------------
bool t_head_controller::connect() {
	//connect to serial ports
	if (!head_motors_controller.connect(3, 115200)) {
		std::cout << ("Error attaching to Jenny 5' head motors!\n");
		return false;
	}

	//jenny5_event connect_to_head_motors_event(IS_ALIVE_EVENT);
	//return head_motors_controller.wait_for_command_completion(connect_to_head_motors_event);

	// now wait to see if I have been connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();

	while (1) {
		if (!head_motors_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (head_motors_controller.query_for_event(IS_ALIVE_EVENT, 0)) { // have we received the event from Serial ?
			break;
		}
		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double) (end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			printf("Head does not respond! Game over!\n");
			return false;
		}
	}
	return true;
}
//----------------------------------------------------------------
bool t_head_controller::setup() {
	//create the motor controllers
	int head_motors_dir_pins[2] = { 2, 5 };
	int head_motors_step_pins[2] = { 3, 6 };
	int head_motors_enable_pins[2] = { 4, 7 };
	head_motors_controller.send_create_stepper_motors(2, head_motors_dir_pins, head_motors_step_pins, head_motors_enable_pins);

	//create the sonar
	int head_sonars_trig_pins[1] = { 8 };
	int head_sonars_echo_pins[1] = { 9 };

	head_motors_controller.send_create_sonars(1, head_sonars_trig_pins, head_sonars_echo_pins);

	//jenny5_event create_head_motors_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT);
	//return head_motors_controller.wait_for_command_completion(create_head_motors_event);

	clock_t start_time = clock();

	bool motors_controller_created = false;
	bool sonars_controller_created = false;

	while (1) {
		if (!head_motors_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (head_motors_controller.query_for_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			motors_controller_created = true;

		if (head_motors_controller.query_for_event(SONARS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			sonars_controller_created = true;

		if (motors_controller_created && sonars_controller_created)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double) (end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!motors_controller_created)
				printf("Cannot create head's motors controller! Game over!\n");
			if (!sonars_controller_created)
				printf("Cannot create head's sonars controller! Game over!\n");
			return false;
		}
	}


	head_motors_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_HEAD_HORIZONTAL, 1000, 50);
	head_motors_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_HEAD_VERTICAL, 1000, 50);

	return true;

}
//----------------------------------------------------------------
void t_head_controller::move_left(unsigned int num_steps) {
	head_motors_controller.send_move_stepper_motor(MOVE_X, DIRECTION_LEFT * num_steps);
	head_motors_controller.set_stepper_motor_state(MOVE_X, COMMAND_SENT);
	printf("move head left: M%d %d# - sent\n", MOVE_X, DIRECTION_LEFT * num_steps);

	jenny5_event motor_done_event(STEPPER_MOTOR_MOVE_DONE_EVENT);
	head_motors_controller.wait_for_command_completion(motor_done_event);
}
//----------------------------------------------------------------
void t_head_controller::move_right(unsigned int num_steps) {
	head_motors_controller.send_move_stepper_motor(MOVE_X, DIRECTION_RIGTH * num_steps);
	head_motors_controller.set_stepper_motor_state(MOVE_X, COMMAND_SENT);
	printf("move head right: M%d %d# - sent\n", MOVE_X, DIRECTION_RIGTH * num_steps);

	jenny5_event motor_done_event(STEPPER_MOTOR_MOVE_DONE_EVENT);
	head_motors_controller.wait_for_command_completion(motor_done_event);

}
//----------------------------------------------------------------
void t_head_controller::move_up(unsigned int num_steps) {
	head_motors_controller.send_move_stepper_motor(MOVE_Y, DIRECTION_UP * num_steps);
	head_motors_controller.set_stepper_motor_state(MOVE_Y, COMMAND_SENT);
	printf("move head up: M%d %d# - sent\n", MOVE_Y, DIRECTION_UP * num_steps);

	jenny5_event motor_done_event(STEPPER_MOTOR_MOVE_DONE_EVENT);
	head_motors_controller.wait_for_command_completion(motor_done_event);
}
//----------------------------------------------------------------
void t_head_controller::move_down(unsigned int num_steps) {
	head_motors_controller.send_move_stepper_motor(MOVE_Y, DIRECTION_DOWN * num_steps);
	head_motors_controller.set_stepper_motor_state(MOVE_Y, COMMAND_SENT);
	printf("move head down: M%d %d# - sent\n", MOVE_Y, DIRECTION_DOWN * num_steps);

	jenny5_event motor_done_event(STEPPER_MOTOR_MOVE_DONE_EVENT);
	head_motors_controller.wait_for_command_completion(motor_done_event);
}
//----------------------------------------------------------------
t_head_controller::~t_head_controller(void) {
	head_motors_controller.close_connection();
}
//----------------------------------------------------------------