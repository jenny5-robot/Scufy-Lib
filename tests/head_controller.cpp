#include "../include/head_controller.h"

//----------------------------------------------------------------
t_head_controller::t_head_controller(void) {
	if (connect()) {

		printf("Connected...\n");
	}
	if (setup()) {
		printf("Setup complete...\n");
	}
}
//----------------------------------------------------------------
bool t_head_controller::connect() {
	//connect to serial ports
	if (!head_motors_controller.connect(3, 115200))
	{
		std::cout << ("Error attaching to Jenny 5' head motors!\n");
	}

	jenny5_event connect_to_head_motors_event(IS_ALIVE_EVENT);

	return head_motors_controller.wait_for_command_completion(connect_to_head_motors_event);
}
//----------------------------------------------------------------
bool t_head_controller::setup() {
	//create the motor controllers
	int head_motors_dir_pins[2] = { 2, 5 };
	int head_motors_step_pins[2] = { 3, 6 };
	int head_motors_enable_pins[2] = { 4, 7 };
	head_motors_controller.send_create_stepper_motors(2, head_motors_dir_pins, head_motors_step_pins, head_motors_enable_pins);

	jenny5_event create_head_motors_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT);
	return head_motors_controller.wait_for_command_completion(create_head_motors_event);
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