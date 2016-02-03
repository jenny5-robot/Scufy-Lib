#include "../include/arm_controller.h"
#include <time.h>

//----------------------------------------------------------------
t_arm_controller::t_arm_controller(void) {
	if (connect()) {

		printf("Connected...\n");
	}
	if (setup()) {
		printf("Setup complete...\n");
	}
}
//----------------------------------------------------------------
bool t_arm_controller::connect() {
	//connect to serial ports
	if (!upper_motors_controller.connect(2, 115200)) {
		std::cout << ("Error attaching to Jenny 5' upper arm motors!\n");
	}

	if (!lower_motors_controller.connect(3, 115200)) {
		std::cout << ("Error attaching to Jenny 5' lower arm motors!\n");
	}

	jenny5_event connect_to_upper_motors_event(IS_ALIVE_EVENT);
	jenny5_event connect_to_lower_motors_event(IS_ALIVE_EVENT);
	return upper_motors_controller.wait_for_command_completion(connect_to_upper_motors_event) &&
		lower_motors_controller.wait_for_command_completion(connect_to_lower_motors_event);

	//// now wait to see if I have been connected
	//// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	//clock_t start_time = clock();
	//bool upper_motors_responded = false;
	//bool lower_motors_responded = false;

	//while (1) {
	//	if (!upper_motors_controller.update_commands_from_serial() && !lower_motors_controller.update_commands_from_serial())
	//		Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

	//	if (!upper_motors_responded)
	//		if (upper_motors_controller.query_for_event(IS_ALIVE_EVENT, 0))  // have we received the event from Serial ?
	//			upper_motors_responded = true;

	//	if (!lower_motors_responded)
	//		if (lower_motors_controller.query_for_event(IS_ALIVE_EVENT, 0))  // have we received the event from Serial ?
	//			lower_motors_responded = true;

	//	if (upper_motors_responded && lower_motors_responded)
	//		break;
	//	// measure the passed time 
	//	clock_t end_time = clock();

	//	double wait_time = (double) (end_time - start_time) / CLOCKS_PER_SEC;
	//	// if more than 3 seconds then game over
	//	if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
	//		if (!upper_motors_responded)
	//			printf("Head does not respond! Game over!\n");

	//		if (!lower_motors_responded)
	//			printf("Foot does not respond! Game over!\n");
	//		return false;
	//	}
	//}
	//return true;
}
//----------------------------------------------------------------
bool t_arm_controller::setup() {
	//create the motor controllers
	int upper_arm_motors_dir_pins[4] = { 2, 5, 8, 11 };
	int upper_arm_motors_step_pins[4] = { 3, 6, 9, 12 };
	int upper_arm_motors_enable_pins[4] = { 4, 7, 10, 13 };
	upper_motors_controller.send_create_motors(4, upper_arm_motors_dir_pins, upper_arm_motors_step_pins, upper_arm_motors_enable_pins);

	int lower_arm_motors_dir_pins[1] = { 5 };
	int lower_arm_motors_step_pins[1] = { 6 };
	int lower_arm_motors_enable_pins[1] = { 7 };
	lower_motors_controller.send_create_motors(1, lower_arm_motors_dir_pins, lower_arm_motors_step_pins, lower_arm_motors_enable_pins);

	jenny5_event create_upper_motors_event(MOTORS_CONTROLLER_CREATED_EVENT);
	return upper_motors_controller.wait_for_command_completion(create_upper_motors_event);

//	// now wait to see if I have been connected
//	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
//	clock_t start_time = clock();
//	bool upper_motors_responded = false;
//	bool lower_motors_responded = false;
//
//	while (1) {
//		if (!upper_motors_controller.update_commands_from_serial())// && !lower_motors_controller.update_commands_from_serial())
//			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor
//
//		if (!upper_motors_responded)
//			if (upper_motors_controller.query_for_event(MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
//				upper_motors_responded = true;
//
//
//		if (upper_motors_responded)// && lower_motors_responded)
//			break;
//		// measure the passed time 
//		clock_t end_time = clock();
//
//		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
//		// if more than 3 seconds then game over
//		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
//			if (!upper_motors_responded)
//				printf("Upper motors do not respond! Game over!\n");
///*
//			if (!lower_motors_responded)
//				printf("Lower motors do not respond! Game over!\n");*/
//			Sleep(1000);
//			return false;
//		
}
//----------------------------------------------------------------
void t_arm_controller::spin_shoulder(int num_steps) {
	upper_motors_controller.send_move_motor(SHOULDER_SPIN, num_steps);
	upper_motors_controller.set_motor_state(SHOULDER_SPIN, COMMAND_SENT);
	printf("shoulder spin: M%d %d# - sent\n", SHOULDER_SPIN, num_steps);

	jenny5_event motor_done_event(MOTOR_DONE_EVENT);
	upper_motors_controller.wait_for_command_completion(motor_done_event);

	//bool arm_motors_responded = false;
	//clock_t start_time = clock();
	//while (1) {
	//	if (!upper_motors_controller.update_commands_from_serial())
	//		Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

	//	if (!arm_motors_responded)
	//		if (upper_motors_controller.query_for_event(MOTOR_DONE_EVENT, 0))  // have we received the event from Serial ?
	//			arm_motors_responded = true;
	//	if(arm_motors_responded)
	//		break;
	//	// measure the passed time 
	//	clock_t end_time = clock();

	//	double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
	//	// if more than 3 seconds then game over
	//	if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
	//		if (!arm_motors_responded)
	//			printf("Head does not respond! Game over!\n");
	//		return;
	//	}
	//}

}
//----------------------------------------------------------------
void t_arm_controller::lift_shoulder(int num_steps) {
	upper_motors_controller.send_move_motor(SHOULDER_LIFT, num_steps);
	upper_motors_controller.set_motor_state(SHOULDER_LIFT, COMMAND_SENT);
	printf("shoulder lift: M%d %d# - sent\n", SHOULDER_LIFT, num_steps);

	jenny5_event motor_done_event(MOTOR_DONE_EVENT);
	upper_motors_controller.wait_for_command_completion(motor_done_event);

	//bool arm_motors_responded = false;
	//clock_t start_time = clock();
	//while (1) {
	//	if (!upper_motors_controller.update_commands_from_serial())
	//		Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

	//	if (!arm_motors_responded)
	//		if (upper_motors_controller.query_for_event(MOTOR_DONE_EVENT, 0))  // have we received the event from Serial ?
	//			arm_motors_responded = true;
	//	if (arm_motors_responded)
	//		break;
	//	 measure the passed time 
	//	clock_t end_time = clock();

	//	double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
	//	 if more than 3 seconds then game over
	//	if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
	//		if (!arm_motors_responded)
	//			printf("Head does not respond! Game over!\n");
	//		return;
	//	}
	//}
}
//----------------------------------------------------------------
void t_arm_controller::rotate_upper_arm(int num_steps) {
	upper_motors_controller.send_move_motor(UPPER_ARM_ROTATE, num_steps);
	upper_motors_controller.set_motor_state(UPPER_ARM_ROTATE, COMMAND_SENT);
	printf("upper arm rotate: M%d %d# - sent\n", UPPER_ARM_ROTATE, num_steps);

	jenny5_event motor_done_event(MOTOR_DONE_EVENT);
	upper_motors_controller.wait_for_command_completion(motor_done_event);
}
//----------------------------------------------------------------
void t_arm_controller::rotate_lower_arm(int num_steps) {
	lower_motors_controller.send_move_motor(LOWER_ARM_ROTATE, num_steps);
	lower_motors_controller.set_motor_state(LOWER_ARM_ROTATE, COMMAND_SENT);
	printf("lower arm rotate: M%d %d# - sent\n", LOWER_ARM_ROTATE, num_steps);

	jenny5_event motor_done_event(MOTOR_DONE_EVENT);
	lower_motors_controller.wait_for_command_completion(motor_done_event);
}
//----------------------------------------------------------------
void t_arm_controller::lift_elbow(int num_steps) {
	upper_motors_controller.send_move_motor(ELBOW_LIFT, num_steps);
	upper_motors_controller.set_motor_state(ELBOW_LIFT, COMMAND_SENT);
	printf("elbow lift: M%d %d# - sent\n", ELBOW_LIFT, num_steps);

	jenny5_event motor_done_event(MOTOR_DONE_EVENT);
	upper_motors_controller.wait_for_command_completion(motor_done_event);
}
//----------------------------------------------------------------
t_arm_controller::~t_arm_controller(void) {
	upper_motors_controller.close_connection();
	lower_motors_controller.close_connection();
}
//----------------------------------------------------------------
