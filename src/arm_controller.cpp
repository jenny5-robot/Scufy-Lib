#include "../include/arm_controller.h"

//----------------------------------------------------------------
t_arm_controller::t_arm_controller(void) {
	//connect to serial ports
	if (!upper_motors_controller.connect(3, 115200)) {
		std::cout<<("Error attaching to Jenny 5' upper arm motors!\n");
	}

	if (!lower_motors_controller.connect(4, 115200)) {
		std::cout << ("Error attaching to Jenny 5' lower arm motors!\n");
	}

	//create the motor controllers
	int upper_arm_motors_dir_pins[4] = { 2, 5, 8, 11 };
	int upper_arm_motors_step_pins[4] = { 3, 6, 9, 12 };
	int upper_arm_motors_enable_pins[4] = { 4, 7, 10, 13 };
	upper_motors_controller.send_create_motors(4, upper_arm_motors_dir_pins, upper_arm_motors_step_pins, upper_arm_motors_enable_pins);

	int lower_arm_motors_dir_pins[1] = { 5 };
	int lower_arm_motors_step_pins[1] = { 6 };
	int lower_arm_motors_enable_pins[1] = { 7 };
	lower_motors_controller.send_create_motors(1, lower_arm_motors_dir_pins, lower_arm_motors_step_pins, lower_arm_motors_enable_pins);
}
//----------------------------------------------------------------
void t_arm_controller::spin_shoulder(int num_steps) {
	upper_motors_controller.send_move_motor(SHOULDER_SPIN, num_steps);
	upper_motors_controller.set_motor_state(SHOULDER_SPIN, COMMAND_SENT);
	printf("shoulder spin: M%d %d# - sent\n", SHOULDER_SPIN, num_steps);
}
//----------------------------------------------------------------
void t_arm_controller::lift_shoulder(int num_steps) {
	upper_motors_controller.send_move_motor(SHOULDER_LIFT, num_steps);
	upper_motors_controller.set_motor_state(SHOULDER_LIFT, COMMAND_SENT);
	printf("shoulder lift: M%d %d# - sent\n", SHOULDER_LIFT, num_steps);
}
//----------------------------------------------------------------
void t_arm_controller::rotate_upper_arm(int num_steps) {
	upper_motors_controller.send_move_motor(UPPER_ARM_ROTATE, num_steps);
	upper_motors_controller.set_motor_state(UPPER_ARM_ROTATE, COMMAND_SENT);
	printf("upper arm rotate: M%d %d# - sent\n", UPPER_ARM_ROTATE, num_steps);
}
//----------------------------------------------------------------
void t_arm_controller::rotate_lower_arm(int num_steps) {
	upper_motors_controller.send_move_motor(LOWER_ARM_ROTATE, num_steps);
	upper_motors_controller.set_motor_state(LOWER_ARM_ROTATE, COMMAND_SENT);
	printf("lower arm rotate: M%d %d# - sent\n", LOWER_ARM_ROTATE, num_steps);
}
//----------------------------------------------------------------
void t_arm_controller::lift_elbow(int num_steps) {
	upper_motors_controller.send_move_motor(ELBOW_LIFT, num_steps);
	upper_motors_controller.set_motor_state(ELBOW_LIFT, COMMAND_SENT);
	printf("elbow lift: M%d %d# - sent\n", ELBOW_LIFT, num_steps);
}
//----------------------------------------------------------------
t_arm_controller::~t_arm_controller(void) {
	upper_motors_controller.close_connection();
	lower_motors_controller.close_connection();
}
//----------------------------------------------------------------