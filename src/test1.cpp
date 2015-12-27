// jenny5_test1.cpp : Defines the entry point for the console application.
//


#include <stdio.h>
#include "jenny5_command_module.h"
#include "jenny5_events.h"

// make sure that you have WIN32 defined in your windows project (preprocessor options)
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x)*1000)
#endif

#define COMMAND_NOT_SENT 0
#define COMMAND_SENT 1
#define COMMAND_DONE 2

//---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	// attach to the robot
	t_jenny5_command_module right_arm_controller, left_arm_controller, head_controller, foot_controller, grippers_controller;

	if (!head_controller.connect(10, 9600)) {
		printf("Error attaching to Jenny 5' head!\n");
		getchar();
		return 0;   
	}
	Sleep(2000); // wait for connection

	// empty the serial buffer
	char sir[1000];
	int num_read = 0;
	num_read = head_controller.clear_data_from_serial(sir, 1000);
	sir[num_read] = 0;
	printf("Serial buffer = %s\n", sir);

	printf("Control module version = %s\n", head_controller.get_version());
	int motor_state = COMMAND_NOT_SENT;
	int distance_to_object = 150;

	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor


		if (distance_to_object > 100) {
			if (motor_state == COMMAND_NOT_SENT) {
				// send a command to the module
				head_controller.send_move_motor(0, 100);
				motor_state = COMMAND_SENT;
			}
			else {
				// now wait for the motor to complete the movement
				if (head_controller.query_for_event(MOTOR_DONE_EVENT, 0)) { // have we received the event from Serial ?
					motor_state = COMMAND_DONE;
					distance_to_object = 50;// this is obtained by other meanings - for instance from camera or infrared or ultrasound
				}
				else {
					// motor is still running and we can supervise it (with a camera?)
				}
			}
		}
		else {
			if (motor_state = COMMAND_DONE)
				motor_state = COMMAND_NOT_SENT;
			// send a new command to disable the motor
			if (motor_state == COMMAND_NOT_SENT) {
				head_controller.send_disable_motor(0);
				motor_state = COMMAND_SENT;
			}
			else {
				// now wait for the motor to complete the movement
				if (head_controller.query_for_event(MOTOR_DISABLED_EVENT, 0)) { // have we received the event from Serial ?
					motor_state = COMMAND_DONE;
					break;// exit while
				}
				else {
					// motor is still locked so we can supervise it
				}
			}
		}
	}
	Sleep(50);

	head_controller.close_connection();

	printf("End of the game ... press Enter ...");
	getchar();

	return 0;
}
//---------------------------------------------------------------------------
