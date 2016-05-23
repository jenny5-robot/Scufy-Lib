#include <iostream>
#include <time.h>


#include "../include/jenny5_command_module.h"
#include "../include/jenny5_events.h"
#include "point_tracker.h"
//----------------------------------------------------------------

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x)*1000)
#endif

using namespace std;


#define MOTOR_HEAD_HORIZONTAL 0
#define MOTOR_HEAD_VERTICAL 1

#define MOTOR_TRACKS_LEFT 0
#define MOTOR_TRACKS_RIGHT 1
#define MOTOR_LIDAR 2

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3

#define TOLERANCE 20

#define DOES_NOTHING_SLEEP 10

#define HEAD_RADIUS_TO_REVERT 70


//----------------------------------------------------------------
bool connect(t_jenny5_command_module &tracks_controller, char* error_string)
{
	//-------------- START INITIALIZATION ------------------------------

	if (!tracks_controller.connect(3, 115200)) {
		sprintf(error_string, "Error attaching to Jenny 5' tracks!");
		return false;
	}

	// now wait to see if I have been connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();
	
	bool tracks_responded = false;

	while (1) {
		if (!tracks_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		
		if (!tracks_responded)
			if (tracks_controller.query_for_event(IS_ALIVE_EVENT, 0))  // have we received the event from Serial ?
				tracks_responded = true;

		if (tracks_responded)
			break;
		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!tracks_responded)
				sprintf(error_string, "Tracks do not respond! Game over!");
			return false;
		}
	}

	// home head's motors

	return true;
}
//----------------------------------------------------------------
bool setup(t_jenny5_command_module &tracks_controller, char* error_string)
{
	int tracks_motors_dir_pins[3] = { 2, 8, 5};
	int tracks_motors_step_pins[3] = { 3, 9, 6};
	int tracks_motors_enable_pins[3] = { 4, 10, 7};
	tracks_controller.send_create_stepper_motors(3, tracks_motors_dir_pins, tracks_motors_step_pins, tracks_motors_enable_pins);

	int lidar_stop_buttons_pins[1] = { 12 };
	int lidar_stop_buttons_dir[1] = { 1 };

	tracks_controller.send_create_buttons(1, lidar_stop_buttons_pins, lidar_stop_buttons_dir);

	clock_t start_time = clock();
	
	bool tracks_motors_controler_created = false;
	bool lidar_stop_buttons_controller_created = false;

	while (1) {
		if (!tracks_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor


		if (!tracks_motors_controler_created)
			if (tracks_controller.query_for_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
				tracks_motors_controler_created = true;

		if (!lidar_stop_buttons_controller_created)
			if (tracks_controller.query_for_event(BUTTONS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
				lidar_stop_buttons_controller_created = true;

		if (tracks_motors_controler_created && lidar_stop_buttons_controller_created)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {

			if (!tracks_motors_controler_created)
				sprintf(error_string, "Cannot create tracks's motor controller! Game over!");

			if (!lidar_stop_buttons_controller_created)
				sprintf(error_string, "Cannot create LIDAR's button controller! Game over!");
			return false;
		}
	}
	
	tracks_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_TRACKS_LEFT, 1300, 500);
	tracks_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_TRACKS_RIGHT, 1300, 500);
	tracks_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_LIDAR, 50, 50);

	tracks_controller.send_attach_sensors_to_stepper_motor(MOTOR_LIDAR, 0, NULL, 0, NULL, 1, 0);

	return true;
}
//----------------------------------------------------------------
bool init(t_jenny5_command_module &tracks_controller, char* error_string)
{
	// must home the LIDAR
	tracks_controller.send_go_home_stepper_motor(MOTOR_LIDAR);

	clock_t start_time = clock();
	bool lidar_homed = false;

	while (1) {
		if (!tracks_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!lidar_homed)
			if (tracks_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_LIDAR))  // have we received the event from Serial ?
				lidar_homed = true;

		if (lidar_homed)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 5 seconds and no home
		if (wait_time > 5) {
			if (!lidar_homed)
				sprintf(error_string, "Cannot home LIDAR! Game over!");
			return false;
		}
	}

	return true;
}
//----------------------------------------------------------------
int	main(void)
{
	t_jenny5_command_module tracks_controller;
	
	
	// initialization
	char error_string[1000];
	if (!connect(tracks_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Initialization succceded.\n");

	// setup
	if (!setup(tracks_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Setup succceded.\n");
	
	//  init
	if (!init(tracks_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Init succceded.\n");


	bool active = true;
	
	while (active){        // starting infinit loop
	
/*
		if (face_found) {// 
			// horizontal movement motor

			// send a command to the module so that the face is in the center of the image
			if (center.x > frame.cols / 2 + TOLERANCE) {
				tracking_data angle_offset = get_offset_angles(920, Point(center.x, center.y));
				int num_steps_x = (int)(angle_offset.degrees_from_center_x / 1.8 * 8);
				
				if (center.range < HEAD_RADIUS_TO_REVERT) {
					// move forward
					tracks_controller.send_move_stepper_motor(MOTOR_TRACKS_LEFT, num_steps_x);
					tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_LEFT, COMMAND_SENT);
					printf("foot: M%d %d# - sent\n", MOTOR_TRACKS_LEFT, num_steps_x);
				}
				else {
					// move backward
					tracks_controller.send_move_stepper_motor(MOTOR_TRACKS_RIGHT, num_steps_x);
					tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_RIGHT, COMMAND_SENT);
					printf("foot: M%d %d# - sent\n", MOTOR_TRACKS_RIGHT, num_steps_x);
				}
				
			}
			else
				if (center.x < frame.cols / 2 - TOLERANCE) {
					
					tracking_data angle_offset = get_offset_angles(920, Point(center.x, center.y));
					int num_steps_x = (int)(angle_offset.degrees_from_center_x / 1.8 * 8);
					
					if (center.range < HEAD_RADIUS_TO_REVERT) {
						// move forward
						tracks_controller.send_move_stepper_motor(MOTOR_TRACKS_RIGHT, num_steps_x);
						tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_RIGHT, COMMAND_SENT);
						printf("foot: M%d %d# - sent\n", MOTOR_TRACKS_RIGHT, num_steps_x);
					}
					else {
						// move backward
						tracks_controller.send_move_stepper_motor(MOTOR_TRACKS_LEFT, num_steps_x);
						tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_LEFT, COMMAND_SENT);
						printf("foot: M%d %d# - sent\n", MOTOR_TRACKS_LEFT, num_steps_x);
					}
					
				}
				else {
					
					// face is in the center, so I move equaly with both motors
					if (center.range < HEAD_RADIUS_TO_REVERT) {
						// move forward
						tracks_controller.send_move_stepper_motor(MOTOR_TRACKS_LEFT, 100);
						tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_LEFT, COMMAND_SENT);
						printf("foot: M%d %d# - sent\n", MOTOR_TRACKS_LEFT, 100);

						tracks_controller.send_move_stepper_motor(MOTOR_TRACKS_RIGHT, -100);
						tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_RIGHT, COMMAND_SENT);
						printf("foot: M%d %d# - sent\n", MOTOR_TRACKS_RIGHT, -100);
					}
					else {
						// move backward
						tracks_controller.send_move_stepper_motor(MOTOR_TRACKS_LEFT, -100);
						tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_LEFT, COMMAND_SENT);
						printf("foot: M%d %d# - sent\n", MOTOR_TRACKS_LEFT, -100);

						tracks_controller.send_move_stepper_motor(MOTOR_TRACKS_RIGHT, 100);
						tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_RIGHT, COMMAND_SENT);
						printf("foot: M%d %d# - sent\n", MOTOR_TRACKS_RIGHT, 100);
					}
					
				}

			
		}
		*/
		/*
		//extract movements for foot
		// now extract the moves done from the queue
		if (tracks_controller.get_stepper_motor_state(MOTOR_TRACKS_LEFT) == COMMAND_SENT) {// if a command has been sent
			if (tracks_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_TRACKS_LEFT)) { // have we received the event from Serial ?
				tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_LEFT, COMMAND_DONE);
				printf("foot: M%d# - done\n", MOTOR_TRACKS_LEFT);
			}
		}
		if (tracks_controller.get_stepper_motor_state(MOTOR_TRACKS_RIGHT) == COMMAND_SENT) {// if a command has been sent
			if (tracks_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_TRACKS_RIGHT)) { // have we received the event from Serial ?
				tracks_controller.set_stepper_motor_state(MOTOR_TRACKS_RIGHT, COMMAND_DONE);
				printf("foot: M%d# - done\n", MOTOR_TRACKS_RIGHT);
			}
		}
		*/

		if (waitKey(1) >= 0)  // break the loop
			active = false;
	}

	tracks_controller.close_connection();
	return 0;
}
//----------------------------------------------------------------