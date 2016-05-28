#include <iostream>
#include <time.h>

#include "jenny5_command_module.h"
#include "jenny5_events.h"
#include "tera_ranger_one_controller.h"

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

#define LIDAR_STEPS_RANGE 100
#define LIDAR_STEP 1
#define LIDAR_SKIP_MARGINS 10 // steps from margin
#define LIDAR_NUM_STEPS (LIDAR_STEPS_RANGE - 2 * LIDAR_SKIP_MARGINS) / LIDAR_STEP


//----------------------------------------------------------------
bool connect(t_jenny5_command_module &tracks_controller, t_teraranger_one_controller& tera_ranger_one, char* error_string)
{
	//-------------- START INITIALIZATION ------------------------------

	if (!tracks_controller.connect(3, 115200)) {
		sprintf(error_string, "Error attaching to Jenny 5' tracks!");
		return false;
	}

	if (!tera_ranger_one.connect(6, 115200)) {
		sprintf(error_string, "Error attaching to Jenny 5' LIDAR!");
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
	int tracks_motors_dir_pins[3] = { 2, 8, 5 };
	int tracks_motors_step_pins[3] = { 3, 9, 6 };
	int tracks_motors_enable_pins[3] = { 4, 10, 7 };
	tracks_controller.send_create_stepper_motors(3, tracks_motors_dir_pins, tracks_motors_step_pins, tracks_motors_enable_pins);

	int lidar_stop_buttons_pins[1] = { 12 };
	int lidar_stop_buttons_dir[1] = { -1 };

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
	tracks_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_LIDAR, 75, 75);

	int buttons_index[1] = { 0 };
	tracks_controller.send_attach_sensors_to_stepper_motor(MOTOR_LIDAR, 0, NULL, 0, NULL, 1, buttons_index);

	return true;
}
//----------------------------------------------------------------
bool init(t_jenny5_command_module &tracks_controller, t_teraranger_one_controller& tera_ranger_one, int * lidar_distances, char* error_string)
{
	// must home the LIDAR
	tracks_controller.send_go_home_stepper_motor(MOTOR_LIDAR);
	printf("LIDAR motor home started ...");
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

	printf("DONE\n");
	// I have been able to home it.
	// now I skip steps from margin
	printf("LIDAR motor skip margins started ...");
	bool skipped_margins = false;
	tracks_controller.send_move_stepper_motor(MOTOR_LIDAR, LIDAR_SKIP_MARGINS);
	start_time = clock();
	while (1) {
		if (!tracks_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!skipped_margins)
			if (tracks_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_LIDAR))  // have we received the event from Serial ?
				skipped_margins = true;

		if (skipped_margins)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 5 seconds and no home
		if (wait_time > 5) {
			if (!skipped_margins)
				sprintf(error_string, "Cannot skip LIDAR margins! Game over!");
			return false;
		}
	}
	printf("DONE\n");

	printf("Fill initial array of distances ...");
	tracks_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_LIDAR, 1000, 500);
	// now fill the distances array
	for (int i = 0; i < LIDAR_NUM_STEPS; i++) {
		tracks_controller.send_move_stepper_motor(MOTOR_LIDAR, LIDAR_STEP);
		bool made_step = false;
		bool tera_ranger_responded = false;
		start_time = clock();
		while (1) {
			if (!tracks_controller.update_commands_from_serial())
				Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

			if (!made_step)
				if (tracks_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_LIDAR))  // have we received the event from Serial ?
					made_step = true;

			if (made_step) {
				if (tera_ranger_one.get_state() == COMMAND_DONE) {// I ping the sonar only if no ping was sent before
					tera_ranger_one.send_request_distance();
					tera_ranger_one.set_state(COMMAND_SENT);
					tera_ranger_responded = false;
					printf("TR# - sent\n");
				}

				if (!tera_ranger_one.update_commands_from_serial())
					Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

								  // read to see if there is any distance received from sonar
				if (tera_ranger_one.get_state() == COMMAND_SENT) {// if a command has been sent
					int distance;
					if (tera_ranger_one.query_for_distance(distance)) { // have we received the event from Serial ?
						tera_ranger_one.set_state(COMMAND_DONE);
						lidar_distances[i] = distance;
						printf("LIDAR distance [%d] = %d cm\n", i, distance);
						tera_ranger_responded = true;
						break;
					}
				}

			//	if (GetAsyncKeyState(VK_ESCAPE))  // break the loop
				//	break;
			}



			// measure the passed time 
			clock_t end_time = clock();

			double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
			// if more than 1 seconds and no home
			if (wait_time > 0.5) {
				if (!made_step) {
					sprintf(error_string, "Cannot make LIDAR step! Game over!");
					return false;
				}
				if (!tera_ranger_responded) {
					lidar_distances[i] = 0;
					printf("LIDAR distance = %d cm\n", 0);
					break;
				}
			}
		}
	}
	printf("DONE\n");

	return true;
}
//----------------------------------------------------------------
int	main(void)
{
	t_jenny5_command_module tracks_controller;
	t_teraranger_one_controller tera_ranger_one;

	int lidar_distances[LIDAR_NUM_STEPS];


	// initialization
	char error_string[1000];
	if (!connect(tracks_controller, tera_ranger_one, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Connection succceded.\n");

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
	clock_t start_time = clock();
	if (!init(tracks_controller, tera_ranger_one, lidar_distances, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else {
		printf("First step succceded.\n");
		clock_t end_time = clock();

		double run_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		printf("First step time = %lf\n", run_time);
	}

	printf("Now running the main loop. Press Escape when want to exit!\n");
	bool active = true;

	int lidar_motor_position = LIDAR_NUM_STEPS - 1;
	int lidar_motor_direction = -1;
	
	
	while (active) {        // starting infinit loop
		tracks_controller.send_move_stepper_motor(MOTOR_LIDAR, LIDAR_STEP * lidar_motor_direction);
		bool made_step = false;

		if (!tracks_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!made_step)
			if (tracks_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_LIDAR))  // have we received the event from Serial ?
				made_step = true;

		if (made_step) {
			if (tera_ranger_one.get_state() == COMMAND_DONE) {// I ping the sonar only if no ping was sent before
				tera_ranger_one.send_request_distance();
				tera_ranger_one.set_state(COMMAND_SENT);
				//printf("TR# - sent\n");
			}
			
			if (!tera_ranger_one.update_commands_from_serial())
				Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

			// read to see if there is any distance received from sonar
			if (tera_ranger_one.get_state() == COMMAND_SENT) {// if a command has been sent
				int distance;
				if (tera_ranger_one.query_for_distance(distance)) { // have we received the event from Serial ?
					tera_ranger_one.set_state(COMMAND_DONE);
					lidar_distances[lidar_motor_position] = distance;
					printf("LIDAR distance [%d] = %d cm\n", lidar_motor_position, distance);
					if (lidar_motor_position == LIDAR_SKIP_MARGINS && lidar_motor_direction == -1)
						lidar_motor_direction = 1;
					else
						if (lidar_motor_position == LIDAR_NUM_STEPS - 1 && lidar_motor_direction == 1)
							lidar_motor_direction = -1;
					lidar_motor_position += lidar_motor_direction;
				}
			}
		}
		if (GetAsyncKeyState(VK_ESCAPE))  // break the loop
			break;
	}
	
	tracks_controller.close_connection();
	tera_ranger_one.close_connection();

	printf("Press Enter!");
	getchar();
	return 0;
}
//----------------------------------------------------------------