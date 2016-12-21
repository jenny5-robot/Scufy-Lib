// last update : 2016.12.20.0 // year.month.day.build number
// MIT License


#define PROGRAM_VERSION "PROGRAM VERSION: 2016.12.21.1\n"

#include <iostream>
#include <time.h>

#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include "jenny5_arduino_controller.h"
#include "jenny5_events.h"
#include "point_tracker.h"
//----------------------------------------------------------------

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x)*1000)
#endif

using namespace std;
using namespace cv;

#define HEAD_COM_PORT 9
#define LEFT_ARM_COM_PORT 4


#define HEAD_MOTOR_HORIZONTAL 0
#define HEAD_MOTOR_VERTICAL 1

#define HEAD_POTENTIOMETER_HORIZONTAL_INDEX 0
#define HEAD_POTENTIOMETER_VERTICAL_INDEX 1

#define LEFT_ARM_BODY_MOTOR 0
#define LEFT_ARM_SHOULDER_UP_DOWN_MOTOR 1
#define LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR 2
#define LEFT_ARM_ELBOW_MOTOR 3
#define LEFT_ARM_FOREARM_MOTOR 4
#define LEFT_ARM_GRIPPER_MOTOR 5

#define LEFT_ARM_BODY_POTENTIOMETER_INDEX 0
#define LEFT_ARM_SHOULDER_UP_DOWN_POTENTIOMETER_INDEX 1
#define LEFT_ARM_SHOULDER_LEFT_RIGHT_POTENTIOMETER_INDEX 2
#define LEFT_ARM_ELBOW_POTENTIOMETER_INDEX 3
#define LEFT_ARM_FOREARM_POTENTIOMETER_INDEX 4
#define LEFT_ARM_GRIPPER_POTENTIOMETER_INDEX 5

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3

#define TOLERANCE 20

#define DOES_NOTHING_SLEEP 10


struct t_CENTER_POINT
{
	int x;
	int y;
	int range;
};

//----------------------------------------------------------------
bool biggest_face(std::vector<Rect> faces, t_CENTER_POINT &center)
{

	center.x = -1;
	center.x = -1;
	center.range = 0;

	bool found_one = false;
	for (unsigned int i = 0; i < faces.size(); i++) {
		if ((faces[i].width) / 2 > center.range) {
			center.range = (faces[i].width) / 2;
			center.x = faces[i].x + center.range;
			center.y = faces[i].y + center.range;
			found_one = true;
		}
	}
	return found_one;
}
//----------------------------------------------------------------
bool init(t_jenny5_arduino_controller &head_controller, t_jenny5_arduino_controller &left_arm_controller, VideoCapture &head_cam, VideoCapture left_hand_cam, CascadeClassifier &face_classifier, char* error_string)
{
	//-------------- START INITIALIZATION ------------------------------

	if (!head_controller.connect(HEAD_COM_PORT - 1, 115200)) { // real - 1
		sprintf(error_string, "Error attaching to Jenny 5' head!");
		return false;
	}

	if (!left_arm_controller.connect(LEFT_ARM_COM_PORT - 1, 115200)) { // real - 1
		sprintf(error_string, "Error attaching to Jenny 5' left arm!");
		return false;
	}

	bool head_responded = false;
	bool left_arm_responded = false;

	// now wait to see if I have been connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();

	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!head_responded)
			if (head_controller.query_for_event(IS_ALIVE_EVENT, 0)) { // have we received the event from Serial ?
				head_responded = true;
				break;
			}

		if (!left_arm_responded)
			if (left_arm_controller.query_for_event(IS_ALIVE_EVENT, 0)) { // have we received the event from Serial ?
				left_arm_responded = true;
				break;
			}
		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!head_responded)
				sprintf(error_string, "Head does not respond! Game over!");

			if (!left_arm_responded)
				sprintf(error_string, "Left arm does not respond! Game over!");

			return false;
		}
	}
	; // create cascade for face reco
	  // load haarcascade library
	if (!face_classifier.load("haarcascade_frontalface_alt.xml")) {
		sprintf(error_string, "Cannot load haarcascade! Please place the file in the correct folder!");
		return false;
	}
	// connect to video camera

	head_cam.open(0);			// link it to the device [0 = default cam] (USBcam is default 'cause I disabled the onbord one IRRELEVANT!)
	if (!head_cam.isOpened())	// check if we succeeded
	{
		sprintf(error_string, "Couldn't open head's video camera!");
		head_controller.close_connection();
		return false;
	}
	else {
		Mat frame;
		head_cam >> frame;
		printf("Head video size: %dx%d\n", frame.rows, frame.cols);
	}
	return true;
}
//----------------------------------------------------------------
bool setup_head(t_jenny5_arduino_controller &head_controller, char* error_string)
{

	int head_motors_dir_pins[2] = { 5, 2 };
	int head_motors_step_pins[2] = { 6, 3 };
	int head_motors_enable_pins[2] = { 7, 4 };
	head_controller.send_create_stepper_motors(2, head_motors_dir_pins, head_motors_step_pins, head_motors_enable_pins);

	//int head_sonars_trig_pins[1] = { 8 };
	//int head_sonars_echo_pins[1] = { 9 };

	//head_controller.send_create_sonars(1, head_sonars_trig_pins, head_sonars_echo_pins);

	int head_potentiometer_pins[2] = { 0, 1 };
	head_controller.send_create_potentiometers(2, head_potentiometer_pins);

	bool motors_controller_created = false;
	bool sonars_controller_created = false;
	bool potentiometers_controller_created = false;

	clock_t start_time = clock();
	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (head_controller.query_for_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			motors_controller_created = true;

		//if (head_controller.query_for_event(SONARS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
		sonars_controller_created = true;

		if (head_controller.query_for_event(POTENTIOMETERS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			potentiometers_controller_created = true;

		if (motors_controller_created && sonars_controller_created && potentiometers_controller_created)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!motors_controller_created)
				sprintf(error_string, "Cannot create head's motors controller! Game over!");
			if (!sonars_controller_created)
				sprintf(error_string, "Cannot create head's sonars controller! Game over!");
			if (!potentiometers_controller_created)
				sprintf(error_string, "Cannot create head's potentiometers controller! Game over!");
			return false;
		}
	}

	head_controller.send_set_stepper_motor_speed_and_acceleration(HEAD_MOTOR_HORIZONTAL, 1500, 500);
	head_controller.send_set_stepper_motor_speed_and_acceleration(HEAD_MOTOR_VERTICAL, 1500, 500);

	bool HEAD_MOTOR_HORIZONTAL_set_speed_accell = false;
	bool HEAD_MOTOR_VERTICAL_set_speed_accell = false;

	start_time = clock();

	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (head_controller.query_for_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, HEAD_MOTOR_HORIZONTAL))  // have we received the event from Serial ?
			HEAD_MOTOR_HORIZONTAL_set_speed_accell = true;
		if (head_controller.query_for_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, HEAD_MOTOR_VERTICAL))  // have we received the event from Serial ?
			HEAD_MOTOR_VERTICAL_set_speed_accell = true;

		if (HEAD_MOTOR_HORIZONTAL_set_speed_accell && HEAD_MOTOR_VERTICAL_set_speed_accell)
			break;

		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!HEAD_MOTOR_HORIZONTAL_set_speed_accell)
				sprintf(error_string, "Cannot HEAD_MOTOR_HORIZONTAL_set_speed_accell! Game over!");
			if (!HEAD_MOTOR_VERTICAL_set_speed_accell)
				sprintf(error_string, "Cannot HEAD_MOTOR_VERTICAL_set_speed_accell! Game over!");
			return false;
		}

	}

	int potentiometer_index_head_horizontal_motor[1] = { 0 };
	int potentiometer_index_head_vertical_motor[1] = { 1 };


	int head_horizontal_motor_potentiometer_min[1] = { 329 };
	int head_horizontal_motor_potentiometer_max[1] = { 829 };
	int head_horizontal_motor_potentiometer_home[1] = { 529 };
	int head_horizontal_motor_potentiometer_dir[1] = { -1 };


	int head_vertical_motor_potentiometer_min[1] = { 332 };
	int head_vertical_motor_potentiometer_max[1] = { 832 };
	int head_vertical_motor_potentiometer_home[1] = { 632 };
	int head_vertical_motor_potentiometer_dir[1] = { 1 };

	head_controller.send_attach_sensors_to_stepper_motor(HEAD_MOTOR_HORIZONTAL, 1, potentiometer_index_head_horizontal_motor, head_horizontal_motor_potentiometer_min, head_horizontal_motor_potentiometer_max, head_horizontal_motor_potentiometer_home, head_horizontal_motor_potentiometer_dir, 0, NULL, 0, NULL);
	head_controller.send_attach_sensors_to_stepper_motor(HEAD_MOTOR_VERTICAL, 1, potentiometer_index_head_vertical_motor, head_vertical_motor_potentiometer_min, head_vertical_motor_potentiometer_max, head_vertical_motor_potentiometer_home, head_vertical_motor_potentiometer_dir, 0, NULL, 0, NULL);

	return true;
}
//----------------------------------------------------------------
bool setup_left_arm(t_jenny5_arduino_controller &left_arm_controller, char* error_string)
{

	int left_arm_motors_dir_pins[6] = { 12, 10, 8, 6, 4, 2 };
	int left_arm_motors_step_pins[6] = { 13, 11, 9, 7, 5, 3 };
	int left_arm_motors_enable_pins[6] = { 14, 14, 14, 14, 14, 14 };
	left_arm_controller.send_create_stepper_motors(5, left_arm_motors_dir_pins, left_arm_motors_step_pins, left_arm_motors_enable_pins);
	Sleep(100);
	int left_arm_potentiometer_pins[5] = { 2, 3, 4, 5, 6 };
	left_arm_controller.send_create_potentiometers(5, left_arm_potentiometer_pins);

	int gripper_infrared_pins[1] = { 7 };

	left_arm_controller.send_create_infrared_sensors(1, gripper_infrared_pins);

	int gripper_button_pins[1] = { 1 };

	left_arm_controller.send_create_buttons(1, gripper_button_pins);

	clock_t start_time = clock();

	bool motors_controller_created = false;
	bool potentiometers_controller_created = false;
	bool infrared_controller_created = false;
	bool buttons_controller_created = false;

	while (1) {
		if (!left_arm_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (left_arm_controller.query_for_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			motors_controller_created = true;

		if (left_arm_controller.query_for_event(INFRARED_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			infrared_controller_created = true;

		if (left_arm_controller.query_for_event(BUTTONS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			buttons_controller_created = true;

		if (left_arm_controller.query_for_event(POTENTIOMETERS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			potentiometers_controller_created = true;

		if (motors_controller_created && potentiometers_controller_created && infrared_controller_created && buttons_controller_created)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!motors_controller_created)
				sprintf(error_string, "Cannot create left arm's motors controller! Game over!");
			if (!potentiometers_controller_created)
				sprintf(error_string, "Cannot create left arm's potentiometers controller! Game over!");
			if (!infrared_controller_created)
				sprintf(error_string, "Cannot create left arm's infrared controller! Game over!");
			if (!buttons_controller_created)
				sprintf(error_string, "Cannot create left arm's buttons controller! Game over!");
			return false;
		}
	}
	
	left_arm_controller.send_set_stepper_motor_speed_and_acceleration(LEFT_ARM_BODY_MOTOR, 1000, 500);
	
	left_arm_controller.send_set_stepper_motor_speed_and_acceleration(LEFT_ARM_SHOULDER_UP_DOWN_MOTOR, 1000, 500);
	
	left_arm_controller.send_set_stepper_motor_speed_and_acceleration(LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR, 1000, 500);
	
	left_arm_controller.send_set_stepper_motor_speed_and_acceleration(LEFT_ARM_ELBOW_MOTOR, 1000, 500);
	
	left_arm_controller.send_set_stepper_motor_speed_and_acceleration(LEFT_ARM_FOREARM_MOTOR, 1000, 500);
	
	left_arm_controller.send_set_stepper_motor_speed_and_acceleration(LEFT_ARM_GRIPPER_MOTOR, 1000, 500);
	

	bool LEFT_ARM_BODY_MOTOR_set_speed_accell = false;
	bool LEFT_ARM_SHOULDER_UP_DOWN_MOTOR_set_speed_accell = false;
	bool LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR_set_speed_accell = false;
	bool LEFT_ARM_ELBOW_MOTOR_set_speed_accell = false;
	bool LEFT_ARM_FOREARM_MOTOR_set_speed_accell = false;
	bool LEFT_ARM_GRIPPER_MOTOR_set_speed_accell = false;

	start_time = clock();

	while (1) {
		if (!left_arm_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (left_arm_controller.query_for_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, LEFT_ARM_BODY_MOTOR))  // have we received the event from Serial ?
			LEFT_ARM_BODY_MOTOR_set_speed_accell = true;
		if (left_arm_controller.query_for_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, LEFT_ARM_SHOULDER_UP_DOWN_MOTOR))  // have we received the event from Serial ?
			LEFT_ARM_SHOULDER_UP_DOWN_MOTOR_set_speed_accell = true;
		if (left_arm_controller.query_for_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR))  // have we received the event from Serial ?
			LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR_set_speed_accell = true;
		if (left_arm_controller.query_for_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, LEFT_ARM_ELBOW_MOTOR))  // have we received the event from Serial ?
			LEFT_ARM_ELBOW_MOTOR_set_speed_accell = true;
		if (left_arm_controller.query_for_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, LEFT_ARM_FOREARM_MOTOR))  // have we received the event from Serial ?
			LEFT_ARM_FOREARM_MOTOR_set_speed_accell = true;
		if (left_arm_controller.query_for_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, LEFT_ARM_GRIPPER_MOTOR))  // have we received the event from Serial ?
			LEFT_ARM_GRIPPER_MOTOR_set_speed_accell = true;


		if (LEFT_ARM_BODY_MOTOR_set_speed_accell && LEFT_ARM_SHOULDER_UP_DOWN_MOTOR_set_speed_accell && LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR_set_speed_accell && 
			LEFT_ARM_ELBOW_MOTOR_set_speed_accell && LEFT_ARM_FOREARM_MOTOR_set_speed_accell && LEFT_ARM_GRIPPER_MOTOR_set_speed_accell)
			break;

		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!LEFT_ARM_BODY_MOTOR_set_speed_accell)
				sprintf(error_string, "Cannot LEFT_ARM_BODY_MOTOR_set_speed_accell! Game over!");
			if (!LEFT_ARM_SHOULDER_UP_DOWN_MOTOR_set_speed_accell)
				sprintf(error_string, "Cannot LEFT_ARM_SHOULDER_UP_DOWN_MOTOR_set_speed_accell! Game over!");
			if (!LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR_set_speed_accell)
				sprintf(error_string, "Cannot LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR_set_speed_accell! Game over!");
			if (!LEFT_ARM_ELBOW_MOTOR_set_speed_accell)
				sprintf(error_string, "Cannot LEFT_ARM_ELBOW_MOTOR_set_speed_accell! Game over!");
			if (!LEFT_ARM_FOREARM_MOTOR_set_speed_accell)
				sprintf(error_string, "Cannot LEFT_ARM_FOREARM_MOTOR_set_speed_accell! Game over!");
			if (!LEFT_ARM_GRIPPER_MOTOR_set_speed_accell)
				sprintf(error_string, "Cannot LEFT_ARM_GRIPPER_MOTOR_set_speed_accell! Game over!");
			return false;
		}

	}

	int potentiometer_index_LEFT_ARM_BODY_MOTOR[1] = { 0 };
	int potentiometer_index_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR[1] = { 1 };
	int potentiometer_index_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR[1] = { 2 };
	int potentiometer_index_LEFT_ARM_ELBOW_MOTOR[1] = { 3 };
	int potentiometer_index_LEFT_ARM_FOREARM_MOTOR[1] = { 4 };
	int potentiometer_index_LEFT_ARM_GRIPPER_MOTOR[1] = { 5 };


	int potentiometer_min_LEFT_ARM_BODY_MOTOR[1] = { 380 };
	int potentiometer_max_LEFT_ARM_BODY_MOTOR[1] = { 800 };
	int potentiometer_home_LEFT_ARM_BODY_MOTOR[1] = { 640 };
	int potentiometer_dir_LEFT_ARM_BODY_MOTOR[1] = { -1 };


	int potentiometer_min_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR[1] = { 170 };
	int potentiometer_max_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR[1] = { 600 };
	int potentiometer_home_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR[1] = { 550 };
	int potentiometer_dir_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR[1] = { -1 };


	int potentiometer_min_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR[1] = { 230 };
	int potentiometer_max_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR[1] = { 760 };
	int potentiometer_home_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR[1] = { 750 };
	int potentiometer_dir_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR[1] = { 1 };

	int potentiometer_min_LEFT_ARM_ELBOW_MOTOR[1] = { 440 };
	int potentiometer_max_LEFT_ARM_ELBOW_MOTOR[1] = { 830 };
	int potentiometer_home_LEFT_ARM_ELBOW_MOTOR[1] = { 630 };
	int potentiometer_dir_LEFT_ARM_ELBOW_MOTOR[1] = { 1 };

	int potentiometer_min_LEFT_ARM_FOREARM_MOTOR[1] = { 230 };
	int potentiometer_max_LEFT_ARM_FOREARM_MOTOR[1] = { 720 };
	int potentiometer_home_LEFT_ARM_FOREARM_MOTOR[1] = { 440 };
	int potentiometer_dir_LEFT_ARM_FOREARM_MOTOR[1] = { 1 };


	left_arm_controller.send_attach_sensors_to_stepper_motor(LEFT_ARM_BODY_MOTOR, 1, 
		potentiometer_index_LEFT_ARM_BODY_MOTOR, 
		potentiometer_min_LEFT_ARM_BODY_MOTOR,
		potentiometer_max_LEFT_ARM_BODY_MOTOR,
		potentiometer_home_LEFT_ARM_BODY_MOTOR,
		potentiometer_dir_LEFT_ARM_BODY_MOTOR, 
		0, NULL, 0, NULL);
	
	left_arm_controller.send_attach_sensors_to_stepper_motor(LEFT_ARM_SHOULDER_UP_DOWN_MOTOR, 1,
		potentiometer_index_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR,
		potentiometer_min_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR,
		potentiometer_max_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR,
		potentiometer_home_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR,
		potentiometer_dir_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR, 0, NULL, 0, NULL);
	
	left_arm_controller.send_attach_sensors_to_stepper_motor(LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR, 1,
		potentiometer_index_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR,
		potentiometer_min_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR,
		potentiometer_max_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR,
		potentiometer_home_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR,
		potentiometer_dir_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR, 0, NULL, 0, NULL);

	left_arm_controller.send_attach_sensors_to_stepper_motor(LEFT_ARM_ELBOW_MOTOR, 1,
		potentiometer_index_LEFT_ARM_ELBOW_MOTOR,
		potentiometer_min_LEFT_ARM_ELBOW_MOTOR,
		potentiometer_max_LEFT_ARM_ELBOW_MOTOR,
		potentiometer_home_LEFT_ARM_ELBOW_MOTOR,
		potentiometer_dir_LEFT_ARM_ELBOW_MOTOR, 0, NULL, 0, NULL);

	left_arm_controller.send_attach_sensors_to_stepper_motor(LEFT_ARM_FOREARM_MOTOR, 1,
		potentiometer_index_LEFT_ARM_FOREARM_MOTOR,
		potentiometer_min_LEFT_ARM_FOREARM_MOTOR,
		potentiometer_max_LEFT_ARM_FOREARM_MOTOR,
		potentiometer_home_LEFT_ARM_FOREARM_MOTOR,
		potentiometer_dir_LEFT_ARM_FOREARM_MOTOR, 0, NULL, 0, NULL);
	
	return true;
}
//----------------------------------------------------------------
bool home_head_motors(t_jenny5_arduino_controller &head_controller, char* error_string)
{
	// must home the head
	head_controller.send_go_home_stepper_motor(HEAD_MOTOR_HORIZONTAL);
	head_controller.send_go_home_stepper_motor(HEAD_MOTOR_VERTICAL);

	printf("Head motors home started ...");
	clock_t start_time = clock();
	bool horizontal_motor_homed = false;
	bool vertical_motor_homed = false;

	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!horizontal_motor_homed)
			if (head_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, HEAD_MOTOR_HORIZONTAL))  // have we received the event from Serial ?
				horizontal_motor_homed = true;

		if (!vertical_motor_homed)
			if (head_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, HEAD_MOTOR_VERTICAL))  // have we received the event from Serial ?
				vertical_motor_homed = true;

		if (horizontal_motor_homed && vertical_motor_homed)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 5 seconds and no home
		if (wait_time > 5) {
			if (!vertical_motor_homed)
				sprintf(error_string, "Cannot home vertical motor! Game over!");
			if (!horizontal_motor_homed)
				sprintf(error_string, "Cannot home horizontal motor! Game over!");
			return false;
		}
	}

	printf("DONE\n");
	return true;
}
//----------------------------------------------------------------
bool home_left_arm_motors(t_jenny5_arduino_controller &left_arm_controller, char* error_string)
{
	// must home the head
	
	left_arm_controller.send_go_home_stepper_motor(LEFT_ARM_BODY_MOTOR);
	
	left_arm_controller.send_go_home_stepper_motor(LEFT_ARM_SHOULDER_UP_DOWN_MOTOR);
	
	left_arm_controller.send_go_home_stepper_motor(LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR);
	
	left_arm_controller.send_go_home_stepper_motor(LEFT_ARM_ELBOW_MOTOR);
	
	left_arm_controller.send_go_home_stepper_motor(LEFT_ARM_FOREARM_MOTOR);
	

	//left_arm_controller.send_go_home_stepper_motor(LEFT_ARM_GRIPPER_MOTOR);

	printf("Left arm motors home started ...");
	clock_t start_time = clock();
	bool motor_homed_LEFT_ARM_BODY_MOTOR = false;
	bool motor_homed_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR = false;
	bool motor_homed_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR = false;
	bool motor_homed_LEFT_ARM_ELBOW_MOTOR = false;
	bool motor_homed_LEFT_ARM_FOREARM_MOTOR = false;
//	bool motor_homed_LEFT_ARM_GRIPPER_MOTOR = false;

	while (1) {
		if (!left_arm_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!motor_homed_LEFT_ARM_BODY_MOTOR)
			if (left_arm_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, LEFT_ARM_BODY_MOTOR))  // have we received the event from Serial ?
				motor_homed_LEFT_ARM_BODY_MOTOR = true;

		if (!motor_homed_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR)
			if (left_arm_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, LEFT_ARM_SHOULDER_UP_DOWN_MOTOR))  // have we received the event from Serial ?
				motor_homed_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR = true;

		if (!motor_homed_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR)
			if (left_arm_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR))  // have we received the event from Serial ?
				motor_homed_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR = true;

		if (!motor_homed_LEFT_ARM_ELBOW_MOTOR)
			if (left_arm_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, LEFT_ARM_ELBOW_MOTOR))  // have we received the event from Serial ?
				motor_homed_LEFT_ARM_ELBOW_MOTOR = true;

		if (!motor_homed_LEFT_ARM_FOREARM_MOTOR)
			if (left_arm_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, LEFT_ARM_FOREARM_MOTOR))  // have we received the event from Serial ?
				motor_homed_LEFT_ARM_FOREARM_MOTOR = true;

	//	if (!motor_homed_LEFT_ARM_GRIPPER_MOTOR)
		//	if (left_arm_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, LEFT_ARM_GRIPPER_MOTOR))  // have we received the event from Serial ?
			//	motor_homed_LEFT_ARM_GRIPPER_MOTOR = true;

		if (motor_homed_LEFT_ARM_BODY_MOTOR && motor_homed_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR && motor_homed_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR && 
			motor_homed_LEFT_ARM_ELBOW_MOTOR && motor_homed_LEFT_ARM_FOREARM_MOTOR)// && motor_homed_LEFT_ARM_GRIPPER_MOTOR)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 5 seconds and no home
		if (wait_time > 5) {
			if (!motor_homed_LEFT_ARM_BODY_MOTOR)
				sprintf(error_string, "Cannot home motor_homed_LEFT_ARM_BODY_MOTOR! Game over!");
			if (!motor_homed_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR)
				sprintf(error_string, "Cannot home motor_homed_LEFT_ARM_SHOULDER_UP_DOWN_MOTOR! Game over!");
			if (!motor_homed_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR)
				sprintf(error_string, "Cannot home motor_homed_LEFT_ARM_SHOULDER_LEFT_RIGHT_MOTOR! Game over!");
			if (!motor_homed_LEFT_ARM_ELBOW_MOTOR)
				sprintf(error_string, "Cannot home motor_homed_LEFT_ARM_ELBOW_MOTOR! Game over!");
			if (!motor_homed_LEFT_ARM_FOREARM_MOTOR)
				sprintf(error_string, "Cannot home motor_homed_LEFT_ARM_FOREARM_MOTOR! Game over!");
			//if (!motor_homed_LEFT_ARM_GRIPPER_MOTOR)
				//sprintf(error_string, "Cannot home motor_homed_LEFT_ARM_GRIPPER_MOTOR! Game over!");
			return false;
		}
	}

	printf("DONE\n");
	return true;
}
//----------------------------------------------------------------
int	main(void)
{
	t_jenny5_arduino_controller head_controller, left_arm_controller;
	VideoCapture head_cam, left_arm_cam;
	CascadeClassifier face_classifier;

	printf(PROGRAM_VERSION);

	// initialization
	char error_string[1000];
	if (!init(head_controller, left_arm_controller, head_cam, left_arm_cam, face_classifier, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate ...");
		getchar();
		return -1;
	}
	else
		printf("Initialization succceded.\n");

	// setup
	if (!setup_head(head_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate ...");
		getchar();
		return -1;
	}
	else
		printf("Head setup succceded.\n");

	if (!setup_left_arm(left_arm_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate ...");
		getchar();
		return -1;
	}
	else
		printf("Left arm setup succceded.\n");
	
	//  home
	if (!home_head_motors(head_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate ...");
		getchar();
		return -1;
	}
	else
		printf("Home head succceded.\n");
		
	if (!home_left_arm_motors(left_arm_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate ...");
		getchar();
		return -1;
	}
	else
		printf("Home left arm succceded.\n");

	Mat cam_frame; // images used in the proces
	Mat gray_frame;

	namedWindow("Head camera", WINDOW_AUTOSIZE); // window to display the results

	bool active = true;
	while (active) {        // starting infinit loop

		if (!head_controller.update_commands_from_serial())
			Sleep(DOES_NOTHING_SLEEP); // no new data from serial ... we make a little pause so that we don't kill the processor

		head_cam >> cam_frame; // put captured-image frame in frame

		cvtColor(cam_frame, gray_frame, CV_BGR2GRAY); // convert to gray and equalize
		equalizeHist(gray_frame, gray_frame);

		std::vector<Rect> faces;// create an array to store the faces found

								// find and store the faces
		face_classifier.detectMultiScale(gray_frame, faces, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(50, 50));

		t_CENTER_POINT head_center;

		bool face_found = biggest_face(faces, head_center);

		if (face_found) {
			Point p1(head_center.x - head_center.range, head_center.y - head_center.range);
			Point p2(head_center.x + head_center.range, head_center.y + head_center.range);
			// draw an outline for the faces
			rectangle(cam_frame, p1, p2, cvScalar(0, 255, 0, 0), 1, 8, 0);
		}
		else {
			Sleep(DOES_NOTHING_SLEEP); // no face found
									   //head_controller.send_move_motor(MOTOR_HEAD_HORIZONTAL, 0);// stops 
									   //head_controller.send_move_motor(MOTOR_HEAD_VERTICAL, 0);
		}

		imshow("Head camera", cam_frame); // display the result

		if (head_controller.get_sonar_state(0) == COMMAND_DONE) {// I ping the sonar only if no ping was sent before
			head_controller.send_get_sonar_distance(0);
			head_controller.set_sonar_state(0, COMMAND_SENT);
			printf("U0# - sent\n");
		}

		if (face_found) {// 
						 // horizontal movement motor

						 // send a command to the module so that the face is in the center of the image
			if (head_center.x < cam_frame.cols / 2 - TOLERANCE) {
				tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
				int num_steps_x = angle_offset.degrees_from_center_x / 1.8 * 27.0;

				head_controller.send_move_stepper_motor(HEAD_MOTOR_HORIZONTAL, num_steps_x);
				head_controller.set_stepper_motor_state(HEAD_MOTOR_HORIZONTAL, COMMAND_SENT);
				printf("M%d %d# - sent\n", HEAD_MOTOR_HORIZONTAL, num_steps_x);

				//	head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
			}
			else
				if (head_center.x > cam_frame.cols / 2 + TOLERANCE) {
					tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
					int num_steps_x = angle_offset.degrees_from_center_x / 1.8 * 27.0;

					head_controller.send_move_stepper_motor(HEAD_MOTOR_HORIZONTAL, num_steps_x);
					head_controller.set_stepper_motor_state(HEAD_MOTOR_HORIZONTAL, COMMAND_SENT);
					printf("M%d %d# - sent\n", HEAD_MOTOR_HORIZONTAL, num_steps_x);

					//	head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
				}
				else {
					// face is in the center, so I do not move
					Sleep(DOES_NOTHING_SLEEP);
				}

				// vertical movement motor
				// send a command to the module so that the face is in the center of the image
				if (head_center.y < cam_frame.rows / 2 - TOLERANCE) {
					tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
					int num_steps_y = angle_offset.degrees_from_center_y / 1.8 * 27.0;

					head_controller.send_move_stepper_motor(HEAD_MOTOR_VERTICAL, num_steps_y);
					head_controller.set_stepper_motor_state(HEAD_MOTOR_VERTICAL, COMMAND_SENT);
					printf("M%d %d# - sent\n", HEAD_MOTOR_VERTICAL, num_steps_y);
					//	head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
				}
				else
					if (head_center.y > cam_frame.rows / 2 + TOLERANCE) {
						tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
						int num_steps_y = angle_offset.degrees_from_center_y / 1.8 * 27.0;

						head_controller.send_move_stepper_motor(HEAD_MOTOR_VERTICAL, num_steps_y);
						head_controller.set_stepper_motor_state(HEAD_MOTOR_VERTICAL, COMMAND_SENT);
						printf("M%d -%d# - sent\n", HEAD_MOTOR_VERTICAL, num_steps_y);
						//		head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
					}

		}

		// now extract the executed moves from the queue ... otherwise they will just stay there
		if (head_controller.get_stepper_motor_state(HEAD_MOTOR_HORIZONTAL) == COMMAND_SENT) {// if a command has been sent
			if (head_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, HEAD_MOTOR_HORIZONTAL)) { // have we received the event from Serial ?
				head_controller.set_stepper_motor_state(HEAD_MOTOR_HORIZONTAL, COMMAND_DONE);
				printf("M%d# - done\n", HEAD_MOTOR_HORIZONTAL);
			}
		}

		// now extract the moves done from the queue
		if (head_controller.get_stepper_motor_state(HEAD_MOTOR_VERTICAL) == COMMAND_SENT) {// if a command has been sent
			if (head_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, HEAD_MOTOR_VERTICAL)) { // have we received the event from Serial ?
				head_controller.set_stepper_motor_state(HEAD_MOTOR_VERTICAL, COMMAND_DONE);
				printf("M%d# - done\n", HEAD_MOTOR_VERTICAL);
			}
		}

		// read to see if there is any distance received from sonar
		if (head_controller.get_sonar_state(0) == COMMAND_SENT) {// if a command has been sent
			int distance;
			if (head_controller.query_for_event(SONAR_EVENT, 0, &distance)) { // have we received the event from Serial ?
				head_controller.set_sonar_state(0, COMMAND_DONE);
				printf("distance = %d cm\n", distance);
			}
		}

		if (waitKey(1) >= 0)  // break the loop
			active = false;
	}

	head_controller.send_move_stepper_motor(HEAD_MOTOR_HORIZONTAL, 0);
	head_controller.send_move_stepper_motor(HEAD_MOTOR_VERTICAL, 0);

	head_controller.close_connection();
	return 0;
}
//----------------------------------------------------------------