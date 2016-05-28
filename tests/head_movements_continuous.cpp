#include <iostream>
#include <time.h>

#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

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
using namespace cv;

typedef struct _CENTER_POINT
{
	int x;
	int y;
	int range;
}CENTER_POINT, *PCENTER_POINT;

#define MOTOR_HEAD_HORIZONTAL 0
#define MOTOR_HEAD_VERTICAL 1

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3

#define TOLERANCE 20

#define DOES_NOTHING_SLEEP 10


//----------------------------------------------------------------
bool biggest_face(std::vector<Rect> faces, CENTER_POINT &center)
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
bool init(t_jenny5_command_module &head_controller, VideoCapture &head_cam, CascadeClassifier &face_classifier, char* error_string)
{
	//-------------- START INITIALIZATION ------------------------------

	if (!head_controller.connect(4, 115200)) {
		sprintf(error_string, "Error attaching to Jenny 5' head!");
		return false;
	}
	// now wait to see if I have been connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();

	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (head_controller.query_for_event(IS_ALIVE_EVENT, 0)) { // have we received the event from Serial ?
			break;
		}
		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			sprintf(error_string, "Head does not respond! Game over!");
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
		sprintf(error_string, "Couldn't open head's video cam!");
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
bool setup(t_jenny5_command_module &head_controller, char* error_string)
{

	int head_motors_dir_pins[2] = { 2, 5 };
	int head_motors_step_pins[2] = { 3, 6 };
	int head_motors_enable_pins[2] = { 4, 7 };
	head_controller.send_create_stepper_motors(2, head_motors_dir_pins, head_motors_step_pins, head_motors_enable_pins);

	int head_sonars_trig_pins[1] = { 8 };
	int head_sonars_echo_pins[1] = { 9 };

	head_controller.send_create_sonars(1, head_sonars_trig_pins, head_sonars_echo_pins);

	int head_infrared_pins[2] = { 0, 1 };
	int head_infrared_min[2] = { 100, 100 };
	int head_infrared_max[2] = { 650, 800 };
	int head_infrared_home[2] = { 470, 400 };
	int head_infrared_dir[2] = { 1, 1 };

	head_controller.send_create_infrared_sensors(2, head_infrared_pins, head_infrared_min, head_infrared_max, head_infrared_home, head_infrared_dir);

	clock_t start_time = clock();

	bool motors_controller_created = false;
	bool sonars_controller_created = false;
	bool infrareds_controller_created = false;

	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (head_controller.query_for_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			motors_controller_created = true;

		if (head_controller.query_for_event(SONARS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			sonars_controller_created = true;

		if (head_controller.query_for_event(INFRARED_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			infrareds_controller_created = true;

		if (motors_controller_created && sonars_controller_created && infrareds_controller_created)
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
			if (!infrareds_controller_created)
				sprintf(error_string, "Cannot create head's infrared controller! Game over!");
			return false;
		}
	}


	head_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_HEAD_HORIZONTAL, 1500, 500);
	head_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_HEAD_VERTICAL, 1500, 500);

	int infrared_index_m1[1] = { 0 };
	int infrared_index_m2[1] = { 1 };
	head_controller.send_attach_sensors_to_stepper_motor(0, 0, NULL, 1, infrared_index_m1, 0, NULL);
	head_controller.send_attach_sensors_to_stepper_motor(1, 0, NULL, 1, infrared_index_m2, 0, NULL);

	return true;
}
//----------------------------------------------------------------
bool init(t_jenny5_command_module &head_controller, char* error_string)
{
	// must home the head
	head_controller.send_go_home_stepper_motor(MOTOR_HEAD_HORIZONTAL);
	head_controller.send_go_home_stepper_motor(MOTOR_HEAD_VERTICAL);

	printf("Head motors home started ...");
	clock_t start_time = clock();
	bool horizontal_motor_homed = false;
	bool vertical_motor_homed = false;

	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!horizontal_motor_homed)
			if (head_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_HEAD_HORIZONTAL))  // have we received the event from Serial ?
				horizontal_motor_homed = true;

		if (!vertical_motor_homed)
			if (head_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_HEAD_VERTICAL))  // have we received the event from Serial ?
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
				sprintf(error_string, "Cannot home vertical motor! Game over!");
			return false;
		}
	}

	printf("DONE\n");
	return true;
}
//----------------------------------------------------------------
int	main(int argc, const char** argv)
{
	t_jenny5_command_module head_controller;
	VideoCapture head_cam;
	CascadeClassifier face_classifier;

	// initialization
	char error_string[1000];
	if (!init(head_controller, head_cam, face_classifier, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Initialization succceded.\n");

	// setup
	if (!setup(head_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Setup succceded.\n");


	//  init
	if (!init(head_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Init succceded.\n");


	Mat frame; // images used in the proces
	Mat grayFrame;

	namedWindow("Head camera", WINDOW_AUTOSIZE); // window to display the results

	bool active = true;
	while (active)        // starting infinit loop
	{
		if (!head_controller.update_commands_from_serial())
			Sleep(DOES_NOTHING_SLEEP); // no new data from serial ... we make a little pause so that we don't kill the processor

		head_cam >> frame; // put captured-image frame in frame

		cvtColor(frame, grayFrame, CV_BGR2GRAY); // convert to gray and equalize
		equalizeHist(grayFrame, grayFrame);

		std::vector<Rect> faces;// create an array to store the faces found

		// find and store the faces
		face_classifier.detectMultiScale(grayFrame, faces, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(50, 50));

		CENTER_POINT head_center;

		bool face_found = biggest_face(faces, head_center);

		if (face_found) {
			Point p1(head_center.x - head_center.range, head_center.y - head_center.range);
			Point p2(head_center.x + head_center.range, head_center.y + head_center.range);
			// draw an outline for the faces
			rectangle(frame, p1, p2, cvScalar(0, 255, 0, 0), 1, 8, 0);
		}
		else {
			Sleep(DOES_NOTHING_SLEEP); // no face found
			//head_controller.send_move_motor(MOTOR_HEAD_HORIZONTAL, 0);// stops 
			//head_controller.send_move_motor(MOTOR_HEAD_VERTICAL, 0);
		}

		imshow("Head camera", frame); // display the result

		if (head_controller.get_sonar_state(0) == COMMAND_DONE) {// I ping the sonar only if no ping was sent before
			head_controller.send_get_sonar_distance(0);
			head_controller.set_sonar_state(0, COMMAND_SENT);
			printf("U0# - sent\n");
		}

		if (face_found) {// 
			// horizontal movement motor

			// send a command to the module so that the face is in the center of the image
			if (head_center.x < frame.cols / 2 - TOLERANCE) {
				tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
				int num_steps_x = angle_offset.degrees_from_center_x / 1.8 * 27.0;

				head_controller.send_move_stepper_motor(MOTOR_HEAD_HORIZONTAL, num_steps_x);
				head_controller.set_stepper_motor_state(MOTOR_HEAD_HORIZONTAL, COMMAND_SENT);
				printf("M%d %d# - sent\n", MOTOR_HEAD_HORIZONTAL, num_steps_x);

				//	head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
			}
			else
				if (head_center.x > frame.cols / 2 + TOLERANCE) {
					tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
					int num_steps_x = angle_offset.degrees_from_center_x / 1.8 * 27.0;

					head_controller.send_move_stepper_motor(MOTOR_HEAD_HORIZONTAL, num_steps_x);
					head_controller.set_stepper_motor_state(MOTOR_HEAD_HORIZONTAL, COMMAND_SENT);
					printf("M%d %d# - sent\n", MOTOR_HEAD_HORIZONTAL, num_steps_x);

					//	head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
				}
				else {
					// face is in the center, so I do not move
					Sleep(DOES_NOTHING_SLEEP);
				}

				// vertical movement motor
				// send a command to the module so that the face is in the center of the image
				if (head_center.y < frame.rows / 2 - TOLERANCE) {
					tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
					int num_steps_y = angle_offset.degrees_from_center_y / 1.8 * 27.0;

					head_controller.send_move_stepper_motor(MOTOR_HEAD_VERTICAL, num_steps_y);
					head_controller.set_stepper_motor_state(MOTOR_HEAD_VERTICAL, COMMAND_SENT);
					printf("M%d %d# - sent\n", MOTOR_HEAD_VERTICAL, num_steps_y);
					//	head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
				}
				else
					if (head_center.y > frame.rows / 2 + TOLERANCE) {
						tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
						int num_steps_y = angle_offset.degrees_from_center_y / 1.8 * 27.0;

						head_controller.send_move_stepper_motor(MOTOR_HEAD_VERTICAL, num_steps_y);
						head_controller.set_stepper_motor_state(MOTOR_HEAD_VERTICAL, COMMAND_SENT);
						printf("M%d -%d# - sent\n", MOTOR_HEAD_VERTICAL, num_steps_y);
						//		head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
					}

		}

		// now extract the executed moves from the queue ... otherwise they will just stay there
		if (head_controller.get_stepper_motor_state(MOTOR_HEAD_HORIZONTAL) == COMMAND_SENT) {// if a command has been sent
			if (head_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_HEAD_HORIZONTAL)) { // have we received the event from Serial ?
				head_controller.set_stepper_motor_state(MOTOR_HEAD_HORIZONTAL, COMMAND_DONE);
				printf("M%d# - done\n", MOTOR_HEAD_HORIZONTAL);
			}
		}

		// now extract the moves done from the queue
		if (head_controller.get_stepper_motor_state(MOTOR_HEAD_VERTICAL) == COMMAND_SENT) {// if a command has been sent
			if (head_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_HEAD_VERTICAL)) { // have we received the event from Serial ?
				head_controller.set_stepper_motor_state(MOTOR_HEAD_VERTICAL, COMMAND_DONE);
				printf("M%d# - done\n", MOTOR_HEAD_VERTICAL);
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

	head_controller.close_connection();
	return 0;
}
//----------------------------------------------------------------