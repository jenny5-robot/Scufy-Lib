#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>
#include <time.h>

#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include "jenny5_arduino_controller.h"
#include "jenny5_events.h"
#include "point_tracker.h"
#include "roboclaw_controller.h"
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

#define MOTOR_tracks_LEFT 0
#define MOTOR_tracks_RIGHT 1

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3

#define TOLERANCE 150

#define DOES_NOTHING_SLEEP 10

#define HEAD_RADIUS_TO_REVERT 70

#define TRACKS_MOTOR_REDUCTION 5

#define LIDAR_NUM_STEPS 200

double scale_factor = 5.0;
int lidar_distances[LIDAR_NUM_STEPS];

#define MOTOR_FULL_SPEED 1500
#define MOTOR_FULL_TORQUE_SPEED 500


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
bool connect(t_jenny5_arduino_controller &head_controller, t_jenny5_arduino_controller &lidar_controller, t_roboclaw_controller &tracks_controller, VideoCapture &head_cam, CascadeClassifier &face_detector, char* error_string)
{
	//-------------- START INITIALIZATION ------------------------------

	if (!head_controller.connect(2, 115200)) { // real - 1
		sprintf(error_string, "Error attaching to Jenny 5' head!");
		return false;
	}

	if (!lidar_controller.connect(3, 115200)) {
		sprintf(error_string, "Error attaching to Jenny 5' LIDAR!");
		return false;
	}

	if (!tracks_controller.connect(3, 115200)) {
		sprintf(error_string, "Error attaching to Jenny 5' tracks!");
		return false;
	}
	

	// now wait to see if I have been connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();
	bool head_responded = false;
	bool tracks_responded = false;

	while (1) {
		if (!head_controller.update_commands_from_serial() && !lidar_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!head_responded)
			if (head_controller.query_for_event(IS_ALIVE_EVENT, 0))  // have we received the event from Serial ?
				head_responded = true;
		
		if (!tracks_responded)
			if (lidar_controller.query_for_event(IS_ALIVE_EVENT, 0))  // have we received the event from Serial ?
				tracks_responded = true;

		if (head_responded && tracks_responded)
			break;
		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!head_responded)
			  sprintf(error_string, "Head does not respond! Game over!");

			if (!tracks_responded)
				sprintf(error_string, "Tracks do not respond! Game over!");
			return false;
		}
	}

	// home head's motors



	// create cascade for face reco
	// load haarcascade library
	if (!face_detector.load("haarcascade_frontalface_alt.xml")) {
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
	return true;
}
//----------------------------------------------------------------
bool setup(t_jenny5_arduino_controller &head_controller, t_jenny5_arduino_controller &lidar_controller, char* error_string)
{
	
	int head_motors_dir_pins[2] = { 5, 2 };
	int head_motors_step_pins[2] = { 6, 3 };
	int head_motors_enable_pins[2] = { 7, 4 };
	head_controller.send_create_stepper_motors(2, head_motors_dir_pins, head_motors_step_pins, head_motors_enable_pins);

	//int head_sonars_trig_pins[1] = { 8 };
	//int head_sonars_echo_pins[1] = { 9 };

	//head_controller.send_create_sonars(1, head_sonars_trig_pins, head_sonars_echo_pins);

	int head_potentiometer_pins[2] = { 0, 1 };
	int head_potentiometer_min[2] = { 329, 332 };
	int head_potentiometer_max[2] = { 829, 832 };
	int head_potentiometer_home[2] = { 529, 632 };
	int head_potentiometer_dir[2] = { -1, 1 };

	head_controller.send_create_potentiometers(2, head_potentiometer_pins, head_potentiometer_min, head_potentiometer_max, head_potentiometer_home, head_potentiometer_dir);

	lidar_controller.send_create_LIDAR(5, 6, 7, 11);// dir, step, enable, IR_pin

	clock_t start_time = clock();
	bool head_motors_controler_created = false;
	//bool head_sonars_controller_created = false;
	bool infrareds_controller_created = false;
	bool lidar_controller_created = false;


	while (1) {
		if (!head_controller.update_commands_from_serial() && !lidar_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!head_motors_controler_created)
			if (head_controller.query_for_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
				head_motors_controler_created = true;

		if (head_controller.query_for_event(INFRARED_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
			infrareds_controller_created = true;

		if (!lidar_controller_created)
			if (lidar_controller.query_for_event(LIDAR_CONTROLLER_CREATED_EVENT))  // have we received the event from Serial ?
				lidar_controller_created = true;

		if (head_motors_controler_created && lidar_controller_created && /*head_sonars_controller_created && */infrareds_controller_created)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!head_motors_controler_created)
				sprintf(error_string, "Cannot create head's motor controller! Game over!");

			//if (!head_sonars_controller_created)
				//sprintf(error_string, "Cannot create head's sonars controller! Game over!");

			if (!infrareds_controller_created)
				sprintf(error_string, "Cannot create head's infrared controller! Game over!");

			if (!lidar_controller_created)
				sprintf(error_string, "Cannot create LIDAR controller! Game over!");

			return false;
		}
	}
	
	head_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_HEAD_HORIZONTAL, 1500, 500);
	head_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_HEAD_VERTICAL, 1500, 500);

	int potentiometer_index_m1[1] = { 0 };
	int potentiometer_index_m2[1] = { 1 };
	head_controller.send_attach_sensors_to_stepper_motor(MOTOR_HEAD_HORIZONTAL, 1, potentiometer_index_m1, 0, NULL, 0, NULL);
	head_controller.send_attach_sensors_to_stepper_motor(MOTOR_HEAD_VERTICAL, 1, potentiometer_index_m2, 0, NULL, 0, NULL);

	lidar_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_tracks_LEFT, MOTOR_FULL_SPEED, 500);
	lidar_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_tracks_RIGHT, MOTOR_FULL_SPEED, 500);
	return true;
}
//----------------------------------------------------------------
bool home_motors(t_jenny5_arduino_controller &head_controller, char* error_string)
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
				sprintf(error_string, "Cannot home horizontal motor! Game over!");
			return false;
		}
	}

	printf("DONE\n");
	return true;
}
//----------------------------------------------------------------
bool clear_ahead(int *lidar_distances)
{
	for (int i = 40; i < 60; i++)
	if (lidar_distances[i] < 800)
		return false;
	return true;
}
//----------------------------------------------------------------
int	main(void)
{
	t_jenny5_arduino_controller head_controller, lidar_controller;
	t_roboclaw_controller tracks_controller;

	VideoCapture head_cam;
	CascadeClassifier face_detector;

	// initialization
	char error_string[1000];
	if (!connect(head_controller, lidar_controller, tracks_controller, head_cam, face_detector, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate ...");
		getchar();
		return -1;
	}
	else
		printf("Initialization succceded.\n");

	// setup
	if (!setup(head_controller, lidar_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate ...");
		getchar();
		return -1;
	}
	else
		printf("Setup succceded.\n");
	
	//  home motors
	if (!home_motors(head_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate ...");
		getchar();
		return -1;
	}
	else
		printf("Home motors succceded.\n");

	lidar_controller.send_set_LIDAR_motor_speed_and_acceleration(30, 100);
	lidar_controller.send_LIDAR_go();

	namedWindow("LIDAR map", WINDOW_AUTOSIZE);
	// draw the robot
	int w = 600;
	Mat lidar_map_image = Mat::zeros(w, w, CV_8UC3);
	Point center(w / 2, w / 2);
	circle(lidar_map_image, center, 20.0, Scalar(0, 255, 0), 1, 8);
	// finish drawing robot

	Mat cam_frame; // images used in the proces
	Mat gray_frame;

	namedWindow("Head camera", WINDOW_AUTOSIZE); // window to display the results

	bool active = true; 
	while (active)        // starting infinit loop
	{
		if (!head_controller.update_commands_from_serial() && !lidar_controller.update_commands_from_serial())
			Sleep(DOES_NOTHING_SLEEP); // no new data from serial ... we take a little break so that we don't kill the processor
		else {
			// extract all data from LIDAR 
			int motor_position, distance;
			bool at_least_one_new_LIDAR_distance = false;
			while (lidar_controller.query_for_event(LIDAR_READ_EVENT, &motor_position, &distance)) {  // have we received the event from Serial ?

																									  // delete old distance
				Point old_p;
				old_p.x = -lidar_distances[motor_position] / scale_factor * sin(motor_position / 100.0 * M_PI - M_PI / 2);
				old_p.y = -lidar_distances[motor_position] / scale_factor * cos(motor_position / 100.0 * M_PI - M_PI / 2);
				circle(lidar_map_image, center + old_p, 5.0, Scalar(0, 0, 0), 1, 8);

				// draw the new point
				lidar_distances[motor_position] = distance;
				Point new_p;
				new_p.x = -distance / scale_factor * sin(motor_position / 100.0 * M_PI - M_PI / 2);
				new_p.y = -distance / scale_factor * cos(motor_position / 100.0 * M_PI - M_PI / 2);
				circle(lidar_map_image, center + new_p, 5.0, Scalar(0, 0, 255), 1, 8);
				//cout << "Motor position = " << motor_position << " LIDAR distance = " << distance << endl;

				at_least_one_new_LIDAR_distance = true;
			}

			if (at_least_one_new_LIDAR_distance)
			  imshow("LIDAR map", lidar_map_image);
		}
		head_cam >> cam_frame; // put captured-image frame in frame

		cvtColor(cam_frame, gray_frame, CV_BGR2GRAY); // convert to gray and equalize
		equalizeHist(gray_frame, gray_frame);

		std::vector<Rect> faces;// create an array to store the faces found

		// find and store the faces
		face_detector.detectMultiScale(gray_frame, faces, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(30, 30));

		CENTER_POINT head_center;

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

		if (face_found) {

			// send a command to the module so that the face is in the center of the image
			if (head_center.x > cam_frame.cols / 2 + TOLERANCE) {

				//tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
				//int num_steps_x = (int)(angle_offset.degrees_from_center_x / 1.8 * 8) * TRACKS_MOTOR_REDUCTION;

				// rotate
				tracks_controller.drive_M1_with_signed_duty_and_acceleration(100, 1);
				tracks_controller.drive_M2_with_signed_duty_and_acceleration(100, 1);
				//lidar_controller.set_stepper_motor_state(MOTOR_tracks_LEFT, COMMAND_SENT);
				//lidar_controller.set_stepper_motor_state(MOTOR_tracks_RIGHT, COMMAND_SENT);
				//printf("tracks: M%d %d M%d %d# - sent\n", MOTOR_tracks_LEFT, num_steps_x, MOTOR_tracks_RIGHT, num_steps_x);


			}
			else
				if (head_center.x < cam_frame.cols / 2 - TOLERANCE) {

//					tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
//					int num_steps_x = (int)(angle_offset.degrees_from_center_x / 1.8 * 8) * TRACKS_MOTOR_REDUCTION;

					// rotate
					tracks_controller.drive_M1_with_signed_duty_and_acceleration(-100, 1);
					tracks_controller.drive_M2_with_signed_duty_and_acceleration(-100, 1);
					/*
					lidar_controller.send_move_stepper_motor2(MOTOR_tracks_RIGHT, num_steps_x, MOTOR_tracks_LEFT, num_steps_x);
					lidar_controller.set_stepper_motor_state(MOTOR_tracks_RIGHT, COMMAND_SENT);
					lidar_controller.set_stepper_motor_state(MOTOR_tracks_LEFT, COMMAND_SENT);
					printf("tracks: M%d %d# - sent\n", MOTOR_tracks_RIGHT, num_steps_x);
					*/
				}
				else {

					// face is in the center, so I move equaly with both motors
					if (head_center.range < HEAD_RADIUS_TO_REVERT) {
						// move forward
						// only if LIDAR distance to the front point is very far from the robot
						if (clear_ahead(lidar_distances)) {
							tracks_controller.drive_M1_with_signed_duty_and_acceleration(100, 1);
							tracks_controller.drive_M2_with_signed_duty_and_acceleration(-100, 1);

							printf("tracks: M%d %d# - sent\n", MOTOR_tracks_LEFT, 1000);


						}
					}
					else {
						// move backward
						tracks_controller.drive_M1_with_signed_duty_and_acceleration(-100, 1);
						tracks_controller.drive_M2_with_signed_duty_and_acceleration(100, 1);
						printf("tracks: M%d %d# - sent\n", MOTOR_tracks_RIGHT, 1000);
					}

				}

				// vertical movement motor
				// send a command to the module so that the face is in the center of the image
				if (head_center.y < cam_frame.rows / 2 - TOLERANCE) {
					tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
					int num_steps_y = angle_offset.degrees_from_center_y / 1.8 * 27.0;

					head_controller.send_move_stepper_motor(MOTOR_HEAD_VERTICAL, num_steps_y);
					head_controller.set_stepper_motor_state(MOTOR_HEAD_VERTICAL, COMMAND_SENT);
					printf("M%d %d# - sent\n", MOTOR_HEAD_VERTICAL, num_steps_y);
					//	head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
				}
				else
					if (head_center.y > cam_frame.rows / 2 + TOLERANCE) {
						tracking_data angle_offset = get_offset_angles(920, Point(head_center.x, head_center.y));
						int num_steps_y = angle_offset.degrees_from_center_y / 1.8 * 27.0;

						head_controller.send_move_stepper_motor(MOTOR_HEAD_VERTICAL, num_steps_y);
						head_controller.set_stepper_motor_state(MOTOR_HEAD_VERTICAL, COMMAND_SENT);
						printf("M%d -%d# - sent\n", MOTOR_HEAD_VERTICAL, num_steps_y);
						//		head_controller.set_sonar_state(0, COMMAND_DONE); // if the motor has been moved the previous distances become invalid
					}
		}

		// now extract the executed moves from the queue ... otherwise they will just sit there and will occupy memory
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

		if (waitKey(1) >= 0)  // break the loop
			active = false;
	}

	// stops all motors
	head_controller.send_move_stepper_motor(MOTOR_HEAD_VERTICAL, 0);
	head_controller.send_move_stepper_motor(MOTOR_HEAD_HORIZONTAL, 0);

	head_controller.send_disable_stepper_motor(MOTOR_HEAD_VERTICAL);
	head_controller.send_disable_stepper_motor(MOTOR_HEAD_HORIZONTAL);

	tracks_controller.drive_M1_with_signed_duty_and_acceleration(0, 1);
	tracks_controller.drive_M2_with_signed_duty_and_acceleration(0, 1);

	lidar_controller.send_LIDAR_stop();

	// close connection
	head_controller.close_connection();
	lidar_controller.close_connection();
	return 0;
}
//----------------------------------------------------------------