// author: Mihai Oltean, www.tcreate.org, mihai.oltean@gmail.com
// More info: www.jenny5.org
// www.github.com/jenny5-robot
// MIT License
//---------------------------------------------------------------

#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>


#define _USE_MATH_DEFINES

#include <time.h>
#include <math.h>

#include "jenny5_arduino_controller.h"
#include "jenny5_events.h"

//----------------------------------------------------------------

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x)*1000)
#endif

using namespace std;
using namespace cv;

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3

#define LIDAR_NUM_STEPS 200

double scale_factor = 0.1;
int lidar_distances[LIDAR_NUM_STEPS];

//----------------------------------------------------------------
bool connect(t_jenny5_arduino_controller &lidar_controller, char* error_string)
{
	//-------------- START INITIALIZATION ------------------------------

	if (!lidar_controller.connect(5, 115200)) { // real number - 1
		sprintf(error_string, "Error attaching to Jenny 5' LIDAR!");
		return false;
	}

	// now wait to see if I have been connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();

	bool LIDAR_responded = false;

	while (1) {
		if (!lidar_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!LIDAR_responded)
			if (lidar_controller.query_for_event(IS_ALIVE_EVENT)) {  // have we received the event from Serial ?
				LIDAR_responded = true;
				break;
			}

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!LIDAR_responded)
				sprintf(error_string, "LIDAR does not respond! Game over!");
			return false;
		}
	}

	return true;
}
//----------------------------------------------------------------
bool setup(t_jenny5_arduino_controller &LIDAR_controller, char* error_string)
{
	LIDAR_controller.send_create_LIDAR(5, 6, 7, 12);// dir, step, enable, IR_pin

	clock_t start_time = clock();

	bool lidar_controller_created = false;

	while (1) {
		if (!LIDAR_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor
			
		if (!lidar_controller_created)
			if (LIDAR_controller.query_for_event(LIDAR_CONTROLLER_CREATED_EVENT))  // have we received the event from Serial ?
				lidar_controller_created = true;

		if (lidar_controller_created)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!lidar_controller_created)
				sprintf(error_string, "Cannot create LIDAR controller! Game over!");
			return false;
		}
	}

	return true;
}
//----------------------------------------------------------------
struct t_user_data
{
	Mat *lidar_image;
	int image_width;
};
//----------------------------------------------------------------
static void on_mouse(int event, int x, int y, int flags, void *userdata)
{
	if (event == EVENT_MOUSEWHEEL) {
		int delta = getMouseWheelDelta(flags);

		t_user_data* user_data = (t_user_data*)userdata;
		Point center(user_data->image_width / 2, user_data->image_width / 2);
		for (int i = 0; i < LIDAR_NUM_STEPS; i++) {
			Point old_p;
			old_p.x = -lidar_distances[i] * scale_factor * sin(i / 100.0 * M_PI - M_PI / 2);
			old_p.y = -lidar_distances[i] * scale_factor * cos(i / 100.0 * M_PI - M_PI / 2);
			circle(*(user_data->lidar_image), center + old_p, 5.0, Scalar(0, 0, 0), 1, 8);
		}
		char text[100];
		sprintf(text, "scale = %.2lf", scale_factor);
		int font_face = FONT_HERSHEY_SCRIPT_SIMPLEX;
		cv::Point text_position(10, 25);

		cv::putText(*(user_data->lidar_image), text, text_position, font_face, 1, Scalar::all(0), 1, 8);
		// delete the robot
		rectangle(*user_data->lidar_image, Point(center.x - 175 * scale_factor, center.y), Point(center.x + scale_factor * 175, center.y + 600 * scale_factor), Scalar(0, 0, 0));


		if (delta > 0)
			scale_factor += 0.01;
		else
			if (delta < 0)
				scale_factor -= 0.01;
			
		if (scale_factor < 0.01)
			scale_factor = 0.01;

		for (int i = 0; i < LIDAR_NUM_STEPS; i++) {
				// draw the new point
			Point new_p;
			new_p.x = -lidar_distances[i] * scale_factor * sin(i / 100.0 * M_PI - M_PI / 2);
			new_p.y = -lidar_distances[i] * scale_factor * cos(i / 100.0 * M_PI - M_PI / 2);
			circle(*(user_data->lidar_image), center + new_p, 5.0, Scalar(0, 0, 255), 1, 8);
		}

		// robot
		rectangle(*user_data->lidar_image, Point(center.x - 175 * scale_factor, center.y), Point(center.x + scale_factor * 175, center.y + 600 * scale_factor), Scalar(0, 255, 0));

		sprintf(text, "scale = %.2lf", scale_factor);
		cv::putText(*user_data->lidar_image, text, text_position, font_face, 1, Scalar::all(255), 1, 8);
	}
}
//----------------------------------------------------------------
int	main(void)
{
	t_jenny5_arduino_controller LIDAR_controller;
	
	for (int i = 0; i < LIDAR_NUM_STEPS; i++)
		lidar_distances[i] = 0;

	// initialization
	char error_string[1000];

	// setup
	if (!connect(LIDAR_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate...");
		getchar();
		return -1;
	}
	else
		printf("Connection OK.\n");

	// setup
	if (!setup(LIDAR_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter to terminate...");
		getchar();
		return -1;
	}
	else
		printf("Setup OK.\n");

	LIDAR_controller.send_set_LIDAR_motor_speed_and_acceleration(60, 100);
	LIDAR_controller.send_LIDAR_go();

	namedWindow("LIDAR map", WINDOW_AUTOSIZE);

	int image_width = 600;
	Mat lidar_image = Mat::zeros(image_width, image_width, CV_8UC3);
	Point center(image_width / 2, image_width / 2);
	//LIDAR
	circle(lidar_image, center, 10.0, Scalar(0, 255, 0), 1, 8);
    // robot
	rectangle(lidar_image, Point(center.x - 175 * scale_factor , center.y), Point(center.x + scale_factor * 175, center.y + 600 * scale_factor), Scalar(0, 255, 0));

	t_user_data user_data;

	user_data.lidar_image = &lidar_image;
	user_data.image_width = image_width;

	setMouseCallback("LIDAR map", on_mouse, &user_data);
	
	char text[100];
	sprintf(text, "scale = %.2lf", scale_factor);
	int font_face = FONT_HERSHEY_SCRIPT_SIMPLEX;
	cv::Point text_position(10, 25);
	cv::putText(lidar_image, text, text_position, font_face, 1, Scalar::all(255), 1, 8);

	printf("Now running the main loop. Press Escape when want to exit!\n");
	bool active = true;

	while (active) {        // starting infinit loop
		
		if (!LIDAR_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		int motor_position, distance;
		while (LIDAR_controller.query_for_event(LIDAR_READ_EVENT, &motor_position, &distance)) {  // have we received the event from Serial ?
			
// delete old distance
			Point old_p;
			old_p.x = -lidar_distances[motor_position] * scale_factor * sin(motor_position / 100.0 * M_PI - M_PI / 2);
			old_p.y = -lidar_distances[motor_position] * scale_factor * cos(motor_position / 100.0 * M_PI - M_PI / 2);
			circle(lidar_image, center + old_p, 5.0, Scalar(0, 0, 0), 1, 8);

// draw the new point
			lidar_distances[motor_position] = distance;
			Point new_p;
			new_p.x = - distance * scale_factor * sin(motor_position / 100.0 * M_PI - M_PI / 2);
			new_p.y = - distance * scale_factor * cos(motor_position / 100.0 * M_PI - M_PI / 2);
			circle(lidar_image, center + new_p, 5.0, Scalar(0, 0, 255), 1, 8);
			cout << "Motor position = " << motor_position << " LIDAR distance = " << distance << endl;
		}

		imshow("LIDAR map", lidar_image);
		
		if (waitKey(1) >= 0)  // break the loop
			active = false;
	}
	
	LIDAR_controller.send_LIDAR_stop();

	LIDAR_controller.close_connection();

	printf("\n\nProgram over. Press Enter!");
	
	//getchar();
	return 0;
}
//----------------------------------------------------------------