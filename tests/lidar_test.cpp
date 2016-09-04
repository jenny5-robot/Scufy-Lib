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

#include "jenny5_command_module.h"
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

double scale_factor = 5.0;
int lidar_distances[LIDAR_NUM_STEPS];



//----------------------------------------------------------------
bool connect(t_jenny5_command_module &lidar_controller, char* error_string)
{
	//-------------- START INITIALIZATION ------------------------------

	if (!lidar_controller.connect(10, 115200)) { // real number - 1
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
bool setup(t_jenny5_command_module &LIDAR_controller, char* error_string)
{
	LIDAR_controller.send_create_LIDAR(5, 6, 7, 11);// dir, step, enable, IR_pin

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
	Mat *image;
	int w;
};
//----------------------------------------------------------------
static void on_mouse(int event, int x, int y, int flags, void *userdata)
{
	if (event == EVENT_MOUSEWHEEL) {
		int delta = getMouseWheelDelta(flags);

		t_user_data* user_data = (t_user_data*)userdata;
		Point center(user_data->w / 2, user_data->w / 2);
		for (int i = 0; i < LIDAR_NUM_STEPS; i++) {
			Point old_p;
			old_p.x = -lidar_distances[i] / scale_factor * sin(i / 100.0 * M_PI - M_PI / 2);
			old_p.y = -lidar_distances[i] / scale_factor * cos(i / 100.0 * M_PI - M_PI / 2);
			circle(*(user_data->image), center + old_p, 5.0, Scalar(0, 0, 0), 1, 8);
		}
		char text[100];
		sprintf(text, "%lfx", scale_factor);
		int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
		double fontScale = 1;
		int thickness = 1;
		cv::Point textOrg(10, 130);

		cv::putText(*(user_data->image), text, textOrg, fontFace, fontScale, Scalar::all(0), thickness, 8);
		
		scale_factor *= delta / 120.0;

		for (int i = 0; i < LIDAR_NUM_STEPS; i++) {
				// draw the new point
			Point new_p;
			new_p.x = -lidar_distances[i] / scale_factor * sin(i / 100.0 * M_PI - M_PI / 2);
			new_p.y = -lidar_distances[i] / scale_factor * cos(i / 100.0 * M_PI - M_PI / 2);
			circle(*(user_data->image), center + new_p, 5.0, Scalar(0, 0, 255), 1, 8);
		}

		sprintf(text, "%lfx", scale_factor);
		cv::putText(*(user_data->image), text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
	}
}
//----------------------------------------------------------------
int	main(void)
{
	t_jenny5_command_module LIDAR_controller;
	
	for (int i = 0; i < LIDAR_NUM_STEPS; i++)
		lidar_distances[i] = 0;

	// initialization
	char error_string[1000];

	// setup
	if (!connect(LIDAR_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Connection OK.\n");

	// setup
	if (!setup(LIDAR_controller, error_string)) {
		printf("%s\n", error_string);
		printf("Press Enter...");
		getchar();
		return -1;
	}
	else
		printf("Setup OK.\n");

	LIDAR_controller.send_set_LIDAR_motor_speed_and_acceleration(30, 100);
	LIDAR_controller.send_LIDAR_go();

	namedWindow("LIDAR map", WINDOW_AUTOSIZE);

	int w = 600;
	Mat image = Mat::zeros(w, w, CV_8UC3);
	Point center(w / 2, w / 2);
	circle(image, center, 20.0, Scalar(0, 255, 0), 1, 8);

	t_user_data user_data;

	user_data.image = &image;
	user_data.w = w;

	setMouseCallback("LIDAR map", on_mouse, &user_data);
	
	char text[100];
	sprintf(text, "%lf", scale_factor);
	int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 130);
	cv::putText(image, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);


	printf("Now running the main loop. Press Escape when want to exit!\n");
	bool active = true;


	while (active) {        // starting infinit loop
		
		if (!LIDAR_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		int motor_position, distance;
		while (LIDAR_controller.query_for_event(LIDAR_READ_EVENT, &motor_position, &distance)) {  // have we received the event from Serial ?
			
// delete old distance
			Point old_p;
			old_p.x = -lidar_distances[motor_position] / scale_factor * sin(motor_position / 100.0 * M_PI - M_PI / 2);
			old_p.y = -lidar_distances[motor_position] / scale_factor * cos(motor_position / 100.0 * M_PI - M_PI / 2);
			circle(image, center + old_p, 5.0, Scalar(0, 0, 0), 1, 8);

// draw the new point
			lidar_distances[motor_position] = distance;
			Point new_p;
			new_p.x = - distance / scale_factor * sin(motor_position / 100.0 * M_PI - M_PI / 2);
			new_p.y = - distance / scale_factor * cos(motor_position / 100.0 * M_PI - M_PI / 2);
			circle(image, center + new_p, 5.0, Scalar(0, 0, 255), 1, 8);
			cout << "Motor position = " << motor_position << " LIDAR distance = " << distance << endl;
		}

		imshow("LIDAR map", image);
		
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