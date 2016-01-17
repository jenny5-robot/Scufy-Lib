#include <iostream>
#include <time.h>

#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

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

typedef struct _CENTER_POINT
{
	int x;
	int y;
	int range;
}CENTER_POINT, *PCENTER_POINT;

#define MOTOR_HEAD_HORIZONTAL 1
#define MOTOR_HEAD_VERTICAL 0

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3

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
int	main(int argc, const char** argv)
{
	//-------------- START INITIALIZATION ------------------------------

	CascadeClassifier face_reco; // create cascade for face reco
	t_jenny5_command_module head_controller;

	if (!head_controller.connect(10, 115200)) {
		printf("Error attaching to Jenny 5' head!\n");
		getchar();
		return 1;
	}
	// now wait to see if I was connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();

	while (1) {
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (head_controller.query_for_event(IS_ALIVE_EVENT, 0)) { // have we received the event from Serial ?
			printf("Connected to head.\n");
			break;
		}
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			printf("Head does not respond! Game over!\n");
			getchar();
			return 2;
			break;
		}
	}

	if (!face_reco.load("c:\\robots\\opencv\\sources\\data\\haarcascades\\haarcascade_frontalface_alt.xml")) { // loading haarcascade library
		printf("Cannot load haarcascade! Please place the file in the correct folder!\n");
		getchar();
		return 3;
	}
	VideoCapture cam;		// setup video capturing device (a.k.a webcam)
	cam.open(0);			// link it to the device [0 = default cam] (USBcam is default 'cause I disabled the onbord one IRRELEVANT!)
	if (!cam.isOpened())	// check if we succeeded
	{
		cout << "Couldn't open the video cam!!" << endl;
		waitKey(1);
		head_controller.close_connection();
		return 4;
	}

	//-------------- INITIALIZATION OVER ------------------------------

	Mat frame; // images used in the proces
	Mat grayFrame;

	namedWindow("display", WINDOW_AUTOSIZE); // window to display the results
	cam >> frame;
	printf("Capture size:%d %d\n", frame.cols, frame.rows);

	bool active = true;

	head_controller.send_set_motor_speed_and_acceleration(MOTOR_HEAD_HORIZONTAL, 50, 50);
	head_controller.send_set_motor_speed_and_acceleration(MOTOR_HEAD_VERTICAL, 50, 50);

	while (active)        // starting infinit loop
	{
		if (!head_controller.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		cam >> frame; // put captured-image frame in frame

		cvtColor(frame, grayFrame, CV_BGR2GRAY); // convert to gray and equalize
		equalizeHist(grayFrame, grayFrame);

		std::vector<Rect> faces;// create an array to store the faces found

		// find and store the faces
		face_reco.detectMultiScale(grayFrame, faces, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, Size(30, 30));

		CENTER_POINT center;

		bool face_found = biggest_face(faces, center);

		if (face_found) {
			Point p1(center.x - center.range, center.y - center.range);
			Point p2(center.x + center.range, center.y + center.range);
			// draw an outline for the faces
			rectangle(frame, p1, p2, cvScalar(0, 255, 0, 0), 1, 8, 0);
		}

		imshow("display", frame); // display the result

		if (face_found) {// 
			// horizontal movement motor
			if (head_controller.get_motor_state(MOTOR_HEAD_HORIZONTAL) == COMMAND_DONE) {
				// send a command to the module so that the face is in the center of the image
				if (center.x < frame.cols / 2) {
					head_controller.send_move_motor(MOTOR_HEAD_HORIZONTAL, 1);
					printf("M1 5# - sent\n");
				}
				else {
					head_controller.send_move_motor(MOTOR_HEAD_HORIZONTAL, -1);
					printf("M1 -5# - sent\n");
				}
				head_controller.set_motor_state(MOTOR_HEAD_HORIZONTAL, COMMAND_SENT);
			}

			// vertical movement motor
			if (head_controller.get_motor_state(MOTOR_HEAD_VERTICAL) == COMMAND_DONE) {
				// send a command to the module so that the face is in the center of the image
				if (center.y < frame.rows / 2) {
					head_controller.send_move_motor(MOTOR_HEAD_VERTICAL, 1);
					printf("M0 1# - sent\n");
				}
				else {
					head_controller.send_move_motor(MOTOR_HEAD_VERTICAL, -1);
					printf("M0 -1# - sent\n");
				}
				head_controller.set_motor_state(MOTOR_HEAD_VERTICAL, COMMAND_SENT);
			}
		}

		// now check if the the horizontal motor has completed the movement
		if (head_controller.get_motor_state(MOTOR_HEAD_HORIZONTAL) == COMMAND_SENT) {// if a command has been sent

			if (head_controller.query_for_event(MOTOR_DONE_EVENT, MOTOR_HEAD_HORIZONTAL)) { // have we received the event from Serial ?
				head_controller.set_motor_state(MOTOR_HEAD_HORIZONTAL, COMMAND_DONE);
				printf("M1# - done\n");
			}
			else {
				// motor is still running and we can supervise it (with a camera?)
			}
		}

		// now check if the the vertical motor has completed the movement
		if (head_controller.get_motor_state(MOTOR_HEAD_VERTICAL) == COMMAND_SENT) {// if a command has been sent

			if (head_controller.query_for_event(MOTOR_DONE_EVENT, MOTOR_HEAD_VERTICAL)) { // have we received the event from Serial ?
				head_controller.set_motor_state(MOTOR_HEAD_VERTICAL, COMMAND_DONE);
				printf("M0# - done\n");
			}
			else {
				// motor is still running and we can supervise it (with a camera?)
			}
		}
		if (waitKey(1) >= 0)  // break the loop
			active = false;
	}

	head_controller.close_connection();
	return 0;
}
//----------------------------------------------------------------