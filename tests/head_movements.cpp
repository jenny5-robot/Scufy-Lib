#include <iostream>

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

#define COMMAND_NOT_SENT 0
#define COMMAND_SENT 1
#define COMMAND_DONE 2

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
	CascadeClassifier face_reco; // create cascade for face reco
	t_jenny5_command_module head_controller;

	if (!head_controller.connect(10, 115200)) {
		printf("Error attaching to Jenny 5' head!\n");
		getchar();
		return 1;
	}
	Sleep(2000);

	// empty the serial buffer
	char sir[1000];
	int num_read = head_controller.clear_data_from_serial(sir, 1000);
	sir[num_read] = 0;
	printf("Serial buffer = %s\n", sir);

	head_controller.send_set_motor_speed_and_acceleration(MOTOR_HEAD_HORIZONTAL, 50, 50);

	face_reco.load("haarcascade_frontalface_alt.xml"); // loading haarcascade library

	VideoCapture cam;		// setup video capturing device (a.k.a webcam)
	cam.open(0);			// link it to the device [0 = default cam] (USBcam is default 'cause I disabled the onbord one IRRELEVANT!)
	if (!cam.isOpened())	// check if we succeeded
	{
		cout << "Couldn't open the video cam!!" << endl;
		waitKey(1);
		head_controller.close_connection();
		return 2;
	}

	Mat frame; // images used in the proces
	Mat grayFrame;

	namedWindow("display", WINDOW_AUTOSIZE); // window to display the results
	cam >> frame;
	printf("capture size:%d %d\n", frame.cols, frame.rows);

	bool active = true;

	int motor_state = COMMAND_DONE; // COMMAND_NOT_SENT;

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
			if (motor_state == COMMAND_DONE) {
				// send a command to the module so that the face is in the center of the image
				if (center.x < frame.cols / 2) {
					head_controller.send_move_motor(MOTOR_HEAD_HORIZONTAL, 5);
					printf("M1 5# - sent\n");
				}
				else {
					head_controller.send_move_motor(MOTOR_HEAD_HORIZONTAL, -5);
					printf("M1 -5# - sent\n");
				}
				motor_state = COMMAND_SENT;
			}
		}
		if (motor_state == COMMAND_SENT) {// if a command has been sent
			// now wait for the motor to complete the movement
			if (head_controller.query_for_event(MOTOR_DONE_EVENT, MOTOR_HEAD_HORIZONTAL)) { // have we received the event from Serial ?
				motor_state = COMMAND_DONE;
				printf("M1# - done\n");
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