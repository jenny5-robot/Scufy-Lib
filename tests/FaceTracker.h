#ifndef face_trackerH
#define face_trackerH


#include "opencv2/video/tracking.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <limits>
#include <ctype.h>
#include <stdio.h>
#include <vector>

using namespace std;
//using namespace cv;

class t_face_tracker {
private:
	/** Classifiers **/
	cv::String face_cascade_name = "classifiers\\lbpcascade_frontalface_visionary.xml",
		nose_cascade_name = "classifiers\\haarcascade_mcs_nose.xml",
		mouth_cascade_name = "classifiers\\haarcascade_mcs_mouth.xml",
		eyeglasses_cascade_name = "classifiers\\haarcascade_eye_tree_eyeglasses.xml",
		profile_cascade_name = "classifiers\\haarcascade_profileface.xml",
		hand_cascade_name = "classifiers\\cascade_hand.xml";
	cv::CascadeClassifier face_cascade, eyes_cascade, profile_cascade, nose_cascade, mouth_cascade, hand_cascade;

	////Features params
	//int maxCorners, blockSize;
	//double qualityLevel, minDistance;
	//// LK params
	//Size winSize;
	//int maxLevel;
	//TermCriteria criteria;
	//vector<Point2f> corners[2];

	/** Coordinates of the face to be tracked **/
	cv::Point biggest_face_origin = cv::Point(0, 0);
	double biggest_face_height = 0.0, biggest_face_width = 0.0, distance_between_faces = 10.0;
	/** Coordinates of the eyes **/
	cv::Point left_eye_origin = cv::Point(0, 0), right_eye_origin = cv::Point(0, 0);
	double eyes_height = 0.0, eyes_width = 0.0;

	/** Coordinates of the nose**/
	cv::Point nose_origin = cv::Point(0, 0);
	double nose_height = 0.0, nose_width = 0.0;
	/** Coordinates of the mouth**/
	cv::Point mouth_origin = cv::Point(0, 0);
	double mouth_height = 0.0, mouth_width = 0.0;

	/** Coordinates of the right/left corners in which to detect the hand signaling a change of distance **/
	cv::Mat right_corner, left_corner;


	//void track_features(Mat&, Mat&, vector<Point2f>[]);
	void detect_eyes(cv::Mat&, vector<cv::Rect_<int> >&);
	void detect_nose(cv::Mat&, vector<cv::Rect_<int> >&);
	void detect_mouth(cv::Mat&, vector<cv::Rect_<int> >&);
	//void detect_hand_in_upper_corners(Mat);

public:
	cv::String window_name;

	t_face_tracker();
	~t_face_tracker();
	
	bool detect_faces_and_display(cv::Mat);
	
	cv::Point get_biggest_face_origin();
	void set_biggest_face_X(int);
	void set_biggest_face_Y(int);
	
	bool right_hand_detected(cv::Mat);
	bool left_hand_detected(cv::Mat);
};


#endif