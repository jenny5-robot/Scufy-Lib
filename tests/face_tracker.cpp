#include "../include/face_tracker.h"

t_face_tracker::t_face_tracker() {
	if (!face_cascade.load(face_cascade_name)) {
		printf("--(!)Error loading face cascade\n");
		return;
	}
	else {
		printf("Loaded face cascade\n");
	}
	if (!eyes_cascade.load(eyeglasses_cascade_name)) {
		printf("--(!)Error loading eyes cascade\n");
		return;
	}
	else {
		printf("Loaded eyes cascade\n");
	}
	if (!nose_cascade.load(nose_cascade_name)) {
		printf("--(!)Error loading nose cascade\n");
		return;
	}
	else {
		printf("Loaded nose cascade\n");
	}
	if (!mouth_cascade.load(mouth_cascade_name)) {
		printf("--(!)Error loading mouth cascade\n");
		return;
	}
	else {
		printf("Loaded mouth cascade\n");
	}
	if (!profile_cascade.load(profile_cascade_name)) {
		printf("--(!)Error loading profile cascade\n");
		return;
	}
	else {
		printf("Loaded profile cascade\n");
	}
	if (!hand_cascade.load(hand_cascade_name)) {
		printf("--(!)Error loading hand cascade\n");
		return;
	}
	else {
		printf("Loaded hand cascade\n");
	}

	window_name = "Person detection";
	//Set the values for the LK params
	maxCorners = 3000;
	blockSize = 3;
	qualityLevel = 0.5;
	minDistance = 3;
	winSize = Size(10, 10);
	maxLevel = 5;
	criteria = TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 0.03);
}

t_face_tracker::~t_face_tracker() {
}

bool t_face_tracker::track_face() {
	VideoCapture capture;
	Mat frame;

	//Read the video stream
	capture.open(0);
	if (!capture.isOpened()) {
		printf("--(!)Error opening video capture\n");
		return false;
	}

	//Analyse every 10th frame
	int frame_number = 0;
	capture.read(frame);
	detect_and_display_faces(frame);
	detect_hand_in_upper_corners(frame);

	while (capture.read(frame)) {
		if (frame.empty()) {
			printf(" --(!) No captured frame -- Break!");
			return false;
		}

		//Apply the classifier to the frame
		if (frame_number == 10) {
			detect_and_display_faces(frame);
			detect_hand_in_upper_corners(frame);
			frame_number = 0;
		}
		else frame_number++;

		int c = waitKey(10);
		if ((char)c == 27) {
			break;
		} //Escape
	}
	return true;
}


/// <summary>
/// Detects a hand in one of the upper corners in the specified frame.
/// If the hand is in the upper right corner => reduce distance to the person being tracked
/// If the hand is in the upper left corner => increase distance to the person being tracked
/// </summary>
/// <param name="frame">The frame.</param>
void t_face_tracker::detect_hand_in_upper_corners(Mat frame) {
	//Set the corners for hand detection and mark them
	left_corner = frame(Rect(0, 0, frame.size().width / 4, frame.size().height / 2));
	right_corner = frame(Rect(frame.size().width / 2 + frame.size().width / 4, 0, frame.size().width / 4, frame.size().height / 2));
	rectangle(frame, Point(0, 0), Point(frame.size().width / 4, frame.size().height / 2), Scalar(200, 0, 0), 8);
	rectangle(frame, Point(frame.size().width / 2 + frame.size().width / 4, 0), Point(frame.size().width, frame.size().height / 2), Scalar(200, 0, 0), 8);

	cvtColor(right_corner, right_corner, COLOR_BGR2GRAY);
	equalizeHist(right_corner, right_corner);
	cvtColor(left_corner, left_corner, COLOR_BGR2GRAY);
	equalizeHist(left_corner, left_corner);

	vector<Rect> hands;

	//-- Detect hands with the classifier
	hand_cascade.detectMultiScale(right_corner, hands, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));
	if (hands.size() > 0) {
		int i = 0;
		rectangle(right_corner, Point(hands[i].x, hands[i].y), Point(hands[i].x + hands[i].width, hands[i].y + hands[i].height), Scalar(0, 200, 100), 2);
		double x = frame.size().width / 4 + frame.size().width / 2 + hands[i].x;
		rectangle(frame, Point(x, hands[i].y), Point(x + hands[i].width, hands[i].y + hands[i].height), Scalar(0, 200, 200), 2);
	}
	hand_cascade.detectMultiScale(left_corner, hands, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));
	if (hands.size() > 0) {
		int i = 0;
		rectangle(left_corner, Point(hands[i].x, hands[i].y), Point(hands[i].x + hands[i].width, hands[i].y + hands[i].height), Scalar(0, 200, 100), 2);
		rectangle(frame, Point(hands[i].x, hands[i].y), Point(hands[i].x + hands[i].width, hands[i].y + hands[i].height), Scalar(200, 200, 200), 2);
	}

	imshow(window_name, frame);
}

bool t_face_tracker::left_hand_detected(Mat frame) {
	//Set the corners for hand detection and mark them
	left_corner = frame(Rect(0, 0, frame.size().width / 4, frame.size().height / 2));
	rectangle(frame, Point(0, 0), Point(frame.size().width / 4, frame.size().height / 2), Scalar(200, 0, 0), 8);

	cvtColor(left_corner, left_corner, COLOR_BGR2GRAY);
	equalizeHist(left_corner, left_corner);

	vector<Rect> hands;

	//Detect hands with the classifier
	hand_cascade.detectMultiScale(left_corner, hands, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));
	if (hands.size() > 0) {
		int i = 0;
		rectangle(left_corner, Point(hands[i].x, hands[i].y), Point(hands[i].x + hands[i].width, hands[i].y + hands[i].height), Scalar(0, 200, 100), 2);
		rectangle(frame, Point(hands[i].x, hands[i].y), Point(hands[i].x + hands[i].width, hands[i].y + hands[i].height), Scalar(200, 200, 200), 2);
		imshow(window_name, frame);
		return true;
	}
	return false;
}

bool t_face_tracker::right_hand_detected(Mat frame) {
	right_corner = frame(Rect(frame.size().width / 2 + frame.size().width / 4, 0, frame.size().width / 4, frame.size().height / 2));
	rectangle(frame, Point(frame.size().width / 2 + frame.size().width / 4, 0), Point(frame.size().width, frame.size().height / 2), Scalar(200, 0, 0), 8);

	cvtColor(right_corner, right_corner, COLOR_BGR2GRAY);
	equalizeHist(right_corner, right_corner);

	vector<Rect> hands;

	hand_cascade.detectMultiScale(right_corner, hands, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));
	if (hands.size() > 0) {
		int i = 0;
		rectangle(right_corner, Point(hands[i].x, hands[i].y), Point(hands[i].x + hands[i].width, hands[i].y + hands[i].height), Scalar(0, 200, 100), 2);
		double x = frame.size().width / 4 + frame.size().width / 2 + hands[i].x;
		rectangle(frame, Point(x, hands[i].y), Point(x + hands[i].width, hands[i].y + hands[i].height), Scalar(0, 200, 200), 2);
		imshow(window_name, frame);
		return true;
	}
	return false;
}

/// <summary>
/// Detect faces (then eyes, nose, mouth) and displays them in the specified frame.
/// </summary>
/// <param name="frame">The frame.</param>
std::vector<Rect> t_face_tracker::detect_and_display_faces(Mat frame) {
	vector<Rect> faces, eyes, noses, mouths, profiles;
	Mat frame_gray;

	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//-- Detect faces and features with the classifiers
	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));
	if (faces.size() > 0) {
		//The index of the first and only face detected
		int i = 0;
		Mat ROI = frame(Rect(faces[i].x, faces[i].y, faces[i].width, faces[i].height));

		Point face_origin = Point(faces[i].x, faces[i].y);
		double distance = norm(biggest_face_origin - face_origin);
		//Check if the face moved more than a certain number of pixels
		if (abs(distance) > distance_between_faces) {
			biggest_face_origin = face_origin;
			biggest_face_height = faces[i].height;
			biggest_face_width = faces[i].width;

			//Mark the face in the frame
			Point upper_left = Point(faces[i].x, faces[i].y);
			Point lower_right = Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
			rectangle(frame, upper_left, lower_right, Scalar(100, 0, 100), 4);

			//Detect and mark the eyes
			Point eye_center;
			detect_eyes(ROI, eyes);
			for (size_t j = 0; j < eyes.size(); j++) {
				eye_center = Point2f(eyes[j].x + eyes[j].width / 2, eyes[j].y + eyes[j].height / 2);
				rectangle(ROI, Point(eyes[j].x, eyes[j].y), Point(eyes[j].x + eyes[j].width, eyes[j].y + eyes[j].height), Scalar(255, 0, 0), 1, 4);
				//Set the features detected to be tracked with LK
				corners[0].push_back(eye_center);
			}
			if (eyes.size() == 2) {
				right_eye_origin = Point(eyes[0].x, eyes[0].y);
				left_eye_origin = Point(eyes[1].x, eyes[1].y);
				eyes_height = eyes[0].height;
				eyes_width = eyes[0].width;
			}
			else if (eyes.size() == 1) {
				//Check which eye was detected and determine the other eye
				if (eyes[0].x > (faces[i].x + faces[i].width / 2)) {
					left_eye_origin = Point(eyes[0].x, eyes[0].y);
					right_eye_origin = Point(faces[i].width / 2 - eyes[0].width - eyes[0].x, eyes[0].y);
				}
				else {
					right_eye_origin = Point(eyes[0].x, eyes[0].y);
					double distance_between_nose_and_eye = faces[i].width / 2 - (eyes[0].x - faces[i].x) - eyes[0].width;
					left_eye_origin = Point(faces[i].width / 2 + distance_between_nose_and_eye, eyes[0].y);
				}
				eyes_height = eyes[0].height;
				eyes_width = eyes[0].width;
			}

			////Detect and mark the nose
			//double nose_center_height = 0.0, nose_center_width = 0.0;
			//Point nose_top, nose_left, nose_right;
			//detect_nose(ROI, noses);
			//if (noses.size() > 0) {
			//	//The index of the only nose detected in the face
			//	int j = 0;
			//	//Set the most recent detected object
			//	nose_origin = Point(noses[0].x, noses[0].y);
			//	nose_height = noses[0].height;
			//	nose_width = noses[0].width;

			//	rectangle(ROI, Point(noses[j].x, noses[j].y), Point(noses[j].x + noses[j].width, noses[j].y + noses[j].height), Scalar(255, 0, 0), 1, 4);
			//	nose_center_height = (noses[j].y + noses[j].height / 2);
			//	nose_center_width = (noses[j].x + noses[j].width / 2);
			//	nose_top = Point(noses[j].x + noses[j].width / 2, noses[j].y);
			//	nose_left = Point(noses[j].x + noses[j].width, noses[j].y + noses[j].height / 2);
			//	nose_right = Point(noses[j].x, noses[j].y + noses[j].height / 2);

			//	//Set the features detected to be tracked with LK
			//	corners[0].push_back(nose_top);
			//	corners[0].push_back(nose_left);
			//	corners[0].push_back(nose_right);
			//	corners[0].push_back(Point2f(nose_center_width, nose_center_height));
			//}

			////Detect and mark the mouth
			//double mouth_center_height = 0.0, mouth_center_width = 0.0;
			//detect_mouth(ROI, mouths);
			//if (mouths.size() > 0) {
			//	//The index of the only mouth detected in the face
			//	int j = 0;

			//	//Set the most recent detected object
			//	mouth_origin = Point(mouths[0].x, mouths[0].y);
			//	mouth_height = mouths[0].height;
			//	mouth_width = mouths[0].width;

			//	mouth_center_height = (mouths[j].y + mouths[j].height / 2);
			//	mouth_center_width = (mouths[j].x + mouths[j].width / 2);

			//	//Set the features detected to be tracked with LK
			//	corners[0].push_back(Point2f(mouth_center_width, mouth_center_height));
			//	// The mouth should lie below the nose
			//	if (mouth_center_height > nose_center_height) {
			//		rectangle(ROI, Point(mouths[j].x, mouths[j].y), Point(mouths[j].x + mouths[j].width, mouths[j].y + mouths[j].height), Scalar(255, 0, 0), 1, 4);
			//	}
			//	else if (mouth_center_height <= nose_center_height) {
			//		//Do nothing
			//	}
			//	else
			//		rectangle(ROI, Point(mouths[j].x, mouths[j].y), Point(mouths[j].x + mouths[j].width, mouths[j].y + mouths[j].height), Scalar(255, 0, 0), 1, 4);
			//}
		}
		else {
			//Mark the face in the frame
			rectangle(frame, biggest_face_origin, Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height), Scalar(100, 0, 10), 4);
			rectangle(ROI, left_eye_origin, Point(left_eye_origin.x + eyes_width, left_eye_origin.y + eyes_height), Scalar(200, 0, 0), 1);
			rectangle(ROI, right_eye_origin, Point(right_eye_origin.x + eyes_width, right_eye_origin.y + eyes_height), Scalar(200, 0, 0), 1);
			/*rectangle(ROI, nose_origin, Point(nose_origin.x + nose_width, nose_origin.y + nose_height), Scalar(200, 0, 0), 1);
			rectangle(ROI, mouth_origin, Point(mouth_origin.x + mouth_width, mouth_origin.y + mouth_height), Scalar(200, 0, 0), 1);*/
		}

		//Set the features to track with LK
		Mat frame_ROI = frame_gray(faces[i]);
		track_features(ROI, frame_ROI, corners);
	}
	else {
		//-- Detect profile faces
		profile_cascade.detectMultiScale(frame_gray, profiles, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));

		if (profiles.size() > 0) {
			//The index of the first and biggest profile face detected
			int j = 0;

			Point face_origin = Point(profiles[j].x, profiles[j].y);
			double distance = norm(biggest_face_origin - face_origin);
			//Check if the face moved more than a certain number of pixels
			if (abs(distance) > distance_between_faces) {
				biggest_face_origin = face_origin;
				biggest_face_height = profiles[j].height;
				biggest_face_width = profiles[j].width;

				Point profile_top_left = Point(profiles[j].x, profiles[j].y);
				Point profile_lower_right = Point(profiles[j].x + profiles[j].width, profiles[j].y + profiles[j].height);
				rectangle(frame, profile_top_left, profile_lower_right, Scalar(100, 100, 0), 4);

				//Set the features detected to be tracked with LK
				corners[0].push_back(profile_top_left);
				corners[0].push_back(profile_lower_right);
				corners[0].push_back(Point2f(profiles[j].x + profiles[j].width / 2, profiles[j].y + profiles[j].height / 2));
			}
			else {
				//Mark the face in the frame
				Point lower_right = Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height);
				rectangle(frame, biggest_face_origin, lower_right, Scalar(100, 10, 0), 4);
			}
			//Set the features to track with LK
			Mat frame_ROI = frame_gray(profiles[j]);
			//Mat ROI = frame(Rect(profiles[j].x, profiles[j].y, profiles[j].width, profiles[j].height));
			//track_features(ROI, frame_ROI, corners);
		}
		else {
			Mat flipped_frame;
			//Flip the image to try and find a profile face
			flip(frame_gray, flipped_frame, 1);
			//-- Detect profile faces
			profile_cascade.detectMultiScale(flipped_frame, profiles, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));
			if (profiles.size() > 0) {
				//The index of the first and biggest profile face detected
				int j = 0;
				//Get the profile face origin coordinates from the mirrored image
				double x = frame.size().width / 2 + profiles[0].x;
				double y = frame.size().height / 2 + profiles[0].y;
				Point face_origin = Point(x, y);
				double distance = norm(biggest_face_origin - face_origin);
				//Check if the face moved more than a certain number of pixels
				if (abs(distance) > distance_between_faces) {
					biggest_face_origin = face_origin;
					biggest_face_height = profiles[j].height;
					biggest_face_width = profiles[j].width;

					Point profile_top_left = Point(x, y);
					Point profile_lower_right = Point(x + profiles[j].width, y + profiles[j].height);
					rectangle(frame, profile_top_left, profile_lower_right, Scalar(100, 100, 0), 4);

					//Set the features detected to be tracked with LK
					corners[0].push_back(profile_top_left);
					corners[0].push_back(profile_lower_right);
					corners[0].push_back(Point2f(x + profiles[j].width / 2, y + profiles[j].height / 2));
				}
				else {
					//Mark the face in the frame
					Point lower_right = Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height);
					rectangle(frame, biggest_face_origin, lower_right, Scalar(100, 10, 0), 4);
				}
				//Set the features to track with LK
				Mat frame_ROI = frame_gray(profiles[j]);
				//Mat ROI = frame(Rect(x, y, profiles[j].width, profiles[j].height));
				//track_features(ROI, frame_ROI, corners);
			}
		}
	}
	//-- Show what you got
	imshow(window_name, frame);

	if (faces.size() > 0)
		return faces;
	else
		return profiles;
}

void t_face_tracker::detect_faces_and_display(Mat frame) {
	vector<Rect> faces, eyes, noses, mouths, profiles;
	Mat frame_gray;

	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//Detect faces and features with the classifiers
	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));
	if (faces.size() > 0) {
		//The index of the first and only face detected
		int i = 0;
		Mat ROI = frame(Rect(faces[i].x, faces[i].y, faces[i].width, faces[i].height));

		//check if face characteristics can be found, to be sure it is a face
		detect_eyes(ROI, eyes);
		detect_nose(ROI, noses);
		detect_mouth(ROI, mouths);
		if (eyes.size() > 0 && mouths.size() > 0 && noses.size() > 0) {
			//it is definitely a face, mark all the features found

			Point face_origin = Point(faces[i].x, faces[i].y);
			double distance = norm(biggest_face_origin - face_origin);
			//Check if the face moved more than a certain number of pixels
			if (abs(distance) > distance_between_faces) {
				biggest_face_origin = face_origin;
				biggest_face_height = faces[i].height;
				biggest_face_width = faces[i].width;

				//Mark the face in the frame
				Point upper_left = Point(faces[i].x, faces[i].y);
				Point lower_right = Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
				rectangle(frame, upper_left, lower_right, Scalar(100, 0, 100), 4);

				//Detect and mark the eyes
				Point eye_center;
				for (size_t j = 0; j < eyes.size(); j++) {
					eye_center = Point2f(eyes[j].x + eyes[j].width / 2, eyes[j].y + eyes[j].height / 2);
					rectangle(ROI, Point(eyes[j].x, eyes[j].y), Point(eyes[j].x + eyes[j].width, eyes[j].y + eyes[j].height), Scalar(255, 0, 0), 1, 4);
					////Set the features detected to be tracked with LK
					//corners[0].push_back(eye_center);
				}
				if (eyes.size() == 2) {
					right_eye_origin = Point(eyes[0].x, eyes[0].y);
					left_eye_origin = Point(eyes[1].x, eyes[1].y);
					eyes_height = eyes[0].height;
					eyes_width = eyes[0].width;
				}
				else if (eyes.size() == 1) {
					//Check which eye was detected and determine the other eye
					if (eyes[0].x > (faces[i].x + faces[i].width / 2)) {
						left_eye_origin = Point(eyes[0].x, eyes[0].y);
						right_eye_origin = Point(faces[i].width / 2 - eyes[0].width - eyes[0].x, eyes[0].y);
					}
					else {
						right_eye_origin = Point(eyes[0].x, eyes[0].y);
						double distance_between_nose_and_eye = faces[i].width / 2 - (eyes[0].x - faces[i].x) - eyes[0].width;
						left_eye_origin = Point(faces[i].width / 2 + distance_between_nose_and_eye, eyes[0].y);
					}
					eyes_height = eyes[0].height;
					eyes_width = eyes[0].width;
				}

				//Detect and mark the nose
				double nose_center_height = 0.0, nose_center_width = 0.0;
				Point nose_top, nose_left, nose_right;
				if (noses.size() > 0) {
					//The index of the only nose detected in the face
					int j = 0;
					//Set the most recent detected object
					nose_origin = Point(noses[0].x, noses[0].y);
					nose_height = noses[0].height;
					nose_width = noses[0].width;
					rectangle(ROI, Point(noses[j].x, noses[j].y), Point(noses[j].x + noses[j].width, noses[j].y + noses[j].height), Scalar(255, 0, 0), 1, 4);
					nose_center_height = (noses[j].y + noses[j].height / 2);
					nose_center_width = (noses[j].x + noses[j].width / 2);
					nose_top = Point(noses[j].x + noses[j].width / 2, noses[j].y);
					nose_left = Point(noses[j].x + noses[j].width, noses[j].y + noses[j].height / 2);
					nose_right = Point(noses[j].x, noses[j].y + noses[j].height / 2);
					////Set the features detected to be tracked with LK
					//corners[0].push_back(nose_top);
					//corners[0].push_back(nose_left);
					//corners[0].push_back(nose_right);
					//corners[0].push_back(Point2f(nose_center_width, nose_center_height));
				}
				//Detect and mark the mouth
				double mouth_center_height = 0.0, mouth_center_width = 0.0;
				if (mouths.size() > 0) {
					//The index of the only mouth detected in the face
					int j = 0;
					//Set the most recent detected object
					mouth_origin = Point(mouths[0].x, mouths[0].y);
					mouth_height = mouths[0].height;
					mouth_width = mouths[0].width;
					mouth_center_height = (mouths[j].y + mouths[j].height / 2);
					mouth_center_width = (mouths[j].x + mouths[j].width / 2);
					////Set the features detected to be tracked with LK
					//corners[0].push_back(Point2f(mouth_center_width, mouth_center_height));
					// The mouth should lie below the nose
					if (mouth_center_height > nose_center_height) {
						rectangle(ROI, mouth_origin, Point(mouths[j].x + mouths[j].width, mouths[j].y + mouths[j].height), Scalar(255, 0, 0), 1, 4);
					}
					else if (mouth_center_height <= nose_center_height) {
						//Do nothing
					}
					else
						rectangle(ROI, mouth_origin, Point(mouths[j].x + mouths[j].width, mouths[j].y + mouths[j].height), Scalar(255, 0, 0), 1, 4);
				}
			}
			else {
				//Mark the face in the frame
				rectangle(frame, biggest_face_origin, Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height), Scalar(100, 0, 10), 4);
				rectangle(ROI, left_eye_origin, Point(left_eye_origin.x + eyes_width, left_eye_origin.y + eyes_height), Scalar(200, 0, 0), 1);
				rectangle(ROI, right_eye_origin, Point(right_eye_origin.x + eyes_width, right_eye_origin.y + eyes_height), Scalar(200, 0, 0), 1);
				rectangle(ROI, nose_origin, Point(nose_origin.x + nose_width, nose_origin.y + nose_height), Scalar(200, 0, 0), 1);
				rectangle(ROI, mouth_origin, Point(mouth_origin.x + mouth_width, mouth_origin.y + mouth_height), Scalar(200, 0, 0), 1);
			}
		}
		////Set the features to track with LK
		//Mat frame_ROI = frame_gray(faces[i]);
		//track_features(ROI, frame_ROI, corners);
	}
	else {
		//Detect profile faces
		profile_cascade.detectMultiScale(frame_gray, profiles, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));

		if (profiles.size() > 0) {
			//The index of the first and biggest profile face detected
			int j = 0;

			Point face_origin = Point(profiles[j].x, profiles[j].y);
			double distance = norm(biggest_face_origin - face_origin);
			//Check if the face moved more than a certain number of pixels
			if (abs(distance) > distance_between_faces) {
				biggest_face_origin = face_origin;
				biggest_face_height = profiles[j].height;
				biggest_face_width = profiles[j].width;

				Point profile_top_left = Point(profiles[j].x, profiles[j].y);
				Point profile_lower_right = Point(profiles[j].x + profiles[j].width, profiles[j].y + profiles[j].height);
				rectangle(frame, profile_top_left, profile_lower_right, Scalar(100, 100, 0), 4);

				////Set the features detected to be tracked with LK
				//corners[0].push_back(profile_top_left);
				//corners[0].push_back(profile_lower_right);
				//corners[0].push_back(Point2f(profiles[j].x + profiles[j].width / 2, profiles[j].y + profiles[j].height / 2));
			}
			else {
				//Mark the face in the frame
				Point lower_right = Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height);
				rectangle(frame, biggest_face_origin, lower_right, Scalar(100, 10, 0), 4);
			}
			////Set the features to track with LK
			//Mat frame_ROI = frame_gray(profiles[j]);
			//Mat ROI = frame(Rect(profiles[j].x, profiles[j].y, profiles[j].width, profiles[j].height));
			//track_features(ROI, frame_ROI, corners);
		}
		else {
			Mat flipped_frame;
			//Flip the image to try and find a profile face
			flip(frame_gray, flipped_frame, 1);
			//-- Detect profile faces
			profile_cascade.detectMultiScale(flipped_frame, profiles, 1.1, 4, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_SCALE_IMAGE, Size(30, 30));
			if (profiles.size() > 0) {
				//The index of the first and biggest profile face detected
				int j = 0;
				//Get the profile face origin coordinates from the mirrored image
				double x = frame.size().width / 2 + profiles[0].x;
				double y = frame.size().height / 2 + profiles[0].y;
				Point face_origin = Point(x, y);
				double distance = norm(biggest_face_origin - face_origin);
				//Check if the face moved more than a certain number of pixels
				if (abs(distance) > distance_between_faces) {
					biggest_face_origin = face_origin;
					biggest_face_height = profiles[j].height;
					biggest_face_width = profiles[j].width;

					Point profile_top_left = Point(x, y);
					Point profile_lower_right = Point(x + profiles[j].width, y + profiles[j].height);
					rectangle(frame, profile_top_left, profile_lower_right, Scalar(100, 100, 0), 4);

					////Set the features detected to be tracked with LK
					//corners[0].push_back(profile_top_left);
					//corners[0].push_back(profile_lower_right);
					//corners[0].push_back(Point2f(x + profiles[j].width / 2, y + profiles[j].height / 2));
				}
				else {
					//Mark the face in the frame
					Point lower_right = Point(biggest_face_origin.x + biggest_face_width, biggest_face_origin.y + biggest_face_height);
					rectangle(frame, biggest_face_origin, lower_right, Scalar(100, 10, 0), 4);
				}
				////Set the features to track with LK
				//Mat frame_ROI = frame_gray(profiles[j]);
				//Mat ROI = frame(Rect(x, y, profiles[j].width, profiles[j].height));
				//track_features(ROI, frame_ROI, corners);
			}
		}
	}
	//Show what you got
	imshow(window_name, frame);
}

/// <summary>
/// Track featureses in the specified image using the Lucas-Kanade algorithm.
/// </summary>
/// <param name="ROI">The image in which the.</param>
/// <param name="gray_ROI">The gray_ roi.</param>
/// <param name="corners">The corners.</param>
void t_face_tracker::track_features(Mat &ROI, Mat &gray_ROI, vector<Point2f> corners[]) {
	////Find features in the image to be tracked
	//goodFeaturesToTrack(gray_ROI, corners[0], maxCorners, qualityLevel, minDistance, noArray(), blockSize);
	////cornerSubPix(gray_ROI, corners[0], winSize, Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

	////Apply LK to the image, according to the features found
	//vector<uchar> status;
	//vector<float> err;
	//calcOpticalFlowPyrLK(gray_ROI, gray_ROI, corners[0], corners[1], status, err, winSize, maxLevel, criteria);
	//for (int i = 0; i < status.size(); i++) {
	//	if (status[i] > 0) {
	//		Point p0(corners[1][i].x, corners[1][i].y);
	//		//Mark the features on the image
	//		circle(ROI, p0, 3, Scalar(100, 200, 0), 4, 8);
	//	}
	//}
	//swap(corners[1], corners[0]);
	//corners[1].clear();
}

void t_face_tracker::detect_eyes(Mat& img, vector<Rect_<int> >& eyes) {
	eyes_cascade.detectMultiScale(img, eyes, 1.2, 3, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	return;
}

void t_face_tracker::detect_nose(Mat& img, vector<Rect_<int> >& nose) {
	nose_cascade.detectMultiScale(img, nose, 1.1, 4, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	return;
}

void t_face_tracker::detect_mouth(Mat& img, vector<Rect_<int> >& mouth) {
	mouth_cascade.detectMultiScale(img, mouth, 1.1, 4, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	return;
}

Point t_face_tracker::get_biggest_face_origin() {
	return biggest_face_origin;
}

void t_face_tracker::set_biggest_face_X(int displacement) {
	biggest_face_origin.x += displacement;
}

void t_face_tracker::set_biggest_face_Y(int displacement) {
	biggest_face_origin.y += displacement;
}