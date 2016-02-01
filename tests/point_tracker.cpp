#include "point_tracker.h"

//-----------------------------------------------------------------------
tracking_data get_offset_angles(int webcam_model_number, double image_ratio, int image_width, int image_height, Point position)
{
	tracking_data deviation;

	if (webcam_model_number == 910)
	{
		if (image_ratio == 4 / 3.0)
		{
			deviation.grades_from_center_x = determine_offset_angle(position.x, C910_4_3_HORIZONTAL_FIELD_OF_VIEW, image_width);
			deviation.grades_from_center_y = determine_offset_angle(position.y, C910_4_3_VERTICAL_FIELD_OF_VIEW, image_height);
		}
		else {
			deviation.grades_from_center_x = determine_offset_angle(position.x, C910_16_9_HORIZONTAL_FIELD_OF_VIEW, image_width);
			deviation.grades_from_center_y = determine_offset_angle(position.y, C910_16_9_VERTICAL_FIELD_OF_VIEW, image_height);
		}
	}
	else if (webcam_model_number == 920)
	{
		if (fabs(image_ratio - 4 / 3.0) < 1e-6)
		{
			deviation.grades_from_center_x = determine_offset_angle(position.x, C920_4_3_HORIZONTAL_FIELD_OF_VIEW, image_width);
			deviation.grades_from_center_y = determine_offset_angle(position.y, C920_4_3_VERTICAL_FIELD_OF_VIEW, image_height);
		}
		else {
			deviation.grades_from_center_x = determine_offset_angle(position.x, C920_16_9_HORIZONTAL_FIELD_OF_VIEW, image_width);
			deviation.grades_from_center_y = determine_offset_angle(position.y, C920_16_9_VERTICAL_FIELD_OF_VIEW, image_height);
		}
	}

	return deviation;
}
//-----------------------------------------------------------------------
tracking_data get_offset_angles(int webcam_model_number, Point position)
{
	tracking_data deviation;

	if (webcam_model_number == 910)
	{
		deviation.grades_from_center_x = determine_offset_angle(position.x, C910_4_3_HORIZONTAL_FIELD_OF_VIEW, 640);
		deviation.grades_from_center_y = determine_offset_angle(position.y, C910_4_3_VERTICAL_FIELD_OF_VIEW, 480);
	}
	else
		if (webcam_model_number == 920)
		{
			deviation.grades_from_center_x = determine_offset_angle(position.x, C920_4_3_HORIZONTAL_FIELD_OF_VIEW, 640);
			deviation.grades_from_center_y = determine_offset_angle(position.y, C920_4_3_VERTICAL_FIELD_OF_VIEW, 480);
		}

	return deviation;
}
//-----------------------------------------------------------------------
double determine_offset_angle(int position, double field_of_view, int number_of_pixels)
{
	double pixel_one_percent = (double)number_of_pixels / 100.0;
	double fov_one_percent = (double)field_of_view / 100.0;
	double offset_from_center = (position - number_of_pixels / 2.0) / pixel_one_percent;

	return offset_from_center * fov_one_percent;
}
//-----------------------------------------------------------------------