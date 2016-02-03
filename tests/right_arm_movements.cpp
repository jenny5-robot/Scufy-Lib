#include "../include/arm_controller.h"
#include "../include/jenny5_events.h"

//----------------------------------------------------------------
int main(void) {
	t_arm_controller arm_controller;

	arm_controller.lift_shoulder(-400, NO_WAIT);
	arm_controller.lift_elbow(200, NO_WAIT);
	arm_controller.rotate_upper_arm(150, ELBOW_LIFT);
	arm_controller.rotate_upper_arm(-300, UPPER_ARM_ROTATE);
	arm_controller.rotate_upper_arm(300, UPPER_ARM_ROTATE);

	return 0;
}
