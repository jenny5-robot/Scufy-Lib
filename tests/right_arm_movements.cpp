#include "../include/arm_controller.h"

//----------------------------------------------------------------
int main(void) {
	t_arm_controller arm_controller;

	arm_controller.spin_shoulder(200);
	arm_controller.lift_shoulder(-200);

	return 0;
}
