#include "../include/arm_controller.h"

//----------------------------------------------------------------
int main(void) {
	t_arm_controller arm_controller;

	arm_controller.spin_shoulder(200);
	Sleep(10000);

	return 0;
}
