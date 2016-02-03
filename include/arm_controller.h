#ifndef arm_controllerH
#define arm_controllerH

#include "jenny5_command_module.h"
#include "jenny5_events.h"
#include <iostream>

//----------------------------------------------------------------
#define NO_WAIT -1
#define SHOULDER_SPIN 0
#define SHOULDER_LIFT 1
#define UPPER_ARM_ROTATE 2
#define ELBOW_LIFT 3
#define LOWER_ARM_ROTATE 1
//----------------------------------------------------------------

class t_arm_controller {
private:
	t_jenny5_command_module upper_motors_controller;
	t_jenny5_command_module lower_motors_controller;

public:
	t_arm_controller(void);
	~t_arm_controller(void);

	bool connect();
	bool setup();

	void spin_shoulder(int num_steps, int wait_for);
	void lift_shoulder(int num_steps, int wait_for);
	void rotate_upper_arm(int num_steps, int wait_for);
	void rotate_lower_arm(int num_steps, int wait_for);
	void lift_elbow(int num_steps, int wait_for);
	void wait_for_action(int wait_for);
};

#endif
