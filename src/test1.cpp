// jenny5_test1.cpp : Defines the entry point for the console application.
//


#include <stdio.h>
#include "jenny5_command_module.h"


int main(int argc, char* argv[])
{
	// attach to the robot
	t_jenny5_command_module right_arm_connection;

	if (!right_arm_connection.connect(9, 9600)) {
		printf("Error attaching to Jenny 5' right arm!\n");
		getchar();
		return 0;   
	}
	Sleep(2000);

	// empty the serial buffer
	char sir[1000];
	int num_read = right_arm_connection.get_data(sir, 1000);
	sir[num_read] = 0;
	printf("Serial buffer = %s\n", sir);
	
	// send a command to the module
	right_arm_connection.move_motor(0, 10);
	right_arm_connection.disable_motor(0);

	Sleep(50);

	right_arm_connection.close_connection();

	printf("program over ... press Enter ...");
	getchar();

	return 0;
}

