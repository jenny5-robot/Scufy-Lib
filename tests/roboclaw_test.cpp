// copyright Mihai Oltean
// www.jenny5.org
// www.tcreate.org
// https://github.com/jenny5-robot
// mihai.oltean@gmail.com

// MIT License

#include "roboclaw_controller.h"

//--------------------------------------------------------------
int main(void)
{
	t_roboclaw_controller roboclaw;

	// connect
	if (!roboclaw.connect(19, 38400)) { // real - 1
		printf("Cannot connect! Game over.");
		getchar();
		return 1;
	}

	char buffer[100];
	buffer[0] = 0;
	int buffer_length = 100;

	// firmware version
	roboclaw.get_firmware_version(buffer);
	printf("Firmware version = %s\n", buffer);

	// temperature
	Sleep(10);
	double temperature = roboclaw.get_temperature();
	Sleep(10);
	printf("Temperature = %lf\n", temperature);

	// main battery voltage
	Sleep(10);
	double main_battery_voltage = roboclaw.get_main_battery_voltage();
	printf("Battery voltage = %lf\n", main_battery_voltage);

	// move motor 2 forward
	Sleep(10);
	//roboclaw.drive_forward_M1(10);

	roboclaw.drive_M1_with_signed_duty_and_acceleration(-3000, 1);
	Sleep(5);
	// read the result
	roboclaw.read_result((unsigned char*)buffer, 100);
	
	while (1) {
		if (GetAsyncKeyState(VK_ESCAPE))
			break;
		double current_motor_1, current_motor_2;
		roboclaw.read_motor_currents(current_motor_1, current_motor_2);
		printf("current_motor_1 = %lf; current_motor_2 = %lf\n", current_motor_1, current_motor_2);
		Sleep(50);
	}

	// stop the motor
	roboclaw.drive_forward_M1(0);

	printf("Program over. Press Enter.");
	getchar();

	return 0;
}
//--------------------------------------------------------------