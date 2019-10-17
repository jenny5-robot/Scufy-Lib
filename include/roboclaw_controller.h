// Author: Mihai Oltean, https://mihaioltean.github.io, mihai.oltean@gmail.com
// More details: https://jenny5.org, https://jenny5-robot.github.io/
// Source code: github.com/jenny5-robot
// License: MIT
// ---------------------------------------------------------------------------
 

// refer to 
// http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf
// for the list of parameters of the commands


#ifndef ROBOCLAW_CONTROLLER_H
#define ROBOCLAW_CONTROLLER_H

#include "c_serial.h"
#include <inttypes.h>
//-------------------------------------------------------------
enum {
	M1FORWARD = 0,
	M1BACKWARD = 1,
	SETMINMB = 2,
	SETMAXMB = 3,
	M2FORWARD = 4,
	M2BACKWARD = 5,
	M17BIT = 6,
	M27BIT = 7,
	MIXEDFORWARD = 8,
	MIXEDBACKWARD = 9,
	MIXEDRIGHT = 10,
	MIXEDLEFT = 11,
	MIXEDFB = 12,
	MIXEDLR = 13,
	GETM1ENC = 16,
	GETM2ENC = 17,
	GETM1SPEED = 18,
	GETM2SPEED = 19,
	RESETENC = 20,
	GETVERSION = 21,
	SETM1ENCCOUNT = 22,
	SETM2ENCCOUNT = 23,
	GETMBATT = 24,
	GETLBATT = 25,
	SETMINLB = 26,
	SETMAXLB = 27,
	SETM1PID = 28,
	SETM2PID = 29,
	GETM1ISPEED = 30,
	GETM2ISPEED = 31,
	M1DUTY = 32,
	M2DUTY = 33,
	MIXEDDUTY = 34,
	M1SPEED = 35,
	M2SPEED = 36,
	MIXEDSPEED = 37,
	M1SPEEDACCEL = 38,
	M2SPEEDACCEL = 39,
	MIXEDSPEEDACCEL = 40,
	M1SPEEDDIST = 41,
	M2SPEEDDIST = 42,
	MIXEDSPEEDDIST = 43,
	M1SPEEDACCELDIST = 44,
	M2SPEEDACCELDIST = 45,
	MIXEDSPEEDACCELDIST = 46,
	GETBUFFERS = 47,
	GETPWMS = 48,
	GETCURRENTS = 49,
	MIXEDSPEED2ACCEL = 50,
	MIXEDSPEED2ACCELDIST = 51,
	M1DUTYACCEL = 52,
	M2DUTYACCEL = 53,
	MIXEDDUTYACCEL = 54,
	READM1PID = 55,
	READM2PID = 56,
	SETMAINVOLTAGES = 57,
	SETLOGICVOLTAGES = 58,
	GETMINMAXMAINVOLTAGES = 59,
	GETMINMAXLOGICVOLTAGES = 60,
	SETM1POSPID = 61,
	SETM2POSPID = 62,
	READM1POSPID = 63,
	READM2POSPID = 64,
	M1SPEEDACCELDECCELPOS = 65,
	M2SPEEDACCELDECCELPOS = 66,
	MIXEDSPEEDACCELDECCELPOS = 67,
	SETM1DEFAULTACCEL = 68,
	SETM2DEFAULTACCEL = 69,
	SETPINFUNCTIONS = 74,
	GETPINFUNCTIONS = 75,
	SETDEADBAND = 76,
	GETDEADBAND = 77,
	GETENCODERS = 78,
	GETISPEEDS = 79,
	RESTOREDEFAULTS = 80,
	GETTEMP = 82,
	GETTEMP2 = 83,	//Only valid on some models
	GETERROR = 90,
	GETENCODERMODE = 91,
	SETM1ENCODERMODE = 92,
	SETM2ENCODERMODE = 93,
	WRITENVM = 94,
	READNVM = 95,	//Reloads values from Flash into Ram
	SETCONFIG = 98,
	GETCONFIG = 99,
	SETM1MAXCURRENT = 133,
	SETM2MAXCURRENT = 134,
	GETM1MAXCURRENT = 135,
	GETM2MAXCURRENT = 136,
	SETPWMMODE = 148,
	GETPWMMODE = 149,
	FLAGBOOTLOADER = 255
};
//-------------------------------------------------------------
class t_roboclaw_controller{

private:
	// version number of the library
	char library_version[20];

	// port number of the serial connection
	c_serial_port_t* m_port;
	c_serial_control_lines_t m_lines;

public:

	t_roboclaw_controller(void);
	~t_roboclaw_controller(void);

	// returns the library version
	// the caller must not delete the pointer
	const char* get_library_version(void);

	bool connect(const char* port, int baud_rate);
	void close_connection(void);
	bool is_open(void);
	void send_command(unsigned char command);
	bool read_result(unsigned char* buffer, int buffer_size);

	// Read the board temperature.Value returned is in 10ths of degrees.
	double get_board_temperature(void);

	// Read the main battery voltage level connected to B+ and B- terminals
	double get_main_battery_voltage(void);

	// Read RoboClaw firmware version. 
	// Returns up to 48 bytes(depending on the Roboclaw model) and 
	// is terminated by a line feed character and a null character.
	void get_firmware_version(char *firmware_version);

	// Drive motor 1 forward.Valid data range is 0 - 127. 
	// A value of 127 = full speed forward, 64 = about half speed forward and 0 = full stop.
	bool drive_forward_M1(unsigned char speed);
	// Drive motor 2 forward.Valid data range is 0 - 127. 
	// A value of 127 = full speed forward, 64 = about half speed forward and 0 = full stop.
	bool drive_forward_M2(unsigned char speed);
	bool drive_backward_M1(unsigned char speed);
	bool drive_backward_M2(unsigned char speed);

	// Read the current draw from each motor in 10ma increments. 
	// The amps value is calculated by	dividing the value by 100.
	void get_motors_current_consumption(double &current_motor_1, double &current_motor_2);

	// Read the current PWM output values for the motor channels.
	// The values returned are + -32767.
	// The duty cycle percent is calculated by dividing the Value by 327.67.
	void read_motor_PWM(double &pwm_motor_1, double &pwm_motor_2);

	// The duty value is signed and the range is - 32768 to + 32767(eg. + -100 % duty).
	// The accel value range is 0 to 655359(eg maximum acceleration rate is - 100 % to 100 % in 100ms).
	bool drive_M1_with_signed_duty_and_acceleration(int16_t duty, uint32_t accel);

	// The duty value is signed and the range is - 32768 to + 32767(eg. + -100 % duty).
	// The accel value range is 0 to 655359(eg maximum acceleration rate is - 100 % to 100 % in 100ms).
	bool drive_M2_with_signed_duty_and_acceleration(int16_t duty, uint32_t accel);

	// Set Motor 1 Maximum Current Limit. Current value is in 10ma units.
	//  To calculate multiply current limit by 100.
	bool set_M1_max_current_limit(double c_max);

	// Set Motor 1 Maximum Current Limit. Current value is in 10ma units.
	//  To calculate multiply current limit by 100.
	bool set_M2_max_current_limit(double c_max);

	// Read the current unit status.
	uint16_t read_status(void);

	// Set config bits for standard settings.
	void set_standard_config_settings(uint16_t config);

	// Read config bits for standard settings.
	uint16_t read_standard_config_settings(void);

};
//-------------------------------------------------------------
#endif