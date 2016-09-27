// copyright Mihai Oltean
// www.jenny5.org
// www.tcreate.org
// https://github.com/jenny5-robot
// mihai.oltean@gmail.com

// MIT License

#include "roboclaw_controller.h"
#include <stdint.h>

//--------------------------------------------------------------
uint16_t CRC16(unsigned char *packet, int nBytes)
{
	uint16_t crc = 0;
	for (int byte = 0; byte < nBytes; byte++) {
		crc = crc ^ ((uint16_t)packet[byte] << 8);
		for (unsigned char bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			}
			else {
				crc = crc << 1;
			}
		}
	}
	return crc;
}
//--------------------------------------------------------------
t_roboclaw_controller::t_roboclaw_controller(void)
{
	strcpy(library_version, "2016.09.27.0"); // year.month.day.build number
}
//--------------------------------------------------------------
t_roboclaw_controller::~t_roboclaw_controller(void)
{

}
//--------------------------------------------------------------
const char* t_roboclaw_controller::get_library_version(void)
{
	return library_version;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::connect(int port, int baud_rate)
{
	char mode[] = { '8', 'N', '1', 0 };

	port_number = port;

	return RS232_OpenComport(port, baud_rate, mode) == 0;
}
//--------------------------------------------------------------
void t_roboclaw_controller::close_connection(void)
{
	RS232_CloseComport(port_number);

}
//--------------------------------------------------------------
void t_roboclaw_controller::send_command(int command)
{
	unsigned char s[2];
	s[0] = 0x80;// port
	s[1] = command;
	RS232_SendBuf(port_number, s, 2);
}
//--------------------------------------------------------------
bool t_roboclaw_controller::read_result(unsigned char* buffer, int buffer_size)
{
	return RS232_PollComport(port_number, buffer, buffer_size);
}
//--------------------------------------------------------------
void t_roboclaw_controller::get_firmware_version(char *firmware_version)
{

	firmware_version[0] = 0x80;// port
	firmware_version[1] = GETVERSION;
	RS232_SendBuf(port_number, (unsigned char*)firmware_version, 2);
	Sleep(10);

	unsigned char s[32];
	RS232_PollComport(port_number, s, 32);
	strcpy(firmware_version, (char*)s);
}
//--------------------------------------------------------------
double t_roboclaw_controller::get_temperature(void)
{
	unsigned char s[10];
	s[0] = 0x80;// port
	s[1] = GETTEMP;
	RS232_SendBuf(port_number, s, 2);
	Sleep(10);

	RS232_PollComport(port_number, s, 10);

	return (double)(s[0] << 8 | s[1]) / 10.0;
}
//--------------------------------------------------------------
double t_roboclaw_controller::get_main_battery_voltage(void)
{
	unsigned char s[10];
	s[0] = 0x80;// port
	s[1] = GETMBATT;
	RS232_SendBuf(port_number, s, 2);
	Sleep(10);

	RS232_PollComport(port_number, s, 10);

	return (double)(s[0] << 8 | s[1]) / 10.0;
}
//--------------------------------------------------------------
void t_roboclaw_controller::drive_forward_M1(unsigned char speed)
{
	unsigned char buffer[5];
	buffer[0] = 0x80; // port
	buffer[1] = M1FORWARD;    // command
	buffer[2] = speed;

	uint16_t crc = CRC16(buffer, 3);
	buffer[3] = crc >> 8;
	buffer[4] = crc;

	RS232_SendBuf(port_number, buffer, 5);

}
//--------------------------------------------------------------
void t_roboclaw_controller::drive_forward_M2(unsigned char speed)
{
	unsigned char buffer[5];
	buffer[0] = 0x80; // port
	buffer[1] = M2FORWARD;    // command
	buffer[2] = speed;

	uint16_t crc = CRC16(buffer, 3);
	buffer[3] = crc >> 8;
	buffer[4] = crc;
	
	RS232_SendBuf(port_number, buffer, 5);
}
//--------------------------------------------------------------
void t_roboclaw_controller::drive_backward_M1(unsigned char speed)
{
	unsigned char buffer[5];
	buffer[0] = 0x80; // port
	buffer[1] = M1BACKWARD;    // command
	buffer[2] = speed;

	uint16_t crc = CRC16(buffer, 3);
	buffer[3] = crc >> 8;
	buffer[4] = crc;

	RS232_SendBuf(port_number, buffer, 5);
}
//--------------------------------------------------------------
void t_roboclaw_controller::drive_backward_M2(unsigned char speed)
{
	unsigned char buffer[5];
	buffer[0] = 0x80; // port
	buffer[1] = M2BACKWARD;    // command
	buffer[2] = speed;

	uint16_t crc = CRC16(buffer, 3);
	buffer[3] = crc >> 8;
	buffer[4] = crc;

	RS232_SendBuf(port_number, buffer, 5);
}
//--------------------------------------------------------------
void t_roboclaw_controller::read_motor_currents(double &current_motor_1, double &current_motor_2)
{
	unsigned char s[10];
	s[0] = 0x80;// port
	s[1] = GETCURRENTS;
	RS232_SendBuf(port_number, s, 2);
	Sleep(10);

	RS232_PollComport(port_number, s, 10);

	current_motor_1 = (double)(s[0] << 8 | s[1]) / 100.0;
	current_motor_2 = (double)(s[2] << 8 | s[3]) / 100.0;

}
//--------------------------------------------------------------
void t_roboclaw_controller::drive_M1_with_signed_duty_and_acceleration(int16_t duty, uint32_t accel)
{
	unsigned char buffer[10];
	buffer[0] = 0x80; // port
	buffer[1] = M1DUTYACCEL;    // command

	buffer[2] = duty >> 8;
	buffer[3] = duty;

	buffer[4] = accel >> 8;
	buffer[5] = accel;

	buffer[6] = accel >> 8;
	buffer[7] = accel;

	uint16_t crc = CRC16(buffer, 8);
	buffer[8] = crc >> 8;
	buffer[9] = crc;

	RS232_SendBuf(port_number, buffer, 10);

}
//--------------------------------------------------------------
void t_roboclaw_controller::drive_M2_with_signed_duty_and_acceleration(int16_t duty, uint32_t accel)
{
	unsigned char buffer[10];
	buffer[0] = 0x80; // port
	buffer[1] = M2DUTYACCEL;    // command

	buffer[2] = duty >> 8;
	buffer[3] = duty;

	buffer[4] = accel >> 8;
	buffer[5] = accel;

	buffer[6] = accel >> 8;
	buffer[7] = accel;

	uint16_t crc = CRC16(buffer, 8);
	buffer[8] = crc >> 8;
	buffer[9] = crc;

	RS232_SendBuf(port_number, buffer, 10);

}
//--------------------------------------------------------------
void t_roboclaw_controller::set_M1_max_current_limit(double c_max)
{
	unsigned char buffer[12];
	buffer[0] = 0x80; // port
	buffer[1] = SETM1MAXCURRENT;    // command

	uint32_t max_v;
	max_v = (uint32_t)(c_max * 100);

	buffer[2] = max_v >> 24;
	buffer[3] = max_v >> 16;
	buffer[4] = max_v >> 8;
	buffer[5] = max_v;

	buffer[6] = 0;
	buffer[7] = 0;
	buffer[8] = 0;
	buffer[9] = 0;

	uint16_t crc = CRC16(buffer, 10);
	buffer[10] = crc >> 8;
	buffer[11] = crc;

	RS232_SendBuf(port_number, buffer, 12);
}
//--------------------------------------------------------------

void t_roboclaw_controller::set_M2_max_current_limit(double c_max)
{
	unsigned char buffer[12];
	buffer[0] = 0x80; // port
	buffer[1] = SETM2MAXCURRENT;    // command

	uint32_t max_v;
	max_v = (uint32_t)(c_max * 100);

	buffer[2] = max_v >> 24;
	buffer[3] = max_v >> 16;
	buffer[4] = max_v >> 8;
	buffer[5] = max_v;

	buffer[6] = 0;
	buffer[7] = 0;
	buffer[8] = 0;
	buffer[9] = 0;

	uint16_t crc = CRC16(buffer, 10);
	buffer[10] = crc >> 8;
	buffer[11] = crc;

	RS232_SendBuf(port_number, buffer, 12);
}
//--------------------------------------------------------------