// copyright Mihai Oltean, mihai.oltean@gmail.com
// www.jenny5.org
// www.tcreate.org
// source code: https://github.com/jenny5-robot

// MIT License
// ---------------------------------------------------------------------------

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
	strcpy(library_version, "2016.01.30.0"); // year.month.day.build number
	b_is_open = false;
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
	if (!b_is_open) {
		char mode[] = { '8', 'N', '1', 0 };

		port_number = port;

		if (RS232_OpenComport(port, baud_rate, mode) == 0) {
			b_is_open = true;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::is_open(void)
{
	return b_is_open;
}
//--------------------------------------------------------------
void t_roboclaw_controller::close_connection(void)
{
	if (b_is_open) {
		RS232_CloseComport(port_number);
		b_is_open = false;
	}
}
//--------------------------------------------------------------
void t_roboclaw_controller::send_command(unsigned char command)
{
	if (b_is_open) {
		unsigned char buffer[2];
		buffer[0] = 0x80;// port
		buffer[1] = command;
		RS232_SendBuf(port_number, buffer, 2);
	}
}
//--------------------------------------------------------------
bool t_roboclaw_controller::read_result(unsigned char* buffer, int buffer_size)
{
	if (b_is_open) {
		return RS232_PollComport(port_number, buffer, buffer_size);
	}
	else {
		buffer[0] = 0;
		return false;
	}
}
//--------------------------------------------------------------
void t_roboclaw_controller::get_firmware_version(char *firmware_version)
{
	if (b_is_open) {
		unsigned char buffer[32];

		buffer[0] = 0x80;// port
		buffer[1] = GETVERSION;
		RS232_SendBuf(port_number, (unsigned char*)buffer, 2);
		Sleep(10);

		RS232_PollComport(port_number, buffer, 32);
		strcpy(firmware_version, (char*)buffer);
	}
	else
		firmware_version[0] = 0;
}
//--------------------------------------------------------------
double t_roboclaw_controller::get_board_temperature(void)
{
	if (b_is_open) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETTEMP;
		RS232_SendBuf(port_number, buffer, 2);
		Sleep(10);

		RS232_PollComport(port_number, buffer, 10);

		return (double)(buffer[0] << 8 | buffer[1]) / 10.0;
	}
	else
		return 0;
}
//--------------------------------------------------------------
double t_roboclaw_controller::get_main_battery_voltage(void)
{
	if (b_is_open) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETMBATT;
		RS232_SendBuf(port_number, buffer, 2);
		Sleep(10);

		RS232_PollComport(port_number, buffer, 10);

		return (double)(buffer[0] << 8 | buffer[1]) / 10.0;
	}
	else
		return 0;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_forward_M1(unsigned char speed)
{
	if (b_is_open) {
		unsigned char buffer[5];
		buffer[0] = 0x80; // port
		buffer[1] = M1FORWARD;    // command
		buffer[2] = speed;

		uint16_t crc = CRC16(buffer, 3);
		buffer[3] = crc >> 8;
		buffer[4] = crc;

		RS232_SendBuf(port_number, buffer, 5);
		Sleep(10);

		int num_read = RS232_PollComport(port_number, buffer, 10);
		if (!num_read)
			return false;
		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_forward_M2(unsigned char speed)
{
	if (b_is_open) {
		unsigned char buffer[5];
		buffer[0] = 0x80; // port
		buffer[1] = M2FORWARD;    // command
		buffer[2] = speed;

		uint16_t crc = CRC16(buffer, 3);
		buffer[3] = crc >> 8;
		buffer[4] = crc;

		RS232_SendBuf(port_number, buffer, 5);
		Sleep(10);

		int num_read = RS232_PollComport(port_number, buffer, 10);
		if (!num_read)
			return false;
		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_backward_M1(unsigned char speed)
{
	if (b_is_open) {
		unsigned char buffer[5];
		buffer[0] = 0x80; // port
		buffer[1] = M1BACKWARD;    // command
		buffer[2] = speed;

		uint16_t crc = CRC16(buffer, 3);
		buffer[3] = crc >> 8;
		buffer[4] = crc;

		RS232_SendBuf(port_number, buffer, 5);
		Sleep(10);

		int num_read = RS232_PollComport(port_number, buffer, 10);
		if (!num_read)
			return false;
		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_backward_M2(unsigned char speed)
{
	if (b_is_open) {
		unsigned char buffer[5];
		buffer[0] = 0x80; // port
		buffer[1] = M2BACKWARD;    // command
		buffer[2] = speed;

		uint16_t crc = CRC16(buffer, 3);
		buffer[3] = crc >> 8;
		buffer[4] = crc;

		RS232_SendBuf(port_number, buffer, 5);
		Sleep(10);

		int num_read = RS232_PollComport(port_number, buffer, 10);
		if (!num_read)
			return false;
		return true;
	}
	return false;
}
//--------------------------------------------------------------
void t_roboclaw_controller::get_motors_current_consumption(double &current_motor_1, double &current_motor_2)
{
	if (b_is_open) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETCURRENTS;
		RS232_SendBuf(port_number, buffer, 2);
		Sleep(10);

		RS232_PollComport(port_number, buffer, 10);

		current_motor_1 = (double)(buffer[0] << 8 | buffer[1]) / 100.0;
		current_motor_2 = (double)(buffer[2] << 8 | buffer[3]) / 100.0;
	}
	else {
		current_motor_1 = 0;
		current_motor_2 = 0;
	}
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_M1_with_signed_duty_and_acceleration(int16_t duty, uint32_t accel)
{
	if (b_is_open) {
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
		Sleep(10);
		
		int num_read = RS232_PollComport(port_number, buffer, 10);
		if (!num_read)
			return false;
		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_M2_with_signed_duty_and_acceleration(int16_t duty, uint32_t accel)
{
	if (b_is_open) {
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
		Sleep(10);
		int num_read = RS232_PollComport(port_number, buffer, 10);
		if (!num_read)
			return false;
		return true;
	}
	return false;
}
//--------------------------------------------------------------
void t_roboclaw_controller::set_M1_max_current_limit(double c_max)
{
	if (b_is_open) {
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
}
//--------------------------------------------------------------
void t_roboclaw_controller::set_M2_max_current_limit(double c_max)
{
	if (b_is_open) {
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
}
//--------------------------------------------------------------
void t_roboclaw_controller::read_motor_PWM(double &pwm_motor_1, double &pwm_motor_2)
{
	if (b_is_open) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETPWMS;
		RS232_SendBuf(port_number, buffer, 2);
		Sleep(10);

		RS232_PollComport(port_number, buffer, 10);

		pwm_motor_1 = (double)(buffer[0] << 8 | buffer[1]) / 327.67;
		pwm_motor_2 = (double)(buffer[2] << 8 | buffer[3]) / 327.67;
	}
	else {
		pwm_motor_1 = 0;
		pwm_motor_2 = 0;
	}
}
//--------------------------------------------------------------
uint16_t t_roboclaw_controller::read_status(void)
{
	if (b_is_open) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETERROR;
		RS232_SendBuf(port_number, buffer, 2);
		Sleep(10);

		RS232_PollComport(port_number, buffer, 10);

		return buffer[0] << 8 | buffer[1];
	}
	else
		return 0;
}
//--------------------------------------------------------------
void t_roboclaw_controller::set_standard_config_settings(uint16_t config)
{
	if (b_is_open) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = SETCONFIG;

		buffer[2] = config >> 8;
		buffer[3] = config;

		uint16_t crc = CRC16(buffer, 4);
		buffer[4] = crc >> 8;
		buffer[5] = crc;

		RS232_SendBuf(port_number, buffer, 6);
		Sleep(10);
	}
}
//--------------------------------------------------------------
uint16_t t_roboclaw_controller::read_standard_config_settings(void)
{
	if (b_is_open) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETCONFIG;
		RS232_SendBuf(port_number, buffer, 2);
		Sleep(10);

		RS232_PollComport(port_number, buffer, 10);

		return buffer[0] << 8 | buffer[1];
	}
	else
		return  0;
}
//--------------------------------------------------------------