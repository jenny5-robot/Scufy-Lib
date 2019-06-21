// author: Mihai Oltean
// email: mihai.oltean@gmail.com
// main website: https://www.jenny5.org
// mirror website: https://jenny5-robot.github.io
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
	strcpy(library_version, "2019.06.01.0"); // year.month.day.build number

	if (c_serial_new(&m_port, NULL) < 0) {
		//fprintf(stderr, "ERROR: Unable to create new serial port\n");
		//		return 1;
	}

}
//--------------------------------------------------------------
t_roboclaw_controller::~t_roboclaw_controller(void)
{
	c_serial_free(m_port);
}
//--------------------------------------------------------------
const char* t_roboclaw_controller::get_library_version(void)
{
	return library_version;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::connect(const char* port, int baud_rate)
{
	if (!c_serial_is_open(m_port)) {

		if (c_serial_set_port_name(m_port, port) < 0) {
			//fprintf(stderr, "ERROR: can't set port name\n");
			return false;
		}

		c_serial_set_baud_rate(m_port, CSERIAL_BAUD_115200);
		c_serial_set_data_bits(m_port, CSERIAL_BITS_8);
		c_serial_set_stop_bits(m_port, CSERIAL_STOP_BITS_1);
		c_serial_set_parity(m_port, CSERIAL_PARITY_NONE);
		c_serial_set_flow_control(m_port, CSERIAL_FLOW_NONE);

		c_serial_set_serial_line_change_flags(m_port, CSERIAL_LINE_FLAG_ALL);

		int status = c_serial_open(m_port);
		if (status < 0) {
			//fprintf(stderr, "ERROR: Can't open serial port\n");
			return false;
		}

		c_serial_flush(m_port);
		return true;
	}
	else
		return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::is_open(void)
{
	return c_serial_is_open(m_port);
}
//--------------------------------------------------------------
void t_roboclaw_controller::close_connection(void)
{
	if (c_serial_is_open(m_port)) {
		c_serial_close(m_port);
	}
}
//--------------------------------------------------------------
void t_roboclaw_controller::send_command(unsigned char command)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[2];
		buffer[0] = 0x80;// port
		buffer[1] = command;
		int data_length = 2;
		c_serial_write_data(m_port, buffer, &data_length);
	}
}
//--------------------------------------------------------------
bool t_roboclaw_controller::read_result(unsigned char* buffer, int buffer_size)
{
	if (c_serial_is_open(m_port)) {
		int num_available;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			buffer[0] = 0;
			return false;
		}
		if (num_available <= 0)
			return false;

		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);
		return true;
	}
	else {
		buffer[0] = 0;
		return false;
	}
}
//--------------------------------------------------------------
void t_roboclaw_controller::get_firmware_version(char *firmware_version)
{
	if (c_serial_is_open(m_port)) {
		int buffer_size = 32;
		unsigned char buffer[32];

		buffer[0] = 0x80;// port
		buffer[1] = GETVERSION;
		
		int data_length = 2;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		
		int num_available;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			buffer[0] = 0;
		}
		if (num_available <= 0)
			buffer[0] = 0;
		
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);
		
		strcpy(firmware_version, (char*)buffer);
	}
	else
		firmware_version[0] = 0;
}
//--------------------------------------------------------------
double t_roboclaw_controller::get_board_temperature(void)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETTEMP;
		int data_length = 2;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return 0;
		}
		if (num_available <= 0)
			return 0;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

		return (double)(buffer[0] << 8 | buffer[1]) / 10.0;
	}
	else
		return 0;
}
//--------------------------------------------------------------
double t_roboclaw_controller::get_main_battery_voltage(void)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETMBATT;
		int data_length = 2;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return 0;
		}
		if (num_available <= 0)
			return 0;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

		return (double)(buffer[0] << 8 | buffer[1]) / 10.0;
	}
	else
		return 0;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_forward_M1(unsigned char speed)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[5];
		buffer[0] = 0x80; // port
		buffer[1] = M1FORWARD;    // command
		buffer[2] = speed;

		uint16_t crc = CRC16(buffer, 3);
		buffer[3] = crc >> 8;
		buffer[4] = (unsigned char)crc;

		int data_length = 5;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return false;
		}
		if (num_available <= 0)
			return false;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_forward_M2(unsigned char speed)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[5];
		buffer[0] = 0x80; // port
		buffer[1] = M2FORWARD;    // command
		buffer[2] = speed;

		uint16_t crc = CRC16(buffer, 3);
		buffer[3] = crc >> 8;
		buffer[4] = (unsigned char)crc;

		int data_length = 5;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return false;
		}
		if (num_available <= 0)
			return false;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_backward_M1(unsigned char speed)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[5];
		buffer[0] = 0x80; // port
		buffer[1] = M1BACKWARD;    // command
		buffer[2] = speed;

		uint16_t crc = CRC16(buffer, 3);
		buffer[3] = crc >> 8;
		buffer[4] = (unsigned char)crc;

		int data_length = 5;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return false;
		}
		if (num_available <= 0)
			return false;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_backward_M2(unsigned char speed)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[5];
		buffer[0] = 0x80; // port
		buffer[1] = M2BACKWARD;    // command
		buffer[2] = speed;

		uint16_t crc = CRC16(buffer, 3);
		buffer[3] = crc >> 8;
		buffer[4] = (unsigned char)crc;

		int data_length = 5;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return false;
		}
		if (num_available <= 0)
			return false;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

		return true;
	}
	return false;
}
//--------------------------------------------------------------
void t_roboclaw_controller::get_motors_current_consumption(double &current_motor_1, double &current_motor_2)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETCURRENTS;
		int data_length = 5;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			current_motor_1 = current_motor_2 = 0;
			return ;
		}
		if (num_available <= 0) {
			current_motor_1 = current_motor_2 = 0;
			return ;
		}
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

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
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80; // port
		buffer[1] = M1DUTYACCEL;    // command

		buffer[2] = duty >> 8;
		buffer[3] = (unsigned char)duty;

		buffer[4] = accel >> 8;
		buffer[5] = (unsigned char)accel;

		buffer[6] = accel >> 8;
		buffer[7] = (unsigned char)accel;

		uint16_t crc = CRC16(buffer, 8);
		buffer[8] = crc >> 8;
		buffer[9] = (unsigned char)crc;

		int data_length = 10;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);
		
		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return false;
		}
		if (num_available <= 0)
			return false;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::drive_M2_with_signed_duty_and_acceleration(int16_t duty, uint32_t accel)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80; // port
		buffer[1] = M2DUTYACCEL;    // command

		buffer[2] = duty >> 8;
		buffer[3] = (unsigned char)duty;

		buffer[4] = accel >> 8;
		buffer[5] = (unsigned char)accel;

		buffer[6] = accel >> 8;
		buffer[7] = (unsigned char)accel;

		uint16_t crc = CRC16(buffer, 8);
		buffer[8] = crc >> 8;
		buffer[9] = (unsigned char)crc;

		int data_length = 10;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);
		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return false;
		}
		if (num_available <= 0)
			return false;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);
		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::set_M1_max_current_limit(double c_max)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[12];
		buffer[0] = 0x80; // port
		buffer[1] = SETM1MAXCURRENT;    // command

		uint32_t max_v;
		max_v = (uint32_t)(c_max * 100);

		buffer[2] = max_v >> 24;
		buffer[3] = max_v >> 16;
		buffer[4] = max_v >> 8;
		buffer[5] = (unsigned char)max_v;

		buffer[6] = 0;
		buffer[7] = 0;
		buffer[8] = 0;
		buffer[9] = 0;

		uint16_t crc = CRC16(buffer, 10);
		buffer[10] = crc >> 8;
		buffer[11] = (unsigned char)crc;

		int data_length = 12;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);
		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return false;
		}
		if (num_available <= 0)
			return false;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);
		return true;
	}
	return false;
}
//--------------------------------------------------------------
bool t_roboclaw_controller::set_M2_max_current_limit(double c_max)
{
	if (c_serial_is_open(m_port)) {
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
		buffer[11] = (unsigned char)crc;

		int data_length = 12;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);
		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return false;
		}
		if (num_available <= 0)
			return false;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);
		return true;
	}
	return false;
}
//--------------------------------------------------------------
void t_roboclaw_controller::read_motor_PWM(double &pwm_motor_1, double &pwm_motor_2)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETPWMS;
		int data_length = 2;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			pwm_motor_1 = pwm_motor_2 = 0;
			return ;
		}
		if (num_available <= 0) {
			pwm_motor_1 = pwm_motor_2 = 0;
			return ;
		}
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

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
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETERROR;
		int data_length = 2;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return 0;
		}
		if (num_available <= 0) {
			return 0;
		}
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);

		return buffer[0] << 8 | buffer[1];
	}
	else
		return 0;
}
//--------------------------------------------------------------
void t_roboclaw_controller::set_standard_config_settings(uint16_t config)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = SETCONFIG;

		buffer[2] = config >> 8;
		buffer[3] = (unsigned char)config;

		uint16_t crc = CRC16(buffer, 4);
		buffer[4] = crc >> 8;
		buffer[5] = (unsigned char)crc;

		int data_length = 6;
		c_serial_write_data(m_port, buffer, &data_length);
		Sleep(10);
	}
}
//--------------------------------------------------------------
uint16_t t_roboclaw_controller::read_standard_config_settings(void)
{
	if (c_serial_is_open(m_port)) {
		unsigned char buffer[10];
		buffer[0] = 0x80;// port
		buffer[1] = GETCONFIG;
		int data_length = 2;
		c_serial_write_data(m_port, buffer, &data_length);

		Sleep(10);

		int num_available;
		int buffer_size = 10;
		if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK) {
			return 0;
		}
		if (num_available <= 0) {
			return 0;
		}

		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);
		return buffer[0] << 8 | buffer[1];
	}
	else
		return  0;
}
//--------------------------------------------------------------