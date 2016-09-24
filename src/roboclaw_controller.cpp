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
void write_n(uint8_t cnt, ...)
{
	uint8_t crc = 0;

	//send data with crc
	va_list marker;
	va_start(marker, cnt);     /* Initialize variable arguments. */
	for (uint8_t index = 0; index<cnt; index++) {
		uint8_t data = va_arg(marker, uint16_t);
		crc += data;
	//	write(data);
	}
	va_end(marker);              /* Reset variable arguments.      */
	crc &= 0x7F;
	//write(crc & 0x7F);
}

//--------------------------------------------------------------
t_roboclaw_controller::t_roboclaw_controller(void)
{
	strcpy(library_version, "2016.09.22.0"); // year.month.day.build number

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