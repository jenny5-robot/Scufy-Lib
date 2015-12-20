
#include "jenny5_command_module.h"
#include <stdio.h>

//--------------------------------------------------------------
t_jenny5_command_module::t_jenny5_command_module()
{
	strcpy(version, "2015.12.20.0"); // year.month.day.built number
}
//--------------------------------------------------------------
t_jenny5_command_module::~t_jenny5_command_module()
{

}
//--------------------------------------------------------------
const char* t_jenny5_command_module::get_version(void)
{
	return version;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::connect(int port, int baud_rate)
{
	if (!serial_connection.IsOpened())
		return serial_connection.Open(port, baud_rate);
	else
		return false;
}
//--------------------------------------------------------------
void t_jenny5_command_module::close_connection(void)
{
	serial_connection.Close();
}
//--------------------------------------------------------------
bool t_jenny5_command_module::move_motor(int motor_index, int num_steps)
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "M%d %d#", motor_index, num_steps);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::move_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2)
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------

bool t_jenny5_command_module::move_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3)
{
	if (serial_connection.IsOpened()) {
		char s[63];
		sprintf(s, "M%d %d M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::move_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4)
{
	if (serial_connection.IsOpened()) {
		char s[63];
		sprintf(s, "M%d %d M%d %d M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3, motor_index4, num_steps4);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::move_motor(int num_motors, int* motor_index, int *num_steps)
{
	if (serial_connection.IsOpened()) {
		char s[63];
		s[0] = 0;
		for (int i = 0; i < num_motors; i++) {
			char tmp_str[20];

		  sprintf(tmp_str, "M%d %d#", motor_index[i], num_steps[i]);
		  strcat(s, tmp_str);
	    }
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::set_motor_speed(int motor_index, int motor_speed, int motor_acceleration)
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "S%d %d %d#", motor_index, motor_speed, motor_acceleration);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::set_motor_acceleration(int motor_index, int motor_acceleration)
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "A%d %d#", motor_index, motor_acceleration);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_ultrasonic_distance(int sensor_index)
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "U%d#", sensor_index);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_button_status(int button_index)
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "B%d#", button_index);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_potentiometer_position(int sensor_index)
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "P%d#", sensor_index);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_infrared_status(int sensor_index)
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "I%d#", sensor_index);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_motors_sensors_statistics()
{
	if (serial_connection.IsOpened()) {
		char s[20];
		sprintf(s, "G#");
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::disable_motor(int motor_index)
{
	if (serial_connection.IsOpened()) {
		char s[10];
		sprintf(s, "D%d#", motor_index);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::lock_motor(int motor_index)
{
	if (serial_connection.IsOpened()) {
		char s[10];
		sprintf(s, "L%d#", motor_index);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::is_connected(void)
{
	return serial_connection.IsOpened();
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_data(char *buffer, int buffer_size)
{
	if (serial_connection.IsOpened())
		return serial_connection.ReadData(buffer, buffer_size);
	else
		return 0;
}
//--------------------------------------------------------------
void t_jenny5_command_module::attach_sensors(int motor_index, int num_potentiometers, int *potentiometers_index)
{
	if (serial_connection.IsOpened()) {
		char s[64];
		sprintf(s, "A%d %d", motor_index, num_potentiometers);
		for (int i = 0; i < num_potentiometers; i++) {
			char tmp_str[64];
			sprintf(s, " P%d#", potentiometers_index[i]);
			strcat(s, tmp_str);
		}
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
}
//--------------------------------------------------------------
void t_jenny5_command_module::remove_attached_sensors(int motor_index)
{
	if (serial_connection.IsOpened()) {
		char s[10];
		sprintf(s, "A%d 0#", motor_index);
		serial_connection.SendData((const char*)s, strlen(s) + 1);
	}
}
//--------------------------------------------------------------
