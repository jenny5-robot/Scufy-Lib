
#include "jenny5_command_module.h"
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

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
	char mode[] = { '8', 'N', '1', 0 };

	port_number = port;
	return RS232_OpenComport(port, baud_rate, mode) == 0;

}
//--------------------------------------------------------------
void t_jenny5_command_module::close_connection(void)
{
	RS232_CloseComport(port_number);
}
//--------------------------------------------------------------
bool t_jenny5_command_module::move_motor(int motor_index, int num_steps)
{
	if (RS232_is_open()) {
		char s[20];
		sprintf(s, "M%d %d#", motor_index, num_steps);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	
	else
		return false;
		
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::move_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2)
{
	if (RS232_is_open()) {
		char s[20];
		sprintf(s, "M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------

bool t_jenny5_command_module::move_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3)
{
	if (RS232_is_open()) {
		char s[63];
		sprintf(s, "M%d %d M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::move_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4)
{
	if (RS232_is_open()) {
		char s[63];
		sprintf(s, "M%d %d M%d %d M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3, motor_index4, num_steps4);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::move_motor_array(int num_motors, int* motor_index, int *num_steps)
{
	if (RS232_is_open()) {
		char s[63];
		s[0] = 0;
		for (int i = 0; i < num_motors; i++) {
			char tmp_str[20];

		  sprintf(tmp_str, "M%d %d#", motor_index[i], num_steps[i]);
		  strcat(s, tmp_str);
	    }
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::set_motor_speed_and_acceleration(int motor_index, int motor_speed, int motor_acceleration)
{
	if (RS232_is_open()) {
		char s[20];
		sprintf(s, "SM%d %d %d#", motor_index, motor_speed, motor_acceleration);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	else
		return false;

	return true;
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_ultrasonic_distance(int sensor_index)
{
	if (RS232_is_open()) {
		char s[20];
		sprintf(s, "U%d#", sensor_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
		// now wait for result
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_button_status(int button_index)
{
	if (RS232_is_open()) {
		char s[20];
		sprintf(s, "B%d#", button_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_potentiometer_position(int sensor_index)
{
	if (RS232_is_open()) {
		char s[20];
		sprintf(s, "P%d#", sensor_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
		// now wait for result
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_infrared_distance(int sensor_index)
{
	if (RS232_is_open()) {
		char s[20];
		sprintf(s, "I%d#", sensor_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_motors_sensors_statistics(void)
{
	if (RS232_is_open()) {
		char s[20];
		sprintf(s, "G#");
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::disable_motor(int motor_index)
{
	if (RS232_is_open()) {
		char s[10];
		sprintf(s, "D%d#", motor_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::lock_motor(int motor_index)
{
	if (RS232_is_open()) {
		char s[10];
		sprintf(s, "L%d#", motor_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
	return true;
}
//--------------------------------------------------------------
int t_jenny5_command_module::is_connected(void)
{
	return RS232_is_open();
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_data(unsigned char *buffer, int buffer_size)
{
	if (RS232_is_open())
		return RS232_PollComport(port_number, buffer, buffer_size);
	else
		return 0;
}
//--------------------------------------------------------------
void t_jenny5_command_module::attach_sensors(int motor_index, int num_potentiometers, int *potentiometers_index)
{
	if (RS232_is_open()) {
		char s[64];
		sprintf(s, "A%d %d", motor_index, num_potentiometers);
		for (int i = 0; i < num_potentiometers; i++) {
			char tmp_str[64];
			sprintf(s, " P%d#", potentiometers_index[i]);
			strcat(s, tmp_str);
		}
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
}
//--------------------------------------------------------------
void t_jenny5_command_module::remove_attached_sensors(int motor_index)
{
	if (RS232_is_open()) {
		char s[10];
		sprintf(s, "A%d 0#", motor_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_motor_parameters(int motor_index)
{
	if (RS232_is_open()) {
		char s[10];
		sprintf(s, "GM%d#", motor_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
}
//--------------------------------------------------------------
bool t_jenny5_command_module::get_potentiometer_parameters(int potentiometer_index)
{
	if (RS232_is_open()) {
		char s[10];
		sprintf(s, "GP%d#", potentiometer_index);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
}
//--------------------------------------------------------------
bool t_jenny5_command_module::set_potentiometer_parameters(int potentiometer_index, int _min, int _max, int _home)
{
	if (RS232_is_open()) {
		char s[30];
		sprintf(s, "SP%d %d %d %d#", potentiometer_index, _min, _max, _home);
		RS232_SendBuf(port_number, (unsigned char*)s, strlen(s) + 1);
	}
}
//--------------------------------------------------------------