
#include "jenny5_command_module.h"
#include "jenny5_events.h"
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

//--------------------------------------------------------------
t_jenny5_command_module::t_jenny5_command_module(void)
{
	strcpy(version, "2016.01.17.0"); // year.month.day.build number
	current_buffer[0] = 0;
	for (int i = 0; i < 4; i++)
	  motor_state[i] = COMMAND_DONE;
}
//--------------------------------------------------------------
t_jenny5_command_module::~t_jenny5_command_module(void)
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
	current_buffer[0] = 0;

	return RS232_OpenComport(port, baud_rate, mode) == 0;
}
//--------------------------------------------------------------
void t_jenny5_command_module::close_connection(void)
{
	RS232_CloseComport(port_number);
	current_buffer[0] = 0;
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_motor(int motor_index, int num_steps)
{
	char s[20];
	sprintf(s, "M%d %d#", motor_index, num_steps);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2)
{
	char s[30];
	sprintf(s, "M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3)
{
	char s[63];
	sprintf(s, "M%d %d M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4)
{

	char s[63];
	sprintf(s, "M%d %d M%d %d M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3, motor_index4, num_steps4);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_motor_array(int num_motors, int* motor_index, int *num_steps)
{
	char s[63];
	s[0] = 0;
	for (int i = 0; i < num_motors; i++) {
		char tmp_str[20];

		sprintf(tmp_str, "M%d %d#", motor_index[i], num_steps[i]);
		strcat(s, tmp_str);
	}
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_set_motor_speed_and_acceleration(int motor_index, int motor_speed, int motor_acceleration)
{
	char s[20];
	sprintf(s, "SM%d %d %d#", motor_index, motor_speed, motor_acceleration);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_ultrasonic_distance(int sensor_index)
{
	char s[20];
	sprintf(s, "U%d#", sensor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_button_status(int button_index)
{
	char s[20];
	sprintf(s, "B%d#", button_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_potentiometer_position(int sensor_index)
{
	char s[20];
	sprintf(s, "P%d#", sensor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_infrared_distance(int sensor_index)
{
	char s[20];
	sprintf(s, "I%d#", sensor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_motors_sensors_statistics(void)
{
	char s[20];
	sprintf(s, "G#");
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_disable_motor(int motor_index)
{
	char s[10];
	sprintf(s, "D%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_lock_motor(int motor_index)
{
	char s[10];
	sprintf(s, "L%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_data_from_serial(char *buffer, int buffer_size)
{
	return RS232_PollComport(port_number, (unsigned char*)buffer, buffer_size);
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_attach_sensors(int motor_index, int num_potentiometers, int *potentiometers_index)
{
	char s[64];
	sprintf(s, "A%d %d", motor_index, num_potentiometers);
	for (int i = 0; i < num_potentiometers; i++) {
		char tmp_str[64];
		sprintf(s, " P%d#", potentiometers_index[i]);
		strcat(s, tmp_str);
	}
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_remove_attached_sensors(int motor_index)
{
	char s[10];
	sprintf(s, "A%d 0#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_motor_parameters(int motor_index)
{
	char s[10];
	sprintf(s, "GM%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_potentiometer_parameters(int potentiometer_index)
{
	char s[10];
	sprintf(s, "GP%d#", potentiometer_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_set_potentiometer_parameters(int potentiometer_index, int _min, int _max, int _home)
{
	char s[30];
	sprintf(s, "SP%d %d %d %d#", potentiometer_index, _min, _max, _home);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_is_alive(void)
{
	char s[3];
	strcpy(s, "T#");
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::parse_and_queue_commands(char* tmp_str, int str_length)
{
	int i = 0;
	while (i < str_length) {
		// can be more than 1 command in a string, so I have to check again for a letter
		if ((tmp_str[i] >= 'A' && tmp_str[i] <= 'Z') || (tmp_str[i] >= 'a' && tmp_str[i] <= 'z')) {

			if (tmp_str[i] == 'M' || tmp_str[i] == 'm') {// motor was moved
				int motor_index, distance_to_go;
				sscanf(tmp_str + i + 1, "%d%d", &motor_index, &distance_to_go);
				i += 5;
				jenny5_event *e = new jenny5_event(MOTOR_DONE_EVENT, motor_index, distance_to_go, 0);
				received_events.Add((void*)e);
			}
			else
				if (tmp_str[i] == 'T' || tmp_str[i] == 't') {// test connection
					jenny5_event *e = new jenny5_event(IS_ALIVE_EVENT, 0, 0, 0);
					received_events.Add((void*)e);
					i += 2;
				}
				else
					if (tmp_str[i] == 'L' || tmp_str[i] == 'l') {// motor was locked
						int motor_index;
						sscanf(tmp_str + i + 1, "%d", &motor_index);
						i += 3;
						jenny5_event *e = new jenny5_event(MOTOR_LOCKED_EVENT, motor_index, 0, 0);
						received_events.Add((void*)e);
					}
					else
						if (tmp_str[i] == 'D' || tmp_str[i] == 'd') {// motor was disabled
							int motor_index;
							sscanf(tmp_str + i + 1, "%d", &motor_index);
							i += 3;
							jenny5_event *e = new jenny5_event(MOTOR_DISABLED_EVENT, motor_index, 0, 0);
							received_events.Add((void*)e);
						}
						else
							i++;
			// more events to add
		}
		else
			i++;
	}
}
//--------------------------------------------------------------
bool t_jenny5_command_module::update_commands_from_serial(void)
{
	// the same code as in firmware
	char tmp_buffer[4096];
	int received_size = get_data_from_serial(tmp_buffer, 4096);
	tmp_buffer[received_size] = 0;
	if (received_size) {
		strcpy(current_buffer + strlen(current_buffer), tmp_buffer);

		int buffer_length = strlen(current_buffer);
		for (int i = 0; i < buffer_length; i++)
			if ((current_buffer[i] >= 'A' && current_buffer[i] <= 'Z') || (current_buffer[i] >= 'a' && current_buffer[i] <= 'z')) {// a command
				// find the terminal character #
				int j = i + 1;
				for (; j < buffer_length && current_buffer[j] != '#'; j++);// parse until I find the termination char
				if (j < buffer_length) {

#ifdef DEBUG
					char tmp_str[64];
					strncpy(tmp_str, current_buffer + i, j - i);
					tmp_str[j - i] = 0;
					printf("current command is= %s", tmp_str);
#endif

					//parse_and_execute_commands(tmp_str, j - i);
					parse_and_queue_commands(current_buffer + i, j - i);


					// remove the current executed command
					strcpy(current_buffer, current_buffer + j + 1);// not sure if this is good due to overlaps

#ifdef DEBUG
					Serial.write("buffer left=");
					Serial.write(current_buffer);
					Serial.write("\n----------------\n");
					//Serial.println(strlen(current_buffer)); // buffer length
#endif

					break; //for i
			}
				else {// the string is not completed ... so I must wait for more...
					break; // for i
				}
	}


		return true;
	}
	else
		return false;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::query_for_event(int event_type, intptr_t param1)
{
	for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type && e->param1 == param1) {
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::query_for_event(int event_type, intptr_t param1, intptr_t param2)
{
	for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type && e->param1 == param1 && e->param2 == param2) {
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::query_for_2_events(int event_type1, intptr_t param1_1, int event_type2, intptr_t param1_2)
{
	bool event1_found = false;
	bool event2_found = false;

	node_double_linked *node_p1, *node_p2;
	for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type1 && e->param1 == param1_1) {
			event1_found = true;
			node_p1 = node_p;
		}
		else
			if (e->type == event_type2 && e->param2 == param1_1) {
				event2_found = true;
				node_p2 = node_p;
			}
	}

	if (event1_found && event2_found) {
		received_events.DeleteCurrent(node_p1);
		received_events.DeleteCurrent(node_p2);
		return true;
	}
	else
	return false;
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_go_home_motor(int motor_index)
{
	char s[20];
	sprintf(s, "H%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, strlen(s));
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_motor_state(int motor_index)
{
	return motor_state[motor_index];
}
//--------------------------------------------------------------
void t_jenny5_command_module::set_motor_state(int motor_index, int state)
{
	motor_state[motor_index] = state;
}
//--------------------------------------------------------------