
#include "../include/jenny5_command_module.h"
#include "../include/jenny5_events.h"
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

//--------------------------------------------------------------
t_jenny5_command_module::t_jenny5_command_module(void)
{
	strcpy(version, "2016.02.17.3"); // year.month.day.build number
	current_buffer[0] = 0;
	for (int i = 0; i < 4; i++)
		stepper_motor_state[i] = COMMAND_DONE;
	for (int i = 0; i < 3; i++)
		dc_motor_state[i] = COMMAND_DONE;
	for (int i = 0; i < 6; i++)
		sonar_state[i] = COMMAND_DONE;
	for (int i = 0; i < 4; i++)
		potentiometer_state[i] = COMMAND_DONE;
	for (int i = 0; i < 4; i++)
		infrared_state[i] = COMMAND_DONE;
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
void t_jenny5_command_module::send_move_stepper_motor(int motor_index, int num_steps)
{
	char s[20];
	sprintf(s, "MS%d %d#", motor_index, num_steps);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_stepper_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2)
{
	char s[30];
	sprintf(s, "MS%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_stepper_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3)
{
	char s[63];
	sprintf(s, "MS%d %d M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_stepper_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4)
{

	char s[63];
	sprintf(s, "MS%d %d M%d %d M%d %d M%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3, motor_index4, num_steps4);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_move_stepper_motor_array(int num_motors, int* motor_index, int *num_steps)
{
	char s[63];
	s[0] = 0;
	for (int i = 0; i < num_motors; i++) {
		char tmp_str[20];

		sprintf(tmp_str, "MS%d %d#", motor_index[i], num_steps[i]);
		strcat(s, tmp_str);
	}
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_set_stepper_motor_speed_and_acceleration(int motor_index, int motor_speed, int motor_acceleration)
{
	char s[20];
	sprintf(s, "SS%d %d %d#", motor_index, motor_speed, motor_acceleration);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_sonar_distance(int sensor_index)
{
	char s[20];
	sprintf(s, "U%d#", sensor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_button_status(int button_index)
{
	char s[20];
	sprintf(s, "B%d#", button_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_potentiometer_position(int sensor_index)
{
	char s[20];
	sprintf(s, "P%d#", sensor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_infrared_distance(int sensor_index)
{
	char s[20];
	sprintf(s, "I%d#", sensor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_motors_sensors_statistics(void)
{
	char s[20];
	sprintf(s, "G#");
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_disable_stepper_motor(int motor_index)
{
	char s[10];
	sprintf(s, "DS%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_lock_stepper_motor(int motor_index)
{
	char s[10];
	sprintf(s, "L%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_data_from_serial(char *buffer, int buffer_size)
{
	return RS232_PollComport(port_number, (unsigned char*)buffer, buffer_size);
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_attach_sensors_to_stepper_motor(int motor_index, int num_potentiometers, int *potentiometers_index)
{
	char s[64];
	sprintf(s, "AS%d %d", motor_index, num_potentiometers);
	for (int i = 0; i < num_potentiometers; i++) {
		char tmp_str[64];
		sprintf(s, " P%d#", potentiometers_index[i]);
		strcat(s, tmp_str);
	}
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_remove_attached_sensors_from_stepper_motor(int motor_index)
{
	char s[10];
	sprintf(s, "AS%d 0#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_motor_parameters(int motor_index)
{
	char s[10];
	sprintf(s, "GS%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_get_potentiometer_parameters(int potentiometer_index)
{
	char s[10];
	sprintf(s, "GP%d#", potentiometer_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_set_potentiometer_parameters(int potentiometer_index, int _min, int _max, int _home, int _direction)
{
	char s[30];
	sprintf(s, "SP%d %d %d %d %d#", potentiometer_index, _min, _max, _home, _direction);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_is_alive(void)
{
	char s[3];
	strcpy(s, "T#");
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::parse_and_queue_commands(char* tmp_str, int str_length)
{
	int i = 0;
	while (i < str_length) {
		// can be more than 1 command in a string, so I have to check again for a letter
		if ((tmp_str[i] >= 'A' && tmp_str[i] <= 'Z') || (tmp_str[i] >= 'a' && tmp_str[i] <= 'z')) {

			if (tmp_str[i] == 'M' || tmp_str[i] == 'm') {// motor finished movement
				if (tmp_str[i + 1] == 'S' || tmp_str[i + 1] == 's') {// stepper motor finished movement
					int motor_index, distance_to_go;
					sscanf(tmp_str + i + 2, "%d%d", &motor_index, &distance_to_go);
					i += 5;
					jenny5_event *e = new jenny5_event(STEPPER_MOTOR_MOVE_DONE_EVENT, motor_index, distance_to_go, 0);
					received_events.Add((void*)e);
				}
				else				
					if (tmp_str[i + 1] == 'D' || tmp_str[i + 1] == 'd') {// DC motor finished movement
						int motor_index, miliseconds_to_go;
						sscanf(tmp_str + i + 2, "%d%d", &motor_index, &miliseconds_to_go);
						i += 5;
						jenny5_event *e = new jenny5_event(DC_MOTOR_MOVE_DONE_EVENT, motor_index, miliseconds_to_go, 0);
						received_events.Add((void*)e);
					}
					else
						i++;

			}
			else
				if (tmp_str[i] == 'U' || tmp_str[i] == 'u') {//sonar reading returned value
					int sonar_index, distance;
					sscanf(tmp_str + i + 1, "%d%d", &sonar_index, &distance);
					i += 4;
					jenny5_event *e = new jenny5_event(SONAR_EVENT, sonar_index, distance, 0);
					received_events.Add((void*)e);
				}
				else
					if (tmp_str[i] == 'P' || tmp_str[i] == 'p') {//potentiometer reading returned value
						int potentiometer_index, position;
						sscanf(tmp_str + i + 1, "%d%d", &potentiometer_index, &position);
						i += 4;
						jenny5_event *e = new jenny5_event(POTENTIOMETER_EVENT, potentiometer_index, position, 0);
						received_events.Add((void*)e);
					}
					else
						if (tmp_str[i] == 'I' || tmp_str[i] == 'i') {//infrared reading returned value
							int infrared_index, distance;
							sscanf(tmp_str + i + 1, "%d%d", &infrared_index, &distance);
							i += 4;
							jenny5_event *e = new jenny5_event(INFRARED_EVENT, infrared_index, distance, 0);
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
									jenny5_event *e = new jenny5_event(STEPPER_MOTOR_LOCKED_EVENT, motor_index, 0, 0);
									received_events.Add((void*)e);
								}
								else
									if (tmp_str[i] == 'D' || tmp_str[i] == 'd') {// motor was disabled
										int motor_index;
										char motor_type = tmp_str[i + 1];
										sscanf(tmp_str + i + 2, "%d", &motor_index);
										i += 4;
										if (motor_type == 'S' || motor_type == 's') { // stepper was disabled
											jenny5_event *e = new jenny5_event(STEPPER_MOTOR_DISABLED_EVENT, motor_index, 0, 0);
											received_events.Add((void*)e);
										}
										else
											if (motor_type == 'D' || motor_type == 'd') { // dc was disabled
												jenny5_event *e = new jenny5_event(DC_MOTOR_DISABLED_EVENT, motor_index, 0, 0);
												received_events.Add((void*)e);
											}
									}
									else
										if (tmp_str[i] == 'C' || tmp_str[i] == 'c') {// something is created
											if (tmp_str[i + 1] == 'S' || tmp_str[i + 1] == 's') {// stepper motors controller created
												i += 3;
												jenny5_event *e = new jenny5_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0, 0, 0);
												received_events.Add((void*)e);
											}
											else
												if (tmp_str[i + 1] == 'D' || tmp_str[i + 1] == 'd') {// DC motors controller created
													i += 3;
													jenny5_event *e = new jenny5_event(DC_MOTORS_CONTROLLER_CREATED_EVENT, 0, 0, 0);
													received_events.Add((void*)e);
												}
												else
													if (tmp_str[i + 1] == 'U' || tmp_str[i + 1] == 'u') {// sonars controller created
													i += 3;
													jenny5_event *e = new jenny5_event(SONARS_CONTROLLER_CREATED_EVENT, 0, 0, 0);
													received_events.Add((void*)e);
												}
												else
													if (tmp_str[i + 1] == 'P' || tmp_str[i + 1] == 'p') {// potentiometere controller created
														i += 3;
														jenny5_event *e = new jenny5_event(POTENTIOMETERS_CONTROLLER_CREATED_EVENT, 0, 0, 0);
														received_events.Add((void*)e);
													}
													else
														if (tmp_str[i + 1] == 'I' || tmp_str[i + 1] == 'i') {// infrared controller created
															i += 3;
															jenny5_event *e = new jenny5_event(INFRARED_CONTROLLER_CREATED_EVENT, 0, 0, 0);
															received_events.Add((void*)e);
														}
														else
															i++;
										}
										else// not an recognized event
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

		size_t buffer_length = strlen(current_buffer);
		for (size_t i = 0; i < buffer_length; i++)
			if ((current_buffer[i] >= 'A' && current_buffer[i] <= 'Z') || (current_buffer[i] >= 'a' && current_buffer[i] <= 'z')) {// a command
				// find the terminal character #
				size_t j = buffer_length - 1;
				for (; j > i && current_buffer[j] != '#'; j--);// parse until I find the termination char
				if (j > i) {

#ifdef DEBUG
					char tmp_str[64];
					strncpy(tmp_str, current_buffer + i, j - i);
					tmp_str[j - i] = 0;
					printf("current command is= %s", tmp_str);
#endif

					//parse_and_execute_commands(tmp_str, j - i);
					parse_and_queue_commands(current_buffer + i, (int)(j - i + 1));


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
bool t_jenny5_command_module::query_for_event(jenny5_event &event, int available_info)
{
	for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next)
	{
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (available_info == (EVENT_INFO_TYPE | EVENT_INFO_PARAM1 | EVENT_INFO_PARAM2))
		{
			if (e->type == event.type && e->type == event.param1 && e->type == event.param2) {
				event.time = e->time;
				received_events.DeleteCurrent(node_p);
				return true;
			}
		}
		else if (available_info == (EVENT_INFO_TYPE | EVENT_INFO_PARAM1)) 
		{
			if (e->type == event.type && e->param1 == event.param1) {
				event.param2 = e->param2;
				event.time = e->time;
				received_events.DeleteCurrent(node_p);
				return true;
			}
		}
		else if (available_info == EVENT_INFO_TYPE)
		{
			if (e->type == event.type) {
				event.param1 = e->param1;
				event.param2 = e->param2;
				event.time = e->time;
				received_events.DeleteCurrent(node_p);
				return true;
			}
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::wait_for_command_completion(jenny5_event &event, int available_info)
{
	clock_t start_time = clock();
	bool event_success = false;

	while (1) 
	{
		if (!update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		if (!event_success) {
			if (query_for_event(event, available_info))  // have we received the event from Serial ?
				event_success = true;
				//for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
				//	jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
				//	std::cout << (int)e->type << " " << e->param1 << std::endl;
				//}
		}

		if (event_success)
			break;

		// measure the passed time 
		clock_t end_time = clock();
		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;

		// if more than SECONDS_UNTIL_TIMEOUT seconds then game over
		if (wait_time > SECONDS_UNTIL_TIMEOUT) {
			if (!event_success)
			{
				std::cout << "Event with CODE" << event.type << " timed-out.\n";
				std::cout << "Exiting...\n";
				exit(0); //TO_DELETE
			}
		}
	}
	return true;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::query_for_event(int event_type)
{
	for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type) {
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::query_for_event(int event_type, int *param1)
{
	for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type) {
			*param1 = e->param1;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::query_for_event(int event_type, int param1)
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
bool t_jenny5_command_module::query_for_event(int event_type, int param1, int* param2)
{
	for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type && e->param1 == param1) {
			*param2 = e->param2;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_jenny5_command_module::query_for_event(int event_type, int param1, int param2)
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
bool t_jenny5_command_module::query_for_2_events(int event_type1, int param1_1, int event_type2, int param1_2)
{
	bool event1_found = false;
	bool event2_found = false;

	node_double_linked *node_p1 = NULL, *node_p2 = NULL;
	for (node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type1 && e->param1 == param1_1) {
			event1_found = true;
			node_p1 = node_p;
		}
		else
			if (e->type == event_type2 && e->param2 == param1_2) {
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
void t_jenny5_command_module::send_go_home_stepper_motor(int motor_index)
{
	char s[20];
	sprintf(s, "HS%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_stepper_motor_state(int motor_index)
{
	return stepper_motor_state[motor_index];
}
//--------------------------------------------------------------
void t_jenny5_command_module::set_stepper_motor_state(int motor_index, int state)
{
	stepper_motor_state[motor_index] = state;
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_create_stepper_motors(int num_motors, int* dir_pins, int* step_pins, int* enable_pins)
{
	char s[100];
	sprintf(s, "CS %d", num_motors);
	char tmp_s[100];
	for (int i = 0; i < num_motors; i++) {
		sprintf(tmp_s, "%d %d %d", dir_pins[i], step_pins[i], enable_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_create_dc_motors(int num_motors, int *pwm_pins, int* dir1_pins, int* dir2_pins, int* enable_pins)
{
	char s[100];
	sprintf(s, "CD %d", num_motors);
	char tmp_s[100];
	for (int i = 0; i < num_motors; i++) {
		sprintf(tmp_s, "%d %d %d %d", pwm_pins[i], dir1_pins[i], dir2_pins[i], enable_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_create_sonars(int num_sonars, int* trig_pins, int* echo_pins)
{
	char s[100];
	sprintf(s, "CU %d", num_sonars);
	char tmp_s[100];
	for (int i = 0; i < num_sonars; i++) {
		sprintf(tmp_s, "%d %d", trig_pins[i], echo_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_create_potentiometers(int num_potentiometers, int* out_pins, int* _low, int* _high, int *_home)
{
	char s[100];
	sprintf(s, "CP %d", num_potentiometers);
	char tmp_s[100];
	for (int i = 0; i < num_potentiometers; i++) {
		sprintf(tmp_s, "%d %d %d %d", out_pins[i], _low[i], _high[i], _home[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
void t_jenny5_command_module::send_create_infrared_sensors(int num_infrared_sensors, int* _pins, int* _low)
{
	char s[100];
	sprintf(s, "CI %d", num_infrared_sensors);
	char tmp_s[100];
	for (int i = 0; i < num_infrared_sensors; i++) {
		sprintf(tmp_s, "%d %d", _pins[i], _low[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_sonar_state(int sonar_index)
{
	return sonar_state[sonar_index];
}
//--------------------------------------------------------------
void t_jenny5_command_module::set_sonar_state(int sonar_index, int state)
{
	sonar_state[sonar_index] = state;
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_potentiometer_state(int potentiometer_index)
{
	return potentiometer_state[potentiometer_index];
}
//--------------------------------------------------------------
void t_jenny5_command_module::set_potentiometer_state(int potentiometer_index, int new_state)
{
	potentiometer_state[potentiometer_index] = new_state;
}
//--------------------------------------------------------------
int t_jenny5_command_module::get_infrared_state(int infrared_index)
{
	return infrared_state[infrared_index];
}
//--------------------------------------------------------------
void t_jenny5_command_module::set_infrared_state(int infrared_index, int new_state)
{
	infrared_state[infrared_index] = new_state;
}
//--------------------------------------------------------------
// sends (to Arduino) a command for moving a DC motor for a given number of microseconds
void t_jenny5_command_module::send_move_dc_motor(int motor_index, int num_miliseconds)
{
	char s[20];
	sprintf(s, "MD%d %d#", motor_index, num_miliseconds);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
// sends (to Arduino) a command for moving a DC motor to home position
void t_jenny5_command_module::send_go_home_dc_motor(int motor_index)
{
	char s[20];
	sprintf(s, "HD%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
// sends (to Arduino) a command for disabling a DC motor
void t_jenny5_command_module::send_disable_dc_motor(int motor_index)
{
	char s[10];
	sprintf(s, "DD%d#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
// sends (to Arduino) a command for setting the speed of a given DC motor
void t_jenny5_command_module::send_set_dc_motor_speed(int motor_index, int motor_speed)
{
	char s[20];
	sprintf(s, "SD%d %d#", motor_index, motor_speed);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
// sends (to Arduino) a command for attaching several sensors to a given DC motor
void t_jenny5_command_module::send_attach_sensors_to_dc_motor(int motor_index, int num_buttons, int *buttons_index)
{
	char s[64];
	sprintf(s, "AD%d %d", motor_index, num_buttons);
	for (int i = 0; i < num_buttons; i++) {
		char tmp_str[64];
		sprintf(s, " B%d#", buttons_index[i]);
		strcat(s, tmp_str);
	}
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
// sends (to Arduino) a command for reading removing all attached sensors of a motor
void t_jenny5_command_module::send_remove_attached_sensors_from_dc_motor(int motor_index)
{
	char s[10];
	sprintf(s, "AD%d 0#", motor_index);
	RS232_SendBuf(port_number, (unsigned char*)s, (int)strlen(s));
}
//--------------------------------------------------------------
// returns the state of a motor
int t_jenny5_command_module::get_dc_motor_state(int motor_index)
{
	return dc_motor_state[motor_index];
}
//--------------------------------------------------------------
// sets the state of a motor
void t_jenny5_command_module::set_dc_motor_state(int motor_index, int new_state)
{
	dc_motor_state[motor_index] = new_state;
}
//--------------------------------------------------------------