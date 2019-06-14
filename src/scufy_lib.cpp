// Scufy Lib - a serial communication library for Scufy Arduino firmware

// author: Mihai Oltean, 
// email: mihai.oltean@gmail.com
// main website: https://www.jenny5.org
// mirror website: https://jenny5-robot.github.io
// source code: https://github.com/jenny5-robot

// MIT License
// ---------------------------------------------------------------------------


#include "scufy_lib.h"
#include "scufy_events.h"
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define DEBUG_ON

#define MAX_BUFFER_LENGTH 16384

//--------------------------------------------------------------
t_scufy_lib::t_scufy_lib(void)
{
	strcpy(library_version, "2019.06.14.0"); // year.month.day.build number
	current_buffer[0] = 0;

	for (int i = 0; i < MAX_NUM_STEPPER_MOTORS; i++)
		stepper_motor_move_state[i] = COMMAND_DONE;
	for (int i = 0; i < MAX_NUM_DC_MOTORS; i++)
		dc_motor_move_state[i] = COMMAND_DONE;
	for (int i = 0; i < MAX_NUM_ULTRASONIC_HC_SR04_SENSORS; i++)
		ultrasonic_HC_SR04_read_state[i] = COMMAND_DONE;
	for (int i = 0; i < MAX_NUM_POTENTIOMETERS; i++)
		potentiometer_read_state[i] = COMMAND_DONE;
	for (int i = 0; i < MAX_NUM_INFRARED_SENSORS; i++)
		infrared_read_state[i] = COMMAND_DONE;
	for (int i = 0; i < MAX_NUM_AS5147_SENSORS; i++)
		AS5147_read_state[i] = COMMAND_DONE;

	tera_ranger_one_read_state = COMMAND_DONE;

	if (c_serial_new(&m_port, NULL) < 0) {
		fprintf(stderr, "ERROR: Unable to create new serial port\n");
//		return 1;
	}

}
//--------------------------------------------------------------
t_scufy_lib::~t_scufy_lib(void)
{
	c_serial_free(m_port);
}
//--------------------------------------------------------------
const char* t_scufy_lib::get_library_version(void)
{
	return library_version;
}
//--------------------------------------------------------------
bool t_scufy_lib::connect(const char* port, int baud_rate)
{
	if (!c_serial_is_open(m_port)) {

		current_buffer[0] = 0;

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
bool t_scufy_lib::is_open(void)
{
	return c_serial_is_open(m_port);
}
//--------------------------------------------------------------
void t_scufy_lib::close_connection(void)
{
	if (c_serial_is_open(m_port)) {
		c_serial_close(m_port);
		current_buffer[0] = 0;
	}
}
//--------------------------------------------------------------
void t_scufy_lib::send_get_firmware_version(void)
{
	char s[3];
	strcpy(s, "v#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
}
//--------------------------------------------------------------
void t_scufy_lib::send_move_stepper_motor(int motor_index, int num_steps)
{
	char s[20];
	sprintf(s, "SM%d %d#", motor_index, num_steps);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
	stepper_motor_move_state[motor_index] = COMMAND_SENT;

}
//--------------------------------------------------------------
void t_scufy_lib::send_move_stepper_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2)
{
	char s[30];
	sprintf(s, "SM%d %d SM%d %d#", motor_index1, num_steps1, motor_index2, num_steps2);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
	stepper_motor_move_state[motor_index1] = COMMAND_SENT;
	stepper_motor_move_state[motor_index2] = COMMAND_SENT;

}
//--------------------------------------------------------------
void t_scufy_lib::send_move_stepper_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3)
{
	char s[63];
	sprintf(s, "SM%d %d SM%d %d SM%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

	stepper_motor_move_state[motor_index1] = COMMAND_SENT;
	stepper_motor_move_state[motor_index2] = COMMAND_SENT;
	stepper_motor_move_state[motor_index3] = COMMAND_SENT;
}
//--------------------------------------------------------------
void t_scufy_lib::send_move_stepper_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4)
{
	char s[63];
	sprintf(s, "SM%d %d SM%d %d SM%d %d SM%d %d#", motor_index1, num_steps1, motor_index2, num_steps2, motor_index3, num_steps3, motor_index4, num_steps4);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
	stepper_motor_move_state[motor_index1] = COMMAND_SENT;
	stepper_motor_move_state[motor_index2] = COMMAND_SENT;
	stepper_motor_move_state[motor_index3] = COMMAND_SENT;
	stepper_motor_move_state[motor_index4] = COMMAND_SENT;
}
//--------------------------------------------------------------
void t_scufy_lib::send_stop_stepper_motor(int motor_index)
{
	char s[20];
	sprintf(s, "ST%d#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_move_stepper_motor_array(int num_motors, int* motor_index, int *num_steps)
{
	char s[63];
	s[0] = 0;
	for (int i = 0; i < num_motors; i++) {
		char tmp_str[20];

		sprintf(tmp_str, "SM%d %d#", motor_index[i], num_steps[i]);
		strcat(s, tmp_str);
	}
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

	for (int i = 0; i < num_motors; i++)
		stepper_motor_move_state[motor_index[i]] = COMMAND_SENT;
}
//--------------------------------------------------------------
void t_scufy_lib::send_stepper_motor_goto_sensor_position(int motor_index, int sensor_position)
{
	char s[20];
	sprintf(s, "SG%d %d#", motor_index, sensor_position);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_set_stepper_motor_speed_and_acceleration(int motor_index, int motor_speed, int motor_acceleration)
{
	char s[20];
	sprintf(s, "SS%d %d %d#", motor_index, motor_speed, motor_acceleration);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_get_ultrasonic_HC_SR04_distance(int sensor_index)
{
	char s[20];
	sprintf(s, "RU%d#", sensor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_get_button_state(int button_index)
{
	char s[20];
	sprintf(s, "RB%d#", button_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_get_potentiometer_position(int sensor_index)
{
	char s[20];
	sprintf(s, "RP%d#", sensor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_get_AS5147_position(int sensor_index)
{
	char s[20];
	sprintf(s, "RA%d#", sensor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_get_infrared_signal_strength(int sensor_index)
{
	char s[20];
	sprintf(s, "RI%d#", sensor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_get_motors_sensors_statistics(void)
{
	char s[20];
	sprintf(s, "G#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_disable_stepper_motor(int motor_index)
{
	char s[10];
	sprintf(s, "SD%d#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_lock_stepper_motor(int motor_index)
{
	char s[10];
	sprintf(s, "SL%d#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
int t_scufy_lib::get_data_from_serial(unsigned char *buffer, int buffer_size)
{
	int num_available;
	if (c_serial_get_available(m_port, &num_available) != CSERIAL_OK)
		return 0;
	if (num_available){
		buffer_size = MAX_BUFFER_LENGTH;
		c_serial_read_data(m_port, buffer, &buffer_size, &m_lines);
		return buffer_size;

	}
	else
		return 0;
		//RS232_PollComport(port_number, (unsigned char*)buffer, buffer_size);
}
//--------------------------------------------------------------
void t_scufy_lib::send_attach_sensors_to_stepper_motor(int motor_index, 
	int num_potentiometers, int *potentiometers_index,
	int* pot_low, int* pot_high, int *pot_home, int *pot_direction,
	int num_AS5147s, int *AS5147_index,
	int* AS5147_low, int* AS5147_high, int *AS5147_home, int *AS5147_direction,
	int num_infrared, int *infrared_index,
	int num_buttons, int *buttons_index, int *button_direction)
{
	char s[63];
	sprintf(s, "AS%d %d", motor_index, num_potentiometers + num_infrared + num_buttons + num_AS5147s);
// link potentiometers
	for (int i = 0; i < num_potentiometers; i++) {
		char tmp_str[63];
		sprintf(tmp_str, " P%d %d %d %d %d", potentiometers_index[i], pot_low[i], pot_high[i], pot_home[i], pot_direction[i]);
		strcat(s, tmp_str);
	}
// AS5147
	for (int i = 0; i < num_AS5147s; i++) {
		char tmp_str[63];
		sprintf(tmp_str, " A%d %d %d %d %d", AS5147_index[i], AS5147_low[i], AS5147_high[i], AS5147_home[i], AS5147_direction[i]);
		strcat(s, tmp_str);
	}
	// infrared
	for (int i = 0; i < num_infrared; i++) {
		char tmp_str[63];
		sprintf(tmp_str, " I%d", infrared_index[i]);
		strcat(s, tmp_str);
	}
	// buttons
	for (int i = 0; i < num_buttons; i++) {
		char tmp_str[63];
		sprintf(tmp_str, " B%d %d", buttons_index[i], button_direction[i]);
		strcat(s, tmp_str);
	}

	strcat(s, "#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_remove_attached_sensors_from_stepper_motor(int motor_index)
{
	char s[10];
	sprintf(s, "AS%d 0#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_get_motor_parameters(int motor_index)
{
	char s[10];
	sprintf(s, "GS%d#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_get_potentiometer_parameters(int potentiometer_index)
{
	char s[10];
	sprintf(s, "GP%d#", potentiometer_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_set_potentiometer_parameters(int potentiometer_index)
{
	char s[30];
	sprintf(s, "SP%d#", potentiometer_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_is_alive(void)
{
	char s[3];
	strcpy(s, "T#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::parse_and_queue_commands(char* tmp_str, int str_length)
{
	int i = 0;
	while (i < str_length) {
		// can be more than 1 command in a string, so I have to check again for a letter
		if ((tmp_str[i] >= 'A' && tmp_str[i] <= 'Z') || (tmp_str[i] >= 'a' && tmp_str[i] <= 'z')) {

			if (tmp_str[i] == 'S' || tmp_str[i] == 's') {// stepper motor finished movement
				if (tmp_str[i + 1] == 'M' || tmp_str[i + 1] == 'm') {// stepper motor finished movement
					int motor_index, distance_to_go, num_consumed;
					sscanf(tmp_str + i + 2, "%d%d%n", &motor_index, &distance_to_go, &num_consumed);
					i += 2 + num_consumed;
					jenny5_event *e = new jenny5_event(STEPPER_MOTOR_MOVE_DONE_EVENT, motor_index, distance_to_go, 0);
					received_events.Add((void*)e);
					printf("Distance to go = %d\n", distance_to_go);
				}
				else
					if (tmp_str[i + 1] == 'L' || tmp_str[i + 1] == 'l') {// motor was locked
						int motor_index;
						sscanf(tmp_str + i + 2, "%d", &motor_index);
						i += 4;
						jenny5_event *e = new jenny5_event(STEPPER_MOTOR_LOCKED_EVENT, motor_index, 0, 0);
						received_events.Add((void*)e);
					}
					else
						if (tmp_str[i + 1] == 'D' || tmp_str[i + 1] == 'd') {// motor was disabled
							int motor_index;
							sscanf(tmp_str + i + 2, "%d", &motor_index);
							i += 4;

							jenny5_event *e = new jenny5_event(STEPPER_MOTOR_DISABLED_EVENT, motor_index, 0, 0);
							received_events.Add((void*)e);
						}
						else
							if (tmp_str[i + 1] == 'S' || tmp_str[i + 1] == 's') {// set speed and acceleration
								int motor_index;
								sscanf(tmp_str + i + 2, "%d", &motor_index);
								i += 4;

								jenny5_event *e = new jenny5_event(STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT, motor_index, 0, 0);
								received_events.Add((void*)e);
							}
							else
								if (tmp_str[i + 1] == 'T' || tmp_str[i + 1] == 't') {// stepper stopped
									int motor_index;
									sscanf(tmp_str + i + 2, "%d", &motor_index);
									i += 4;

									jenny5_event *e = new jenny5_event(STEPPER_STOPPED_EVENT, motor_index, 0, 0);
									received_events.Add((void*)e);
									printf("Stepper stopped.\n");
								}
							
								
							

				/*
				if (tmp_str[i + 1] == 'D' || tmp_str[i + 1] == 'd') {// DC motor finished movement
					int motor_index, miliseconds_to_go;
					sscanf(tmp_str + i + 2, "%d%d", &motor_index, &miliseconds_to_go);
					i += 5;
					jenny5_event *e = new jenny5_event(DC_MOTOR_MOVE_DONE_EVENT, motor_index, miliseconds_to_go, 0);
					received_events.Add((void*)e);
				}
				*/
							else
								i++;

			}
			else
				if (tmp_str[i] == 'R' || tmp_str[i] == 'r') { // reads something
					if (tmp_str[i + 1] == 'U' || tmp_str[i + 1] == 'u') {//ultrasonic reading returned value
						int ultrasonic_index, distance;
						sscanf(tmp_str + i + 2, "%d%d", &ultrasonic_index, &distance);
						i += 5;
						jenny5_event *e = new jenny5_event(ULTRASONIC_EVENT, ultrasonic_index, distance, 0);
						received_events.Add((void*)e);
					}
					else
						if (tmp_str[i + 1] == 'P' || tmp_str[i + 1] == 'p') {//potentiometer reading returned value
							int potentiometer_index, position;
							sscanf(tmp_str + i + 2, "%d%d", &potentiometer_index, &position);
							i += 5;
							jenny5_event *e = new jenny5_event(POTENTIOMETER_EVENT, potentiometer_index, position, 0);
							received_events.Add((void*)e);
						}
						else
							if (tmp_str[i + 1] == 'A' || tmp_str[i + 1] == 'a') {//AS5147 reading returned value
								int as5147_index, position;
								sscanf(tmp_str + i + 2, "%d%d", &as5147_index, &position);
								i += 5;
								jenny5_event *e = new jenny5_event(AS5147_EVENT, as5147_index, position, 0);
								received_events.Add((void*)e);
							}
							else
								if (tmp_str[i + 1] == 'I' || tmp_str[i + 1] == 'i') {//infrared reading returned value
									int infrared_index, distance;
									sscanf(tmp_str + i + 2, "%d%d", &infrared_index, &distance);
									i += 5;
									jenny5_event *e = new jenny5_event(INFRARED_EVENT, infrared_index, distance, 0);
									received_events.Add((void*)e);
								}
								else
									if (tmp_str[i + 1] == 'B' || tmp_str[i + 1] == 'b') {//button state
										int button_index, button_state;
										sscanf(tmp_str + i + 2, "%d%d", &button_index, &button_state);
										i += 5;
										jenny5_event *e = new jenny5_event(BUTTON_EVENT, button_index, button_state, 0);
										received_events.Add((void*)e);
									}
									else
										if (tmp_str[i + 1] == 'T' || tmp_str[i + 1] == 't') { // tera ranger one
											int distance;
											int num_read;
											sscanf(tmp_str + i + 2, "%d%n", &distance, &num_read);
											jenny5_event *e = new jenny5_event(TERA_RANGER_ONE_EVENT, distance, 0, 0);
											received_events.Add((void*)e);
											i += 2 + num_read + 1;
										}
										else
											if (tmp_str[i + 1] == 'M' || tmp_str[i + 1] == 'm') {// version number
											//scan until #

												int num_consumed;
												int free_memory;
												sscanf(tmp_str + i + 2, "%d%n", &free_memory, &num_consumed);
												i += 2 + num_consumed + 1;
												jenny5_event *e = new jenny5_event(FREE_MEMORY_EVENT, free_memory, 0, 0);
												received_events.Add((void*)e);

											}
				}// end read something
				else
					if (tmp_str[i] == 'T' || tmp_str[i] == 't') {// test connection
						jenny5_event *e = new jenny5_event(IS_ALIVE_EVENT, 0, 0, 0);
						received_events.Add((void*)e);
						i += 2;
					}
					else
						if (tmp_str[i] == 'L' || tmp_str[i] == 'l') {//LIDAR reading returned value
									int motor_position, distance;
									int num_consumed;
									sscanf(tmp_str + i + 1, "%d%d%n", &motor_position, &distance, &num_consumed);
									i += 2 + num_consumed;
									jenny5_event *e = new jenny5_event(LIDAR_READ_EVENT, motor_position, distance, 0);
									received_events.Add((void*)e);
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
												if (tmp_str[i + 1] == 'U' || tmp_str[i + 1] == 'u') {// ultrasonics controller created
													i += 3;
													jenny5_event *e = new jenny5_event(ULTRASONICS_CONTROLLER_CREATED_EVENT, 0, 0, 0);
													received_events.Add((void*)e);
												}
												else
													if (tmp_str[i + 1] == 'P' || tmp_str[i + 1] == 'p') {// potentiometers controller created
														i += 3;
														jenny5_event *e = new jenny5_event(POTENTIOMETERS_CONTROLLER_CREATED_EVENT, 0, 0, 0);
														received_events.Add((void*)e);
													}
													else
														if (tmp_str[i + 1] == 'A' || tmp_str[i + 1] == 'a') {// as5147 controller created
															i += 3;
															jenny5_event *e = new jenny5_event(AS5147S_CONTROLLER_CREATED_EVENT, 0, 0, 0);
															received_events.Add((void*)e);
														}
														else
															if (tmp_str[i + 1] == 'I' || tmp_str[i + 1] == 'i') {// infrared controller created
															i += 3;
															jenny5_event *e = new jenny5_event(INFRARED_CONTROLLER_CREATED_EVENT, 0, 0, 0);
															received_events.Add((void*)e);
														}
														else
															if (tmp_str[i + 1] == 'B' || tmp_str[i + 1] == 'b') {// infrared controller created
																i += 3;
																jenny5_event *e = new jenny5_event(BUTTONS_CONTROLLER_CREATED_EVENT, 0, 0, 0);
																received_events.Add((void*)e);
															}
															else
																if (tmp_str[i + 1] == 'T' || tmp_str[i + 1] == 't') {// tera ranger one controller created
																	i += 3;
																	jenny5_event *e = new jenny5_event(TERA_RANGER_ONE_CONTROLLER_CREATED_EVENT, 0, 0, 0);
																	received_events.Add((void*)e);
																}
																else
																	if (tmp_str[i + 1] == 'L' || tmp_str[i + 1] == 'l') {// LIDAR controller created
																		i += 3;
																		jenny5_event *e = new jenny5_event(LIDAR_CONTROLLER_CREATED_EVENT, 0, 0, 0);
																		received_events.Add((void*)e);
																	}
																	else
																		i++;
									}
							else
								if (tmp_str[i] == 'V' || tmp_str[i] == 'v') {// version number
											//scan until #
											char *stop_index = strchr(tmp_str + i, '#');
											char *s_version = new char[stop_index - (tmp_str + i + 1) + 1];
											strncpy(s_version, tmp_str + i + 1, stop_index - (tmp_str + i + 1));
											s_version[stop_index - (tmp_str + i + 1)] = 0;
											jenny5_event *e = new jenny5_event(ARDUINO_FIRMWARE_VERSION_EVENT, 0, (intptr_t)s_version, 0);
											received_events.Add((void*)e);
											i += stop_index - (tmp_str + i + 1) + 2;
										}
								else
									if (tmp_str[i] == 'I' || tmp_str[i] == 'i') {// information from firmware
											//scan until #
												char *stop_index = strchr(tmp_str + i, '#');
												if (stop_index) {
													char *s_info = new char[stop_index - (tmp_str + i + 1) + 1];
													strncpy(s_info, tmp_str + i + 1, stop_index - (tmp_str + i + 1));
													s_info[stop_index - (tmp_str + i + 1)] = 0;
													//jenny5_event *e = new jenny5_event(INFO_EVENT, 0, (intptr_t)s_info, 0);
													//received_events.Add((void*)e);
													i += stop_index - (tmp_str + i + 1) + 2;
													printf("INFO from firmware = %s\n", s_info);
												}
											}
									else
										if (tmp_str[i] == 'E' || tmp_str[i] == 'e') {// information from firmware
												//scan until #
											char *stop_index = strchr(tmp_str + i, '#');
											//char *s_info = new char[stop_index - (tmp_str + i + 1) + 1];
											//strncpy(s_info, tmp_str + i + 1, stop_index - (tmp_str + i + 1));
											//s_info[stop_index - (tmp_str + i + 1)] = 0;
											//jenny5_event *e = new jenny5_event(INFO_EVENT, 0, (intptr_t)s_info, 0);
											//received_events.Add((void*)e);
											i += stop_index - (tmp_str + i + 1) + 2;
											printf("Error from firmware.\n");
										}
										else
											if (tmp_str[i] == 'A' || tmp_str[i] == 'a') {//attach
												int motor_index;
												int num_consumed;
												sscanf(tmp_str + i + 1, "%d%n", &motor_index, &num_consumed);
												i += 2 + num_consumed;
												jenny5_event *e = new jenny5_event(ATTACH_SENSORS_EVENT, motor_index, 0);
												received_events.Add((void*)e);
											}
											else// not an recognized event// not an recognized event
												i++;
			// more events to add
		}
		else
			i++;
	}
}
//--------------------------------------------------------------
bool t_scufy_lib::update_commands_from_serial(void)
{
	// the same code as in firmware
	unsigned char tmp_buffer[MAX_BUFFER_LENGTH];
	int received_size = get_data_from_serial(tmp_buffer, MAX_BUFFER_LENGTH);
	if (received_size >= MAX_BUFFER_LENGTH)
		return false;
	tmp_buffer[received_size] = 0;
	if (received_size) {
		strcpy(current_buffer + strlen(current_buffer), (char*)tmp_buffer);
		//	printf("%s\n", current_buffer);

		size_t buffer_length = strlen(current_buffer);
		for (size_t i = 0; i < buffer_length; i++)
			if ((current_buffer[i] >= 'A' && current_buffer[i] <= 'Z') || (current_buffer[i] >= 'a' && current_buffer[i] <= 'z')) {// a command
				// find the terminal character #
				size_t j = buffer_length - 1;
				for (; j > i && current_buffer[j] != '#'; j--);// parse until I find the termination other commands
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
void t_scufy_lib::clear_events_list(void)
{
	while (received_events.head){
		jenny5_event* e = (jenny5_event*)received_events.GetHeadInfo();
		delete e;
		received_events.DeleteHead();
	}
}
//--------------------------------------------------------------
bool t_scufy_lib::query_for_event(int event_type)
{
	for (t_node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type) {
			delete e;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_scufy_lib::query_for_firmware_version_event(char *arduino_firmware_version)
{
	for (t_node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == ARDUINO_FIRMWARE_VERSION_EVENT) { // test for firmware version event because that one contains a string
			strcpy(arduino_firmware_version, (char*)e->param2);
			delete[] (char*)e->param2;
			
			delete e;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_scufy_lib::query_for_event(int event_type, int *param1)
{
	for (t_node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type) {
			*param1 = e->param1;
			delete e;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_scufy_lib::query_for_event(int event_type, int *param1, intptr_t *param2)
{
	for (t_node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type) {
			*param1 = e->param1;
			*param2 = e->param2;
			delete e;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_scufy_lib::query_for_event(int event_type, int param1)
{
	for (t_node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type && e->param1 == param1) {
			delete e;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_scufy_lib::query_for_event(int event_type, int param1, intptr_t* param2)
{
	for (t_node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type && e->param1 == param1) {
			*param2 = e->param2;
			delete e;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_scufy_lib::query_for_event(int event_type, int param1, int param2)
{
	for (t_node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p);
		if (e->type == event_type && e->param1 == param1 && e->param2 == param2) {
			delete e;
			received_events.DeleteCurrent(node_p);
			return true;
		}
	}
	return false;
}
//--------------------------------------------------------------
bool t_scufy_lib::query_for_2_events(int event_type1, int param1_1, int event_type2, int param1_2)
{
	bool event1_found = false;
	bool event2_found = false;

	t_node_double_linked *node_p1 = NULL, *node_p2 = NULL;
	for (t_node_double_linked *node_p = received_events.head; node_p; node_p = node_p->next) {
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
		jenny5_event* e = (jenny5_event*)received_events.GetCurrentInfo(node_p1);
		delete e;
		e = (jenny5_event*)received_events.GetCurrentInfo(node_p2);
		delete e;
		received_events.DeleteCurrent(node_p1);
		received_events.DeleteCurrent(node_p2);
		return true;
	}
	else
		return false;
}
//--------------------------------------------------------------
void t_scufy_lib::send_go_home_stepper_motor(int motor_index)
{
	char s[20];
	sprintf(s, "SH%d#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
int t_scufy_lib::get_stepper_motor_state(int motor_index)
{
	return stepper_motor_move_state[motor_index];
}
//--------------------------------------------------------------
void t_scufy_lib::set_stepper_motor_state(int motor_index, int state)
{
	stepper_motor_move_state[motor_index] = state;
}
//--------------------------------------------------------------
void t_scufy_lib::send_create_stepper_motors(int num_motors, int* dir_pins, int* step_pins, int* enable_pins)
{
	char s[63];
	sprintf(s, "CS %d", num_motors);
	char tmp_s[100];
	for (int i = 0; i < num_motors; i++) {
		sprintf(tmp_s, "%d %d %d", dir_pins[i], step_pins[i], enable_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_create_dc_motors(int num_motors, int *pwm_pins, int* dir1_pins, int* dir2_pins, int* enable_pins)
{
	char s[63];
	sprintf(s, "CD %d", num_motors);
	char tmp_s[100];
	for (int i = 0; i < num_motors; i++) {
		sprintf(tmp_s, "%d %d %d %d", pwm_pins[i], dir1_pins[i], dir2_pins[i], enable_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_create_ultrasonics_HC_SR04(int num_ultrasonics, int* trig_pins, int* echo_pins)
{
	char s[63];
	sprintf(s, "CU %d", num_ultrasonics);
	char tmp_s[100];
	for (int i = 0; i < num_ultrasonics; i++) {
		sprintf(tmp_s, "%d %d", trig_pins[i], echo_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_create_potentiometers(int num_potentiometers, int* out_pins)
{
	char s[63];
	sprintf(s, "CP %d", num_potentiometers);
	char tmp_s[100];
	for (int i = 0; i < num_potentiometers; i++) {
		sprintf(tmp_s, "%d", out_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_create_as5147s(int num_as5147s, int* out_pins)
{
	char s[63];
	sprintf(s, "CA %d", num_as5147s);
	char tmp_s[100];
	for (int i = 0; i < num_as5147s; i++) {
		sprintf(tmp_s, "%d", out_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_create_infrared_sensors(int num_infrared_sensors, int* _out_pins)
{
	char s[63];
	sprintf(s, "CI %d", num_infrared_sensors);
	char tmp_s[100];
	for (int i = 0; i < num_infrared_sensors; i++) {
		sprintf(tmp_s, "%d", _out_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_create_buttons(int num_buttons, int* out_pins)
{
	char s[63];
	sprintf(s, "CB %d", num_buttons);
	char tmp_s[100];
	for (int i = 0; i < num_buttons; i++) {
		sprintf(tmp_s, "%d", out_pins[i]);
		strcat(s, " ");
		strcat(s, tmp_s);
	}
	strcat(s, "#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
int t_scufy_lib::get_ultrasonic_HC_SR04_state(int ultrasonic_index)
{
	return ultrasonic_HC_SR04_read_state[ultrasonic_index];
}
//--------------------------------------------------------------
void t_scufy_lib::set_ultrasonic_HC_SR04_state(int ultrasonic_index, int state)
{
	ultrasonic_HC_SR04_read_state[ultrasonic_index] = state;
}
//--------------------------------------------------------------
int t_scufy_lib::get_potentiometer_state(int potentiometer_index)
{
	return potentiometer_read_state[potentiometer_index];
}
//--------------------------------------------------------------
void t_scufy_lib::set_potentiometer_state(int potentiometer_index, int new_state)
{
	potentiometer_read_state[potentiometer_index] = new_state;
}
//--------------------------------------------------------------
int t_scufy_lib::get_infrared_state(int infrared_index)
{
	return infrared_read_state[infrared_index];
}
//--------------------------------------------------------------
void t_scufy_lib::set_infrared_state(int infrared_index, int new_state)
{
	infrared_read_state[infrared_index] = new_state;
}
//--------------------------------------------------------------
int t_scufy_lib::get_AS5147_state(int AS5147_index)
{
	return AS5147_read_state[AS5147_index];
}
//--------------------------------------------------------------
void t_scufy_lib::set_AS5147_state(int AS5147_index, int new_state)
{
	AS5147_read_state[AS5147_index] = new_state;
}
//--------------------------------------------------------------
// sends (to Arduino) a command for moving a DC motor for a given number of microseconds
void t_scufy_lib::send_move_dc_motor(int motor_index, int num_miliseconds)
{
	char s[20];
	sprintf(s, "MD%d %d#", motor_index, num_miliseconds);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
// sends (to Arduino) a command for moving a DC motor to home position
void t_scufy_lib::send_go_home_dc_motor(int motor_index)
{
	char s[20];
	sprintf(s, "HD%d#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
// sends (to Arduino) a command for disabling a DC motor
void t_scufy_lib::send_disable_dc_motor(int motor_index)
{
	char s[10];
	sprintf(s, "DD%d#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
// sends (to Arduino) a command for setting the speed of a given DC motor
void t_scufy_lib::send_set_dc_motor_speed(int motor_index, int motor_speed)
{
	char s[20];
	sprintf(s, "SD%d %d#", motor_index, motor_speed);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
// sends (to Arduino) a command for attaching several sensors to a given DC motor
void t_scufy_lib::send_attach_sensors_to_dc_motor(int motor_index, int num_buttons, int *buttons_index)
{
	char s[64];
	sprintf(s, "AD%d %d", motor_index, num_buttons);
	for (int i = 0; i < num_buttons; i++) {
		char tmp_str[64];
		sprintf(tmp_str, " B%d#", buttons_index[i]);
		strcat(s, tmp_str);
	}
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
// sends (to Arduino) a command for reading removing all attached sensors of a motor
void t_scufy_lib::send_remove_attached_sensors_from_dc_motor(int motor_index)
{
	char s[10];
	sprintf(s, "AD%d 0#", motor_index);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
// returns the state of a motor
int t_scufy_lib::get_dc_motor_state(int motor_index)
{
	return dc_motor_move_state[motor_index];
}
//--------------------------------------------------------------
// sets the state of a motor
void t_scufy_lib::set_dc_motor_state(int motor_index, int new_state)
{
	dc_motor_move_state[motor_index] = new_state;
}
//--------------------------------------------------------------
void t_scufy_lib::send_create_tera_ranger_one(void)
{
	char s[10];
	sprintf(s, "CT#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------
void t_scufy_lib::send_create_LiDAR(int dir_pin, int step_pin, int enable_pin, int ir_pin)
{
	char s[30];
	sprintf(s, "CL %d %d %d %d#", dir_pin, step_pin, enable_pin, ir_pin);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif

}
//--------------------------------------------------------------

// returns the state of the tera ranger one sensor
int t_scufy_lib::get_tera_ranger_one_state(void)
{
	return tera_ranger_one_read_state;
}
//--------------------------------------------------------------
// sets the state of the tera ranger one sensor
void t_scufy_lib::set_tera_ranger_one_state(int new_state)
{
	tera_ranger_one_read_state = new_state;
}
//--------------------------------------------------------------
// sends (to Arduino) a command for reading the Tera Ranger One sensor
void t_scufy_lib::send_get_tera_ranger_one_distance(void)
{
	char s[10];
	sprintf(s, "TR#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
}
//--------------------------------------------------------------
void t_scufy_lib::send_LiDAR_go(void)
{
	char s[10];
	sprintf(s, "LG#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
}
//--------------------------------------------------------------
void t_scufy_lib::send_LiDAR_stop(void)
{
	char s[10];
	sprintf(s, "LH#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
}
//--------------------------------------------------------------
void t_scufy_lib::send_set_LiDAR_motor_speed_and_acceleration(int motor_speed, int motor_acceleration)
{
	char s[20];
	sprintf(s, "LS %d %d#", motor_speed, motor_acceleration);
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
}
//--------------------------------------------------------------
void t_scufy_lib::send_get_free_memory(void)
{
	char s[20];
	sprintf(s, "RM#");
	int data_length = strlen(s);
	c_serial_write_data(m_port, (unsigned char*)s, &data_length);
#ifdef DEBUG_ON
	printf("%s\n", s);
#endif
}
//--------------------------------------------------------------
int t_scufy_lib::get_free_memory(void)
{
	send_get_free_memory();

	clock_t start_time = clock();
	bool free_memory_set = false;

	int free_memory = 0;

	while (1) {
		if (!update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor
		if (query_for_event(FREE_MEMORY_EVENT, &free_memory)) {  // have we received the event from Serial ?
			free_memory_set = true;
			break;
		}

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 1 seconds then game over
		if (wait_time > 1) {
			break;
		}
	}
	return free_memory;
}
//--------------------------------------------------------------


