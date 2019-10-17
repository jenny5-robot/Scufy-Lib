// Author: Mihai Oltean, https://mihaioltean.github.io, mihai.oltean@gmail.com
// More details: https://jenny5.org, https://jenny5-robot.github.io/
// Source code: github.com/jenny5-robot
// License: MIT
// ---------------------------------------------------------------------------

#ifndef scufy_lib_H
#define scufy_lib_H


#include "lista_voidp.h"
#include "scufy_events.h"
#include "c_serial.h"

#include <iostream>
#include <time.h>

#define COMMAND_NOT_SENT 0
#define COMMAND_SENT 1
#define COMMAND_DONE 2

#define MAX_NUM_STEPPER_MOTORS 6
#define MAX_NUM_DC_MOTORS_TB6612FNG 3
#define MAX_NUM_ULTRASONIC_HC_SR04_SENSORS 6
#define MAX_NUM_POTENTIOMETERS 6
#define MAX_NUM_AS5147_SENSORS 6
#define MAX_NUM_INFRARED_SENSORS 6


//----------------------------------------------------------------
class t_scufy_lib{
private:
	// version number of the library
	char library_version[20];

	// port number of the serial connection
	c_serial_port_t* m_port;
	c_serial_control_lines_t m_lines;
	
	// a list with received events from Arduino
	t_lista received_events;

	// current buffer of characters received from Arduino
	char current_buffer[4096]; 

	// each motor can be in one of 2 states: COMMAND_DONE and COMMAND_SENT
	int stepper_motor_move_state[MAX_NUM_STEPPER_MOTORS]; // max 6 motors 

	// each dc motor can be in one of 2 states: COMMAND_DONE and COMMAND_SENT
	int dc_motor_TB6612FNG_move_state[MAX_NUM_DC_MOTORS_TB6612FNG]; // max dc 3 motors (each motor occupy 4 digital pins ... so 4x3 = 12 digital pins = which Arduino Nano has)

	int ultrasonic_HC_SR04_read_state[MAX_NUM_ULTRASONIC_HC_SR04_SENSORS]; // max HC-SR04 6 ultrasounds (each ultrasonic occupy 2 digital pins ... so 6x2 = 12 digital pins = which Arduino Nano has)

	int potentiometer_read_state[MAX_NUM_POTENTIOMETERS]; // if I have max 6 motors, the number of potentiometers is not higher because each potentiometer is attached to 1 motor

	int AS5147_read_state[MAX_NUM_POTENTIOMETERS]; // if I have max 6 motors, the number of potentiometers is not higher because each potentiometer is attached to 1 motor

	int infrared_read_state[MAX_NUM_INFRARED_SENSORS]; // I suppose that I don't have so many infrared sensors attached to 1 controller

	int tera_ranger_one_read_state;

	// parse the string for events
	void parse_and_queue_commands(char* tmp_str, int str_length);

	// gets an unformated string of chars from serial
	// should be used only in extreme cases
	// normally an application must call update_commands_from_serial
	int get_data_from_serial(unsigned char* buffer, int buffer_size);

public:
	t_scufy_lib(void);
	~t_scufy_lib(void);

	// connects to given serial port
	bool connect(const char* port, int baud_rate);

	// test if the connection is open; For testing if the Arduino is alive please send_is_alive method and wait for IS_ALIVE_EVENT event
	bool is_open(void);
	
	// close serial connection
	void close_connection(void);
	
	// returns a string containing the version number of this library
	const char* get_library_version(void);
	
	// send a command to Arduino for obtaining the version number of the Scufy firmware 
	void send_get_firmware_version(void);

	// reads data from serial and updates the list of received events from Arduino
	// this should be called frequently from the main loop of the program in order to read the data received from Arduino
	bool update_commands_from_serial(void);

	// clear the list of received events
	void clear_events_list(void);

	// ---------------------QUERY LIST of COMMANDS ---------------


	// search in the list of events for a particular event type
	// it returns true if the event is found in list
	// the first occurrence of the event is removed from the list
	bool query_for_event(int event_type);
	
	// search in the list of events for a firmware version event type
	// it returns true if the event is found in list
	bool query_for_firmware_version_event(char *firmware_version);
		
	// search in the list of events for a particular event type
	// it returns true if the event is found in list
	// param1 parameter will be set to the information from the param1 member of the event
	bool query_for_event(int event_type, int* param1);

	// search in the list of events for a particular event type
	// it returns true if the event is found in list
	// param1 parameter will be set to the information from the param1 member of the event
	// param2 parameter will be set to the information from the param2 member of the event
	bool query_for_event(int event_type, int *param1, intptr_t *param2);

	// search in the list of events for a particular event type
	// returns true if the event type matches and if the param1 member is equal to the parameter given to this function
	bool query_for_event(int event_type, int param1);

	// search in the list of events for a particular event type
	// returns true if the event type matches and if the param1 member is equal to the parameter given to this function
	// param2 parameter will be set to the information from the param2 member of the event
	bool query_for_event(int event_type, int param1, intptr_t *param2);

	// search in the list of events for two event types
	// returns true only if both events are found
	bool query_for_2_events(int event_type1, int param1_1, int event_type2, int param1_2);

	// search in the list of events for a particular event type
	bool query_for_event(int event_type, int param1, int param2);

	// -------------- CREATE COMMANDS ----------------
	// sends (to Arduino) a command for creating a stepper motor controller
	// several arrays of pin indexes for direction, step and enable must be specified
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memory
	void send_create_stepper_motors(int num_motors, int* dir_pins, int* step_pins, int* enable_pins);
	
	// sends (to Arduino) a command for creating a DC motor controller (by using TB6612FNG board)
	// several arrays of pin indexes for direction, step and enable must be specified
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memory
	void send_create_dc_motors_TB6612FNG(int num_motors, int *pwm_pins, int* dir1_pins, int* dir2_pins, int* enable_pins);

	// sends (to Arduino) a command for creating a ultrasonic controller
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memory
	void send_create_ultrasonics_HC_SR04(int num_ultrasonics, int* trig_pins, int* echo_pins);

	// sends (to Arduino) a command for creating a potentiometer controller
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memory
	void send_create_potentiometers(int num_potentiometers, int* out_pins);

	// sends (to Arduino) a command for creating a as5147s controller
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memory
	void send_create_as5147s(int num_as5147s, int* out_pins);

	// sends (to Arduino) a command for creating an infrared controller
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memory
	void send_create_infrared_sensors(int num_infrared_sensors, int* out_pins);

	// sends (to Arduino) a command for creating a buttons controller
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memory
	void send_create_buttons(int num_buttons_sensors, int* out_pins);

	// sends (to Arduino) a command for creating a Tera Ranger One controller
	// this method should be called once at the beginning of the program
	// only one sensor is permitted per Arduino board
	void send_create_tera_ranger_one(void);

	// sends (to Arduino) a command for creating a Tera Ranger One LiDAR controller
	// this method should be called once at the beginning of the program
	// only one sensor is permitted per Arduino board
	// only 1 LiDAR per Arduino board is possible due to the use of SDA and SLC pins for the signal
	void send_create_LiDAR(int dir_pin, int step_pin, int enable_pin, int ir_pin);

	// sends (to Arduino) a command (T#) for testing if the connection is alive
	// when the Arduino will respond, the event will be added in the list
	void send_is_alive(void);

	// ------------------ STEPPER MOTORS MOVEMENT --------------------
	// sends (to Arduino) a command for moving a stepper motor to home position
	void send_go_home_stepper_motor(int motor_index);

	// sends (to Arduino) a command for moving a motor with a given number of steps
	void send_move_stepper_motor(int motor_index, int num_steps);
	
	// sends (to Arduino) a command for moving two motors
	void send_move_stepper_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2);
	
	// sends (to Arduino) a command for moving three motors
	void send_move_stepper_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3);
	
	// sends (to Arduino) a command for moving four motors
	void send_move_stepper_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4);
	
	// sends (to Arduino) a command for moving multiple motors
	void send_move_stepper_motor_array(int num_motors, int* motor_index, int *num_steps);

	// sends (to Arduino) a command for stopping a stepper motor
	void send_stop_stepper_motor(int motor_index);

	// sends (to Arduino) a command for moving a motor to a new sensor position
	void send_stepper_motor_goto_sensor_position(int motor_index, int sensor_position);

	// sends (to Arduino) a command for blocking a motor to current position
	void send_lock_stepper_motor(int motor_index);
	
	// sends (to Arduino) a command for disabling a motor
	void send_disable_stepper_motor(int motor_index);

	// sends (to Arduino) a command for setting the speed and acceleration of a given motor
	void send_set_stepper_motor_speed_and_acceleration(int motor_index, int motor_speed, int motor_acceleration);
	
	// sends (to Arduino) a command for attaching several sensors to a given motor
	void send_attach_sensors_to_stepper_motor(int motor_index, 
		int num_potentiometers, int *potentiometers_index, 
		int* _low, int* _high, int *home, int *_direction, 
		int num_AS5147s, int *AS5147_index,
		int* AS5147_low, int* AS5147_high, int *AS5147_home, int *AS5147_direction,
		int num_infrared, int *infrared_index,
		int num_buttons, int *buttons_index, int *button_direction
	);

	// sends (to Arduino) a command for reading removing all attached sensors of a motor
	void send_remove_attached_sensors_from_stepper_motor(int motor_index);

	// returns the state of a motor
	int get_stepper_motor_state(int motor_index);

	// sets the state of a motor
	void set_stepper_motor_state(int motor_index, int new_state);

	// ---------------------- DC motors ----------------------
	// sends (to Arduino) a command for moving a DC motor for a given number of microseconds
	void send_move_dc_motor_TB6612FNG(int motor_index, int num_miliseconds);
	// sends (to Arduino) a command for moving a DC motor to home position
	void send_go_home_dc_motor_TB6612FNG(int motor_index);
	// sends (to Arduino) a command for disabling a DC motor
	void send_disable_dc_motor_TB6612FNG(int motor_index);
	// sends (to Arduino) a command for setting the speed of a given DC motor
	void send_set_dc_motor_speed_TB6612FNG(int motor_index, int motor_speed);
	// sends (to Arduino) a command for attaching several sensors to a given DC motor
	void send_attach_sensors_to_dc_motor_TB6612FNG(int motor_index, int num_buttons, int *buttons_index);
	// sends (to Arduino) a command for reading removing all attached sensors of a motor
	void send_remove_attached_sensors_from_dc_motor_TB6612FNG(int motor_index);
	// returns the state of a motor
	int get_dc_motor_state_TB6612FNG(int motor_index);
	// sets the state of a motor
	void set_dc_motor_state_TB6612FNG(int motor_index, int new_state);


	// ----------------------- SENSORS ----------------------
	// sends (to Arduino) a command for reading a HC_SR04 ultrasonic
	void send_get_ultrasonic_HC_SR04_distance(int sensor_index);
	
	// sends (to Arduino) a command for a button state
	void send_get_button_state(int button_index);

	// sends (to Arduino) a command for reading a potentiometer position
	void send_get_potentiometer_position(int sensor_index);

	// sends (to Arduino) a command for reading a AS5147 position
	void send_get_AS5147_position(int sensor_index);

	// sends (to Arduino) a command for reading a infrared value
	void send_get_infrared_signal_strength(int sensor_index);

	// sends (to Arduino) a command for reading the Tera Ranger One sensor
	void send_get_tera_ranger_one_distance(void);

	// sends (to Arduino) a command for starting the LiDAR rotation
	void send_LiDAR_go(void);

	// sends (to Arduino) a command for stopping LiDAR rotation
	void send_LiDAR_stop(void);

	// sends (to Arduino) a command for setting the speed and acceleration of the LiDAR motor
	void send_set_LiDAR_motor_speed_and_acceleration(int motor_speed, int motor_acceleration);

	// ---------------------- DEBUG ---------------------------
	// sends (to Arduino) a command for reading parameters of a motor; debug purposes
	void send_get_motors_sensors_statistics(void);

	// sends (to Arduino) a command for reading parameters of a motor
	void send_get_motor_parameters(int motor_index);

	// sends (to Arduino) a command for reading parameters of a potentiometer
	void send_get_potentiometer_parameters(int potentiometer_index);

	// sends (to Arduino) a command for setting parameters of a potentiometer
	void send_set_potentiometer_parameters(int potentiometer_index);

	// sends a command for reading the size of free memory of Arduino
	void send_get_free_memory(void);

	// returns the size of the free memory from Arduino. 
	// This method will not return until will not receive the event from Arduino or timeout
	int get_free_memory(void);
	

	//  ----------------------- STATE -----------------------
	// returns the state of a ultrasonic
	int get_ultrasonic_HC_SR04_state(int ultrasonic_index);

	// sets the state of a ultrasonic
	void set_ultrasonic_HC_SR04_state(int ultrasonic_index, int new_state);

	// returns the state of a potentiometer
	int get_potentiometer_state(int potentiometer_index);

	// gets the state of a potentiometer
	void set_potentiometer_state(int potentiometer_index, int new_state);

	// returns the state of a AS5147 sensor
	int get_AS5147_state(int potentiometer_index);

	// gets the state of a AS5147 sensor
	void set_AS5147_state(int potentiometer_index, int new_state);

	// returns the state of an infrared sensor
	int get_infrared_state(int infrared_index);

	// sets the state of an infrared sensor
	void set_infrared_state(int infrared_index, int new_state);

	// returns the state of the Tera Ranger One sensor
	int get_tera_ranger_one_state(void);

	// sets the state of the Tera Ranger One sensor
	void set_tera_ranger_one_state(int new_state);
};
//----------------------------------------------------------------
#endif