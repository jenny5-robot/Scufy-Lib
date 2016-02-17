#ifndef jenny5_command_moduleH
#define jenny5_command_moduleH

#include "rs232.h"
#include "lista_voidp.h"
#include "../include/jenny5_events.h"

#include <iostream>
#include <time.h>

#define COMMAND_NOT_SENT 0
#define COMMAND_SENT 1
#define COMMAND_DONE 2

#define EVENT_INFO_TYPE 1
#define EVENT_INFO_PARAM1 2
#define EVENT_INFO_PARAM2 4

//----------------------------------------------------------------
class t_jenny5_command_module{
private:
	// version number of the library
	char version[20];

	// port number of the serial connection
	int port_number;
	
	// a list with received events from Arduino
	TLista received_events;

	// current buffer of characters received from Arduino
	char current_buffer[4096]; // I should not need so much

	// each motor can be in one of 2 states: COMMAND_DONE and COMMAND_SENT
	int stepper_motor_state[4]; // max 4 motors (each motor occupy 3 digital pins ... so 4x3 = 12 digital pins = which arduino nano has)

								// each dc motor can be in one of 2 states: COMMAND_DONE and COMMAND_SENT
	int dc_motor_state[3]; // max dc 3 motors (each motor occupy 4 digital pins ... so 4x3 = 12 digital pins = which arduino nano has)

	int sonar_state[6]; // max 6 ultrasounds (each sonar occupy 2 digital pins ... so 6x2 = 12 digital pins = which arduino nano has)

	int potentiometer_state[4]; // if I have max 4 motors, the number of potentiometers is not higher because each potentiometer is attached to 1 motor

	int infrared_state[4]; // I suppose that I don't have so many infrared sensors attached to 1 controller

	// parse the string for events
	void parse_and_queue_commands(char* tmp_str, int str_length);

public:
	t_jenny5_command_module(void);
	~t_jenny5_command_module(void);

	// connects to given serial port
	bool connect(int port, int baud_rate);
	
	// close serial connection
	void close_connection(void);
	
	// returns a string containing the version number of this library
	const char* get_version(void);
	
	// reads data from serial and updates the list of received events from Arduino
	// this should be called frequently from the main loop of the program in order to read the data received from Arduino
	bool update_commands_from_serial(void);

	// gets an unformated string of chars from serial
	// should be used only in extreme cases
	// normally an application must call update_commands_from_serial
	int get_data_from_serial(char *buffer, int buffer_size);

	// -search in the list of events for a particular event type
	// -if found returns true and, event param will be updated with the
	// found event, else false is returned.
	bool query_for_event(jenny5_event &event, int available_info = EVENT_INFO_TYPE);

	//-waits for the completion of the event passed as a parameter,
	//also updates all the event data if it was found
	//-if waits more than SECONDS_UNTIL_TIMEOUT times out and ends the program
	bool wait_for_command_completion(jenny5_event &event, int available_info = EVENT_INFO_TYPE);

	//// search in the list of events for a particular event type
	bool query_for_event(int event_type);
	//
	//// search in the list of events for a particular event type
	bool query_for_event(int event_type, int* param1);

	//// search in the list of events for a particular event type
	bool query_for_event(int event_type, int param1);

	//// search in the list of events for a particular event type
	bool query_for_event(int event_type, int param1, int *param2);

	//// search in the list of events for a particular event type
	bool query_for_2_events(int event_type1, int param1_1, int event_type2, int param1_2);

	//// search in the list of events for a particular event type
	bool query_for_event(int event_type, int param1, int param2);

	// sends (to Arduino) a command for creating a stepper motor controller
	// several arrays of pin indecses for direction, step and enable must be specified
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memmory
	void send_create_stepper_motors(int num_motors, int* dir_pins, int* step_pins, int* enable_pins);
	
	// sends (to Arduino) a command for creating a DC motor controller
	// several arrays of pin indecses for direction, step and enable must be specified
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memmory
	void send_create_dc_motors(int num_motors, int *pwm_pins, int* dir1_pins, int* dir2_pins, int* enable_pins);

	// sends (to Arduino) a command for creating a sonar controller
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memmory
	void send_create_sonars(int num_sonars, int* trig_pins, int* echo_pins);

	// sends (to Arduino) a command for creating a potentiometer controller
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memmory
	void send_create_potentiometers(int num_potentiometers, int* out_pins, int* _low, int* _high, int *home);

	// sends (to Arduino) a command for creating an infrared controller
	// this method should be called once at the beginning of the program
	// calling it multiple times is allowed, but this will only fragment the Arduino memmory
	void send_create_infrared_sensors(int num_infrared_sensors, int* _pins, int *_low);

	// sends (to Arduino) a command (T#) for testing if the connection is alive
	void send_is_alive(void);

	// STEPPER MOTORS
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

	// sends (to Arduino) a command for blocking a motor to current position
	void send_lock_stepper_motor(int motor_index);
	
	// sends (to Arduino) a command for disabling a motor
	void send_disable_stepper_motor(int motor_index);

	// sends (to Arduino) a command for setting the speed and acceleration of a given motor
	void send_set_stepper_motor_speed_and_acceleration(int motor_index, int motor_speed, int motor_acceleration);
	
	// sends (to Arduino) a command for attaching several sensors to a given motor
	void send_attach_sensors_to_stepper_motor(int motor_index, int num_potentiometers, int *potentiometers_index);

	// sends (to Arduino) a command for reading removing all attached sensors of a motor
	void send_remove_attached_sensors_from_stepper_motor(int motor_index);

	// returns the state of a motor
	int get_stepper_motor_state(int motor_index);

	// sets the state of a motor
	void set_stepper_motor_state(int motor_index, int new_state);

	//DC motors
	// sends (to Arduino) a command for moving a DC motor for a given number of microseconds
	void send_move_dc_motor(int motor_index, int num_miliseconds);
	// sends (to Arduino) a command for moving a DC motor to home position
	void send_go_home_dc_motor(int motor_index);
	// sends (to Arduino) a command for disabling a DC motor
	void send_disable_dc_motor(int motor_index);
	// sends (to Arduino) a command for setting the speed of a given DC motor
	void send_set_dc_motor_speed(int motor_index, int motor_speed);
	// sends (to Arduino) a command for attaching several sensors to a given DC motor
	void send_attach_sensors_to_dc_motor(int motor_index, int num_buttons, int *buttons_index);
	// sends (to Arduino) a command for reading removing all attached sensors of a motor
	void send_remove_attached_sensors_from_dc_motor(int motor_index);
	// returns the state of a motor
	int get_dc_motor_state(int motor_index);
	// sets the state of a motor
	void set_dc_motor_state(int motor_index, int new_state);


	// SENSORS
	// sends (to Arduino) a command for reading a sonar
	void send_get_sonar_distance(int sensor_index);
	
	// sends (to Arduino) a command for a button state
	void send_get_button_status(int button_index);

	// sends (to Arduino) a command for reading a potentiometer position
	void send_get_potentiometer_position(int sensor_index);

	// sends (to Arduino) a command for reading a infrared value
	void send_get_infrared_distance(int sensor_index);

	// sends (to Arduino) a command for reading parameters of a motor; debug purposes
	void send_get_motors_sensors_statistics(void);

	// sends (to Arduino) a command for reading parameters of a motor
	void send_get_motor_parameters(int motor_index);

	// sends (to Arduino) a command for reading parameters of a potentiometer
	void send_get_potentiometer_parameters(int potentiometer_index);

	// sends (to Arduino) a command for setting parameters of a potentiometer
	void send_set_potentiometer_parameters(int potentiometer_index, int min, int max, int home, int direction);
	

	// returns the state of a sonar
	int get_sonar_state(int sonar_index);

	// sets the state of a sonar
	void set_sonar_state(int sonar_index, int new_state);

	// returns the state of a potentiometer
	int get_potentiometer_state(int potentiometer_index);

	// gets the state of a potentiometer
	void set_potentiometer_state(int potentiometer_index, int new_state);

	// returns the state of an infrared sensor
	int get_infrared_state(int infrared_index);

	// sets the state of an infrared sensor
	void set_infrared_state(int infrared_index, int new_state);
};
//----------------------------------------------------------------
#endif