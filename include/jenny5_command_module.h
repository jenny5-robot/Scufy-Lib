#ifndef jenny5_command_moduleH
#define jenny5_command_moduleH

#include "rs232.h"
#include "lista_voidp.h"

#define COMMAND_NOT_SENT 0
#define COMMAND_SENT 1
#define COMMAND_DONE 2


//----------------------------------------------------------------
class t_jenny5_command_module{
private:

	char version[20];

	int port_number;
	
	TLista received_events;

	char current_buffer[4096]; // I should not need this size

	int motor_state[4]; // max 4 motors (each motor occupy 3 digital pins ... so 4x3 = 12 digital pins = which arduino nano has)

	int sonar_state[6]; // max 6 ultrasounds (each sonar occupy 2 digital pins ... so 6x2 = 12 digital pins = which arduino nano has)

	int potentiometer_state[4]; // if I have max 4 motors, the number of potentiometers is not higher because each potentiometer is attached to 1 motor

	void parse_and_queue_commands(char* tmp_str, int str_length);

public:
	t_jenny5_command_module(void);
	~t_jenny5_command_module(void);

	bool connect(int port, int baud_rate);
	void close_connection(void);

	const char* get_version(void);
	
	bool update_commands_from_serial(void);
	int get_data_from_serial(char *buffer, int buffer_size);

	bool query_for_event(int event_type);
	bool query_for_event(int event_type, int* param1);
	bool query_for_event(int event_type, int param1);
	bool query_for_event(int event_type, int param1, int *param2);
	bool query_for_2_events(int event_type1, int param1_1, int event_type2, int param1_2);
	bool query_for_event(int event_type, int param1, int param2);
	
	void send_create_motors(int num_motors, int* dir_pins, int* step_pins, int* enable_pins);
	void send_create_sonars(int num_sonars, int* trig_pins, int* echo_pins);
	void send_create_potentiometers(int num_potentiometers, int* out_pins, int* _low, int* _high, int *home);

	void send_is_alive(void);

	void send_go_home_motor(int motor_index);

	void send_move_motor(int motor_index, int num_steps);
	void send_move_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2);
	void send_move_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3);
	void send_move_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4);
	void send_move_motor_array(int num_motors, int* motor_index, int *num_steps);

	void send_lock_motor(int motor_index);
	void send_disable_motor(int motor_index);

	void send_set_motor_speed_and_acceleration(int motor_index, int motor_speed, int motor_acceleration);
	
	void send_attach_sensors(int motor_index, int num_potentiometers, int *potentiometers_index);
	
	void send_get_sonar_distance(int sensor_index);
	void send_get_button_status(int button_index);
	void send_get_potentiometer_position(int sensor_index);
	void send_get_infrared_distance(int sensor_index);

	void send_get_motors_sensors_statistics(void);

	void send_get_motor_parameters(int motor_index);

	void send_get_potentiometer_parameters(int potentiometer_index);
	void send_set_potentiometer_parameters(int potentiometer_index, int min, int max, int home);
	
	void send_remove_attached_sensors(int motor_index);

	int get_motor_state(int motor_index);
	void set_motor_state(int motor_index, int state);

	int get_sonar_state(int sonar_index);
	void set_sonar_state(int sonar_index, int state);

	int get_potentiometer_state(int potentiometer_index);
	void set_potentiometer_state(int potentiometer_index, int state);
};
//----------------------------------------------------------------
#endif