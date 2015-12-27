#include "rs232.h"
#include "lista_voidp.h"



//----------------------------------------------------------------
class t_jenny5_command_module{
private:

	char version[20];

	int port_number;
	
	TLista received_events;

	char current_buffer[4096]; // I should not need this size

	int get_data_from_serial(char *buffer, int buffer_size);
	void parse_and_queue_commands(char* tmp_str, int str_length);

public:
	t_jenny5_command_module(void);
	~t_jenny5_command_module(void);

	bool connect(int port, int baud_rate);
	void close_connection(void);

	const char* get_version(void);
	
	bool update_commands_from_serial(void);

	int clear_data_from_serial(char *buffer, int buffer_size);

	bool query_for_event(int event_type, intptr_t param1);
	bool query_for_event(int event_type, intptr_t param1, intptr_t param2);
	
	void send_is_alive(void);

	void send_move_motor(int motor_index, int num_steps);
	void send_move_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2);
	void send_move_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3);
	void send_move_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4);
	void send_move_motor_array(int num_motors, int* motor_index, int *num_steps);

	void send_lock_motor(int motor_index);
	void send_disable_motor(int motor_index);

	void send_set_motor_speed_and_acceleration(int motor_index, int motor_speed, int motor_acceleration);
	
	void send_attach_sensors(int motor_index, int num_potentiometers, int *potentiometers_index);
	
	void send_get_ultrasonic_distance(int sensor_index);
	void send_get_button_status(int button_index);
	void send_get_potentiometer_position(int sensor_index);
	void send_get_infrared_distance(int sensor_index);

	void send_get_motors_sensors_statistics(void);

	void send_get_motor_parameters(int motor_index);

	void send_get_potentiometer_parameters(int potentiometer_index);
	void send_set_potentiometer_parameters(int potentiometer_index, int min, int max, int home);
	
	void send_remove_attached_sensors(int motor_index);
};
//----------------------------------------------------------------