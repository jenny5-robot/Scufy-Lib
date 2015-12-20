#include "rs232.h"



class t_jenny5_command_module{
private:
//	CSerial serial_connection;
	char version[20];

	int port_number;
	

public:
	t_jenny5_command_module();
	~t_jenny5_command_module();

	bool connect(int port, int baud_rate);
	void close_connection(void);
	
	bool move_motor(int motor_index, int num_steps);
	bool move_motor2(int motor_index1, int num_steps1, int motor_index2, int num_steps2);
	bool move_motor3(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3);
	bool move_motor4(int motor_index1, int num_steps1, int motor_index2, int num_steps2, int motor_index3, int num_steps3, int motor_index4, int num_steps4);
	bool move_motor(int num_motors, int* motor_index, int *num_steps);

	bool set_motor_speed(int motor_index, int motor_speed, int motor_acceleration);
	bool set_motor_acceleration(int motor_index, int motor_acceleration);
	int get_ultrasonic_distance(int sensor_index);
	bool get_button_status(int button_index);
	int get_potentiometer_position(int sensor_index);
	bool get_infrared_status(int sensor_index);
	bool get_motors_sensors_statistics();
	bool disable_motor(int motor_index);
	bool lock_motor(int motor_index);
	int is_connected(void);
	int get_data(unsigned char *buffer, int buffer_size);

	void attach_sensors(int motor_index, int num_potentiometers, int *potentiometers_index);
	void remove_attached_sensors(int motor_index);
	const char* get_version(void);

};