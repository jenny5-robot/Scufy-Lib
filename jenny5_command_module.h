#include "serial.h"

// version 2015.11.12.1

class t_jenny5_command_module{
private:
	CSerial serial_connection;
public:
	t_jenny5_command_module();
	~t_jenny5_command_module();
	bool connect(int port, int baud_rate);
	void close_connection(void);
	bool move_motor(int motor_index, int num_steps);
	bool set_motor_speed(int motor_index, int motor_speed);
	bool set_motor_acceleration(int motor_index, int motor_acceleration);
	bool disable_motor(int motor_index);
	bool lock_motor(int motor_index);
	bool is_connected(void);
	int get_data(char *buffer, int buffer_size);

};