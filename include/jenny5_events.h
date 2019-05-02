#ifndef jenny5_eventsH
#define jenny5_eventsH

#include <stdio.h>

#define IS_ALIVE_EVENT 0
#define STEPPER_MOTOR_MOVE_DONE_EVENT 1
#define DC_MOTOR_MOVE_DONE_EVENT 2

#define SONAR_EVENT 3
#define POTENTIOMETER_EVENT 4
#define BUTTON_EVENT 5
#define INFRARED_EVENT 6
#define AS5147_EVENT 7

#define STEPPER_MOTOR_LOCKED_EVENT 8
#define STEPPER_MOTOR_DISABLED_EVENT 9

#define DC_MOTOR_DISABLED_EVENT 10

#define STEPPER_MOTORS_CONTROLLER_CREATED_EVENT 11
#define SONARS_CONTROLLER_CREATED_EVENT 12
#define POTENTIOMETERS_CONTROLLER_CREATED_EVENT 13
#define INFRARED_CONTROLLER_CREATED_EVENT 14
#define DC_MOTORS_CONTROLLER_CREATED_EVENT 15
#define BUTTONS_CONTROLLER_CREATED_EVENT 16
#define TERA_RANGER_ONE_CONTROLLER_CREATED_EVENT 17
#define LIDAR_CONTROLLER_CREATED_EVENT 18
#define AS5147S_CONTROLLER_CREATED_EVENT 19
#define ATTACH_SENSORS_EVENT 20
#define STEPPER_STOPPED_EVENT 21



#define TERA_RANGER_ONE_EVENT 30
#define LIDAR_READ_EVENT 31
#define STEPPER_MOTOR_SET_SPEED_ACCELL_EVENT 32
#define ARDUINO_FIRMWARE_VERSION_EVENT 33
#define FREE_MEMORY_EVENT 34
#define INFO_EVENT 35

#define SECONDS_UNTIL_TIMEOUT 10

//-----------------------------------------------------------------------
class jenny5_event{
public:
	char type;
	int param1;
	intptr_t param2;
	int time;

public:
	jenny5_event(char _type) : 
		type(_type),
		param1(-1),
		param2(-1),
		time(-1)
	{}

	jenny5_event(char _type, int _param1) :
		type(_type),
		param1(_param1),
		param2(-1),
		time(-1)
	{}

	jenny5_event(char _type, int _param1, intptr_t _param2) :
		type(_type),
		param1(_param1),
		param2(_param2),
		time(-1)
	{}

	jenny5_event(char _type, int _param1, intptr_t _param2, int _time)
	{
		type = _type;
		param1 = _param1;
		param2 = _param2;
		time = _time;
	}
};
//-----------------------------------------------------------------------

#endif