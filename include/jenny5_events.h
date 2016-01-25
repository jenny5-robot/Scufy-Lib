#ifndef jenny5_eventsH
#define jenny5_eventsH

#include <stdio.h>

#define IS_ALIVE_EVENT 0
#define MOTOR_DONE_EVENT 1
#define SONAR_EVENT 2
#define POTENTIOMETER_EVENT 3
#define BUTTON_EVENT 4
#define MOTOR_LOCKED_EVENT 5
#define MOTOR_DISABLED_EVENT 6
#define MOTORS_CONTROLLER_CREATED_EVENT 7
#define SONARS_CONTROLLER_CREATED_EVENT 8

//-----------------------------------------------------------------------
class jenny5_event{
public:
	char type;
	int param1, param2;
	int time;

public:
	jenny5_event(char _type, int _param1, int _param2, int _time)
	{
		type = _type;
		param1 = _param1;
		param2 = _param2;
		time = _time;
	}
};
//-----------------------------------------------------------------------

#endif