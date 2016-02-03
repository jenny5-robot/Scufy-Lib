#ifndef EVENT_HANDLER_H_
#define EVENT_HANDLER_H_

#include "jenny5_events.h"
#include <iostream>

#define TYPE 0x001
#define PARAM1 0x010
#define PARAM2 0x100

class t_event_handler {
private:
	t_event_handler();

public:
	static jenny5_event query_for_event(jenny5_event &event, int available_info);
};

#endif