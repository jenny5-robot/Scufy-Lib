
#include "tera_ranger_one_controller.h"
//-----------------------------------------------------------------------
int main(void)
{
	t_teraranger_one_controller tera_ranger_one;

	if (!tera_ranger_one.connect(11, 115200)) {
		printf("Error attaching to tera ranger one!");
		getchar();
		return 1;
	}

	if (tera_ranger_one.get_state() == COMMAND_DONE) {// I ping the sonar only if no ping was sent before
		tera_ranger_one.send_request_distance();
		tera_ranger_one.set_state(COMMAND_SENT);
		printf("TR# - sent\n");
	}

	while (1){        // starting infinit loop
		if (!tera_ranger_one.update_commands_from_serial())
			Sleep(5); // no new data from serial ... we make a little pause so that we don't kill the processor

		// read to see if there is any distance received from sonar
		if (tera_ranger_one.get_state() == COMMAND_SENT) {// if a command has been sent
			int distance;
			if (tera_ranger_one.query_for_distance(distance)) { // have we received the event from Serial ?
				tera_ranger_one.set_state(COMMAND_DONE);
				printf("distance = %d cm\n", distance);
			}
		}
			if (tera_ranger_one.get_state() == COMMAND_DONE) {// I ping the sonar only if no ping was sent before
				tera_ranger_one.send_request_distance();
				tera_ranger_one.set_state(COMMAND_SENT);
				printf("TR# - sent\n");
			}

		if (GetAsyncKeyState(VK_ESCAPE))  // break the loop
			break;
	}

	tera_ranger_one.close_connection();


	return 0;
}
//-----------------------------------------------------------------------