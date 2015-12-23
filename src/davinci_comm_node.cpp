#include "davinci_comm_node.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "davinci_comm");
// 	Default input arguments
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 115200;

// 	do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);

//	PORT and THREAD STARTUP
	Serial_Port serial_port(uart_name, baudrate);
	Qbot_Interface qbot_interface(&serial_port);

//	Setup interrupt signal handler. Responds to early exits signaled with Ctrl-C.
	signal(SIGINT,quit_handler);	
	
	serial_port_quit	= &serial_port;
	qbot_interface_quit	= &qbot_interface;

//	Start the port and qbot_interface
	serial_port.start();
	qbot_interface.start();
	

//	RUN COMMANDS
//	commands(qbot_interface);	
	ros::spin();
//	THREAD and PORT SHUTDOWN
//	qbot_interface.stop();
//	serial_port.stop();



	return 0;	
}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------
void commands(Qbot_Interface &api)
{

	ros::Timer	pub_timer;
//	pub_timer = api.qbot_comm.comm_nh.createTimer(ros::Duration(2.0),(const ros::TimerCallback &)pub_timer_cb);

	ros::spin();

	return;

}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	// qbot interface
	try {
		qbot_interface_quit->handle_quit(sig);
	}
	catch (int error){
	}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){	
	}

	// ROS
	ros::shutdown();
	printf("..........\n");
	// end program here
	exit(0);

}

