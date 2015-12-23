
#define __need_timeval
// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "qbot_interface.h"

int count_write=0;
timeval t_start,t_end;
long last_time=0;
long count=0,total_time=0;
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
extern uint64_t get_time_usec();

Qbot_Interface::
Qbot_Interface(Serial_Port *serial_port_)
{
	
	// initialize attributes
	qbot_comm.write_count = 0;

	reading_status = 0;      // whether the read thread is running
	qbot_comm.writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	qbot_comm.serial_port = serial_port_; // serial port management object

	RC_Flag = 0;
	rx_wr_index = 0;
	checksum = 0;
	memset(rx_buffer, RX_BUFFER_SIZE, 0);
}

Qbot_Interface::
~Qbot_Interface()
{}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Qbot_Interface::
read_messages()
{	
	uint8_t message;

	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	bool header00_flag = false, header01_flag = false, data_flag = false;
	//printf("Qbot_Interface::read_messages()\n");
	// Blocking wait for new data
	while ( not received_all and not time_to_exit ) {
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------		
		success = qbot_comm.serial_port->read_message(message);
		//printf("%d\n", message);
		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success ) {
			if (message == 0xAA) {
				header00_flag = true;
				RC_Flag |= b_uart_head;
				rx_buffer[rx_wr_index++] = message;
			} else if (message == 0x55) {
				if (RC_Flag & b_uart_head) {
					header01_flag = true;
					rx_wr_index = 0;
					RC_Flag &= ~b_rx_over;
				} else
					rx_buffer[rx_wr_index++] = message;
				RC_Flag &= ~b_uart_head;
			} else {
				rx_buffer[rx_wr_index++] = message;
				RC_Flag &= ~b_uart_head;
				if (rx_wr_index == (rx_buffer[0] + 2)) {
					RC_Flag ^= b_rx_over;
					data_flag = true;
					if (Sum_check()) {
						int i = 1;
						if ((rx_buffer[i] == 0x01) && (rx_buffer[i+1] == 0x06)) {
							// UltraSound
							i += 2;
							current_messages.ultrasound.left = (int) rx_buffer[i+1] << 8 | rx_buffer[i];i += 2;
							current_messages.ultrasound.central = (int) rx_buffer[i+1] << 8 | rx_buffer[i];i += 2;
							current_messages.ultrasound.right = (int) rx_buffer[i+1] << 8 | rx_buffer[i];i += 2;
							//printf("ultrasound\n");
							qbot_comm.publish_ultrasound(current_messages.ultrasound);
						}
						if ((rx_buffer[i] == 0x02) && (rx_buffer[i+1] == 0x01)) {
							// InfraRed
							i+=2;
							//printf("Infrared\n");
							current_messages.infrared.status = (int) rx_buffer[i];i+=1;
							qbot_comm.publish_infrared(current_messages.infrared);
						}
						if ((rx_buffer[i] == 0x03) && (rx_buffer[i+1] == 0x06)) {
							// IMU
							i+=2;
							//printf("IMU\n");
							current_messages.imu.yaw = (int) rx_buffer[i+1] << 8 | rx_buffer[i];i+=2;
							current_messages.imu.pitch = (int) rx_buffer[i+1] << 8 | rx_buffer[i];i+=2;
							current_messages.imu.roll = (int) rx_buffer[i+1] << 8 | rx_buffer[i];i+=2;
							qbot_comm.publish_imu(current_messages.imu);
						}
						if ((rx_buffer[i] == 0x04) && (rx_buffer[i+1] == 0x02)) {
							// Power
							i+=2;
							//printf("power\n");
							current_messages.power.voltage = (int) rx_buffer[i+1] << 8 | rx_buffer[i];i+=2;
							qbot_comm.publish_power(current_messages.power);
						}
						if ((rx_buffer[i] == 0x06) && (rx_buffer[i+1] == 0x01)) {
							//Charger
							i+=2;
							//printf("charger\n");
							current_messages.charger.status = (int) rx_buffer[i];i+=1;
							qbot_comm.publish_charger(current_messages.charger);
						}
					}
				}
			}
		} // end: if read message

		// Check for receipt of all items
		received_all = 
				header00_flag	&&
				header01_flag	&&
				data_flag;

		// give the write thread time to use the port
		if ( qbot_comm.writing_status > false )
			usleep(100); // look for components of batches at 10kHz

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Qbot_Interface::
write_message(qbot_message_t message)
{
	// do the write
	int len = qbot_comm.serial_port->write_message(message);

	// Done!
	return len;
}



// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Qbot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( not qbot_comm.serial_port->status == 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}
	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_qbot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_qbot_interface_write_thread, this );
	if ( result ) throw result;
	// wait for it to be started

	while ( not qbot_comm.writing_status )
		usleep(10000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Qbot_Interface::
stop()
{

	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	

	// signal exit
	time_to_exit = true;


	// wait for exit
	if (0 == pthread_join(read_tid ,NULL)) {
		printf("read pthread close\n");
	} else {
		printf("Error: close read pthread fail\n");
	}
	
	if (0 == pthread_join(write_tid,NULL)) {
		printf("write pthread close\n");
	} else {
		printf("Error: close write pthread fail\n");
	}
	printf("\n");
	exit(0);
	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Qbot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Qbot_Interface::
start_write_thread(void)
{
	if ( not qbot_comm.writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Qbot_Interface::
handle_quit( int sig )
{

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Qbot_Interface::
read_thread()
{
	reading_status = true;

	while ( not time_to_exit )
	{
		read_messages();
		usleep(250000); // Read batches at 40Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Qbot_Interface::
write_thread(void)
{
	// signal startup
	qbot_comm.writing_status = 2;

	// write a message and signal writing
//	write_setpoint();
	qbot_comm.writing_status = true;
	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( not time_to_exit )
	{
		usleep(250000);   // Stream at 40Hz
		count_write++;
//		usleep(4000);   // Stream at 25Hz

	}
	// signal end
	qbot_comm.writing_status = false;

	return;

}


uint8_t
Qbot_Interface::
Sum_check() {
	uint8_t i;
	unsigned int checksum = 0;
	for (i = 0; i < rx_buffer[0] + 2; i++)
		checksum ^= rx_buffer[i];

	return checksum ? 0 : 1;
}


// End Qbot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_qbot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Qbot_Interface *qbot_Interface = (Qbot_Interface *)args;

	// run the object's read thread
	qbot_Interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_qbot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Qbot_Interface *qbot_Interface = (Qbot_Interface *)args;

	// run the object's read thread
	qbot_Interface->start_write_thread();

	// done!
	return NULL;
}


