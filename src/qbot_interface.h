
#ifndef QBOT_INTERFACE_H_
#define QBOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "qbot_types.h"
#include "qbot_comm.h"

#define b_uart_head  0x80
#define b_rx_over    0x40
#define RX_BUFFER_SIZE 128

// helper functions
uint64_t get_time_usec();

void* start_qbot_interface_read_thread(void *args);
void* start_qbot_interface_write_thread(void *args);

// Struct containing information on the MAV we are currently connected to
struct Qbot_Messages {
	qbot_power_t power;
	qbot_ultrasound_t ultrasound;
	qbot_infrared_t infrared;	
	qbot_imu_t imu;
	qbot_charger_t charger; 
};


class Qbot_Interface
{

public:
	Qbot_Interface();
	Qbot_Interface(Serial_Port *serial_port_);
	~Qbot_Interface();

	char reading_status;
//	char writing_status;
	char control_status;

	uint8_t RC_Flag;
	uint8_t rx_buffer[RX_BUFFER_SIZE];
	uint8_t rx_wr_index;
	uint8_t checksum;
	
	Qbot_Messages current_messages;
	Qbot_Communication qbot_comm;
	void read_messages();
	int write_message(qbot_message_t message);

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );
private:

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;


	void read_thread();
	void write_thread(void);
	uint8_t Sum_check();	
};



#endif // QBOT_INTERFACE_H_




