#ifndef QBOT_COMM_H_
#define QBOT_COMM_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sstream>

#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#include "serial_port.h"
#include "qbot_types.h"


// ----------------------------------------------------------------------------------
//   Qbot Interface Class
// ----------------------------------------------------------------------------------
class Qbot_Communication{

public:
	Qbot_Communication();
	~Qbot_Communication();
	
	char writing_status;
	uint64_t write_count;
	int i;
	ros::NodeHandle comm_nh;

	ros::Publisher power_pub, ultrasound_pub, infrared_pub, imu_pub, charger_pub;
	ros::Subscriber	pulse_led_sub, state_led_sub, line_laser_sub, motor_state_sub, base_ctr_sub;
		
	int  write_message(qbot_message_t &message);
	
	void publish_power(qbot_power_t power);
	void publish_ultrasound(qbot_ultrasound_t ultrasound);
	void publish_infrared(qbot_infrared_t infrared);
	void publish_imu(qbot_imu_t imu);
	void publish_charger(qbot_charger_t charger);

	void pulse_led_Callback(const std_msgs::String::ConstPtr& pulse_led);
	void state_led_Callback(const std_msgs::Byte::ConstPtr& state_led);
	void line_laser_Callback(const std_msgs::Byte::ConstPtr& line_laser);
	void motor_state_Callback(const std_msgs::Byte::ConstPtr& motor_state);	
	void base_ctr_Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel);

	Serial_Port *serial_port;

private:
};


#endif // QBOT_COMM_H_
