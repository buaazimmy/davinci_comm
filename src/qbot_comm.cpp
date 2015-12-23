/**
 * @file Qbot_comm.cpp
 *
 * @brief Qbot interface functions
 *
 * Functions for publish and subscribe message from Qbot
 *
 *
 */

#include "qbot_comm.h"

uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

Qbot_Communication::Qbot_Communication()
{
	// initialize attributes
	// start ros publisher
    	ultrasound_pub	= comm_nh.advertise<std_msgs::String>("/qfeel_follow/ultrasound", 10);
	infrared_pub	= comm_nh.advertise<std_msgs::Byte>("/qfeel_follow/infrared", 10);
	imu_pub		= comm_nh.advertise<sensor_msgs::Imu>("/qfeel_follow/imu", 10);
    	power_pub	= comm_nh.advertise<std_msgs::Byte>("/qfeel_node/power", 10);
    	charger_pub	= comm_nh.advertise<std_msgs::Byte>("/qfeel_follow/charger", 10);
/*
	pulse_led_sub	= comm_nh.subscribe<std_msgs::Byte>("/qfeel_follow/pulse_led",100,&Qbot_Communication::pulse_led_Callback,this);
	state_led_sub	= comm_nh.subscribe<std_msgs::Byte>("/qfeel_follow/state_led",100,&Qbot_Communication::state_led_Callback,this);
	line_laser_sub	= comm_nh.subscribe<std_msgs::Byte>("/qfeel_follow/line_laser",100,&Qbot_Communication::line_laser_Callback,this);
	motor_state_sub	= comm_nh.subscribe<std_msgs::Byte>("/qfeel_follow/motor_state",100,&Qbot_Communication::motor_state_Callback,this);
	base_ctr_sub	= comm_nh.subscribe<geometry_msgs::Twist>("/qfeel_follow/cmd_vel",100,&Qbot_Communication::base_ctr_Callback,this); 
*/
	pulse_led_sub	= comm_nh.subscribe("/qfeel_follow/pulse_led",100,&Qbot_Communication::pulse_led_Callback,this);
	state_led_sub	= comm_nh.subscribe("/qfeel_follow/state_led",100,&Qbot_Communication::state_led_Callback,this);
	line_laser_sub	= comm_nh.subscribe("/qfeel_follow/line_laser",100,&Qbot_Communication::line_laser_Callback,this);
	motor_state_sub	= comm_nh.subscribe("/qfeel_follow/motor_state",100,&Qbot_Communication::motor_state_Callback,this);
	base_ctr_sub	= comm_nh.subscribe("/qfeel_follow/cmd_vel",100,&Qbot_Communication::base_ctr_Callback,this); 

}


Qbot_Communication::~Qbot_Communication(){;}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int Qbot_Communication::write_message(qbot_message_t &message)
{
	// do the write
	int len = serial_port->write_message(message);

	// Done!
	return len;
}

/*************** advertise publish ******************/

void
Qbot_Communication::
publish_power(qbot_power_t power) {
	std_msgs::Byte msg;
	msg.data=power.voltage;
	printf("publish power: %d\n",msg.data);
	power_pub.publish(msg);

}

void
Qbot_Communication::
publish_ultrasound(qbot_ultrasound_t ultrasound) {
	std_msgs::String msg;
	std::stringstream ss;
	ss  << (int)ultrasound.left << " " << (int)ultrasound.central << " " << (int)ultrasound.central;
	msg.data = ss.str();
//	ROS_INFO("publish ultrasound:%s\n",ss.str());
	//printf("publish ultrasound:%s\n", ss.str());
	std::cout<< i++ << "\npublish ultrasound: " << ss.str() << "\n";
	ultrasound_pub.publish(msg);
	ss.str("");
}

void
Qbot_Communication::
publish_infrared(qbot_infrared_t infrared) {
	std_msgs::Byte msg;
	msg.data = infrared.status;
	printf("publish infrared: %d\n",msg.data);
	infrared_pub.publish(msg);
}

void
Qbot_Communication::
publish_imu(qbot_imu_t imu){
	sensor_msgs::Imu msg;
	float q0, q1, q2, q3, yaw, pitch, roll;
	
	yaw = imu.yaw / 100.0;
	pitch = imu.pitch / 100.0;
	roll = imu.roll / 100.0; 
	
	q0 = cos(0.5*roll)*cos(0.5*pitch)*cos(0.5*yaw) - sin(0.5*roll)*sin(0.5*pitch)*sin(0.5*yaw);  //w
	q1 = cos(0.5*roll)*sin(0.5*pitch)*cos(0.5*yaw) - sin(0.5*roll)*cos(0.5*pitch)*sin(0.5*yaw);  //x   pitch
	q2 = sin(0.5*roll)*cos(0.5*pitch)*cos(0.5*yaw) + cos(0.5*roll)*sin(0.5*pitch)*sin(0.5*yaw);  //y   roll
	q3 = cos(0.5*roll)*cos(0.5*pitch)*sin(0.5*yaw) + sin(0.5*roll)*sin(0.5*pitch)*cos(0.5*yaw);  //z   Yaw
	
	msg.orientation.w = q0;
	msg.orientation.x = q1;
	msg.orientation.y = q2;
	msg.orientation.z = q3;	
 	
	imu_pub.publish(msg);
	printf("publish imu: %f %f %f %f\n",q0, q1, q2, q3);
}

void
Qbot_Communication::
publish_charger(qbot_charger_t charger){
	std_msgs::Byte msg;	
	msg.data = charger.status;
	printf("publish charger: %d\n\n",msg.data);
	charger_pub.publish(msg);
}

/*************** subscribe CallBack ******************/

void
Qbot_Communication::
base_ctr_Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	qbot_message_t message;
	message.id = 0x11;
	message.len = 0x04;
	int linear_tmp, angular_tmp;
	linear_tmp = cmd_vel->linear.x;
	angular_tmp = cmd_vel->angular.z;
	printf("subscribe base_ctr:linear_tmp->%d angular_tmp->%d\n",linear_tmp, angular_tmp);
	message.payload64[0] = linear_tmp & 0xFF;
	message.payload64[1] = linear_tmp >> 8;
	message.payload64[2] = angular_tmp & 0xFF;
	message.payload64[3] = angular_tmp >> 8;
	//message.payload64[1] = motor_state->linear.z;
	write_message(message);
}


void
Qbot_Communication::
state_led_Callback(const std_msgs::Byte::ConstPtr& state_led)
{
	qbot_message_t message;
	message.id = 0x12;
	message.len = 0x01;
	message.payload64[0] = state_led->data;
	printf("subscribe state_led:%d\n",message.payload64[0]);
	write_message(message);
}

void
Qbot_Communication::
pulse_led_Callback(const std_msgs::String::ConstPtr& pulse_led)
{
	int i = 0;
	std::stringstream ss(pulse_led->data.c_str());
	qbot_message_t message;
	message.id = 0x13;
	message.len = 0x04;
	printf("subscribe pulse_led:\n");
	while ( ss ) {
		ss >> message.payload64[i++];
		printf("\t%d", message.payload64[i-1]);
	}
	printf("\n");
	write_message(message);
		
}

void
Qbot_Communication::
line_laser_Callback(const std_msgs::Byte::ConstPtr& line_laser)
{
	qbot_message_t message;
	message.id = 0x14;
	message.len = 0x01;
	message.payload64[0] = line_laser->data;
	printf("subscribe line_laser:%d\n", message.payload64[0]);
	write_message(message);
}

void
Qbot_Communication::
motor_state_Callback(const std_msgs::Byte::ConstPtr& motor_state)
{
	qbot_message_t message;
	message.id = 0x15;
	message.len = 0x01;
	message.payload64[0] = motor_state->data;
	printf("subscribe motor_state:%d\n", message.payload64[0]);
	write_message(message);
}


