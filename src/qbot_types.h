#ifndef __QBOT_TYPES_H
#define __QBOT_TYPES_H

#include <stdint.h>

#define QBOT_LINK_MAX_PAYLOAD_LEN   255

typedef struct __qbot_message {
	uint8_t id;
	uint8_t len;
	uint8_t payload64[(QBOT_LINK_MAX_PAYLOAD_LEN)/8];
} qbot_message_t;


/***********************************************/
typedef struct __qbot_power {
	int32_t voltage;	
} qbot_power_t;


typedef struct __qbot_ultrasound {
	int left;
	int central;
	int right;	
} qbot_ultrasound_t;


typedef struct __qbot_infrared {
	char status;	
} qbot_infrared_t;

typedef struct __qbot_imu {
	int yaw;
	int pitch;
	int roll;	
} qbot_imu_t;

typedef struct __qbot_charger {
	char status;
} qbot_charger_t;


#endif	/* __QBOT_TYPES_H */
