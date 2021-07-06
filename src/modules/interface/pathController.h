#ifndef __PATH_CONTROLLER_H__
#define __PATH_CONTROLLER_H__

#include <stdint.h>
#include "stabilizer_types.h"
#include "crtp_commander.h"

#define MAX_PATH_LENGTH 4
#define RATE_PATH_CTRL RATE_25_HZ

///** path Data */
typedef struct pathPacket_s {
	uint8_t pathLength;

	float xPos[MAX_PATH_LENGTH];
	float yPos[MAX_PATH_LENGTH];
	float zPos[MAX_PATH_LENGTH];

} pathPacket_t;


/** path Control Data */
typedef struct pathContolData_s {
	int8_t pathCounter;
	uint8_t  planStatus;
	uint16_t nodeCounter;

	float pathTimeNext;

	bool initFlag;    // path controller flag whether last WP reached yet

	float timePath;
	float accTime;
	float speedVec[3];
	float waypoint1[3];
	float waypoint2[3];
} pathControlData_t;

void initPathController(state_t *state, pathControlData_t* pathControlData, pathPacket_t* pathPacket );
void pathController(setpoint_t *setpoint, state_t *state, pathPacket_t *pathPacket, pathControlData_t *pathControlData, float deltaT);

#endif /* __PATH_CONTROLLER_H__ */
