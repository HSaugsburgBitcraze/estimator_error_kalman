
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "pathController.h"
#include "log.h"
#include "param.h"
#include "math.h"

#ifndef MATLAB_MEX_FILE
#include "FreeRTOS.h"
#include "task.h"
#endif



static float path_Kp = 2.0f;

static float xLow   = -1.2f;
static float yLow   = -1.2f;
static float xHigh  =  1.2f;
static float yHigh  =  1.2f;
static float zLow   =  1.0f;
static float zHigh  =  1.0f;
static float speed  =  0.5f;

//static bool isInit;
//static bool isInitPlanner;

static float euclidianDistance(float* point1, float* point2);

void initPathController(state_t *state, pathControlData_t* pathControlData, pathPacket_t* pathPacket ){
	float deltaTSegment;

  pathPacket->pathLength = 4;
  pathPacket->xPos[0]= xLow;
  pathPacket->yPos[0]= yLow;
  pathPacket->zPos[0]= zLow;
  
  pathPacket->xPos[1]= xHigh;
  pathPacket->yPos[1]= yLow;
  pathPacket->zPos[1]= zLow;
  
  pathPacket->xPos[2]= xHigh;
  pathPacket->yPos[2]= yHigh;
  pathPacket->zPos[2]= zHigh;

  pathPacket->xPos[3]= xLow;
  pathPacket->yPos[3]= yHigh;
  pathPacket->zPos[3]= zHigh;

	// compute path waypoint information for the first segment
	pathControlData->pathCounter = -1;
	pathControlData->waypoint1[0] = state->position.x;
	pathControlData->waypoint1[1] = state->position.y;
	pathControlData->waypoint1[2] = state->position.z;

	pathControlData->waypoint2[0] = xLow;
	pathControlData->waypoint2[1] = yLow;
	pathControlData->waypoint2[2] = zLow;

  float distance = euclidianDistance(&pathControlData->waypoint1[0], &pathControlData->waypoint2[0]);

	deltaTSegment = distance/speed;

	// set path control settings to initial value
	pathControlData->timePath = 0.0f;
	pathControlData->initFlag = true;
	//pathControlData->finishFlag    = false;
	//pathControlData->pathRedImproveCounter = 0;

	// compute path speed information for the first segment
	pathControlData->speedVec[0] = (pathControlData->waypoint2[0] - pathControlData->waypoint1[0]) / deltaTSegment;
	pathControlData->speedVec[1] = (pathControlData->waypoint2[1] - pathControlData->waypoint1[1]) / deltaTSegment;
	pathControlData->speedVec[2] = (pathControlData->waypoint2[2] - pathControlData->waypoint1[2]) / deltaTSegment;

  pathControlData->pathTimeNext = deltaTSegment;  
}
 


void pathController(setpoint_t *setpoint, state_t *state, pathPacket_t *pathPacket, pathControlData_t *pathControlData, float deltaT) {

	float refPos[3];// , deltaTime;

	pathControlData->timePath = pathControlData->timePath + deltaT;

	// if path time is such that endpoint of segmenet has not been reached yet and finish path flag was not set
	if (pathControlData->timePath > pathControlData->pathTimeNext  ){
		volatile int8_t currId, nextId;
    // initial segment waypoints are considered after finishing planning stage as well as speed information for initial segment
		currId = pathControlData->pathCounter+1;
    
    if(currId>=pathPacket->pathLength){
      currId = 0;
    }
    nextId = currId+1;
    if(nextId >= (int8_t)pathPacket->pathLength){
      nextId = 0;
    }
    pathControlData->pathCounter = currId;

    pathControlData->waypoint1[0] = pathPacket->xPos[(uint8_t)currId];
		pathControlData->waypoint1[1] = pathPacket->yPos[(uint8_t)currId];
		pathControlData->waypoint1[2] = pathPacket->zPos[(uint8_t)currId];

		pathControlData->waypoint2[0] = pathPacket->xPos[(uint8_t)nextId];
		pathControlData->waypoint2[1] = pathPacket->yPos[(uint8_t)nextId];
		pathControlData->waypoint2[2] = pathPacket->zPos[(uint8_t)nextId];

    pathControlData->timePath = 0.0f;

    float deltaTSegment = euclidianDistance(&pathControlData->waypoint1[0], &pathControlData->waypoint2[0])/speed;
    pathControlData->pathTimeNext = deltaTSegment;

    pathControlData->speedVec[0] = (pathControlData->waypoint2[0] - pathControlData->waypoint1[0]) / deltaTSegment;
	  pathControlData->speedVec[1] = (pathControlData->waypoint2[1] - pathControlData->waypoint1[1]) / deltaTSegment;
	  pathControlData->speedVec[2] = (pathControlData->waypoint2[2] - pathControlData->waypoint1[2]) / deltaTSegment;
	}

	// compute reference position for path control
	refPos[0] = pathControlData->speedVec[0] * pathControlData->timePath + pathControlData->waypoint1[0];
	refPos[1] = pathControlData->speedVec[1] * pathControlData->timePath + pathControlData->waypoint1[1];
	refPos[2] = pathControlData->speedVec[2] * pathControlData->timePath + pathControlData->waypoint1[2];
	
	// sum up formation controller
	setpoint->mode.x = modeVelocity;
	setpoint->mode.y = modeVelocity;
	setpoint->mode.z = modeVelocity;

	// compute control output
	setpoint->velocity.x = pathControlData->speedVec[0] + path_Kp * (refPos[0] - state->position.x);
	setpoint->velocity.y = pathControlData->speedVec[1] + path_Kp * (refPos[1] - state->position.y);
	setpoint->velocity.z = pathControlData->speedVec[2] + path_Kp * (refPos[2] - state->position.z);

	setpoint->mode.yaw = modeVelocity;

	setpoint->attitudeRate.yaw = 0.0;
}


float euclidianDistance(float* point1, float* point2) {

	float distance;

	distance = sqrt((point1[0] - point2[0])*(point1[0] - point2[0]) + (point1[1] - point2[1])*(point1[1] - point2[1])
						+ (point1[2] - point2[2])*(point1[2] - point2[2]));

	return distance;
}


PARAM_GROUP_START(pathCtr)
PARAM_ADD_CORE(PARAM_FLOAT, xLow, &xLow)
PARAM_ADD_CORE(PARAM_FLOAT, yLow, &yLow)
PARAM_ADD_CORE(PARAM_FLOAT, zLow, &zLow)
PARAM_ADD_CORE(PARAM_FLOAT, xHigh, &xHigh)
PARAM_ADD_CORE(PARAM_FLOAT, yHigh, &yHigh)
PARAM_ADD_CORE(PARAM_FLOAT, zHigh, &zHigh)
PARAM_ADD_CORE(PARAM_FLOAT, path_Kp, &path_Kp)
PARAM_ADD_CORE(PARAM_FLOAT, speed, &speed)
PARAM_GROUP_STOP(pathCtr)
