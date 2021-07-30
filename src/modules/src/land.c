/**
*                                             __
*    __  __ ____      ____  __  __ __________/ /_  __   _____________
*   / /_/ / ___/____ / __ `/ / / /´__  / ___/ __ \/ /  / / ___/ __  /
*  / __  (__  )/___// /_/ / /_/ / /_/ (__  ) /_/ / /__/ / /  / /_/ /
* /_/ /_/____/      \__,_/_____/\__  /____/\____/______/_/   \__  /
*                              /____/                       /____/
* Crazyflie Project
*
* Copyright (C) 2019-2021 University of Applied Sciences Augsburg
*
*/

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stabilizer_types.h"
#include "param.h"
#include "commander.h"
#include "debug.h"

static uint32_t lastCallTick = 0;
static float speedZ = 0;

void landGetSetpoint(setpoint_t *setpoint, const state_t *state) {
	// uint32_t osTick = 0;
	uint32_t osTick = xTaskGetTickCount();
	float heightTreshold = 0.2f;
	float landingTime = 1.0f / 2.0f;
	float currentHeight = state->position.z;

	if ((osTick - lastCallTick) >= configTICK_RATE_HZ / 25) {

		if(currentHeight > heightTreshold) {
			speedZ = (-currentHeight * landingTime) - 0.2f;
		}

		lastCallTick = osTick;
	}

		setpoint->mode.x = modeVelocity;
		setpoint->mode.y = modeVelocity;
		setpoint->mode.z = modeVelocity;

		setpoint->velocity.x = 0;
		setpoint->velocity.y = 0;
		setpoint->velocity.z = speedZ;

		setpoint->mode.yaw = modeVelocity;

	if(currentHeight <= heightTreshold) {
		setpoint->velocity.z = speedZ;
		setpoint->timestamp = 0;
		commanderSetSetpoint(setpoint, COMMANDER_PRIORITY_EXTRX);
		paramSetInt(paramGetVarId("commander", "enLand"), 0);
	}
}
