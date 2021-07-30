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

#ifndef __LAND_H__
#define __LAND_H__

#include "stabilizer_types.h"

void landGetSetpoint(setpoint_t *setpoint, const state_t *state);

#endif /* __LAND_H__ */
