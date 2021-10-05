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
* Authored by Klaus Kefferpütz, August December 2021
*
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, in version 3.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* ================================================================================================
* Error-State Unscented Kalman Filter 
* ================================================================================================
*
* MAJOR CHANGELOG:
* 2021.09.03 Initial Commit
*/


#ifndef __ERRORESTIMATOR_KALMAN_H__
#define __ERRORESTIMATOR_KALMAN_H__

#include <stdint.h>
#include "stabilizer_types.h"

void errorEstimatorKalmanInit(void);
bool errorEstimatorKalmanTest(void);
void errorEstimatorKalman(state_t *state, const uint32_t tick);


void errorEstimatorKalmanTaskInit();
bool errorEstimatorKalmanTaskTest();


void errorEstimatorKalmanGetEstimatedPos(point_t* pos);

/**
 * Copies 9 floats representing the current state rotation matrix
 */
void errorEstimatorKalmanGetEstimatedRot(float * rotationMatrix);

#endif // __ESTIMATOR_KALMAN_H__
