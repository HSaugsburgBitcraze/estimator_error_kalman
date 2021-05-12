/**
 * Authored by Klaus Kefferpütz (https://www.hs-augsburg.de/fmv/kefferpuetz-klaus.html), December 2020
 *
 * Auxiliary functions developed for the implementation of
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * by Michael Hamer / Kristoffer Richardsson are re-used within this implemenation.
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
 * Error State Kalman Filter Approach combining strapdown navigation and estimated state error
 * as discribed in
 * Sola - Quaternion kinematics for the error-state Kalman Filter: https://arxiv.org/abs/1711.02508
 * =========================================================================== =====================
 *
 * MAJOR CHANGELOG:
 * 2020.12.23 Initial Commit
 *
 */

#ifndef __ERRORESTIMATOR_KALMAN_H__
#define __ERRORESTIMATOR_KALMAN_H__

#include <stdint.h>
#include "stabilizer_types.h"

void errorEstimatorKalmanInit(void);
bool errorEstimatorKalmanTest(void);
void errorEstimatorKalman(state_t *state, const uint32_t tick);

void errorEstimatorKalmanTaskInit();

/**
 * The filter supports the incorporation of additional sensors into the state estimate via the following functions:
 */
//bool errorEstimatorKalmanEnqueueTDOA(const tdoaMeasurement_t *uwb);
//bool errorEstimatorKalmanEnqueuePosition(const positionMeasurement_t *pos);
//bool errorEstimatorKalmanEnqueuePose(const poseMeasurement_t *pose);
//bool errorEstimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist);
//bool errorEstimatorKalmanEnqueueTOF(const tofMeasurement_t *tof);
//bool errorEstimatorKalmanEnqueueAbsoluteHeight(const heightMeasurement_t *height);
//bool errorEstimatorKalmanEnqueueFlow(const flowMeasurement_t *flow);
//bool errorEstimatorKalmanEnqueueYawError(const yawErrorMeasurement_t* error);
//bool errorEstimatorKalmanEnqueueSweepAngles(const sweepAngleMeasurement_t *angles);

void errorEstimatorKalmanGetEstimatedPos(point_t* pos);

/**
 * Copies 9 floats representing the current state rotation matrix
 */
void errorEstimatorKalmanGetEstimatedRot(float * rotationMatrix);

#endif // __ESTIMATOR_KALMAN_H__