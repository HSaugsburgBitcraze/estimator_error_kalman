/**
 * Authored by Klaus Kefferpütz (https://www.hs-augsburg.de/fmv/kefferpuetz-klaus.html), December 2020
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
 * Error State Kalman Filter Approach combining strapdown navigation and estimated state error
 * as discribed in
 * Sola - Quaternion kinematics for the error-state Kalman Filter: https://arxiv.org/abs/1711.02508
 * =========================================================================== =====================
 *
 * MAJOR CHANGELOG:
 * 2020.12.23 Initial Commit
 *
 */
//#include "kalman_core.h"
#include "errorEstimator_kalman.h"
#include "estimator.h"
#include "kalman_supervisor.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"
#include "outlierFilter.h"

#include "statsCnt.h"

#define DEBUG_MODULE "ESTKALMAN"
#include "debug.h"

// Use the robust implementations of TWR and TDoA, off by default but can be turned on through a parameter.
// The robust implementations use around 10% more CPU VS the standard flavours
//static bool robustTwr = false;
//static bool robustTdoa = false;

// #define KALMAN_USE_BARO_UPDATE

// Semaphore to signal that we got data from the stabilzer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

#define PREDICT_RATE RATE_500_HZ // this is slower than the IMU update rate of 500Hz
#define BARO_RATE RATE_25_HZ

#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */
static bool isInit = false;
static Axis3f accAccumulator;
static Axis3f gyroAccumulator;
static float baroAslAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static uint32_t baroAccumulatorCount;
static Axis3f accLatest;
static Axis3f gyroLatest;

// Data used to enable the task and stabilizer loop to run with minimal locking
static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilzer when needed.

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(baroUpdateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

//#ifdef KALMAN_USE_BARO_UPDATE
//static const bool useBaroUpdate = true;
//#else
//static const bool useBaroUpdate = false;
//#endif

// for error filter version
#define DIM_FILTER 9
#define DIM_STRAPDOWN 10

static float stateNav[DIM_STRAPDOWN];
static bool initializedNav = false;
static int32_t numberInitSteps = 1000;
static Axis3f accBias;
static Axis3f omegaBias;
static float baroAslBias;

static float covNavFilter[DIM_FILTER][DIM_FILTER];
static arm_matrix_instance_f32 covNavFilter_Mat = {DIM_FILTER, DIM_FILTER, (float *)covNavFilter};

static float accNed[3];
static float dcm[3][3],dcmTp[3][3];

static float stdDevInitialPosition_xy = 50.0f;// 50.0f;// were static const
static float stdDevInitialPosition_z  = 0.5f; //50.0f;// were static const
static float stdDevInitialVelocity    = 0.0001f;// were static const
static float stdDevInitialAtt         = 0.01f;  // were static const

// variances of process noise (accelerometer and gyroscope noise)
static float procA_h    = 4.4755e-6f; //0.05059f*0.05059f;// 0.1f*0.1f;
static float procA_z    = 8.5343e-6f; //0.05059f*0.05059f;//0.1f*0.1f;
static float procVel_h  = 0.00f;
static float procVel_z  = 0.00f;
static float procRate_h = 9.2495e-7f; //0.0007f*0.0007f; //0.1f*0.1f;
static float procRate_z = 2.3124e-7f; //0.0007f*0.0007f; // 0.1f*0.1f;

// measurement noise baro - variance
static float measNoiseBaro = 0.7f*0.7f;

// quality gates
//static float qualGateTof  = 100.63f; // for CF 2.0
static float qualGateTof  = 100.63f; // for CF 2.1
static float qualGateFlow = 1000.63f;

static float qualGateTdoa = 1000.63f; // should not be lowered currently
static float qualGateBaro = 1.64f;

//static bool activateFlowDeck = true;
static uint32_t nanCounterFilter = 0;

static float accLog[3];
static float omega[3];
static float eulerOut[3];
static float pred_NX;
static float pred_NY;
static float meas_NX;
static float meas_NY;
static float predDist;
static float measDist;
static float baroOut;

static float PxOut;
static float PvxOut;
static float PattxOut;
static uint32_t tdoaCount;

static bool useNavigationFilter = true;
static bool resetNavigation = true;


static void updateStrapdownAlgorithm(float *stateNav, Axis3f* accAverage, Axis3f* gyroAverage, float dt);
static void predictNavigationFilter(float *stateNav, Axis3f *acc, Axis3f *gyro, float dt);
static bool updateNavigationFilter(arm_matrix_instance_f32 *Hk_Mat, float *innovation, float *R, float qualityGate);
static bool updateNavigationFilter2(arm_matrix_instance_f32 *Hk_Mat,float *H2, float *innovation, float *R, float qualityGate);

//static bool updateNavigationFilter2D(arm_matrix_instance_f32 *Hk_Mat, float *innovation, float *R, float qualityGate);
static void navigationInit(void);
static void updateWithBaro( float baroAsl);
static void updateWithTofMeasurement(tofMeasurement_t *tof);
static void updateWithFlowMeasurement(flowMeasurement_t *flow, Axis3f *omegaBody);
static void updateWithTdoaMeasurement(tdoaMeasurement_t *tdoa);
static void resetNavigationStates( float *errorState);

static bool updateQueuedMeasurements(const uint32_t tick, Axis3f* gyroAverage);

static void quatToEuler(float *quat, float *eulerAngles);
static void quatFromAtt(float *attVec, float *quat);
static void transposeMatrix( float *mat, float *matTp);
static void directionCosineMatrix(float *quat, float *dcm);
static void quatToEuler(float *quat, float *eulerAngles);
static void multQuat(float *q1, float *q2, float *quatRes);
/**
 * Supporting and utility functions
 */
//
//static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
//  { configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
//static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
//  { configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
//static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
//  { configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
//static inline float arm_sqrt(float32_t in)
//  { float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }
//

static void errorKalmanTask(void* parameters);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(errorKalmanTask, 7 * configMINIMAL_STACK_SIZE);
//STATIC_MEM_TASK_ALLOC(errorKalmanTask, 9 * configMINIMAL_STACK_SIZE);

// --------------------------------------------------

// Called one time during system startup
void errorEstimatorKalmanTaskInit() {
  vSemaphoreCreateBinary(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  navigationInit();

  STATIC_MEM_TASK_CREATE(errorKalmanTask, errorKalmanTask, ERROR_KALMAN_TASK_NAME, NULL, ERROR_KALMAN_TASK_PRI);

  isInit = true;
}

bool errorEstimatorKalmanTaskTest() {
  return isInit;
}

static void errorKalmanTask(void* parameters) {
  systemWaitStart();

  uint32_t lastPrediction = xTaskGetTickCount();
  uint32_t nextPrediction = xTaskGetTickCount();
  //uint32_t lastPNUpdate = xTaskGetTickCount();
  //uint32_t nextBaroUpdate = xTaskGetTickCount();

  // Tracks whether an update to the state has been made, and the state therefore requires finalization
  //bool doneUpdate = false;

  PattxOut = 0.0f;
  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

    // compute bias error for first ticks averaging the measurements
    if ((accAccumulatorCount>numberInitSteps)&&(gyroAccumulatorCount>numberInitSteps)&&(!initializedNav)){

    	accBias.x = accAccumulator.x/((float)accAccumulatorCount);
    	accBias.y = accAccumulator.y/((float)accAccumulatorCount);
    	accBias.z = accAccumulator.z/((float)accAccumulatorCount)-1.0f;   // CF is on ground, so compensate gravity to not enter bias computation

    	omegaBias.x = gyroAccumulator.x/((float)gyroAccumulatorCount);
    	omegaBias.y = gyroAccumulator.y/((float)gyroAccumulatorCount);
    	omegaBias.z = gyroAccumulator.z/((float)gyroAccumulatorCount);

    	baroAslBias = baroAslAccumulator / ((float)baroAccumulatorCount);

    	initializedNav = true;

    	accAccumulator = (Axis3f){.axis={0}};
    	accAccumulatorCount = 0;
    	gyroAccumulator = (Axis3f){.axis={0}};
    	gyroAccumulatorCount = 0;
    	baroAslAccumulator   = 0.0f;
    	baroAccumulatorCount = 0;

    	navigationInit();
    	lastPrediction = xTaskGetTickCount();
    }

    // If the client triggers an estimator reset via parameter update "kalman.resetEstimation"
    if (resetNavigation) {
      errorEstimatorKalmanInit();
      resetNavigation = false;

      // set bias accumalation counters to zero
      initializedNav = false;
      accBias   = (Axis3f){.axis={0}};
      accAccumulatorCount = 0;
      omegaBias = (Axis3f){.axis={0}};
      gyroAccumulatorCount = 0;

      accNed[0] = 0.0f;
      accNed[1] = 0.0f;
      accNed[2] = -1.0f;

      tdoaCount = 0;
      nanCounterFilter = 0;
    }

    // Tracks whether an update to the state has been made, and the state therefore requires finalization
    uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...
	Axis3f gyroAverage;
	if(initializedNav && (accAccumulatorCount>0)&&(gyroAccumulatorCount>0)){

		// Run the system dynamics to predict the state forward.
		if (osTick >= nextPrediction) { // update at the PREDICT_RATE
			float dt = T2S(osTick - lastPrediction);

			xSemaphoreTake(dataMutex, portMAX_DELAY);

			// gyro is in deg/sec but the estimator requires rad/sec

			gyroAverage.x = (gyroAccumulator.x/((float)gyroAccumulatorCount)-omegaBias.x ) * DEG_TO_RAD;
			gyroAverage.y = (gyroAccumulator.y/((float)gyroAccumulatorCount)-omegaBias.y ) * DEG_TO_RAD;
			gyroAverage.z = (gyroAccumulator.z/((float)gyroAccumulatorCount)-omegaBias.z ) * DEG_TO_RAD;

			// accelerometer is in Gs but the estimator requires ms^-2
			Axis3f accAverage;
			accAverage.x = (accAccumulator.x /((float)accAccumulatorCount)-accBias.x)* GRAVITY_MAGNITUDE;
			accAverage.y = (accAccumulator.y /((float)accAccumulatorCount)-accBias.y) * GRAVITY_MAGNITUDE;
			accAverage.z = (accAccumulator.z /((float)accAccumulatorCount)-accBias.z) * GRAVITY_MAGNITUDE;

			accAccumulator = (Axis3f){.axis={0}};
			accAccumulatorCount = 0;
			gyroAccumulator = (Axis3f){.axis={0}};
			gyroAccumulatorCount = 0;

			xSemaphoreGive(dataMutex);

			//prediction of strapdown navigation, setting also accumlator back to zero!
			updateStrapdownAlgorithm(&stateNav[0], &accAverage, &gyroAverage, dt);

			// prediction step of error state Kalman Filter
			predictNavigationFilter(&stateNav[0], &accAverage, &gyroAverage, dt);

			accLog[0] = accAverage.x; // Logging Data
			accLog[1] = accAverage.y;
			accLog[2] = accAverage.z;

			omega[0] = gyroAverage.x; // Logging Data
			omega[1] = gyroAverage.y;
			omega[2] = gyroAverage.z;

			//PxOut    =  covNavFilter[0][0]; // Logging Data
			//PvxOut   =  covNavFilter[3][3];

			lastPrediction = osTick;
			STATS_CNT_RATE_EVENT(&predictionCounter);

			nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);

			//Axis3f gyro;
			xSemaphoreTake(dataMutex, portMAX_DELAY);
			//memcpy(&gyro, &gyroSnapshot, sizeof(gyro));
			xSemaphoreGive(dataMutex);

		} // end if update at the PREDICT_RATE
	} // end if bias errors are initialized

	directionCosineMatrix(&stateNav[6], &dcm[0][0]);
	transposeMatrix(&dcm[0][0], &dcmTp[0][0]);

  if(updateQueuedMeasurements(osTick, &gyroAverage)) {
   	STATS_CNT_RATE_EVENT(&updateCounter);
   	//      doneUpdate = true;
  }
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  quatToEuler(&stateNav[6], &eulerOut[0]);

	eulerOut[0] = eulerOut[0]*RAD_TO_DEG;
	eulerOut[1] = eulerOut[1]*RAD_TO_DEG;
	eulerOut[2] = eulerOut[2]*RAD_TO_DEG;



    // output global position
    taskEstimatorState.position.timestamp = osTick;
    taskEstimatorState.position.x = stateNav[0];
    taskEstimatorState.position.y = stateNav[1];
    taskEstimatorState.position.z = stateNav[2];

    // output global velocity
    taskEstimatorState.velocity.timestamp = osTick;
    taskEstimatorState.velocity.x = stateNav[3];
    taskEstimatorState.velocity.y = stateNav[4];
    taskEstimatorState.velocity.z = stateNav[5];

    taskEstimatorState.acc.timestamp = osTick;
    // transform into lab frame
    accNed[0] = (dcmTp[0][0]*accLog[0]+dcmTp[0][1]*accLog[1]+dcmTp[0][2]*accLog[2])/GRAVITY_MAGNITUDE;
    accNed[1] = (dcmTp[1][0]*accLog[0]+dcmTp[1][1]*accLog[1]+dcmTp[1][2]*accLog[2])/GRAVITY_MAGNITUDE;
    accNed[2] = (dcmTp[2][0]*accLog[0]+dcmTp[2][1]*accLog[1]+dcmTp[2][2]*accLog[2]-GRAVITY_MAGNITUDE)/GRAVITY_MAGNITUDE;

    taskEstimatorState.acc.x = accNed[0];///GRAVITY_MAGNITUDE;
    taskEstimatorState.acc.y = accNed[1];///GRAVITY_MAGNITUDE;
    taskEstimatorState.acc.z = accNed[2];///GRAVITY_MAGNITUDE;

    // from https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/
    //roll and yaw are clockwise rotating around the axis looking from the origin (right-hand-thumb)
    //pitch are counter-clockwise rotating around the axis looking from the origin (left-hand-thumb)
    taskEstimatorState.attitude.timestamp = osTick;
    taskEstimatorState.attitude.roll  = eulerOut[0];
    taskEstimatorState.attitude.pitch = -eulerOut[1];
    taskEstimatorState.attitude.yaw   = eulerOut[2];

    taskEstimatorState.attitudeQuaternion.timestamp = osTick;
    taskEstimatorState.attitudeQuaternion.w = stateNav[6];
    taskEstimatorState.attitudeQuaternion.x = stateNav[7];
    taskEstimatorState.attitudeQuaternion.y = stateNav[8];
    taskEstimatorState.attitudeQuaternion.z = stateNav[9];

    xSemaphoreGive(dataMutex);
  } // END infinite loop
}

//void errorEstimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
void errorEstimatorKalman(state_t *state, const uint32_t tick)
{
  systemWaitStart();
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Copy the latest state, calculated by the task
  memcpy(state, &taskEstimatorState, sizeof(state_t));
  xSemaphoreGive(dataMutex);

  xSemaphoreGive(runTaskSemaphore);
}


// Called when this estimator is activated
void errorEstimatorKalmanInit(void) {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  accAccumulator = (Axis3f){.axis={0}};
  gyroAccumulator = (Axis3f){.axis={0}};
  baroAslAccumulator = 0;

  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  baroAccumulatorCount = 0;
  xSemaphoreGive(dataMutex);

  navigationInit();

  isInit = true;
}

bool errorEstimatorKalmanTest(void)
{
  return isInit;
}

void errorEstimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = stateNav[0];
  pos->y = stateNav[1];
  pos->z = stateNav[2];
}

void errorEstimatorKalmanGetEstimatedRot(float * rotationMatrix) {
//  memcpy(rotationMatrix, coreData.R, 9*sizeof(float));
}

//update step for strapdown navigation
static void updateStrapdownAlgorithm(float *stateNav, Axis3f* accAverage, Axis3f* gyroAverage, float dt)
{
	float stateNew[DIM_STRAPDOWN];
	float quat[4] = {stateNav[6], stateNav[7], stateNav[8], stateNav[9]};
	float accNed[3], deltaQ[4], quatNew[4];
	float deltaOmAtt[3] = {gyroAverage->x*dt, gyroAverage->y*dt, gyroAverage->z*dt};

	// direction cosine matrix for current attitude and  transformation matrix from body to ned
	directionCosineMatrix(&quat[0], &dcm[0][0]);
	transposeMatrix(&dcm[0][0], &dcmTp[0][0]);

	// transform into ned frame
	accNed[0] = dcmTp[0][0]*accAverage->x+dcmTp[0][1]*accAverage->y+dcmTp[0][2]*accAverage->z;
	accNed[1] = dcmTp[1][0]*accAverage->x+dcmTp[1][1]*accAverage->y+dcmTp[1][2]*accAverage->z;
	accNed[2] = dcmTp[2][0]*accAverage->x+dcmTp[2][1]*accAverage->y+dcmTp[2][2]*accAverage->z-GRAVITY_MAGNITUDE;

	// position update
	stateNew[0] = stateNav[0]+stateNav[3]*dt;//+0.5f*accNed[0]*dt*dt;
	stateNew[1] = stateNav[1]+stateNav[4]*dt;//+0.5f*accNed[1]*dt*dt;
	stateNew[2] = stateNav[2]+stateNav[5]*dt;//+0.5f*accNed[2]*dt*dt;

	// velocity update
	stateNew[3] = stateNav[3]+accNed[0]*dt;
	stateNew[4] = stateNav[4]+accNed[1]*dt;
	stateNew[5] = stateNav[5]+accNed[2]*dt;

	quat[0] = stateNav[6];
	quat[1] = stateNav[7];
	quat[2] = stateNav[8];
	quat[3] = stateNav[9];

	// compute delta attitude from omega Measurements
	quatFromAtt( &deltaOmAtt[0], &deltaQ[0]);

	// update quaternion
	float scale;
	multQuat(&quat[0], &deltaQ[0], &quatNew[0]);
	scale = quatNew[0]*quatNew[0]+quatNew[1]*quatNew[1]+quatNew[2]*quatNew[2]+quatNew[3]*quatNew[3];

	// normalize quaternion
	stateNew[6] = 1/scale*quatNew[0];
	stateNew[7] = 1/scale*quatNew[1];
	stateNew[8] = 1/scale*quatNew[2];
	stateNew[9] = 1/scale*quatNew[3];

	uint32_t ii;
	for(ii=0; ii<DIM_STRAPDOWN; ii++){
		stateNav[ii]=stateNew[ii];
	}
}

// reset step for navigation Filter, called externally (via Basestation and navigationInit)
static void navigationInit(void){
	// initialize state of strapdown navigation algorithm
	uint32_t ii, jj;
	for(ii=0; ii<DIM_STRAPDOWN; ii++){
		stateNav[ii]=0.0f;
	}
	stateNav[6]     = 1.0f;   // no rotation initially

	// initialize covariance matrix of navigation Filter
	for(ii=0; ii<DIM_FILTER; ii++){
	   for(jj=0; jj<DIM_FILTER; jj++){
		   covNavFilter[ii][jj] = 0.0f;
	   }
	}

	// set initial parameters
	covNavFilter[0][0] = stdDevInitialPosition_xy*stdDevInitialPosition_xy;
	covNavFilter[1][1] = stdDevInitialPosition_xy*stdDevInitialPosition_xy;
	covNavFilter[2][2] = stdDevInitialPosition_z*stdDevInitialPosition_z;

	covNavFilter[3][3] = stdDevInitialVelocity*stdDevInitialVelocity;
    covNavFilter[4][4] = stdDevInitialVelocity*stdDevInitialVelocity;
    covNavFilter[5][5] = stdDevInitialVelocity*stdDevInitialVelocity;

	covNavFilter[6][6] = stdDevInitialAtt*stdDevInitialAtt;
	covNavFilter[7][7] = stdDevInitialAtt*stdDevInitialAtt;
	covNavFilter[8][8] = stdDevInitialAtt*stdDevInitialAtt;
}

// prediction step of error Kalman Filter
static void predictNavigationFilter(float *stateNav, Axis3f *acc, Axis3f *gyro, float dt){
	float accTs[3] = {acc->x*dt, acc->y*dt, acc->z*dt};
	float omegaTs[3] = {gyro->x*dt, gyro->y*dt, gyro->z*dt};

	float errorTransMat[DIM_FILTER][DIM_FILTER], errorTransMatTp[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 errorTransMat_Mat = {DIM_FILTER, DIM_FILTER, (float *)errorTransMat};
	arm_matrix_instance_f32 errorTransMatTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)errorTransMatTp};
	float errorTrMatTimesCovMat[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 errorTrMatTimesCovMat_Mat = {DIM_FILTER, DIM_FILTER, (float *)errorTrMatTimesCovMat};

	// compute error State transition matrix
	uint16_t ii, jj;
	for(ii=0; ii<DIM_FILTER; ii++){
	   for(jj=0; jj<DIM_FILTER; jj++){
		   errorTransMat[ii][jj] = 0.0f;
	   }
	}

	//_________________________________________________________
	//Compute transition matrix for error state
	//_________________________________________________________
	// this is row [  I   I*Ts   0    ]
	errorTransMat[0][0] = 1.0f;
	errorTransMat[0][3] = dt;
	errorTransMat[1][1] = 1.0f;
	errorTransMat[1][4] = dt;
	errorTransMat[2][2] = 1.0f;
	errorTransMat[2][5] = dt;

	// this is row [ 0    I   -Tbn*(I-[a]_x*Ts ) ]
	errorTransMat[3][3]  = 1.0f;
	errorTransMat[3][6]  = -( dcmTp[0][1]*accTs[2]-dcmTp[0][2]*accTs[1]);
	errorTransMat[3][7]  = -(-dcmTp[0][0]*accTs[2]+dcmTp[0][2]*accTs[0]);
	errorTransMat[3][8]  = -( dcmTp[0][0]*accTs[1]-dcmTp[0][1]*accTs[0]);
  	errorTransMat[4][4]  = 1.0f;
	errorTransMat[4][6]  = -( dcmTp[1][1]*accTs[2]-dcmTp[1][2]*accTs[1]);
	errorTransMat[4][7]  = -(-dcmTp[1][0]*accTs[2]+dcmTp[1][2]*accTs[0]);
	errorTransMat[4][8]  = -( dcmTp[1][0]*accTs[1]-dcmTp[1][1]*accTs[0]);
  	errorTransMat[5][5]  = 1.0f;
	errorTransMat[5][6]  = -( dcmTp[2][1]*accTs[2]-dcmTp[2][2]*accTs[1]);
	errorTransMat[5][7]  = -(-dcmTp[2][0]*accTs[2]+dcmTp[2][2]*accTs[0]);
	errorTransMat[5][8]  = -( dcmTp[2][0]*accTs[1]-dcmTp[2][1]*accTs[0]);


	// this is row [  0    0  (I-[omega]_x*Ts) ]
	errorTransMat[6][6]	 =  1.0f;
	errorTransMat[6][7]	 =  omegaTs[2];
	errorTransMat[6][8]	 = -omegaTs[1];
	errorTransMat[7][6]	 = -omegaTs[2];
	errorTransMat[7][7]	 =  1.0f;
	errorTransMat[7][8]	 =  omegaTs[0];
	errorTransMat[8][6]	 =  omegaTs[1];
	errorTransMat[8][7]	 = -omegaTs[0];
	errorTransMat[8][8]	 =  1.0f;

	//compute errorTransition*covMat*errorTransMatTp
	mat_trans(&errorTransMat_Mat, &errorTransMatTp_Mat);
	mat_mult(&errorTransMat_Mat, &covNavFilter_Mat, &errorTrMatTimesCovMat_Mat);
	mat_mult(&errorTrMatTimesCovMat_Mat, &errorTransMatTp_Mat, &covNavFilter_Mat);

	covNavFilter[3][4] += procA_h*dt*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+procA_z*dt*dcmTp[0][2]*dcmTp[1][2];
	covNavFilter[4][3] += procA_h*dt*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+procA_z*dt*dcmTp[0][2]*dcmTp[1][2];

	covNavFilter[4][5] += procA_h*dt*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+procA_z*dt*dcmTp[1][2]*dcmTp[2][2];
	covNavFilter[5][4] += procA_h*dt*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+procA_z*dt*dcmTp[1][2]*dcmTp[2][2];

	covNavFilter[3][3] += procA_h*dt*(dcmTp[0][0]*dcmTp[0][0]+dcmTp[0][1]*dcmTp[0][1])+procA_z*dt*dcmTp[0][2]*dcmTp[0][2];
	covNavFilter[4][4] += procA_h*dt*(dcmTp[1][0]*dcmTp[1][0]+dcmTp[1][1]*dcmTp[1][1])+procA_z*dt*dcmTp[1][2]*dcmTp[1][2];
	covNavFilter[5][5] += procA_h*dt*(dcmTp[2][0]*dcmTp[2][0]+dcmTp[2][1]*dcmTp[2][1])+procA_z*dt*dcmTp[2][2]*dcmTp[2][2];


	covNavFilter[6][6] += procRate_h * dt;
	covNavFilter[7][7] += procRate_h * dt;
	covNavFilter[8][8] += procRate_z * dt;

	// add process variances to covariance matrix
//	covNavFilter[0][0] += powf(0.5f*procA_h*dt*dt + procVel_h*dt + 0.0f, 2);
//	covNavFilter[1][1] += powf(0.5f*procA_h*dt*dt + procVel_h*dt + 0.0f, 2);
//	covNavFilter[2][2] += powf(0.5f*procA_z*dt*dt + procVel_z*dt + 0.0f, 2);
//	covNavFilter[3][3] += powf(procA_h*dt + procVel_h, 2);
//	covNavFilter[4][4] += powf(procA_h*dt + procVel_h, 2);
//	covNavFilter[5][5] += powf(procA_z*dt + procVel_z, 2);
//	covNavFilter[6][6] += powf(procRate_h * dt, 2);
//	covNavFilter[7][7] += powf(procRate_h * dt, 2);
//	covNavFilter[8][8] += powf(procRate_z * dt, 2);

//	// diagonal covariance matrix
//	float preFactor_h = 0.333f*procA_h*dt*dt*dt+procVel_h*dt;
//	float preFactor_z = 0.333f*procA_z*dt*dt*dt+procVel_z*dt;
//
//	covNavFilter[0][0] += preFactor_h;
//	covNavFilter[1][1] += preFactor_h;
//	covNavFilter[2][2] += preFactor_z;
//
//	preFactor_h = procA_h*dt;
//	preFactor_z = procA_z*dt;

//	covNavFilter[0][0] += preFactor_h;
//	covNavFilter[1][1] += preFactor_h;
//	covNavFilter[2][2] += preFactor_z;


	// computed Qk matrix
	// add process variances to covariance matrix
	// float preFactor_h = 0.5f*procA_h*dt*dt*dt;
	// float preFactor_z = 0.5f*procA_z*dt*dt*dt;

  // covNavFilter[0][0] += preFactor_h*(dcmTp[0][0]*dcmTp[0][0]+dcmTp[0][1]*dcmTp[0][1])+preFactor_z*(dcmTp[0][2]*dcmTp[0][2]);
  // covNavFilter[1][1] += preFactor_h*(dcmTp[1][0]*dcmTp[1][0]+dcmTp[1][1]*dcmTp[1][1])+preFactor_z*(dcmTp[1][2]*dcmTp[1][2]);
	// covNavFilter[2][2] += preFactor_h*(dcmTp[2][0]*dcmTp[2][0]+dcmTp[2][1]*dcmTp[2][1])+preFactor_z*(dcmTp[2][2]*dcmTp[2][2]);

  // covNavFilter[0][1] += preFactor_h*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+preFactor_z*(dcmTp[0][2]*dcmTp[1][2]);
	// covNavFilter[1][0] += preFactor_h*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+preFactor_z*(dcmTp[0][2]*dcmTp[1][2]);

  // covNavFilter[0][2] += preFactor_h*(dcmTp[0][0]*dcmTp[2][0]+dcmTp[0][1]*dcmTp[2][1])+preFactor_z*(dcmTp[0][2]*dcmTp[2][2]);
	// covNavFilter[2][0] += preFactor_h*(dcmTp[0][0]*dcmTp[2][0]+dcmTp[0][1]*dcmTp[2][1])+preFactor_z*(dcmTp[0][2]*dcmTp[2][2]);

	// covNavFilter[1][2] += preFactor_h*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+preFactor_z*(dcmTp[1][2]*dcmTp[2][2]);
	// covNavFilter[2][1] += preFactor_h*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+preFactor_z*(dcmTp[1][2]*dcmTp[2][2]);

	// preFactor_h = 0.5f*procA_h*dt*dt;
	// preFactor_z = 0.5f*procA_z*dt*dt;


  //  covNavFilter[0][3] += preFactor_h*(dcmTp[0][0]*dcmTp[0][0]+dcmTp[0][1]*dcmTp[0][1])+preFactor_z*(dcmTp[0][2]*dcmTp[0][2]);
  //  covNavFilter[3][0] += preFactor_h*(dcmTp[0][0]*dcmTp[0][0]+dcmTp[0][1]*dcmTp[0][1])+preFactor_z*(dcmTp[0][2]*dcmTp[0][2]);

  //  covNavFilter[0][4] += preFactor_h*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+preFactor_z*(dcmTp[0][2]*dcmTp[1][2]);
  //  covNavFilter[1][3] += preFactor_h*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+preFactor_z*(dcmTp[0][2]*dcmTp[1][2]);
  //  covNavFilter[3][1] += preFactor_h*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+preFactor_z*(dcmTp[0][2]*dcmTp[1][2]);
  //  covNavFilter[4][0] += preFactor_h*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+preFactor_z*(dcmTp[0][2]*dcmTp[1][2]);

  //  covNavFilter[0][5] += preFactor_h*(dcmTp[0][0]*dcmTp[2][0]+dcmTp[0][1]*dcmTp[2][1])+preFactor_z*(dcmTp[0][2]*dcmTp[2][2]);
  //  covNavFilter[2][3] += preFactor_h*(dcmTp[0][0]*dcmTp[2][0]+dcmTp[0][1]*dcmTp[2][1])+preFactor_z*(dcmTp[0][2]*dcmTp[2][2]);
  //  covNavFilter[3][2] += preFactor_h*(dcmTp[0][0]*dcmTp[2][0]+dcmTp[0][1]*dcmTp[2][1])+preFactor_z*(dcmTp[0][2]*dcmTp[2][2]);
  //  covNavFilter[5][0] += preFactor_h*(dcmTp[0][0]*dcmTp[2][0]+dcmTp[0][1]*dcmTp[2][1])+preFactor_z*(dcmTp[0][2]*dcmTp[2][2]);

  //  covNavFilter[1][4] += preFactor_h*(dcmTp[1][0]*dcmTp[1][0]+dcmTp[1][1]*dcmTp[1][1])+preFactor_z*(dcmTp[1][2]*dcmTp[1][2]);
  //  covNavFilter[4][1] += preFactor_h*(dcmTp[1][0]*dcmTp[1][0]+dcmTp[1][1]*dcmTp[1][1])+preFactor_z*(dcmTp[1][2]*dcmTp[1][2]);

  //  covNavFilter[1][5] += preFactor_h*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+preFactor_z*(dcmTp[1][2]*dcmTp[2][2]);
  //  covNavFilter[2][4] += preFactor_h*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+preFactor_z*(dcmTp[1][2]*dcmTp[2][2]);
  //  covNavFilter[4][2] += preFactor_h*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+preFactor_z*(dcmTp[1][2]*dcmTp[2][2]);
  //  covNavFilter[5][1] += preFactor_h*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+preFactor_z*(dcmTp[1][2]*dcmTp[2][2]);

  //  covNavFilter[2][5] += preFactor_h*(dcmTp[2][0]*dcmTp[2][0]+dcmTp[2][1]*dcmTp[2][1])+preFactor_z*(dcmTp[2][2]*dcmTp[2][2]);
  //  covNavFilter[5][2] += preFactor_h*(dcmTp[2][0]*dcmTp[2][0]+dcmTp[2][1]*dcmTp[2][1])+preFactor_z*(dcmTp[2][2]*dcmTp[2][2]);

  //  preFactor_h = procA_h*dt;
  //  preFactor_z = procA_z*dt;
  //  covNavFilter[3][3] += preFactor_h*(dcmTp[0][0]*dcmTp[0][0]+dcmTp[0][1]*dcmTp[0][1])+preFactor_z*(dcmTp[0][2]*dcmTp[0][2]);
  //  covNavFilter[4][4] += preFactor_h*(dcmTp[1][0]*dcmTp[1][0]+dcmTp[1][1]*dcmTp[1][1])+preFactor_z*(dcmTp[1][2]*dcmTp[1][2]);
  //  covNavFilter[5][5] += preFactor_z*(dcmTp[2][0]*dcmTp[2][0]+dcmTp[2][1]*dcmTp[2][1])+preFactor_z*(dcmTp[2][2]*dcmTp[2][2]);

  //  covNavFilter[3][4] += preFactor_h*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+preFactor_z*(dcmTp[0][2]*dcmTp[1][2]);
  //  covNavFilter[4][3] += preFactor_h*(dcmTp[0][0]*dcmTp[1][0]+dcmTp[0][1]*dcmTp[1][1])+preFactor_z*(dcmTp[0][2]*dcmTp[1][2]);

  //  covNavFilter[3][5] += preFactor_h*(dcmTp[0][0]*dcmTp[2][0]+dcmTp[0][1]*dcmTp[2][1])+preFactor_z*(dcmTp[0][2]*dcmTp[2][2]);
  //  covNavFilter[5][3] += preFactor_h*(dcmTp[0][0]*dcmTp[2][0]+dcmTp[0][1]*dcmTp[2][1])+preFactor_z*(dcmTp[0][2]*dcmTp[2][2]);

  //  covNavFilter[4][5] += preFactor_h*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+preFactor_z*(dcmTp[1][2]*dcmTp[2][2]);
  //  covNavFilter[5][4] += preFactor_h*(dcmTp[1][0]*dcmTp[2][0]+dcmTp[1][1]*dcmTp[2][1])+preFactor_z*(dcmTp[1][2]*dcmTp[2][2]);

  //  covNavFilter[6][6] += procRate_h * dt;
	// covNavFilter[7][7] += procRate_h * dt;
	// covNavFilter[8][8] += procRate_z * dt;
}

// Common update step for filter -> todo incorporate multidimensional measurement
//static bool updateNavigationFilter(float *H, float *innovation, float *R, float *errorState, float qualityGate){
static bool updateNavigationFilter(arm_matrix_instance_f32 *Hk_Mat, float *innovation, float *R, float qualityGate){
	float HkTp[DIM_FILTER][1];
	float errorState[DIM_FILTER];

	arm_matrix_instance_f32 HkTp_Mat = { DIM_FILTER, 1, (float *)HkTp};

	float PTimesHkTp[DIM_FILTER][1];
	arm_matrix_instance_f32 PTimesHkTp_Mat = { DIM_FILTER, 1, (float *)PTimesHkTp};
	float Sk[1][1];
	arm_matrix_instance_f32 Sk_Mat = {1, 1, (float *)Sk};
	float SkInv[1][1];
	arm_matrix_instance_f32 SkInv_Mat = {1, 1, (float *)SkInv};

	float Kk[DIM_FILTER][1];
	arm_matrix_instance_f32 Kk_Mat = {DIM_FILTER, 1, (float *)Kk};
	float KkTp[1][DIM_FILTER];
	arm_matrix_instance_f32 KkTp_Mat = {1, DIM_FILTER, (float *)KkTp};
	float KkRKkTp[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 KkRKkTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)KkRKkTp};

	float IMinusKkTimesHk[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 IMinusKkTimesHk_Mat = {DIM_FILTER, DIM_FILTER, (float *)IMinusKkTimesHk};
	float IMinusKkTimesHkTp[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 IMinusKkTimesHkTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)IMinusKkTimesHkTp};

	float IKkHkP[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 IKkHkP_Mat = {DIM_FILTER, DIM_FILTER, (float *)IKkHkP};

	float covNew[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 covNew_Mat = {DIM_FILTER, DIM_FILTER, (float *)covNew};
	float covNewTp[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 covNewTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)covNewTp};

	ASSERT(Hk_Mat->numRows == 1);
	ASSERT(Hk_Mat->numCols == DIM_FILTER);

	uint16_t ii, jj;

	//compute Hk*covMat*HkTp
	mat_trans(Hk_Mat, &HkTp_Mat);
	mat_mult(&covNavFilter_Mat, &HkTp_Mat, &PTimesHkTp_Mat);
	mat_mult(Hk_Mat, &PTimesHkTp_Mat, &Sk_Mat);

	//compute Sk = Hk*covMat*HkTp+R
	for(ii=0; ii<1; ii++){
		for(jj=0; jj<1; jj++){
			Sk[ii][jj] = Sk[ii][jj]+R[ii*1+jj]; // check ok
		}
	}
	float innoCheck = innovation[0]*innovation[0]/Sk[0][0];

	if(innoCheck<qualityGate){
		// compute Kalman gain: Kk=covMat*HkTp*Sk^(-1)
		mat_inv(&Sk_Mat, &SkInv_Mat);
		mat_mult(&PTimesHkTp_Mat, &SkInv_Mat, &Kk_Mat);

		for(ii=0; ii<DIM_FILTER; ii++){
			errorState[ii] = Kk[ii][0]*innovation[0];
		}

		// correct covariance matrix:
		mat_mult(&Kk_Mat, Hk_Mat, &IMinusKkTimesHk_Mat);

		//compute I-Kk*Hk
		for(ii=0; ii<DIM_FILTER; ii++){
			for(jj=0; jj<DIM_FILTER; jj++){
				IMinusKkTimesHk[ii][jj] = -IMinusKkTimesHk[ii][jj];
			}
		}
		for(ii=0; ii<DIM_FILTER; ii++){
			IMinusKkTimesHk[ii][ii] = 1.0f + IMinusKkTimesHk[ii][ii];
		}

		mat_trans(&Kk_Mat, &KkTp_Mat);
		mat_mult(&Kk_Mat, &KkTp_Mat, &KkRKkTp_Mat);

		mat_trans(&IMinusKkTimesHk_Mat, &IMinusKkTimesHkTp_Mat);
		mat_mult(&IMinusKkTimesHk_Mat, &covNavFilter_Mat, &IKkHkP_Mat);
		mat_mult(&IKkHkP_Mat, &IMinusKkTimesHkTp_Mat, &covNew_Mat);

		for(ii=0; ii<DIM_FILTER; ii++){
			for(jj=0; jj<DIM_FILTER; jj++){
				covNew[ii][jj] = covNew[ii][jj]+KkRKkTp[ii][jj]*R[0];
			}
		}

		if ((isnan(errorState[0]))||(isnan(errorState[1]))||
				(isnan(errorState[2]))||(isnan(errorState[3]))||
				(isnan(errorState[4]))||(isnan(errorState[5]))||
				(isnan(errorState[6]))||(isnan(errorState[7]))||
				(isnan(errorState[8]))){

				for(jj=0; jj<DIM_FILTER; jj++){
				   errorState[jj] = 0.0f;
				}
				nanCounterFilter++;
		}
		else{
			resetNavigationStates(&errorState[0]);
		}

		//enforce symmetry and update covariance matrix
		mat_trans(&covNew_Mat, &covNewTp_Mat);
		for(ii=0; ii<DIM_FILTER; ii++){

			for(jj=0; jj<DIM_FILTER; jj++){
				covNavFilter[ii][jj] = 0.5f*(covNew[ii][jj]+covNewTp[ii][jj]);
				if (isnan(covNavFilter[ii][jj])||(covNavFilter[ii][jj] > MAX_COVARIANCE)){
					covNavFilter[ii][jj] = MAX_COVARIANCE;
					nanCounterFilter++;
				}
				else if(ii==jj && covNavFilter[ii][jj] < MIN_COVARIANCE){
					covNavFilter[ii][jj] = MIN_COVARIANCE;
					nanCounterFilter++;
				}
			}
		}
		return true;
	}
	else	{
		return false;
	}
}
// Common update step for filter -> todo incorporate multidimensional measurement
//static bool updateNavigationFilter2(float *H,float *H2, float *innovation, float *R, float *errorState, float qualityGate){
static bool updateNavigationFilter2(arm_matrix_instance_f32 *Hk_Mat,float *H2, float *innovation, float *R, float qualityGate){
	float HkTp[DIM_FILTER][1];
	arm_matrix_instance_f32 HkTp_Mat = { DIM_FILTER, 1, (float *)HkTp};
	float errorState[DIM_FILTER];

	float PTimesHkTp[DIM_FILTER][1];
	arm_matrix_instance_f32 PTimesHkTp_Mat = { DIM_FILTER, 1, (float *)PTimesHkTp};
	float Sk[1][1];
	arm_matrix_instance_f32 Sk_Mat = {1, 1, (float *)Sk};
	float SkInv[1][1];
	arm_matrix_instance_f32 SkInv_Mat = {1, 1, (float *)SkInv};

	float Kk[DIM_FILTER][1];
	arm_matrix_instance_f32 Kk_Mat = {DIM_FILTER, 1, (float *)Kk};
	float KkTp[1][DIM_FILTER];
	arm_matrix_instance_f32 KkTp_Mat = {1, DIM_FILTER, (float *)KkTp};
	float KkRKkTp[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 KkRKkTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)KkRKkTp};

	float IMinusKkTimesHk[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 IMinusKkTimesHk_Mat = {DIM_FILTER, DIM_FILTER, (float *)IMinusKkTimesHk};
	float IMinusKkTimesHkTp[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 IMinusKkTimesHkTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)IMinusKkTimesHkTp};

	float IKkHkP[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 IKkHkP_Mat = {DIM_FILTER, DIM_FILTER, (float *)IKkHkP};

	float covNew[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 covNew_Mat = {DIM_FILTER, DIM_FILTER, (float *)covNew};
	float covNewTp[DIM_FILTER][DIM_FILTER];
	arm_matrix_instance_f32 covNewTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)covNewTp};

	ASSERT(Hk_Mat->numRows == 1);
	ASSERT(Hk_Mat->numCols == DIM_FILTER);

	uint16_t ii, jj;

	//compute Hk*covMat*HkTp
	mat_trans(Hk_Mat, &HkTp_Mat);
	mat_mult(&covNavFilter_Mat, &HkTp_Mat, &PTimesHkTp_Mat);
	mat_mult(Hk_Mat, &PTimesHkTp_Mat, &Sk_Mat);

	float tmp = H2[0]*H2[0]*covNew[0][0]+2*H2[0]*H2[1]*covNew[0][1]+2*H2[0]*H2[2]*covNew[0][2]+
				+ H2[1]*H2[1]*covNew[1][1]+2*H2[1]*H2[2]*covNew[1][2]+H2[2]*H2[2]*covNew[2][2];

	PxOut  = Sk[0][0];
	PvxOut = 0.25f*tmp;

	Sk[0][0] = Sk[0][0]+R[0]+0.25f*tmp;

	float innoCheck = innovation[0]*innovation[0]/Sk[0][0];

	if(innoCheck<qualityGate){
		// compute Kalman gain: Kk=covMat*HkTp*Sk^(-1)
		mat_inv(&Sk_Mat, &SkInv_Mat);
		mat_mult(&PTimesHkTp_Mat, &SkInv_Mat, &Kk_Mat);

		for(ii=0; ii<DIM_FILTER; ii++){
			errorState[ii] = Kk[ii][0]*innovation[0];
		}

		// correct covariance matrix:
		mat_mult(&Kk_Mat, Hk_Mat, &IMinusKkTimesHk_Mat);

		//compute I-Kk*Hk
		for(ii=0; ii<DIM_FILTER; ii++){
			for(jj=0; jj<DIM_FILTER; jj++){
				IMinusKkTimesHk[ii][jj] = -IMinusKkTimesHk[ii][jj];
			}
		}
		for(ii=0; ii<DIM_FILTER; ii++){
			IMinusKkTimesHk[ii][ii] = 1.0f + IMinusKkTimesHk[ii][ii];
		}

		mat_trans(&Kk_Mat, &KkTp_Mat);
		mat_mult(&Kk_Mat, &KkTp_Mat, &KkRKkTp_Mat);

		mat_trans(&IMinusKkTimesHk_Mat, &IMinusKkTimesHkTp_Mat);
		mat_mult(&IMinusKkTimesHk_Mat, &covNavFilter_Mat, &IKkHkP_Mat);
		mat_mult(&IKkHkP_Mat, &IMinusKkTimesHkTp_Mat, &covNew_Mat);

		for(ii=0; ii<DIM_FILTER; ii++){
			for(jj=0; jj<DIM_FILTER; jj++){
				covNew[ii][jj] = covNew[ii][jj]+KkRKkTp[ii][jj]*R[0]; // remark: since R is scalar, it can be considered in this step!
			}
		}

		if ((isnan(errorState[0]))||(isnan(errorState[1]))||
			(isnan(errorState[2]))||(isnan(errorState[3]))||
			(isnan(errorState[4]))||(isnan(errorState[5]))||
			(isnan(errorState[6]))||(isnan(errorState[7]))||
			(isnan(errorState[8]))){
			for(jj=0; jj<DIM_FILTER; jj++){
			   errorState[jj] = 0.0f;
			}
			nanCounterFilter++;
		}
		else{
			resetNavigationStates(&errorState[0]);
		}

		//enforce symmetry and update covariance matrix
		mat_trans(&covNew_Mat, &covNewTp_Mat);
		for(ii=0; ii<DIM_FILTER; ii++){
			for(jj=0; jj<DIM_FILTER; jj++){
				covNavFilter[ii][jj] = 0.5f*(covNew[ii][jj]+covNewTp[ii][jj]);
				if (isnan(covNavFilter[ii][jj])||(covNavFilter[ii][jj] > MAX_COVARIANCE)){
					covNavFilter[ii][jj] = MAX_COVARIANCE;
					nanCounterFilter++;
				}
				else if(ii==jj && covNavFilter[ii][jj] < MIN_COVARIANCE){
					covNavFilter[ii][jj] = MIN_COVARIANCE;
					nanCounterFilter++;
				}
			}
		}
		return true;
	}
	else	{
		return false;
	}
}

//static void updateNavigationFilter2D(arm_matrix_instance_f32 *Hk_Mat, float *innovation, float *R, float qualityGate){
//	uint8_t dimMeas=2;
//
//	float HkTp[DIM_FILTER][2];
//	arm_matrix_instance_f32 HkTp_Mat = { DIM_FILTER, 2, (float *)HkTp};
//
//  float errorState[DIM_FILTER];
//	float PTimesHkTp[DIM_FILTER][1];
//	arm_matrix_instance_f32 PTimesHkTp_Mat = { DIM_FILTER, 2, (float *)PTimesHkTp};
//	float Sk[2][2];
//	arm_matrix_instance_f32 Sk_Mat = {2, 2, (float *)Sk};
//	float SkInv[2][2];
//	arm_matrix_instance_f32 SkInv_Mat = {2, 2, (float *)SkInv};
//
//	float Kk[DIM_FILTER][2];
//	arm_matrix_instance_f32 Kk_Mat = {DIM_FILTER, 2, (float *)Kk};
//	float KkTp[2][DIM_FILTER];
//	arm_matrix_instance_f32 KkTp_Mat = {2, DIM_FILTER, (float *)KkTp};
//	float KkRKkTp[DIM_FILTER][DIM_FILTER];
//	arm_matrix_instance_f32 KkRKkTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)KkRKkTp};
//
//	float IMinusKkTimesHk[DIM_FILTER][DIM_FILTER];
//	arm_matrix_instance_f32 IMinusKkTimesHk_Mat = {DIM_FILTER, DIM_FILTER, (float *)IMinusKkTimesHk};
//	float IMinusKkTimesHkTp[DIM_FILTER][DIM_FILTER];
//	arm_matrix_instance_f32 IMinusKkTimesHkTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)IMinusKkTimesHkTp};
//
//	float IKkHkP[DIM_FILTER][DIM_FILTER];
//	arm_matrix_instance_f32 IKkHkP_Mat = {DIM_FILTER, DIM_FILTER, (float *)IKkHkP};
//
//	float covNew[DIM_FILTER][DIM_FILTER];
//	arm_matrix_instance_f32 covNew_Mat = {DIM_FILTER, DIM_FILTER, (float *)covNew};
//	float covNewTp[DIM_FILTER][DIM_FILTER];
//	arm_matrix_instance_f32 covNewTp_Mat = {DIM_FILTER, DIM_FILTER, (float *)covNewTp};
//
//	uint16_t ii, jj;
//
//	//compute Hk*covMat*HkTp
//	mat_trans(Hk_Mat, &HkTp_Mat);
//	mat_mult(&covNavFilter_Mat, &HkTp_Mat, &PTimesHkTp_Mat);
//	mat_mult(Hk_Mat, &PTimesHkTp_Mat, &Sk_Mat);
//
//	//compute Sk = Hk*covMat*HkTp+R
//	for(ii=0; ii<dimMeas; ii++){
//		for(jj=0; jj<dimMeas; jj++){
//			Sk[ii][jj] = Sk[ii][jj]+R[ii*dimMeas+jj]; // check ok
//		}
//	}
//	float detSk = Sk[0][0]*Sk[1][1]-Sk[1][0]*Sk[0][1];
//
//	SkInv[0][0] = 1.0f/detSk*Sk[1][1];
//	SkInv[0][1] = -1.0f/detSk*Sk[0][1];
//	SkInv[1][0] = -1.0f/detSk*Sk[1][0];
//	SkInv[1][1] = 1.0f/detSk*Sk[0][0];
//
//	float innoCheck = innovation[0]*innovation[0]*SkInv[0][0]+innovation[0]*innovation[1]*SkInv[0][1]+innovation[0]*innovation[1]*SkInv[1][0]+innovation[1]*innovation[1]*SkInv[1][1];
//
//
//	if(innoCheck<qualityGate){
//		mat_mult(&PTimesHkTp_Mat, &SkInv_Mat, &Kk_Mat);
//
//		for(ii=0; ii<DIM_FILTER; ii++){
//			errorState[ii] = Kk[ii][0]*innovation[0]+Kk[ii][1]*innovation[1];
//		}
//		// correct covariance matrix:
//		mat_mult(&Kk_Mat, Hk_Mat, &IMinusKkTimesHk_Mat);
//
//		//compute I-Kk*Hk
//		for(ii=0; ii<DIM_FILTER; ii++){
//			// compute Kk*R with R being 2x2 and diagonal, otherwise, equation must be modified!
//			Kk[ii][0] = Kk[ii][0]* R[0*dimMeas+0];
//			Kk[ii][1] = Kk[ii][1]* R[1*dimMeas+1];
//
//			for(jj=0; jj<DIM_FILTER; jj++){
//				IMinusKkTimesHk[ii][jj] = -IMinusKkTimesHk[ii][jj];
//			}
//		}
//		for(ii=0; ii<DIM_FILTER; ii++){
//			IMinusKkTimesHk[ii][ii] = 1.0f + IMinusKkTimesHk[ii][ii];
//		}
//
//		//compute Kk*R*Kk'
//		mat_trans(&Kk_Mat, &KkTp_Mat);
//		mat_mult(&Kk_Mat, &KkTp_Mat, &KkRKkTp_Mat);
//
//
//		mat_trans(&IMinusKkTimesHk_Mat, &IMinusKkTimesHkTp_Mat);
//		mat_mult(&IMinusKkTimesHk_Mat, &covNavFilter_Mat, &IKkHkP_Mat);
//		mat_mult(&IKkHkP_Mat, &IMinusKkTimesHkTp_Mat, &covNew_Mat);
//
//		for(ii=0; ii<DIM_FILTER; ii++){
//			for(jj=0; jj<DIM_FILTER; jj++){
//				covNew[ii][jj] = covNew[ii][jj]+KkRKkTp[ii][jj];
//			}
//		}
//
//		// handle Filter errors if filter state become nan
//		if ((isnan(errorState[0]))||(isnan(errorState[1]))||
//			(isnan(errorState[2]))||(isnan(errorState[3]))||
//			(isnan(errorState[4]))||(isnan(errorState[5]))||
//			(isnan(errorState[6]))||(isnan(errorState[7]))||
//			(isnan(errorState[8]))){
//
//			for(jj=0; jj<DIM_FILTER; jj++){
//			   errorState[jj] = 0.0f;
//			}
//			nanCounterFilter++;
//		}
//      else{
//	        resetNavigationStates(&errorState[0]);
//      }
//
//		//enforce symmetry and update covariance matrix
//		mat_trans(&covNew_Mat, &covNewTp_Mat);
//		for(ii=0; ii<DIM_FILTER; ii++){
//
//			for(jj=0; jj<DIM_FILTER; jj++){
//				covNavFilter[ii][jj] = 0.5f*(covNew[ii][jj]+covNewTp[ii][jj]);
//				if (isnan(covNavFilter[ii][jj])||(covNavFilter[ii][jj] > MAX_COVARIANCE)){
//					covNavFilter[ii][jj] = MAX_COVARIANCE;
//					nanCounterFilter++;
//				}
//				else if(ii==jj && covNavFilter[ii][jj] < MIN_COVARIANCE){
//					covNavFilter[ii][jj] = MIN_COVARIANCE;
//					nanCounterFilter++;
//				}
//			}
//		}
//		return true;
//	}
//	else	{
//		return false;
//	}
//}

// reset strapdown navigation after filter update step if measurements were obtained
static void resetNavigationStates( float *errorState){

	float attVec[3]  = {errorState[6], errorState[7], errorState[8]};
	float quatNav[4] = {stateNav[6], stateNav[7], stateNav[8], stateNav[9]};
	float attQuat[4], quatRes[4];

	// update position and velocity state
	uint16_t ii;
	for(ii=0; ii<6; ii++){
		stateNav[ii] = stateNav[ii] + errorState[ii];
	}

	quatFromAtt(&attVec[0], &attQuat[0]);
	multQuat(&quatNav[0], &attQuat[0], &quatRes[0]);

	for(ii=6; ii<10; ii++){
		stateNav[ii] = quatRes[ii-6];
	}

	for(ii=0; ii<DIM_FILTER; ii++){
		errorState[ii] = 0.0f;
	}

	directionCosineMatrix(&quatRes[0], &dcm[0][0]);
	transposeMatrix(&dcm[0][0], &dcmTp[0][0]);
}



static bool updateQueuedMeasurements(const uint32_t tick, Axis3f* gyroAverage) {
  bool doneUpdate = false;
  // Pull the latest sensors values of interest; discard the rest
   measurement_t m;
   while (estimatorDequeue(&m)) {
     switch (m.type) {
       case MeasurementTypeTDOA:
//         if(robustTdoa){
//           // robust KF update with TDOA measurements
//           kalmanCoreRobustUpdateWithTDOA(&coreData, &m.data.tdoa);
//         }else{
           // standard KF update
    	   if(useNavigationFilter){
    		   updateWithTdoaMeasurement(&m.data.tdoa);
    	   }
        	 //kalmanCoreUpdateWithTDOA(&coreData, &m.data.tdoa);
 //        }
         doneUpdate = true;
         break;
//       case MeasurementTypePosition:
//         kalmanCoreUpdateWithPosition(&coreData, &m.data.position);
//         doneUpdate = true;
//         break;
//       case MeasurementTypePose:
//         kalmanCoreUpdateWithPose(&coreData, &m.data.pose);
//         doneUpdate = true;
//         break;
//       case MeasurementTypeDistance:
//         if(robustTwr){
//             // robust KF update with UWB TWR measurements
//             kalmanCoreRobustUpdateWithDistance(&coreData, &m.data.distance);
//         }else{
//             // standard KF update
//             kalmanCoreUpdateWithDistance(&coreData, &m.data.distance);
//         }
//         doneUpdate = true;
//         break;
       case MeasurementTypeTOF:
    	   if(useNavigationFilter){
    		   updateWithTofMeasurement(&m.data.tof);
    	   }
         //kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
         doneUpdate = true;
         break;
//       case MeasurementTypeAbsoluteHeight:
//         kalmanCoreUpdateWithAbsoluteHeight(&coreData, &m.data.height);
//         doneUpdate = true;
//         break;
       case MeasurementTypeFlow:
    	   if(useNavigationFilter){
    		   updateWithFlowMeasurement(&m.data.flow, gyroAverage);
    		   //kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest);
    		   doneUpdate = true;
    	   }
         break;
//       case MeasurementTypeYawError:
//         kalmanCoreUpdateWithYawError(&coreData, &m.data.yawError);
//         doneUpdate = true;
//         break;
//       case MeasurementTypeSweepAngle:
//         kalmanCoreUpdateWithSweepAngles(&coreData, &m.data.sweepAngle, tick, &sweepOutlierFilterState);
//         doneUpdate = true;
//         break;
       case MeasurementTypeGyroscope:
         gyroAccumulator.x += m.data.gyroscope.gyro.x;
         gyroAccumulator.y += m.data.gyroscope.gyro.y;
         gyroAccumulator.z += m.data.gyroscope.gyro.z;
         gyroLatest = m.data.gyroscope.gyro;
         gyroAccumulatorCount++;
         break;
       case MeasurementTypeAcceleration:
         accAccumulator.x += m.data.acceleration.acc.x;
         accAccumulator.y += m.data.acceleration.acc.y;
         accAccumulator.z += m.data.acceleration.acc.z;
         accLatest = m.data.acceleration.acc;
         accAccumulatorCount++;
         break;
       case MeasurementTypeBarometer:
         //if (useBaroUpdate) {
    	   if(useNavigationFilter){
    		   updateWithBaro(m.data.barometer.baro.asl);
    		   //kalmanCoreUpdateWithBaro(&coreData, m.data.barometer.baro.asl, quadIsFlying);
    		   doneUpdate = true;
    	   }
         //}
         break;
       default:
         break;
     }
   }
   return doneUpdate;
}

static void updateWithBaro(float baroMeas){
	float hBaro[DIM_FILTER] = {0};
	arm_matrix_instance_f32 HBaro = {1, DIM_FILTER, hBaro};

    float innovation;
    float R = measNoiseBaro;

    hBaro[2] = 1.0f;

    innovation = baroMeas-stateNav[2];

    updateNavigationFilter(&HBaro, &innovation, &R, qualGateBaro);
}


static void updateWithTofMeasurement(tofMeasurement_t *tof){
	float hTof[DIM_FILTER] = {0};
	arm_matrix_instance_f32 HTof = {1, DIM_FILTER, hTof};
    float innovation;
    float R;
   // bool doneUpdate = false;

    // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
    if ((fabs(dcm[2][2]) > 0.1) && (dcm[2][2] > 0.0f)){

      float angle = fabsf(acosf(dcm[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
			if(angle < 0.0f) {
				angle = 0.0f;
			}

			float predictedDistance = stateNav[2]/dcm[2][2];
			float measuredDistance  = tof->distance; // [m]

			if(measuredDistance < 0.03f){
				measuredDistance = 0.0f;
			}

			predDist   = predictedDistance;
			measDist   = measuredDistance;
			innovation = measuredDistance - predictedDistance;

			hTof[2] = 1/dcm[2][2];
			R = (tof->stdDev)*(tof->stdDev);

		//	doneUpdate = updateNavigationFilter(&HTof, &innovation, &R, qualGateTof);
		updateNavigationFilter(&HTof, &innovation, &R, qualGateTof);
		// if(doneUpdate){
		// 	activateFlowDeck = true;
		// }
		// else{
		// 	activateFlowDeck = false;
		// }
  }
}

// successive one-dimemnsional measurements
static void updateWithFlowMeasurement(flowMeasurement_t *flow, Axis3f *omegaBody){
	float thetapix = DEG_TO_RAD * 4.2f;
	float Npix = 30.0f;                      // [pixels] (same in x and y)
	float velocityBody[2];
	float h_g;
	float omegaFactor = 1.0f;
    float innovation;
    float R;

  //  if(activateFlowDeck){
		// Saturate elevation in prediction and correction to avoid singularities
		if (  stateNav[2] < 0.1f ) {
			h_g = 0.1f;
		}
		else {
			h_g = stateNav[2];
		}

		velocityBody[0] =  dcm[0][0]*stateNav[3]+dcm[0][1]*stateNav[4]+dcm[0][2]*stateNav[5];
		velocityBody[1] =  dcm[1][0]*stateNav[3]+dcm[1][1]*stateNav[4]+dcm[1][2]*stateNav[5];

		// ~~~ X velocity prediction and update ~~~
		float flowNX  = flow->dpixelx;
		float flowNY  = flow->dpixely;

		pred_NX = (flow->dt * Npix / thetapix ) * ((velocityBody[0]/h_g*dcm[2][2]) - omegaFactor * omegaBody->y);

		float hx[DIM_FILTER] = {0};
		arm_matrix_instance_f32 Hx= {1, DIM_FILTER, hx};

		// Compute innovation
		meas_NX = flowNX;
		meas_NY = flowNY;

		innovation = flowNX - pred_NX;
		//R          = 4.0f; // flow deck has 0.25 px for heights around 1 m
		R          = (flow->stdDevX)*(flow->stdDevX);

		hx[2] = (flow->dt * Npix / thetapix )*(-velocityBody[0]/(h_g*h_g)*dcm[2][2]);
		hx[3] = (flow->dt * Npix / thetapix )*(dcm[0][0]*dcm[2][2]/h_g);
		hx[4] = (flow->dt * Npix / thetapix )*(dcm[0][1]*dcm[2][2]/h_g);
		hx[5] = (flow->dt * Npix / thetapix )*(dcm[0][2]*dcm[2][2]/h_g);

		updateNavigationFilter(&Hx, &innovation, &R, qualGateFlow);

		pred_NY = (flow->dt * Npix / thetapix ) * ((velocityBody[1]/h_g*dcm[2][2]) + omegaFactor * omegaBody->x);

		float hy[DIM_FILTER] = {0};
		arm_matrix_instance_f32 Hy= {1, DIM_FILTER, hy};

		innovation = flowNY - pred_NY;
		//R          = 4.0f; // flow deck has 0.25 px for heights around 1 m
		R          = (flow->stdDevY)*(flow->stdDevY);

		hy[2] = (flow->dt * Npix / thetapix )*(-velocityBody[1]/(h_g*h_g)*dcm[2][2]);
		hy[3] = (flow->dt * Npix / thetapix )*(dcm[1][0]*dcm[2][2]/h_g);
		hy[4] = (flow->dt * Npix / thetapix )*(dcm[1][1]*dcm[2][2]/h_g);
		hy[5] = (flow->dt * Npix / thetapix )*(dcm[1][2]*dcm[2][2]/h_g);

		updateNavigationFilter(&Hy, &innovation, &R, qualGateFlow);
//  }
}

// multidimensional measurement
//static void updateWithFlowMeasurement(flowMeasurement_t *flow, Axis3f *omegaBody)
//{
//	float thetapix = DEG_TO_RAD * 4.2f;
//	float Npix = 30.0f;                      // [pixels] (same in x and y)
//	float velocityBody[2];
//	float h_g;
//
//	float omegaFactor = 1.25f;//1.25f;
//    float innovation[2];
//    float R[2][2];
//
//    float hFlow[2*DIM_FILTER] = { 0};
//    arm_matrix_instance_f32 HFlow = {2, DIM_FILTER, hFlow};
//
//    if(activateFlowDeck){//&&(tdoaCount>2000)){
//		// Saturate elevation in prediction and correction to avoid singularities
//		if (  stateNav[2] < 0.1f ) {
//			h_g = 0.1f;
//		}
//		else {
//			h_g = stateNav[2];
//		}
//
//		velocityBody[0] =  dcm[0][0]*stateNav[3]+dcm[0][1]*stateNav[4]+dcm[0][2]*stateNav[5];
//		velocityBody[1] =  dcm[1][0]*stateNav[3]+dcm[1][1]*stateNav[4]+dcm[1][2]*stateNav[5];
//
//		// ~~~ X velocity prediction and update ~~~
//		float flowNX  = flow->dpixelx;
//		float flowNY  = flow->dpixely;
//
//		pred_NX = (flow->dt * Npix / thetapix ) * ((velocityBody[0]/h_g*dcm[2][2]) - omegaFactor * omegaBody->y);
//		pred_NY = (flow->dt * Npix / thetapix ) * ((velocityBody[1]/h_g*dcm[2][2]) + omegaFactor * omegaBody->x);
//
//
//		meas_NX = flowNX;
//		meas_NY = flowNY;
//
//		// Compute innovation
//		innovation[0] = flowNX - pred_NX;
//		innovation[1] = flowNY - pred_NY;
//
//		R[0][0]    = 4.0f;//(flow->stdDevX)*(flow->stdDevX);
//		R[1][1]    = 4.0f;//(flow->stdDevY)*(flow->stdDevY);
//		R[0][1]    = 0.0f;
//	    R[1][0]    = 0.0f;
//
//	    hFlow[0*DIM_FILTER+2] = (flow->dt * Npix / thetapix )*(-velocityBody[0]/(h_g*h_g)*dcm[2][2]);
//	    hFlow[0*DIM_FILTER+3] = (flow->dt * Npix / thetapix )*(dcm[0][0]*dcm[2][2]/h_g);
//	    hFlow[0*DIM_FILTER+4] = (flow->dt * Npix / thetapix )*(dcm[0][1]*dcm[2][2]/h_g);
//	    hFlow[0*DIM_FILTER+5] = (flow->dt * Npix / thetapix )*(dcm[0][2]*dcm[2][2]/h_g);
//
//	    hFlow[1*DIM_FILTER+2] = (flow->dt * Npix / thetapix )*(-velocityBody[1]/(h_g*h_g)*dcm[2][2]);
//	    hFlow[1*DIM_FILTER+3] = (flow->dt * Npix / thetapix )*(dcm[1][0]*dcm[2][2]/h_g);
//	    hFlow[1*DIM_FILTER+4] = (flow->dt * Npix / thetapix )*(dcm[1][1]*dcm[2][2]/h_g);
//	    hFlow[1*DIM_FILTER+5] = (flow->dt * Npix / thetapix )*(dcm[1][2]*dcm[2][2]/h_g);
//
//		updateNavigationFilter2D(&HFlow, &innovation[0], &R[0][0], qualGateFlow);
//    }
//}

// update with TdoA (lps) measurement -> todo modify to one multidimensional observation instead of two different one dimensional update steps
static void updateWithTdoaMeasurement(tdoaMeasurement_t *tdoa){
  if (tdoaCount >= 100)  {
     float measurement = tdoa->distanceDiff;

     // predict based on current state
     float x = stateNav[0];
     float y = stateNav[1];
     float z = stateNav[2];

     // rephrase in north east down coordinate frame?
     float x1 = tdoa->anchorPositions[1].x, y1 = tdoa->anchorPositions[1].y, z1 = tdoa->anchorPositions[1].z;
     float x0 = tdoa->anchorPositions[0].x, y0 = tdoa->anchorPositions[0].y, z0 = tdoa->anchorPositions[0].z;

     float dx1 = x - x1;
     float dy1 = y - y1;
     float dz1 = z - z1;

     float dx0 = x - x0;
     float dy0 = y - y0;
     float dz0 = z - z0;

     float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
     float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));
     float divd0_2 = 1/(d0*d0);
     float divd1_2 = 1/(d1*d1);

     float predicted  = d1 - d0;
     float innovation = measurement - predicted;

	 float htdoa1[DIM_FILTER] = {0};
 	 arm_matrix_instance_f32 Htdoa1= {1, DIM_FILTER, htdoa1};

     float H2[1][DIM_FILTER];

     if ((d0 != 0.0f) && (d1 != 0.0f)) {
        htdoa1[0] = (dx1 / d1 - dx0 / d0);
        htdoa1[1] = (dy1 / d1 - dy0 / d0);
        htdoa1[2] = (dz1 / d1 - dz0 / d0);

        // second order approximation
        H2[0][0] = ((1+divd1_2)*dx1 / d1 - (1+divd0_2)*dx1 / d0);
        H2[0][1] = ((1+divd1_2)*dy1 / d1 - (1+divd0_2)*dy1 / d0);
        H2[0][2] = ((1+divd1_2)*dz1 / d1 - (1+divd0_2)*dz1 / d0);

        vector_t jacobian = {
            .x = htdoa1[0],
            .y = htdoa1[1],
            .z = htdoa1[2],
        };

        point_t estimatedPosition = {
            .x = stateNav[0],
            .y = stateNav[1],
            .z = stateNav[2],
        };
        float R = tdoa->stdDev*tdoa->stdDev;
        bool sampleIsGood = outlierFilterValidateTdoaSteps(tdoa, innovation, &jacobian, &estimatedPosition);
        if (sampleIsGood) {
    	  // standard EKF measurement update
          //updateNavigationFilter(&Htdoa1, &innovation, &R, qualGateTdoa);
    	  // standard EKF measurement update considering second order term in linearization
    	  updateNavigationFilter2(&Htdoa1, &H2[0][0], &innovation, &R, qualGateTdoa);
        }
     }
  }
  tdoaCount++;
}


// transform quaternion into euler angles
static void quatToEuler(float *quat, float *eulerAngles){
	float q0_2 = quat[0]*quat[0];
	float q1_2 = quat[1]*quat[1];
	float q2_2 = quat[2]*quat[2];
	float q3_2 = quat[3]*quat[3];

	// compute elements of the direction cosine matrix
	float ele11 = q0_2+q1_2-q2_2-q3_2;
	float ele21 = 2*(quat[1]*quat[2]+quat[0]*quat[3]);
	float ele31 = 2*(quat[1]*quat[3]-quat[0]*quat[2]);
	float ele32 = 2*(quat[2]*quat[3]+quat[0]*quat[1]);
	float ele33 = q0_2-q1_2-q2_2+q3_2;

	//compute eulerAngles
	if((ele32 == 0)&(ele33 == 0)){
		eulerAngles[0] = 0;
	}
	else{
		eulerAngles[0] = atan2f(ele32, ele33);
	}
	eulerAngles[1] = asinf(-ele31);

	if((ele21 == 0)&(ele11 == 0)){
		eulerAngles[2] = 0;
	}
	else{
		eulerAngles[2] = atan2f(ele21, ele11);
	}
}


// compute direction cosine matrix to transfer between lab and body frames
static void directionCosineMatrix(float *quat, float *dcm){
	float q0_2 = quat[0]*quat[0];
	float q1_2 = quat[1]*quat[1];
	float q2_2 = quat[2]*quat[2];
	float q3_2 = quat[3]*quat[3];

	// compute elements of the direction cosine matrix, ned to body
	dcm[0*3+0] = q0_2+q1_2-q2_2-q3_2;
	dcm[0*3+1] = 2*(quat[1]*quat[2]+quat[0]*quat[3]);
	dcm[0*3+2] = 2*(quat[1]*quat[3]-quat[0]*quat[2]);

	dcm[1*3+0] = 2*(quat[1]*quat[2]-quat[0]*quat[3]);
	dcm[1*3+1] = q0_2-q1_2+q2_2-q3_2;
	dcm[1*3+2] = 2*(quat[2]*quat[3]+quat[0]*quat[1]);

	dcm[2*3+0] = 2*(quat[1]*quat[3]+quat[0]*quat[2]);
  dcm[2*3+1] = 2*(quat[2]*quat[3]-quat[0]*quat[1]);
	dcm[2*3+2] = q0_2-q1_2-q2_2+q3_2;
}

// compute quaternion product
static void multQuat(float *q1, float *q2, float *quatRes){

	quatRes[0] = q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
	quatRes[1] = q1[1]*q2[0]+q1[0]*q2[1]-q1[3]*q2[2]+q1[2]*q2[3];
	quatRes[2] = q1[2]*q2[0]+q1[3]*q2[1]+q1[0]*q2[2]-q1[1]*q2[3];
	quatRes[3] = q1[3]*q2[0]-q1[2]*q2[1]+q1[1]*q2[2]+q1[0]*q2[3];
}

// compute quaternion from attitude error
static void quatFromAtt(float *attVec, float *quat){
	float absError = sqrtf(attVec[0]*attVec[0]+attVec[1]*attVec[1]+attVec[2]*attVec[2]);
	float absError2 = absError*absError;
	float absError3 = absError2*absError;
	float absError4 = absError3*absError;
	//float absError5 = absError4*absError;
	//float absError6 = absError5*absError;

	float scale = 0.5f-0.02083333f*absError2+0.0002604166f*absError4;//-0.0000015501f*absError6;
	quat[0] = 1.0f-0.125f*absError2+0.002604166f*absError4;//-0.0000217014f*absError6;

	quat[1] = scale*attVec[0];
	quat[2] = scale*attVec[1];
	quat[3] = scale*attVec[2];
}

static void transposeMatrix( float *mat, float *matTp){
	matTp[0*3+0] = mat[0];
	matTp[0*3+1] = mat[1*3+0];
	matTp[0*3+2] = mat[2*3+0];

	matTp[1*3+0] = mat[0*3+1];
	matTp[1*3+1] = mat[1*3+1];
	matTp[1*3+2] = mat[2*3+1];

	matTp[2*3+0] = mat[0*3+2];
	matTp[2*3+1] = mat[1*3+2];
	matTp[2*3+2] = mat[2*3+2];
}

// Temporary development groups
LOG_GROUP_START(kalman_states)
//  LOG_ADD(LOG_FLOAT, ox, &coreData.S[KC_STATE_X])
//  LOG_ADD(LOG_FLOAT, oy, &coreData.S[KC_STATE_Y])
//  LOG_ADD(LOG_FLOAT, vx, &coreData.S[KC_STATE_PX])
//  LOG_ADD(LOG_FLOAT, vy, &coreData.S[KC_STATE_PY])
LOG_GROUP_STOP(kalman_states)

LOG_GROUP_START(navFilter)
  LOG_ADD(LOG_FLOAT, posX, &stateNav[0])
  LOG_ADD(LOG_FLOAT, posY, &stateNav[1])
  LOG_ADD(LOG_FLOAT, posZ, &stateNav[2])
  LOG_ADD(LOG_FLOAT, accX, &accLog[0])
  LOG_ADD(LOG_FLOAT, accY, &accLog[1])
  LOG_ADD(LOG_FLOAT, accZ, &accLog[2])
  LOG_ADD(LOG_FLOAT, omegaX, &omega[0])
  LOG_ADD(LOG_FLOAT, omegaY, &omega[1])
  LOG_ADD(LOG_FLOAT, omegaZ, &omega[2])
  LOG_ADD(LOG_FLOAT, Phi, &eulerOut[0])
  LOG_ADD(LOG_FLOAT, Theta, &eulerOut[1])
  LOG_ADD(LOG_FLOAT, Psi, &eulerOut[2])
  LOG_ADD(LOG_FLOAT, Px, &PxOut)
  LOG_ADD(LOG_FLOAT, Pvx, &PvxOut)
  LOG_ADD(LOG_FLOAT, Pattx, &PattxOut)
  LOG_ADD(LOG_UINT32, nanCounter, &nanCounterFilter)
LOG_GROUP_STOP(navFilter)

LOG_GROUP_START(sensorFilter)
  LOG_ADD(LOG_FLOAT, dxPx, &meas_NX)
  LOG_ADD(LOG_FLOAT, dyPx, &meas_NY)
  LOG_ADD(LOG_FLOAT, dxPxPred, &pred_NX)
  LOG_ADD(LOG_FLOAT, dyPxPred, &pred_NY)
  LOG_ADD(LOG_FLOAT, distPred, &predDist)
  LOG_ADD(LOG_FLOAT, distMeas, &measDist)
  LOG_ADD(LOG_FLOAT, baroHeight, &baroOut)
LOG_GROUP_STOP(sensorFilter)

// Stock log groups
LOG_GROUP_START(kalman)
  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  STATS_CNT_RATE_LOG_ADD(rtBaro, &baroUpdateCounter)
  STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
  STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
  STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &resetNavigation)
  PARAM_ADD(PARAM_UINT8, useNavFilter, &useNavigationFilter)
  PARAM_ADD(PARAM_FLOAT, sigmaInitPos_xy, &stdDevInitialPosition_xy)
  PARAM_ADD(PARAM_FLOAT, sigmaInitPos_z, &stdDevInitialPosition_z)
  PARAM_ADD(PARAM_FLOAT, sigmaInitVel, &stdDevInitialVelocity)
  PARAM_ADD(PARAM_FLOAT, sigmaInitAtt, &stdDevInitialAtt)
  PARAM_ADD(PARAM_FLOAT, procNoiseA_h, &procA_h)
  PARAM_ADD(PARAM_FLOAT, procNoiseA_z, &procA_z)
  PARAM_ADD(PARAM_FLOAT, procNoiseVel_h, &procVel_h)
  PARAM_ADD(PARAM_FLOAT, procNoiseVel_z, &procVel_z)
  PARAM_ADD(PARAM_FLOAT, procNoiseRate_h, &procRate_h)
  PARAM_ADD(PARAM_FLOAT, procNoiseRate_z, &procRate_z)
  PARAM_ADD(PARAM_FLOAT, baroNoise, &measNoiseBaro)
  PARAM_ADD(PARAM_FLOAT, qualityGateTof, &qualGateTof)
  PARAM_ADD(PARAM_FLOAT, qualityGateFlow , &qualGateFlow)
  PARAM_ADD(PARAM_FLOAT, qualityGateTdoa , &qualGateTdoa)
  PARAM_ADD(PARAM_FLOAT, qualityGateBaro, &qualGateBaro)
PARAM_GROUP_STOP(kalman)
