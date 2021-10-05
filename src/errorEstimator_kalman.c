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
/**
 * Authored by Klaus Kefferpütz, August December 2021
 *
 *
 * ================================================================================================
 * Error-State Unscented Kalman Filter 
 * ================================================================================================
 *
 * MAJOR CHANGELOG:
 * 2021.09.03 Initial Commit
 *
 */

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

// Semaphore to signal that we got data from the stabilzer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
#define BARO_RATE RATE_25_HZ

#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)
//#define KAPPA_UKF       3.0f // TODO Adapt    9-6 = 3;
//#define w0              KAPPA_UKF/((float) DIM_FILTER+KAPPA_UKF)
//#define w1              0.5f/((float) DIM_FILTER+KAPPA_UKF)
/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */
static bool isInit = false;
static bool flowActive = true;
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

// for error filter version
#define DIM_FILTER 9
#define DIM_STRAPDOWN 10

static float scaleSigma = 3.674f;
static float w0    = 0.333f;
static float w1    = 0.03704;


static float stateNav[DIM_STRAPDOWN];
static bool initializedNav = false;
static int32_t numberInitSteps = 1000;
static Axis3f accBias;
static Axis3f omegaBias;
static float baroAslBias;

static float covNavFilter[DIM_FILTER][DIM_FILTER];

static float xEst[DIM_FILTER] = {0};
static float sigmaPointsPlus[DIM_FILTER][DIM_FILTER] = {0};
static float sigmaPointsMinus[DIM_FILTER][DIM_FILTER] = {0};
static float state0[DIM_FILTER] = {0};

static float accNed[3];
static float dcm[3][3],dcmTp[3][3];

// values for flying UWB Based
// If you want to rely on LH Sweep Angles and fuse them in the filter
// use commented values to avoid issues in initial convergence 
static float stdDevInitialPosition_xy = 50.0f;  // 10.0f;
static float stdDevInitialPosition_z  = 1.0f;   // 0.2f; 
static float stdDevInitialVelocity    = 0.0001f; 
static float stdDevInitialAtt         = 0.01f;   

// these parameters are for CF2.1 only! 
// variances of process noise (accelerometer and gyroscope noise)
// derived from the datasheets (contains sqrt(R)) times a scaling factor for inflight vibrations
// vibrations are different for horizontal and vertical components 
static float procA_h    = 4.4755e-6f; 
static float procA_z    = 3.4137e-5f; 
static float procVel_h  = 0.00f;
static float procVel_z  = 0.00f;
static float procRate_h = 9.2495e-7f; 
static float procRate_z = 2.3124e-7f;

// measurement noise baro - variance
static float measNoiseBaro = 6.25;//0.7f*0.7f;

// quality gates, currently only tof-Gate is used
// when tof measurement is not compliant to outlier detection
// flow-measurements are also not processed. 
static float qualGateTof  = 100.0f; // for CF 2.1
//static float qualGateTof  = 1000.63f; // for CF 2.0 a much larger gate is required but not tuned yet!

static float qualGateFlow = 1000.63f;
static float qualGateTdoa = 1000.63f; // should not be lowered currently
static float qualGateBaro = 3.64f;  // was 1.64

static float innoCheckTof = 0.0f;
static float innoCheckFlow_x = 0.0f;
static float innoCheckFlow_y = 0.0f;

static float qualGateSweep = 1000.63f;
static OutlierFilterLhState_t sweepOutlierFilterState;

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

static void navigationInit(void);
static void resetNavigationStates( void );

static bool updateQueuedMeasurements(const uint32_t tick, Axis3f* gyroAverage);

static void computeOutputTof(float *output, float* state);
static void computeOutputFlow_x(float *output, float* state, flowMeasurement_t *flow, Axis3f *omegaBody);
static void computeOutputFlow_y(float *output, float* state, flowMeasurement_t *flow, Axis3f *omegaBody);
static void computeOutputTdoa(float *output, float* state,tdoaMeasurement_t *tdoa);
static void computeOutputBaro(float *output, float* state);

static void computeOutputSweep(float *output, float* state, sweepAngleMeasurement_t *sweepInfo, float *xy);

static bool ukfUpdate(float* Pxy, float* Pyy, float innovation);
//static void computeMeanFromSigmaPoints(void);
static void computeSigmaPoints(void );
static uint8_t cholesky(float *A, float *L, uint8_t n); 
static void quatToEuler(float *quat, float *eulerAngles);
static void quatFromAtt(float *attVec, float *quat);
static void directionCosineMatrix(float *quat, float *dcm);
static void quatToEuler(float *quat, float *eulerAngles);
static void multQuat(float *q1, float *q2, float *quatRes);
static void transposeMatrix( float *mat, float *matTp);

static void errorKalmanTask(void* parameters);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(errorKalmanTask, 6 * configMINIMAL_STACK_SIZE);

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
 
  // Tracks whether an update to the state has been made, and the state therefore requires finalization
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

    	//navigationInit();
    	lastPrediction = xTaskGetTickCount();
    }

    // If the client triggers an estimator reset via parameter update "kalman.resetEstimation"
    if (resetNavigation) {
      errorEstimatorKalmanInit();
			paramSetInt(paramGetVarId("ukf", "resetEstimation"), 0);
      resetNavigation = false;

			DEBUG_PRINT("Reset UKF\n") ;

      // set bias accumalation counters to zero
      initializedNav = false;
      accBias   = (Axis3f){.axis={0}};
      accAccumulatorCount = 0;
      omegaBias = (Axis3f){.axis={0}};
      gyroAccumulatorCount = 0;

      accNed[0] = 0.0f;
      accNed[1] = 0.0f;
      accNed[2] = 0.0f;

      tdoaCount = 0;
      nanCounterFilter = 0;

			navigationInit();
    }

    // Tracks whether an update to the state has been made, and the state therefore requires finalization
  uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...
	Axis3f gyroAverage;
	Axis3f accAverage;

	if (osTick >= nextPrediction) { // update at the PREDICT_RATE
		float dt = T2S(osTick - lastPrediction);
		
		if(initializedNav && (accAccumulatorCount>0)&&(gyroAccumulatorCount>0)){

			// gyro is in deg/sec but the estimator requires rad/sec
			gyroAverage.x = (gyroAccumulator.x/((float)gyroAccumulatorCount)-omegaBias.x ) * DEG_TO_RAD;
			gyroAverage.y = (gyroAccumulator.y/((float)gyroAccumulatorCount)-omegaBias.y ) * DEG_TO_RAD;
			gyroAverage.z = (gyroAccumulator.z/((float)gyroAccumulatorCount)-omegaBias.z ) * DEG_TO_RAD;

			// accelerometer is in Gs but the estimator requires ms^-2
			
			accAverage.x = (accAccumulator.x /((float)accAccumulatorCount)-accBias.x)* GRAVITY_MAGNITUDE;
			accAverage.y = (accAccumulator.y /((float)accAccumulatorCount)-accBias.y) * GRAVITY_MAGNITUDE;
			accAverage.z = (accAccumulator.z /((float)accAccumulatorCount)-accBias.z) * GRAVITY_MAGNITUDE;

			accAccumulator = (Axis3f){.axis={0}};
			accAccumulatorCount = 0;
			gyroAccumulator = (Axis3f){.axis={0}};
			gyroAccumulatorCount = 0;
		}
		else{

				// gyro is in deg/sec but the estimator requires rad/sec
			// gyroAverage.x = gyroAccumulator.x/((float)gyroAccumulatorCount)* DEG_TO_RAD;
			// gyroAverage.y = gyroAccumulator.y/((float)gyroAccumulatorCount) * DEG_TO_RAD;
			// gyroAverage.z = gyroAccumulator.z/((float)gyroAccumulatorCount) * DEG_TO_RAD;

			// // accelerometer is in Gs but the estimator requires ms^-2
			
			// accAverage.x = accAccumulator.x /((float)accAccumulatorCount) * GRAVITY_MAGNITUDE;
			// accAverage.y = accAccumulator.y /((float)accAccumulatorCount) * GRAVITY_MAGNITUDE;
			// accAverage.z = accAccumulator.z /((float)accAccumulatorCount) * GRAVITY_MAGNITUDE;
			accAverage   = (Axis3f){.axis={0}};
			accAverage.z = GRAVITY_MAGNITUDE;
			gyroAverage  = (Axis3f){.axis={0}};
		} // end if bias errors are initialized

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

		lastPrediction = osTick;
		STATS_CNT_RATE_EVENT(&predictionCounter);

		nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);
	} // end if update at the PREDICT_RATE

	directionCosineMatrix(&stateNav[6], &dcm[0][0]);
	transposeMatrix(&dcm[0][0], &dcmTp[0][0]);

  if(updateQueuedMeasurements(osTick, &gyroAverage)) {
   	STATS_CNT_RATE_EVENT(&updateCounter);
   	//      doneUpdate = true;
  }

  quatToEuler(&stateNav[6], &eulerOut[0]);

	eulerOut[0] = eulerOut[0]*RAD_TO_DEG;
	eulerOut[1] = eulerOut[1]*RAD_TO_DEG;
	eulerOut[2] = eulerOut[2]*RAD_TO_DEG;

  xSemaphoreTake(dataMutex, portMAX_DELAY);

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

  taskEstimatorState.acc.x = accNed[0];
  taskEstimatorState.acc.y = accNed[1];
  taskEstimatorState.acc.z = accNed[2];

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

  outlierFilterReset(&sweepOutlierFilterState, 0);
	//sweepOutlierFilterState.openingWindow=2*sweepOutlierFilterState.openingWindow;
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
	stateNew[0] = stateNav[0]+stateNav[3]*dt+0.5f*accNed[0]*dt*dt;
	stateNew[1] = stateNav[1]+stateNav[4]*dt+0.5f*accNed[1]*dt*dt;
	stateNew[2] = stateNav[2]+stateNav[5]*dt+0.5f*accNed[2]*dt*dt;

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
	float scaleQf;
	multQuat(&quat[0], &deltaQ[0], &quatNew[0]);
	scaleQf = quatNew[0]*quatNew[0]+quatNew[1]*quatNew[1]+quatNew[2]*quatNew[2]+quatNew[3]*quatNew[3];

	// normalize quaternion
	stateNew[6] = 1/scaleQf*quatNew[0];
	stateNew[7] = 1/scaleQf*quatNew[1];
	stateNew[8] = 1/scaleQf*quatNew[2];
	stateNew[9] = 1/scaleQf*quatNew[3];

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


	directionCosineMatrix(&stateNav[6], &dcm[0][0]);
	transposeMatrix(&dcm[0][0], &dcmTp[0][0]);
	
	// initialize covariance matrix of navigation Filter
	for(ii=0; ii<DIM_FILTER; ii++){
		xEst[ii] = 0.0f;
	   for(jj=0; jj<DIM_FILTER; jj++){
		   covNavFilter[ii][jj] = 0.0f;
	   }
	}
	flowActive = true;

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

	//computeSigmaPoints();
}

// prediction step of error Kalman Filter
static void predictNavigationFilter(float *stateNav, Axis3f *acc, Axis3f *gyro, float dt){
	float accTs[3] = {acc->x*dt, acc->y*dt, acc->z*dt};
	float omegaTs[3] = {gyro->x*dt, gyro->y*dt, gyro->z*dt};
	float covNew[DIM_FILTER][DIM_FILTER] = {0};
	float errorTransMat[DIM_FILTER][DIM_FILTER] = {0};
	float sigmaTmpPlus[DIM_FILTER][DIM_FILTER] = {0};
	float sigmaTmpMinus[DIM_FILTER][DIM_FILTER] = {0};
	// Commented since xEst, state 0 are both zero
	//float diffStatePlus[DIM_FILTER][DIM_FILTER] = {0};
	//float diffStateMinus[DIM_FILTER][DIM_FILTER] = {0};

	// compute error State transition matrix
	uint8_t ii, jj, kk;
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

	// compute sigma points of UKF
	computeSigmaPoints();

	// Predict Sigma Points
  for(ii=0; ii<DIM_FILTER; ii++){
		for(jj=0; jj<DIM_FILTER; jj++){
			//state0[ii]        = 0.0f; // TODO CHECK xEst[ii]+   errorTransMat[ii][jj]*xEst[jj];
			for(kk=0; kk<DIM_FILTER; kk++){
				sigmaTmpPlus[ii][jj]  = sigmaTmpPlus[ii][jj]  + errorTransMat[ii][kk]*sigmaPointsPlus[kk][jj];
				sigmaTmpMinus[ii][jj] = sigmaTmpMinus[ii][jj] + errorTransMat[ii][kk]*sigmaPointsMinus[kk][jj];
			}
		}
	}

	for(ii=0; ii<DIM_FILTER; ii++){
		for(jj=0; jj<DIM_FILTER; jj++){
			sigmaPointsPlus[ii][jj]  = sigmaTmpPlus[ii][jj];
			sigmaPointsMinus[ii][jj] = sigmaTmpMinus[ii][jj];
		}
	}

	//compute mean from sigma points ( xEst[ii] ) commented since xEst zero
	//computeMeanFromSigmaPoints();

	// //commented since xEst zero after reset and assuming linear prediction model
	// //covMatPr = w0*(state0-statePr)*(state0-statePr)';
	// for(ii=0; ii<DIM_FILTER; ii++){
	// 	for(jj=0; jj<DIM_FILTER; jj++){
	// 		covNew[ii][jj] = w0*(state0[ii]-xEst[ii])*(state0[jj]-xEst[jj]); // state0 and xEst are zero
	// 		diffStatePlus[ii][jj]     = sigmaPointsPlus[ii][jj]-xEst[ii];
	// 		diffStateMinus[ii][jj]    = sigmaPointsMinus[ii][jj]-xEst[ii];
	// 	}
	// }

	// state0 and xEst are zero so 
	// diffStatePlus[ii][jj]     = sigmaPointsPlus[ii][jj]-xEst[ii]= sigmaPointsPlus[ii][jj]
	for(ii=0; ii<DIM_FILTER; ii++){
		for(jj=0; jj<DIM_FILTER; jj++){
			covNew[ii][jj] = 0.0f; 
      for(kk=0; kk<DIM_FILTER; kk++){
				//covNew[ii][jj]  = covNew[ii][jj]  + w1*(diffStatePlus[ii][kk]*diffStatePlus[jj][kk]);
				//covNew[ii][jj]  = covNew[ii][jj]  + w1*(diffStateMinus[ii][kk]*diffStateMinus[jj][kk]);
				covNew[ii][jj]  = covNew[ii][jj]  + w1*(sigmaPointsPlus[ii][kk]*sigmaPointsPlus[jj][kk]);
				covNew[ii][jj]  = covNew[ii][jj]  + w1*(sigmaPointsMinus[ii][kk]*sigmaPointsMinus[jj][kk]);
			}
		}
  }

	// account for process noise covariance Qk  
	covNew[0][0] += procA_h*dt*dt*dt*0.33f;
	covNew[1][1] += procA_h*dt*dt*dt*0.33f;
	covNew[2][2] += procA_z*dt*dt*dt*0.33f;

	covNew[0][3] += procA_h*dt*dt*0.5f;
	covNew[3][0] += procA_h*dt*dt*0.5f;
	covNew[1][4] += procA_h*dt*dt*0.5f;
	covNew[4][1] += procA_h*dt*dt*0.5f;
	covNew[2][5] += procA_z*dt*dt*0.5f;
	covNew[5][2] += procA_z*dt*dt*0.5f;

	covNew[3][3] += procA_h*dt;
	covNew[4][4] += procA_h*dt;
	covNew[5][5] += procA_z*dt;

	covNew[6][6] += procRate_h * dt;
	covNew[7][7] += procRate_h * dt;
	covNew[8][8] += procRate_z * dt;
	

	for(ii=0; ii<DIM_FILTER; ii++){
		for(jj=0; jj<DIM_FILTER; jj++){
			covNavFilter[ii][jj] = 0.5f*(covNew[ii][jj]+covNew[jj][ii]);
		}
	}
	
	computeSigmaPoints();

}

static bool updateQueuedMeasurements(const uint32_t tick, Axis3f* gyroAverage) {
	uint8_t ii, jj, kk;
	
	//float state0[DIM_FILTER] = {0};
	//float covNew[DIM_FILTER][DIM_FILTER];
	float Pyy = 0.0f;
	float Pxy[DIM_FILTER] = {0};

	float xyz[3];

	float observation = 0;
	float outTmp, tmpSigmaVecPlus[DIM_FILTER], tmpSigmaVecMinus[DIM_FILTER];
	float innovation, innoCheck;
  bool  doneUpdate = false;

  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {

		if(m.type==MeasurementTypeGyroscope){
		  gyroAccumulator.x += m.data.gyroscope.gyro.x;
      gyroAccumulator.y += m.data.gyroscope.gyro.y;
      gyroAccumulator.z += m.data.gyroscope.gyro.z;
      gyroLatest = m.data.gyroscope.gyro;
      gyroAccumulatorCount++;
		}
		if(m.type==MeasurementTypeAcceleration){
      accAccumulator.x += m.data.acceleration.acc.x;
      accAccumulator.y += m.data.acceleration.acc.y;
      accAccumulator.z += m.data.acceleration.acc.z;
      accLatest = m.data.acceleration.acc;
      accAccumulatorCount++;
		}


    switch (m.type) {
      case MeasurementTypeTDOA:
				if (tdoaCount >= 100)  {
					//_________________________________________________________________________________
					// UKF update - TDOA
					//_________________________________________________________________________________
					computeOutputTdoa( &outTmp, &state0[0],&m.data.tdoa);
					observation = w0*outTmp;
					for(jj=0; jj<DIM_FILTER; jj++){
						for(ii=0; ii<DIM_FILTER; ii++){
							tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
							tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
						}
						computeOutputTdoa( &outTmp, &tmpSigmaVecPlus[0],&m.data.tdoa);
						observation = observation + w1*outTmp;
						computeOutputTdoa( &outTmp, &tmpSigmaVecMinus[0],&m.data.tdoa);
						observation = observation + w1*outTmp;
					}
					computeOutputTdoa( &outTmp, &state0[0], &m.data.tdoa);
					Pyy =  w0*(outTmp-observation)*(outTmp-observation);

					// xEst and state0 are zero so no contribution for w0 weighted terms
					for(jj=0; jj<DIM_FILTER; jj++){
					 	Pxy[jj]=0.0f;//w0*(state0[jj]-xEst[jj])*(outTmp-observation);
					}

					for(jj=0; jj<DIM_FILTER; jj++){
						for(ii=0; ii<DIM_FILTER; ii++){
							tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
							tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
						}
						computeOutputTdoa( &outTmp, &tmpSigmaVecPlus[0], &m.data.tdoa);
						Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);
								
						for(kk=0; kk<DIM_FILTER; kk++){
							Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecPlus[kk]-xEst[kk])*(outTmp-observation);
						}

						computeOutputTdoa( &outTmp, &tmpSigmaVecMinus[0], &m.data.tdoa);
						Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);

						for(kk=0; kk<DIM_FILTER; kk++){
							Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecMinus[kk]-xEst[kk])*(outTmp-observation);
						}
					}
						// Add TDOA Noise R
					Pyy = Pyy + m.data.tdoa.stdDev*m.data.tdoa.stdDev;
					innovation = m.data.tdoa.distanceDiff-observation;

					float dx1 = stateNav[0] - m.data.tdoa.anchorPositions[1].x;
					float dy1 = stateNav[1] - m.data.tdoa.anchorPositions[1].y;
					float dz1 = stateNav[2] - m.data.tdoa.anchorPositions[1].z;

					float dx0 = stateNav[0] - m.data.tdoa.anchorPositions[0].x;
					float dy0 = stateNav[1] - m.data.tdoa.anchorPositions[0].y;
					float dz0 = stateNav[2] - m.data.tdoa.anchorPositions[0].z;

					float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
					float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));	
					vector_t jacobian = {
							.x = (dx1 / d1 - dx0 / d0),
							.y = (dy1 / d1 - dy0 / d0),
							.z = (dz1 / d1 - dz0 / d0),
					};

					point_t estimatedPosition = {
							.x = stateNav[0],
							.y = stateNav[1],
							.z = stateNav[2],
					};

					innoCheck = innovation *innovation/Pyy;
					if (outlierFilterValidateTdoaSteps(&m.data.tdoa, innovation, &jacobian, &estimatedPosition))
					{
						//	if(innoCheck<qualGateTdoa){ // TdoA outlier rejection 
						ukfUpdate(&Pxy[0], &Pyy, innovation);
						//	}
					}
				}
				tdoaCount++;
				break;

   	 case MeasurementTypeTOF:
    	  //_________________________________________________________________________________
        // UKF update - TOF
				//_________________________________________________________________________________
				if ((fabs(dcm[2][2]) > 0.1) && (dcm[2][2] > 0.0f)){
					
					computeOutputTof( &outTmp, &state0[0]);
					observation = w0*outTmp;
					for(jj=0; jj<DIM_FILTER; jj++){
						for(ii=0; ii<DIM_FILTER; ii++){
							tmpSigmaVecPlus[ii] = sigmaPointsPlus[ii][jj];
							tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
						}
						computeOutputTof( &outTmp, &tmpSigmaVecPlus[0]);
						observation = observation + w1*outTmp;
						computeOutputTof( &outTmp, &tmpSigmaVecMinus[0]);
						observation = observation + w1*outTmp;
					}
					computeOutputTof( &outTmp, &state0[0]);
					Pyy =  w0*(outTmp-observation)*(outTmp-observation);

					// xEst and state0 are zero so no contribution for w0 weighted terms
					for(jj=0; jj<DIM_FILTER; jj++){
						Pxy[jj]=0.0f;//w0*(state0[jj]-xEst[jj])*(outTmp-observation);
					}

					for(jj=0; jj<DIM_FILTER; jj++){
						for(ii=0; ii<DIM_FILTER; ii++){
							tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
							tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
						}
						computeOutputTof( &outTmp, &tmpSigmaVecPlus[0]);
						Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);
								
						for(kk=0; kk<DIM_FILTER; kk++){
							Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecPlus[kk]-xEst[kk])*(outTmp-observation);
						}

						computeOutputTof( &outTmp, &tmpSigmaVecMinus[0]);
						Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);

						for(kk=0; kk<DIM_FILTER; kk++){
							Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecMinus[kk]-xEst[kk])*(outTmp-observation);
						}
					}
					
					if(m.data.tof.distance < 0.03f){
						innovation = 0.0f-observation;
					}
					else{
						innovation = m.data.tof.distance-observation;
					}
					// Add TOF Noise R
					Pyy = Pyy + (m.data.tof.stdDev)*(m.data.tof.stdDev);
					predDist = m.data.tof.distance;
					measDist = observation;

					innoCheck = innovation *innovation/Pyy;
					innoCheckTof = innoCheck;

					// if an outlier is detected, flowActive flag prevents fusing optical flow too
					if(innoCheck<qualGateTof){
						ukfUpdate(&Pxy[0], &Pyy, innovation);
						flowActive = true;
					}
					else{
						flowActive = false;
					}
				}
				break;

       case MeasurementTypeFlow:

				if(flowActive){
					//_________________________________________________________________________________
					// UKF update - Flow - body x
					//_________________________________________________________________________________
					computeOutputFlow_x( &outTmp, &state0[0],&m.data.flow, gyroAverage);
					observation = w0*outTmp;
					for(jj=0; jj<DIM_FILTER; jj++){
						for(ii=0; ii<DIM_FILTER; ii++){
							tmpSigmaVecPlus[ii] = sigmaPointsPlus[ii][jj];
							tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
						}
						computeOutputFlow_x( &outTmp, &tmpSigmaVecPlus[0],&m.data.flow, gyroAverage);
						observation = observation + w1*outTmp;
						computeOutputFlow_x( &outTmp, &tmpSigmaVecMinus[0],&m.data.flow, gyroAverage);
						observation = observation + w1*outTmp;
					}
					computeOutputFlow_x( &outTmp, &state0[0],&m.data.flow, gyroAverage);
					Pyy =  w0*(outTmp-observation)*(outTmp-observation);

					// xEst and state0 are zero so no contribution for w0 weighted terms
					for(jj=0; jj<DIM_FILTER; jj++){
						Pxy[jj]=0.0f;//w0*(state0[jj]-xEst[jj])*(outTmp-observation);
					}

					for(jj=0; jj<DIM_FILTER; jj++){
						for(ii=0; ii<DIM_FILTER; ii++){
							tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
							tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
						}
						computeOutputFlow_x( &outTmp, &tmpSigmaVecPlus[0],&m.data.flow, gyroAverage);
						Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);
								
						for(kk=0; kk<DIM_FILTER; kk++){
							Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecPlus[kk]-xEst[kk])*(outTmp-observation);
						}

						computeOutputFlow_x( &outTmp, &tmpSigmaVecMinus[0],&m.data.flow, gyroAverage);
						Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);

						for(kk=0; kk<DIM_FILTER; kk++){
							Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecMinus[kk]-xEst[kk])*(outTmp-observation);
						}
					}
					
					Pyy = Pyy + (m.data.flow.stdDevX)*(m.data.flow.stdDevX);

					innovation = m.data.flow.dpixelx-observation;
					meas_NX = m.data.flow.dpixelx;
					pred_NX = observation;

					ukfUpdate(&Pxy[0], &Pyy, innovation);
				
					// compute sigma points of UKF after previous update in x Direction
					computeSigmaPoints();

					//_________________________________________________________________________________
					// UKF update - Flow - body y
					//_________________________________________________________________________________
					computeOutputFlow_y( &outTmp, &state0[0],&m.data.flow, gyroAverage);
					observation = w0*outTmp;
					for(jj=0; jj<DIM_FILTER; jj++){
						for(ii=0; ii<DIM_FILTER; ii++){
							tmpSigmaVecPlus[ii] = sigmaPointsPlus[ii][jj];
							tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
						}
						computeOutputFlow_y( &outTmp, &tmpSigmaVecPlus[0],&m.data.flow, gyroAverage);
						observation = observation + w1*outTmp;
						computeOutputFlow_y( &outTmp, &tmpSigmaVecMinus[0],&m.data.flow, gyroAverage);
						observation = observation + w1*outTmp;
					}
					computeOutputFlow_y( &outTmp, &state0[0],&m.data.flow, gyroAverage);
					Pyy =  w0*(outTmp-observation)*(outTmp-observation);
					// xEst and state0 are zero so no contribution for w0 weighted terms
					for(jj=0; jj<DIM_FILTER; jj++){
						Pxy[jj]=0.0f;//w0*(state0[jj]-xEst[jj])*(outTmp-observation);
					}

					for(jj=0; jj<DIM_FILTER; jj++){
						for(ii=0; ii<DIM_FILTER; ii++){
							tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
							tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
						}
						computeOutputFlow_y( &outTmp, &tmpSigmaVecPlus[0],&m.data.flow, gyroAverage);
						Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);
								
						for(kk=0; kk<DIM_FILTER; kk++){
							Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecPlus[kk]-xEst[kk])*(outTmp-observation);
						}

						computeOutputFlow_y( &outTmp, &tmpSigmaVecMinus[0],&m.data.flow, gyroAverage);
						Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);

						for(kk=0; kk<DIM_FILTER; kk++){
							Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecMinus[kk]-xEst[kk])*(outTmp-observation);
						}
					}
					
					// Add TOF Noise R
					Pyy = Pyy + (m.data.flow.stdDevY)*(m.data.flow.stdDevY);

					innovation = m.data.flow.dpixely-observation;
					meas_NY = m.data.flow.dpixely;
					pred_NY = observation;

					ukfUpdate(&Pxy[0], &Pyy, innovation);
				}
        break;
      case MeasurementTypeBarometer:
				//_________________________________________________________________________________
        // UKF update - Baro
				//_________________________________________________________________________________
				computeOutputBaro( &outTmp, &state0[0]);
				observation = w0*outTmp;
				for(jj=0; jj<DIM_FILTER; jj++){
					for(ii=0; ii<DIM_FILTER; ii++){
						tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
						tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
					}
					computeOutputBaro( &outTmp, &tmpSigmaVecPlus[0]);
					observation = observation + w1*outTmp;
					computeOutputBaro( &outTmp, &tmpSigmaVecMinus[0]);
					observation = observation + w1*outTmp;
				}
				
				computeOutputBaro( &outTmp, &state0[0]);
				Pyy =  w0*(outTmp-observation)*(outTmp-observation);
				// xEst and state0 are zero so no contribution for w0 weighted terms
				for(jj=0; jj<DIM_FILTER; jj++){
				 	Pxy[jj]=0.0f;//w0*(state0[jj]-xEst[jj])*(outTmp-observation);
				}

				for(jj=0; jj<DIM_FILTER; jj++){
					for(ii=0; ii<DIM_FILTER; ii++){
						tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
						tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
					}
					computeOutputBaro( &outTmp, &tmpSigmaVecPlus[0]);
					Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);
							
							
					for(kk=0; kk<DIM_FILTER; kk++){
						Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecPlus[kk]-xEst[kk])*(outTmp-observation);
					}

					computeOutputBaro( &outTmp, &tmpSigmaVecMinus[0]);
					Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);

					for(kk=0; kk<DIM_FILTER; kk++){
						Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecMinus[kk]-xEst[kk])*(outTmp-observation);
					}
				}
				
				// Add Baronoise R
				Pyy = Pyy +  measNoiseBaro;

				innovation = m.data.barometer.baro.asl-observation;
				innoCheck = innovation *innovation/Pyy;

				if(innoCheck<qualGateBaro){
					ukfUpdate(&Pxy[0], &Pyy, innovation);
				}
        break;

				case MeasurementTypeSweepAngle:

					computeOutputSweep( &outTmp, &state0[0],&m.data.sweepAngle, &xyz[0]);
					const float r = arm_sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);

					const float tan_t = tanf(	m.data.sweepAngle.t);
					const float z_tan_t = xyz[2] * tan_t;
					const float qNum = r*r - z_tan_t * z_tan_t;

					// Avoid singularity
					if (qNum > 0.0001f){
						observation = w0*outTmp;
						for(jj=0; jj<DIM_FILTER; jj++){
							for(ii=0; ii<DIM_FILTER; ii++){
								tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
								tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
							}
							computeOutputSweep( &outTmp, &tmpSigmaVecPlus[0],&m.data.sweepAngle,&xyz[0]);
							observation = observation + w1*outTmp;
							computeOutputSweep( &outTmp, &tmpSigmaVecMinus[0],&m.data.sweepAngle,&xyz[0]);
							observation = observation + w1*outTmp;
						}
						computeOutputSweep( &outTmp, &state0[0], &m.data.sweepAngle, &xyz[0]);
						Pyy =  w0*(outTmp-observation)*(outTmp-observation);
						// xEst and state0 are zero so no contribution for w0 weighted terms
						for(jj=0; jj<DIM_FILTER; jj++){
							Pxy[jj]=0.0f;//w0*(state0[jj]-xEst[jj])*(outTmp-observation);
						}

						for(jj=0; jj<DIM_FILTER; jj++){
							for(ii=0; ii<DIM_FILTER; ii++){
								tmpSigmaVecPlus[ii]  = sigmaPointsPlus[ii][jj];
								tmpSigmaVecMinus[ii] = sigmaPointsMinus[ii][jj];
							}
							computeOutputSweep( &outTmp, &tmpSigmaVecPlus[0], &m.data.sweepAngle, &xyz[0]);
							Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);
									
							for(kk=0; kk<DIM_FILTER; kk++){
								Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecPlus[kk]-xEst[kk])*(outTmp-observation);
							}

							computeOutputSweep( &outTmp, &tmpSigmaVecMinus[0], &m.data.sweepAngle, &xyz[0]);
							Pyy = Pyy + w1*( outTmp-observation)*( outTmp-observation);

							for(kk=0; kk<DIM_FILTER; kk++){
								Pxy[kk] = Pxy[kk] + w1*(tmpSigmaVecMinus[kk]-xEst[kk])*(outTmp-observation);
							}
						}
							// Add TDOA Noise R
						Pyy = Pyy + m.data.sweepAngle.stdDev*m.data.sweepAngle.stdDev;
						innovation = m.data.sweepAngle.measuredSweepAngle-observation;
						//DEBUG_PRINT("In Sweep Angle Measurement...\n");
						// pred_NX = Pyy;
						// pred_NY = innovation;

						innoCheck = innovation *innovation/Pyy;
						if(outlierFilterValidateLighthouseSweep(&sweepOutlierFilterState, r, innovation, tick)){
							//if(innoCheck<qualGateSweep){	
								ukfUpdate(&Pxy[0], &Pyy, innovation);
						//	}
							  //DEBUG_PRINT("In Sweep Angle Measurement Update...\n");
						}
					}
				break;

      default:
         break;
		}
  }


  return doneUpdate;
}

static void computeOutputBaro(float *output, float* state){
	output[0] = stateNav[2]+state[2];
}

static void computeOutputTof(float *output, float* state){
	output[0] = (stateNav[2]+state[2])/dcm[2][2];
}

static void computeOutputFlow_x(float *output, float* state, flowMeasurement_t *flow, Axis3f *omegaBody){
	float thetapix = DEG_TO_RAD * 4.2f;
	float Npix = 30.0f;                      // [pixels] (same in x and y)
	float velocityBody_x;
	float h_g;
	float omegaFactor = 1.25f;

	velocityBody_x =  dcm[0][0]*(stateNav[3]+state[3])+dcm[0][1]*(stateNav[4]+state[4])+dcm[0][2]*(stateNav[5]+state[5]);

	if (  stateNav[2] < 0.1f ) {
		h_g = 0.1f;
	}
	else {
		h_g = stateNav[2]+state[2];
	}

	output[0]  = (flow->dt * Npix / thetapix ) * ((velocityBody_x/h_g*dcm[2][2]) -  omegaFactor * omegaBody->y);
}

static void computeOutputFlow_y(float *output, float* state, flowMeasurement_t *flow, Axis3f *omegaBody){
	float thetapix = DEG_TO_RAD * 4.2f;
	float Npix = 30.0f;                      // [pixels] (same in x and y)
	float velocityBody_y;
	float h_g;
	float omegaFactor = 1.25f;

	velocityBody_y =  dcm[1][0]*(stateNav[3]+state[3])+dcm[1][1]*(stateNav[4]+state[4])+dcm[1][2]*(stateNav[5]+state[5]);

	if (  stateNav[2] < 0.1f ) {
		h_g = 0.1f;
	}
	else {
		h_g = stateNav[2]+state[2];
	}

	output[0]  = (flow->dt * Npix / thetapix ) * ((velocityBody_y/h_g*dcm[2][2]) +  omegaFactor * omegaBody->x);
}

static void computeOutputTdoa(float *output, float* state, tdoaMeasurement_t *tdoa){
     // predict based on current state
     float x = stateNav[0]+state[0];
     float y = stateNav[1]+state[1];
     float z = stateNav[2]+state[2];

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

     output[0]  = d1 - d0;
}

static void computeOutputSweep(float *output, float* state, sweepAngleMeasurement_t *sweepInfo, float *xyz){
 		float posSensor[3];
 		//	m.data.sweepAngle
 		// Rotate the sensor position from CF reference frame to global reference frame,
 		// using the CF roatation matrix
 		posSensor[0] = dcmTp[0][0]*((float32_t)(*sweepInfo->sensorPos)[0])+dcmTp[0][1]*((float32_t)(*sweepInfo->sensorPos)[1])+dcmTp[0][2]*((float32_t)(*sweepInfo->sensorPos)[2]);
 		posSensor[1] = dcmTp[1][0]*((float32_t)(*sweepInfo->sensorPos)[0])+dcmTp[1][1]*((float32_t)(*sweepInfo->sensorPos)[1])+dcmTp[1][2]*((float32_t)(*sweepInfo->sensorPos)[2]);
 		posSensor[2] = dcmTp[2][0]*((float32_t)(*sweepInfo->sensorPos)[0])+dcmTp[2][1]*((float32_t)(*sweepInfo->sensorPos)[1])+dcmTp[2][2]*((float32_t)(*sweepInfo->sensorPos)[2]);

		
		//DEBUG_PRINT("posSensor_1 %f \n", (double)posSensor[0]);
 		//DEBUG_PRINT("posSensor_2 %f \n", (double)posSensor[1]);
 		//DEBUG_PRINT("posSensor_3 %f \n", (double)posSensor[2]);
 		
		 // Get the current state values of the position of the crazyflie (global reference frame) and add the relative sensor pos
 		vec3d pcf = {stateNav[0]+state[0] + posSensor[0], stateNav[1]+state[1] + posSensor[1], stateNav[2]+state[2] + posSensor[2]};
							
 		// Calculate the difference between the rotor and the sensor on the CF (global reference frame)
 		pcf[0] = pcf[0]-((float32_t)(*sweepInfo->rotorPos)[0]);
 		pcf[1] = pcf[1]-((float32_t)(*sweepInfo->rotorPos)[1]);
 		pcf[2] = pcf[2]-((float32_t)(*sweepInfo->rotorPos)[2]);

			
		// DEBUG_PRINT("pcf_1 %f \n", (double)pcf[0]);
 		// DEBUG_PRINT("pcf_2 %f \n", (double)pcf[1]);
 		// DEBUG_PRINT("pcf_3 %f \n", (double)pcf[2]);

 		arm_matrix_instance_f32 pcf_ = {3, 1, pcf};				
	  // Rotate the difference in position to the rotor reference frame,
 		// using the rotor inverse rotation matrix
 		vec3d sr;
		arm_matrix_instance_f32 Rr_inv_ = {3, 3, (float32_t *)(*sweepInfo->rotorRotInv)};
		arm_matrix_instance_f32 sr_ = {3, 1, sr};
 		mat_mult(&Rr_inv_, &pcf_, &sr_);

 		// The following computations are in the rotor reference frame
 		const float t = sweepInfo->t;

 		xyz[0] = sr[0];
 		xyz[1] = sr[1];
		xyz[2] = sr[2];

 		output[0] = sweepInfo->calibrationMeasurementModel(sr[0], sr[1], sr[2], t, sweepInfo->calib);					
}




static void computeSigmaPoints(void ){
	uint8_t ii, jj;// kk;
	float L[DIM_FILTER][DIM_FILTER] = {0};

	cholesky(&covNavFilter[0][0], &L[0][0] ,  DIM_FILTER); 

	// Compute Sigma Points from Covariance, note that 
	for(ii=0; ii<DIM_FILTER; ii++){
		for(jj=0; jj<DIM_FILTER; jj++){
   	  sigmaPointsPlus[ii][jj]  = xEst[ii]+scaleSigma*L[ii][jj];
      sigmaPointsMinus[ii][jj] = xEst[ii]-scaleSigma*L[ii][jj];
		}
  }
}

static bool ukfUpdate(float* Pxy, float* Pyy, float innovation){
	uint8_t ii, jj;
	float Kk[DIM_FILTER] = {0};
	float covNew[DIM_FILTER][DIM_FILTER] = {0};
	float KkRKkTp[DIM_FILTER][DIM_FILTER] = {0};
	bool  doneUpdate = false;

	for(ii=0; ii<DIM_FILTER; ii++){
		for(jj=0; jj<DIM_FILTER; jj++){
			covNew[ii][jj]  = covNavFilter[ii][jj];
		}
	}


	for(ii=0; ii<DIM_FILTER; ii++){
		Kk[ii] = Pxy[ii]/Pyy[0];
		xEst[ii] = xEst[ii] + Kk[ii] *innovation;
	}
	for(ii=0; ii<DIM_FILTER; ii++){
		for(jj=0; jj<DIM_FILTER; jj++){
			KkRKkTp[ii][jj] = Kk[ii]*Kk[jj]*Pyy[0];
			covNew[ii][jj]  = covNew[ii][jj]-KkRKkTp[ii][jj];
		}
	}
	for(ii=0; ii<DIM_FILTER; ii++){
		for(jj=0; jj<DIM_FILTER; jj++){
			covNavFilter[ii][jj] = 0.5f*covNew[ii][jj]+0.5f*covNew[jj][ii];
		}
	}

	if(useNavigationFilter){
		resetNavigationStates();
		doneUpdate = true;
	}
	return doneUpdate;
}

// reset strapdown navigation after filter update step if measurements were obtained
static void resetNavigationStates( ){
	uint8_t ii;

	float attVec[3]  = {xEst[6], xEst[7], xEst[8]};
	float quatNav[4] = {stateNav[6], stateNav[7], stateNav[8], stateNav[9]};
	float attQuat[4], quatRes[4];

	if((isnan(xEst[0]))||(isnan(xEst[1]))||
		(isnan(xEst[2]))||(isnan(xEst[3]))||
		(isnan(xEst[4]))||(isnan(xEst[5]))||
		(isnan(xEst[6]))||(isnan(xEst[7]))||
		(isnan(xEst[8]))){
		for(ii=0; ii<DIM_FILTER; ii++){
			xEst[ii] = 0.0f;
		}
		nanCounterFilter++;
	}	
	else{
		// update position and velocity state
		for(ii=0; ii<6; ii++){
			stateNav[ii] = stateNav[ii] + xEst[ii];
		}

		quatFromAtt(&attVec[0], &attQuat[0]);
		multQuat(&quatNav[0], &attQuat[0], &quatRes[0]);

		for(ii=6; ii<10; ii++){
			stateNav[ii] = quatRes[ii-6];
		}

		for(ii=0; ii<DIM_FILTER; ii++){	
			xEst[ii] = 0.0f;
		}

		directionCosineMatrix(&quatRes[0], &dcm[0][0]);
		transposeMatrix(&dcm[0][0], &dcmTp[0][0]);
		computeSigmaPoints();
	}
}

static uint8_t cholesky(float *A, float *L, uint8_t n) { 
  for (uint8_t i = 0; i < n; i++){
        for (uint8_t j = 0; j < (i+1); j++) {
            float s = 0.0f;
            for (uint8_t  k = 0; k < j; k++)
                s += L[i * n + k] * L[j * n + k];
            L[i * n + j] = (i == j) ? sqrtf(A[i * n + i] - s) : (1.0f / L[j * n + j] * (A[i * n + j] - s));
        }
	}
    return 1;
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
	LOG_ADD(LOG_FLOAT, innoChFlow_x, &innoCheckFlow_x)
  LOG_ADD(LOG_FLOAT, innoChFlow_y, &innoCheckFlow_y)
  LOG_ADD(LOG_FLOAT, innoChTof, &innoCheckTof)
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

PARAM_GROUP_START(ukf)
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
	PARAM_ADD(PARAM_FLOAT, qualityGateSweep, &qualGateSweep)
	PARAM_ADD(PARAM_FLOAT, ukfScale , &scaleSigma)
  PARAM_ADD(PARAM_FLOAT, ukfw0, &w0)
	PARAM_ADD(PARAM_FLOAT, ukfw1 , &w1)
PARAM_GROUP_STOP(ukf)
