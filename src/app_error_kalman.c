#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"

#include "radiolink.h"
#include "configblock.h"

#include "errorEstimator_kalman.h"

#define DEBUG_MODULE "KALMAN_ERROR"
#include "debug.h"

#include "param.h"
#include "system.h"
static bool isInit = false;


void estimatorOutOfTreeTaskInit(void)
{
    errorEstimatorKalmanTaskInit();
}

bool estimatorOutOfTreeTest(void)
{
  return isInit;
}

void estimatorOutOfTree(state_t *state, const uint32_t tick)
{
    errorEstimatorKalman(state, tick);

}

void estimatorOutOfTreeInit(void)
{
    errorEstimatorKalmanInit();
}


void appMain()
{
    //systemWaitStart();
    DEBUG_PRINT("Initalizing Error Kalman Filter\n");
    errorEstimatorKalmanTaskInit();

  while(1) {
    vTaskDelay(M2T(2000));
    DEBUG_PRINT("Hello World!\n");
  }
}

