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

#include "estimator_error_kalman.h"

#define DEBUG_MODULE "KALMAN_ERROR"
#include "debug.h"

#include "param.h"


void appMain()
{
    DEBUG_PRINT("Initalizing Error Kalman Filter\n");
    errorEstimatorKalmanTaskInit();
}

