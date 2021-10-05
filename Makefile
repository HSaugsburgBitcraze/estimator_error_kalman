# To enable out of tree estimator
CFLAGS += -DOOT_ESTIMATOR

# If lighthouse is used as ground truth
CFLAGS += -DLIGHTHOUSE_AS_GROUNDTRUTH

# To use the modified LPS deck
 #CFLAGS += -DLOCODECK_USE_ALT_PINS
 #CFLAGS += -DLOCODECK_ALT_PIN_RESET=DECK_GPIO_IO4

# to use loco deck at full power
#CFLAGS += -DLPS_FULL_TX_POWER

APP=1
APP_STACKSIZE=300

VPATH += src/
PROJ_OBJ += errorEstimator_kalman.o app_error_kalman.o

CFLAGS += -DERROR_KALMAN_TASK_PRI=5

CRAZYFLIE_BASE=crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile