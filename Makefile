
CFLAGS += -DOOT_ESTIMATOR

VPATH += src/
PROJ_OBJ += estimator_error_kalman.o

CRAZYFLIE_BASE=crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile