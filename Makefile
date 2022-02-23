# If lighthouse is used as ground truth
EXTRA_CFLAGS += -DLIGHTHOUSE_AS_GROUNDTRUTH

# To use the modified LPS deck
 #EXTRA_CFLAGS += -DLOCODECK_USE_ALT_PINS
 #EXTRA_CFLAGS += -DLOCODECK_ALT_PIN_RESET=DECK_GPIO_IO4

# to use loco deck at full power
#EXTRA_CFLAGS += -DLPS_FULL_TX_POWER

EXTRA_CFLAGS += -DERROR_KALMAN_TASK_PRI=5

CRAZYFLIE_BASE=crazyflie-firmware
include $(CRAZYFLIE_BASE)/tools/make/oot.mk
