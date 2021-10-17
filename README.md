Crazyflie Firmware with error-state UKF - out of tree version

This project contains the source code for the firmware used in the Crazyflie range of platforms, including the Crazyflie 2.X and the Roadrunner with the following modifications: Alternate Navigation Filter: error-state UKF and path following controller implemented in the stabilizer task, to activate set pathCtrMode to >=1, setting 0 should return to position controlled mode


error-state UKF - Usage/related parameters

Switch to error-state UKF by setting stabilizer.estimator=3 -> console should tell Estimator 3 is active Error state UKF might need a reset after switching: To this end, set the parameter ukf.resetEstimation to 1 Wait around 1 second before take-off (algorithm estimates IMU bias erros), if you want to be reduce the risk of crashes check the correct initializsation of the CF state variables to the current position before take-off. Remark: Only switch estimators prior to take-off, do not switch during flight!

The setup in this repository is for use with LPS and/or Flow-Deck v2. The outlier rejection quality gate, (parameter ukf.qualityGateTof), is currently set for LPS & Flow Deck v2. If you run into initial convergence issues (when purely relying on Flow-Deck) increase ukf.qualityGateTof. Set it to very high values for deactivating the outlier rejection.

The algorithm is also capable of fusing Lighthouse sweep angle measurements. However, to assure initial convergence, set ukf.sigmaInitPos_xy = 10 and ukf.sigmaInitPos_xy = 0.2

For using lighthouse as ground truth measurement, you will have to use a modified LPS deck and set the following compile flags within the config.mk file: CFLAGS += -DLOCODECK_USE_ALT_PINS CFLAGS += -DLOCODECK_ALT_PIN_RESET=DECK_GPIO_IO4 CFLAGS += -DLIGHTHOUSE_AS_GROUNDTRUTH and set the parameter lighthouse.method = 0


Path Following Controller

Apply your usual takeoff command. Then activate this controller by setting stabilizer.pathCtrMode to >=1. The CF will now start to fly a rectangular shaped trajectory with corner points defined by the parameters pathCtr.xLow, pathCtr.xHigh etc. The segment speed can be set with the parameter pathCtr.speed and the acceleration/deceleration distances by pathCtr.accDist

To leave the mode just set stabilizer.pathCtrMode=0, you should do this before using your usual land-command. Do not toggle between those modes since there might be odd behavior of the path controller due to the internal state after the previous run (hopefully solved currently, however, no guarantee).


Building and Flashing

For building and flashing see common bitcraze advices. See the building and flashing instructions in the github docs folder.
License

The code is licensed under LGPL-3.0
