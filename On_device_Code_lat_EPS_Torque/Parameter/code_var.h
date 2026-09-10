#ifndef _CONTROLCOFF_
#define _CONTROLCOFF_

//Setting parameters
#define MODE  NORMAL_TEST
#define ESTIMATOR NO_ESTIMATOR
#define CONTROL LQ_CONTROL
#define CAR_TYPE HMC_LM
#define STEERING MANDO_CEPS
#define CAMERA_TYPE MOBILEYE
#define CONTROL_FF NO_FEEDFORWARD
#define PREDICTION_PVL NO_PVL
#define QUALITY_PVL QUALITY_PVL_OFF
#define CONTROL_MODE CONTROL_LXC
#define STEERING_CONTROLLER BACKSTEPPING


 /* STATE FEEDBACK GAIN */ 

#define STEER_N    15.300000
#define L_FRONT     0.967000
#define Caf 118800.000000
#define multi_N            15.3
#define SYS_SAMPLETIME     0.010000
#define PI     3.141593
#define Xv_size      3
#define Yv_size      2
#define Xmu_size      1

#endif
