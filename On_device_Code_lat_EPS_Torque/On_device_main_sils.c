#define S_FUNCTION_NAME On_device_main_sils
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "./Parameter/car.par"  
#include "./Parameter/Type_def.h"
#include "./Parameter/code_var.h"
#include "./Parameter/NumericalOperator.h"
#include "./Perception/Perception.h"
#include "./Perception/Input_Mapping/Input_Mapping.h"
#include "./Control/Control.h"
#include "./Control/Lateral_Controller/Lane_Keeping_Control/LKSControl.h"
#include "./Control/Lateral_Controller/EPS_Torque_Control/SteeringControlBackstepping.h"

#define NUM_PARAMS        0

#define NumberInputPort   4 

#define WidthInputPort0   14 
#define WidthInputPort1   3
#define WidthInputPort2   13
#define WidthInputPort3   2

#define NumberOutputPort  1
 
#define WidthOutputPort0	1

void Initial_Reset(void);

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, NumberInputPort)) return;
    
    ssSetInputPortWidth(S,  0, WidthInputPort0);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    ssSetInputPortWidth(S,  1, WidthInputPort1);
    ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    ssSetInputPortWidth(S,  2, WidthInputPort2);
    ssSetInputPortRequiredContiguous(S, 2, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    
    ssSetInputPortWidth(S,  3, WidthInputPort3);
    ssSetInputPortRequiredContiguous(S, 3, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    
    if (!ssSetNumOutputPorts(S, NumberOutputPort)) return;
  
    ssSetOutputPortWidth(S, 0, WidthOutputPort0);

    ssSetNumSampleTimes(S, 1);

    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE|SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE));
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, SYS_SAMPLETIME);
	ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  static void mdlInitializeConditions(SimStruct *S)
  {
    int i;

    ECAN_SWA_Angle=0;
    Logic_ref_SWA=0;
        
    Logic_EPS_Obs_xh[0] 	  	 = 	0.000000;
    Logic_EPS_Obs_xh[1] 	  	 = 	0.000000;
    Logic_EPS_Obs_xh[2] 	  	 = 	0.000000;
    Logic_EPS_Obs_xh[3] 	  	 = 	0.000000;
    Logic_EPS_Obs_xh[4] 	  	 = 	0.000000;
        
    for(i=0;i<4;i++)
    {               
        Logic_EPS_x_des[i] = 0;
        Logic_EPS_x_d_old[i] = 0;
        Logic_EPS_dx_d[i] = 0;
        Logic_EPS_dx_d_old[i] = 0;
        Logic_EPS_error_x[i] = 0;
    }
    
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  static void mdlStart(SimStruct *S)
  {
  	Initial_Reset();
  }
#endif 

void    Initial_Reset(void)
{
}
 
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T       *u0 = (real_T*) ssGetInputPortSignal(S,0);
    real_T       *u1 = (real_T*) ssGetInputPortSignal(S,1);
    real_T       *u2 = (real_T*) ssGetInputPortSignal(S,2);
    real_T       *u3 = (real_T*) ssGetInputPortSignal(S,3);
    
    real_T       *y0 =           ssGetOutputPortSignal(S,0);

    /* E-CAN SUB - lateral sensor inputs */
    ECAN_IMU_LatAccel                   = (float64_t)(u0[0]);                 // unit : [g]                 Chassis lateral acceleration                 (IMU_LatAccelVal)       E-CAN
    ECAN_IMU_YawRt                      = (float64_t)(u0[1])*PI/180;          // unit : [deg/s -> rad/s]    Chassis IMU yawrate                          (IMU_YawRtVal)          E-CAN
    ECAN_SWA_Angle                      = (float64_t)(u0[2])*PI/180;          // unit : [deg -> rad]        Steering wheel angle                         (SWA_AnglVal)           E-CAN
    ECAN_SWA_Spd                        = (float64_t)(u0[3])*PI/180;          // unit : [deg/s -> rad/s]    Steering wheel angular speed                 (SWA_SpdVal)            E-CAN
    ECAN_WHL_SpdRR                      = (float64_t)(u0[4])/3.6;             // unit : [kph -> mps]        Vehicle wheel rear right speed               (WHL_SpdRRVal)          E-CAN
    ECAN_WHL_SpdRL                      = (float64_t)(u0[5])/3.6;             // unit : [kph -> mps]        Vehicle wheel rear left speed                (WHL_SpdRLVal)          E-CAN
    ECAN_CMR_Right_C[0]                 = (float64_t)(u0[6]);                 // unit : [m]                 Camera - Right lane lateral offset : C0      (Right_C0)              E-CAN
    ECAN_CMR_Right_C[1]                 = (float64_t)(u0[7]);                 // unit : [rad]               Camera - Right lane heading angle C1         (Right_C1)              E-CAN
    ECAN_CMR_Right_C[2]                 = (float64_t)(u0[8]);                 // unit : [1/m]               Camera - Right lane curvature C2             (Right_C2)              E-CAN
    ECAN_CMR_Right_C[3]                 = (float64_t)(u0[9]);                 // unit : [1/m^2]             Camera - Right lane curvature rate C3        (Right_C3)              E-CAN
    ECAN_CMR_Left_C[0]                  = (float64_t)(u0[10]);                // unit : [m]                 Camera - Left lane lateral offset C0         (Left_C0)               E-CAN
    ECAN_CMR_Left_C[1]                  = (float64_t)(u0[11]);                // unit : [rad]               Camera - Left lane heading angle C1          (Left_C1)               E-CAN
    ECAN_CMR_Left_C[2]                  = (float64_t)(u0[12]);                // unit : [1/m]               Camera - Left lane curvature C2              (Left_C2)               E-CAN
    ECAN_CMR_Left_C[3]                  = (float64_t)(u0[13]);                // unit : [1/m^2]             Camera - Left lane curvature rate C3         (Left_C3)               E-CAN

    /* Scenario CAN SUB - lateral command inputs */
    ScenarioCAN_Driver_ID               = (float64_t)(u1[0]);                 // unit : [-]                 Driver ID                                    (Driver_ID)             Scenario CAN
    ScenarioCAN_LKS_ON_OFF_SEND         = (float64_t)(u1[1]);                 // unit : [-]                 sLLM LKS on/off command (0 - off/ 1 - on)    (LKS_ON_OFF_SEND)       Scenario CAN
    ScenarioCAN_LKS_LEVEL_SEND          = (float64_t)(u1[2]);                 // unit : [-]                 sLLM LKS level command (1 ~ 3)               (LKS_LEVEL_SEND)        Scenario CAN
    
    /* Controller gain SUB - u2: LKS */
                      
    /* LKS LQR gain */
    MAB_LKS_L1_LQR_K[0]                 = (float64_t)(u2[0]);                 // unit : [-]                 LQR level:1 gain 1                           (Control desk)          MAB3 LAN
    MAB_LKS_L1_LQR_K[1]                 = (float64_t)(u2[1]);                 // unit : [-]                 LQR level:1 gain 2                           (Control desk)          MAB3 LAN
    MAB_LKS_L1_LQR_K[2]                 = (float64_t)(u2[2]);                 // unit : [-]                 LQR level:1 gain 3                           (Control desk)          MAB3 LAN
    MAB_LKS_L1_LQR_K[3]                 = (float64_t)(u2[3]);                 // unit : [-]                 LQR level:1 gain 4                           (Control desk)          MAB3 LAN

    MAB_LKS_L2_LQR_K[0]                 = (float64_t)(u2[4]);                 // unit : [-]                 LQR level:2 gain 1                           (Control desk)          MAB3 LAN
    MAB_LKS_L2_LQR_K[1]                 = (float64_t)(u2[5]);                 // unit : [-]                 LQR level:2 gain 2                           (Control desk)          MAB3 LAN
    MAB_LKS_L2_LQR_K[2]                 = (float64_t)(u2[6]);                 // unit : [-]                 LQR level:2 gain 3                           (Control desk)          MAB3 LAN
    MAB_LKS_L2_LQR_K[3]                 = (float64_t)(u2[7]);                 // unit : [-]                 LQR level:2 gain 4                           (Control desk)          MAB3 LAN

    MAB_LKS_L3_LQR_K[0]                 = (float64_t)(u2[8]);                 // unit : [-]                 LQR level:3 gain 1                           (Control desk)          MAB3 LAN
    MAB_LKS_L3_LQR_K[1]                 = (float64_t)(u2[9]);                 // unit : [-]                 LQR level:3 gain 2                           (Control desk)          MAB3 LAN
    MAB_LKS_L3_LQR_K[2]                 = (float64_t)(u2[10]);                // unit : [-]                 LQR level:3 gain 3                           (Control desk)          MAB3 LAN
    MAB_LKS_L3_LQR_K[3]                 = (float64_t)(u2[11]);                // unit : [-]                 LQR level:3 gain 4                           (Control desk)          MAB3 LAN
  
    /* LKS preview parameter */
    MAB_LKS_LOOKAHEAD_DISTANCE          = (float64_t)(u2[12]);                // unit : [m]                 Lateral lookahead distance                   (Control desk)          MAB3 LAN

    /* Lateral-only config */
    MAB_Lateral_flag                    = (float64_t)(u3[0]);                 // unit : [-]                 Lateral control flag (0 - off/ 1 - on)       (Control desk)          MAB3 LAN
    MAB_Lateral_Level                   = (float64_t)(u3[1]);                 // unit : [-]                 Lateral control level (1 ~ 3)                (Control desk)          MAB3 LAN

    Perception();
    Control();                                                                // Lateral controller integration and gating

    /* Gateway CAN Tx - to Vehicle */
    y0[0]                               = Lateral_Cont_Torque;                // unit : [N*m]               EPS torque request                           (StrToqReq)             Gateway CAN
} 

#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)

  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */

#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)

  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */

static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#include "./Perception/Input_Mapping/Input_Mapping.c"
#include "./Perception/Make_Vehicle_stat/Make_lane/Make_lane.c"
#include "./Perception/Make_Vehicle_stat/Make_Vehicle_Speed/Make_Vehicle_Speed.c"
#include "./Perception/Perception.c"
#include "./Control/Control.c"
#include "./Control/Lateral_Controller/EPS_Torque_Control/SteeringControlBackstepping.c"
#include "./Perception/Path_Command/PathCommand.c"
#include "./Control/Lateral_Controller/Lane_Keeping_Control/LKSControl.c"
#include "./Parameter/NumericalOperator.c"

#else
#include "cg_sfun.h"       /* Code generation registration function */
#include "./Perception/Input_Mapping/Input_Mapping.c"
#include "./Perception/Make_Vehicle_stat/Make_lane/Make_lane.c"
#include "./Perception/Make_Vehicle_stat/Make_Vehicle_Speed/Make_Vehicle_Speed.c"
#include "./Perception/Perception.c"
#include "./Control/Control.c"
#include "./Control/Lateral_Controller/EPS_Torque_Control/SteeringControlBackstepping.c"
#include "./Perception/Path_Command/PathCommand.c"
#include "./Control/Lateral_Controller/Lane_Keeping_Control/LKSControl.c"
#include "./Parameter/NumericalOperator.c"
#endif
