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
#include "./Control/Longitudinal_Controller/Longi_Con.h"
#include "./Control/Longitudinal_Controller/Transition_Logic/Longi_Transition.h"
#include "./Control/Longitudinal_Controller/Crusie_Controller/CruiseControl.h"
#include "./Control/Longitudinal_Controller/Adaptive_Crusie_Controller/AdaptiveCruiseControl.h"
#include "./Control/Longitudinal_Controller/Safety_Logic/Longi_safe_function.h"

#define NUM_PARAMS        0

#define NumberInputPort   3

#define WidthInputPort0   4
#define WidthInputPort1   12
#define WidthInputPort2   4

#define NumberOutputPort  1

#define WidthOutputPort0  2

void Initial_Reset(void);

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, NumberInputPort)) return;

    ssSetInputPortWidth(S, 0, WidthInputPort0);
    ssSetInputPortRequiredContiguous(S, 0, true);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    ssSetInputPortWidth(S, 1, WidthInputPort1);
    ssSetInputPortRequiredContiguous(S, 1, true);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    ssSetInputPortWidth(S, 2, WidthInputPort2);
    ssSetInputPortRequiredContiguous(S, 2, true);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

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

#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
    Logic_accel_OUT             = 0.0;
    Logic_CC_PID_out            = 0.0;
    Logic_CC_p_error            = 0.0;
    Logic_CC_i_error            = 0.0;
    Logic_CC_V_error            = 0.0;
    Logic_CC_LEVEL_SEND         = 0.0;
    Logic_CC_Smooth_PI_gain[0]  = 0.0;
    Logic_CC_Smooth_PI_gain[1]  = 0.0;
    Logic_CC_transition_flag    = 0.0;
    Logic_CC_sum_weight         = 0.0;
    Logic_CC_prev_level         = 0.0;
    Logic_CC_target_level       = 0.0;
    Logic_CC_level_init_flag    = 0.0;
    Logic_ACC_Smooth_gain[0]    = 0.0;
    Logic_ACC_Smooth_gain[1]    = 0.0;
    Logic_ACC_Smooth_Time_headway[0] = 0.0;
    Logic_ACC_THW_transition_flag = 0.0;
    Logic_ACC_THW_weight_rate = 0.0;
    Logic_ACC_THW_sum_weight    = 0.0;
    Logic_ACC_THW_prev_level    = 0.0;
    Logic_ACC_THW_target_level  = 0.0;
    Logic_ACC_THW_level_init_flag = 0.0;
    Logic_ACC_transition_flag   = 0.0;
    Logic_ACC_sum_weight        = 0.0;
    Logic_ACC_prev_level        = 0.0;
    Logic_ACC_target_level      = 0.0;
    Logic_ACC_level_init_flag   = 0.0;
    Logic_TR_FLAG               = 0.0;
    Logic_Longi_current_mode    = 0.0;
    Logic_Longi_target_mode     = 0.0;
    Logic_Longi_transition_flag = 0.0;
    Logic_Longi_sum_weight      = 0.0;
    Logic_Longi_weight_rate     = 0.0;
    Logic_Longi_start_accel     = 0.0;
    Logic_Longi_Smooth_accel    = 0.0;
    Logic_Longi_mode_init_flag  = 0.0;
}
#endif

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    Initial_Reset();
}
#endif

void Initial_Reset(void)
{
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *u0 = (real_T*) ssGetInputPortSignal(S, 0);
    real_T *u1 = (real_T*) ssGetInputPortSignal(S, 1);
    real_T *u2 = (real_T*) ssGetInputPortSignal(S, 2);

    real_T *y0 = ssGetOutputPortSignal(S, 0);

    /* E-CAN longitudinal sensor input */
    ECAN_WHL_SpdRR                  = (float64_t)(u0[0]) / 3.6;
    ECAN_WHL_SpdRL                  = (float64_t)(u0[1]) / 3.6;
    ECAN_SCC_ObjRelSpd              = (float64_t)(u0[2]) / 3.6;
    ECAN_SCC_ObjDst                 = (float64_t)(u0[3]);

    /* CC PI controller gain */
    MAB_CC_L1_P                     = (float64_t)(u1[0]);
    MAB_CC_L1_I                     = (float64_t)(u1[1]);
    MAB_CC_L2_P                     = (float64_t)(u1[2]);
    MAB_CC_L2_I                     = (float64_t)(u1[3]);
    MAB_CC_L3_P                     = (float64_t)(u1[4]);
    MAB_CC_L3_I                     = (float64_t)(u1[5]);
    MAB_ACC_L1_Kp                   = (float64_t)(u1[6]);
    MAB_ACC_L1_Kd                   = (float64_t)(u1[7]);
    MAB_ACC_L2_Kp                   = (float64_t)(u1[8]);
    MAB_ACC_L2_Kd                   = (float64_t)(u1[9]);
    MAB_ACC_L3_Kp                   = (float64_t)(u1[10]);
    MAB_ACC_L3_Kd                   = (float64_t)(u1[11]);

    /* Longitudinal controller command */
    MAB_CC_Level                    = (float64_t)(u2[0]);
    MAB_ACC_Level                   = (float64_t)(u2[1]);
    MAB_ACC_RESP_LEVEL              = (float64_t)(u2[2]);
    MAB_Longi_CC_TARG_SPD           = (float64_t)(u2[3]) / 3.6;


    Perception();
    Control();

    /* Gateway CAN Tx - longitudinal only */
    y0[0]                           = Logic_accel_OUT;
    y0[1]                           = Logic_TR_FLAG;
}

#undef MDL_UPDATE
#if defined(MDL_UPDATE)
static void mdlUpdate(SimStruct *S, int_T tid)
{
}
#endif

#undef MDL_DERIVATIVES
#if defined(MDL_DERIVATIVES)
static void mdlDerivatives(SimStruct *S)
{
}
#endif

static void mdlTerminate(SimStruct *S)
{
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#include "./Perception/Input_Mapping/Input_Mapping.c"
#include "./Perception/Make_Vehicle_stat/Make_Vehicle_Speed/Make_Vehicle_Speed.c"
#include "./Perception/Perception.c"
#include "./Control/Control.c"
#include "./Control/Longitudinal_Controller/Longi_Con.c"
#include "./Control/Longitudinal_Controller/Transition_Logic/Longi_Transition.c"
#include "./Control/Longitudinal_Controller/Crusie_Controller/CruiseControl.c"
#include "./Control/Longitudinal_Controller/Adaptive_Crusie_Controller/AdaptiveCruiseControl.c"
#include "./Control/Longitudinal_Controller/Safety_Logic/Longi_safe_function.c"
#include "./Parameter/NumericalOperator.c"
#else
#include "cg_sfun.h"
#include "./Perception/Input_Mapping/Input_Mapping.c"
#include "./Perception/Make_Vehicle_stat/Make_Vehicle_Speed/Make_Vehicle_Speed.c"
#include "./Perception/Perception.c"
#include "./Control/Control.c"
#include "./Control/Longitudinal_Controller/Longi_Con.c"
#include "./Control/Longitudinal_Controller/Transition_Logic/Longi_Transition.c"
#include "./Control/Longitudinal_Controller/Crusie_Controller/CruiseControl.c"
#include "./Control/Longitudinal_Controller/Adaptive_Crusie_Controller/AdaptiveCruiseControl.c"
#include "./Control/Longitudinal_Controller/Safety_Logic/Longi_safe_function.c"
#include "./Parameter/NumericalOperator.c"
#endif
