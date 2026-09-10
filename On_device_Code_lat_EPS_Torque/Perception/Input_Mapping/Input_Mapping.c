#include "../../Parameter/Type_def.h"
#include "../../Parameter/car.par"
#include "../../Parameter/code_var.h"
#include "../../Control/Lateral_Controller/EPS_Torque_Control/SteeringControlBackstepping.h"
#include "../../Control/Lateral_Controller/Lane_Keeping_Control/LKSControl.h"
#include "../Path_Command/PathCommand.h"
#include "Input_Mapping.h"

float64_t ECAN_IMU_LatAccel;
float64_t ECAN_IMU_YawRt;
float64_t ECAN_SWA_Angle;
float64_t ECAN_SWA_Spd;
float64_t ECAN_WHL_SpdRR;
float64_t ECAN_WHL_SpdRL;
float64_t ECAN_CMR_Right_C[4];
float64_t ECAN_CMR_Left_C[4];

float64_t ScenarioCAN_Driver_ID;
float64_t ScenarioCAN_LKS_ON_OFF_SEND;
float64_t ScenarioCAN_LKS_LEVEL_SEND;

float64_t MAB_LKS_L1_LQR_K[4];
float64_t MAB_LKS_L2_LQR_K[4];
float64_t MAB_LKS_L3_LQR_K[4];
float64_t MAB_LKS_LOOKAHEAD_DISTANCE;

float64_t MAB_Lateral_flag;
float64_t MAB_Lateral_Level;

float64_t Logic_IMU_LatAccel;
float64_t Logic_SWA_Spd;

float64_t Logic_Driver_ID;

void Input_Mapping(void)
{
    /* E-CAN interface -> Logic interface */
    Logic_IMU_LatAccel            = ECAN_IMU_LatAccel;                                                          /* Use: lateral acceleration monitor output. */
    Logic_IMU_YawRt               = ECAN_IMU_YawRt;                                                             /* Use: PathCommand() lateral state x4. */
    Logic_SWA_Angle               = ECAN_SWA_Angle;                                                             /* Use: Logic_SWA_Angle -> SteeringControlBackstepping() observer input. */
    Logic_SWA_Spd                 = ECAN_SWA_Spd;                                                               /* Use: steering speed monitor output. */
    Logic_WHL_SpdRR               = ECAN_WHL_SpdRR;                                                             /* Use: averaged into Logic_wheel_velocity. */
    Logic_WHL_SpdRL               = ECAN_WHL_SpdRL;                                                             /* Use: averaged into Logic_wheel_velocity. */
    Logic_CMR_Right_C[0]          = ECAN_CMR_Right_C[0];                                                        /* Use: averaged into Logic_C0 -> PathCommand(). */
    Logic_CMR_Right_C[1]          = ECAN_CMR_Right_C[1];                                                        /* Use: averaged into Logic_C1 -> PathCommand(). */
    Logic_CMR_Right_C[2]          = ECAN_CMR_Right_C[2];                                                        /* Use: averaged into Logic_C2 -> PathCommand(). */
    Logic_CMR_Right_C[3]          = ECAN_CMR_Right_C[3];                                                        /* Use: averaged into Logic_C3 -> PathCommand(). */
    Logic_CMR_Left_C[0]           = ECAN_CMR_Left_C[0];                                                         /* Use: averaged into Logic_C0 -> PathCommand(). */
    Logic_CMR_Left_C[1]           = ECAN_CMR_Left_C[1];                                                         /* Use: averaged into Logic_C1 -> PathCommand(). */
    Logic_CMR_Left_C[2]           = ECAN_CMR_Left_C[2];                                                         /* Use: averaged into Logic_C2 -> PathCommand(). */
    Logic_CMR_Left_C[3]           = ECAN_CMR_Left_C[3];                                                         /* Use: averaged into Logic_C3 -> PathCommand(). */

    /* Scenario CAN interface -> Logic interface */
    Logic_Driver_ID               = ScenarioCAN_Driver_ID;                                                      /* Use: driver ID monitor output. */
    Logic_LKS_ON_OFF_SEND         = ScenarioCAN_LKS_ON_OFF_SEND;                                                /* Use: LaneKeepingController() and SteeringControlBackstepping() on/off logic. */

    /* MAB LKS gain interface -> Logic interface */
    Logic_LKS_L1_LQR_K[0]         = MAB_LKS_L1_LQR_K[0];                                                        /* Use: Logic_LKS_L1_LQR_K[0] -> LaneKeepingController() level 1 x1 gain. */
    Logic_LKS_L1_LQR_K[1]         = MAB_LKS_L1_LQR_K[1];                                                        /* Use: Logic_LKS_L1_LQR_K[1] -> LaneKeepingController() level 1 x2 gain. */
    Logic_LKS_L1_LQR_K[2]         = MAB_LKS_L1_LQR_K[2];                                                        /* Use: Logic_LKS_L1_LQR_K[2] -> LaneKeepingController() level 1 x3 gain. */
    Logic_LKS_L1_LQR_K[3]         = MAB_LKS_L1_LQR_K[3];                                                        /* Use: Logic_LKS_L1_LQR_K[3] -> LaneKeepingController() level 1 x4 gain. */
    Logic_LKS_L2_LQR_K[0]         = MAB_LKS_L2_LQR_K[0];                                                        /* Use: Logic_LKS_L2_LQR_K[0] -> LaneKeepingController() level 2 x1 gain. */
    Logic_LKS_L2_LQR_K[1]         = MAB_LKS_L2_LQR_K[1];                                                        /* Use: Logic_LKS_L2_LQR_K[1] -> LaneKeepingController() level 2 x2 gain. */
    Logic_LKS_L2_LQR_K[2]         = MAB_LKS_L2_LQR_K[2];                                                        /* Use: Logic_LKS_L2_LQR_K[2] -> LaneKeepingController() level 2 x3 gain. */
    Logic_LKS_L2_LQR_K[3]         = MAB_LKS_L2_LQR_K[3];                                                        /* Use: Logic_LKS_L2_LQR_K[3] -> LaneKeepingController() level 2 x4 gain. */
    Logic_LKS_L3_LQR_K[0]         = MAB_LKS_L3_LQR_K[0];                                                        /* Use: Logic_LKS_L3_LQR_K[0] -> LaneKeepingController() level 3 x1 gain. */
    Logic_LKS_L3_LQR_K[1]         = MAB_LKS_L3_LQR_K[1];                                                        /* Use: Logic_LKS_L3_LQR_K[1] -> LaneKeepingController() level 3 x2 gain. */
    Logic_LKS_L3_LQR_K[2]         = MAB_LKS_L3_LQR_K[2];                                                        /* Use: Logic_LKS_L3_LQR_K[2] -> LaneKeepingController() level 3 x3 gain. */
    Logic_LKS_L3_LQR_K[3]         = MAB_LKS_L3_LQR_K[3];                                                        /* Use: Logic_LKS_L3_LQR_K[3] -> LaneKeepingController() level 3 x4 gain. */
    Logic_LKS_LOOKAHEAD_DISTANCE  = MAB_LKS_LOOKAHEAD_DISTANCE;                                                 /* Use: PathCommand() preview point. */

    /* MAB config interface -> Logic interface */
    Logic_Lateral_flag            = MAB_Lateral_flag;                                                           /* Use: lateral controller on/off gating. */
    Logic_LKS_Level               = MAB_Lateral_Level;                                                          /* Use: LaneKeepingController() level selection. */
}
