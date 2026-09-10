#include "../../../Parameter/car.par"
#include "../../../Parameter/Type_def.h"
#include "../EPS_Torque_Control/SteeringControlBackstepping.h"
#include "../../../Parameter/code_var.h"
#include "../../../Parameter/NumericalOperator.h"
#include "../../../Perception/Path_Command/PathCommand.h"
#include "LKSControl.h"

float64_t Logic_ref_SWA;

float64_t Logic_Lateral_x[4];                            // Lateral state 0 ~ 3
float64_t Logic_LKS_ON_OFF_SEND;
float64_t Logic_LKS_Level;
float64_t Logic_LKS_L1_LQR_K[4];
float64_t Logic_LKS_L2_LQR_K[4];
float64_t Logic_LKS_L3_LQR_K[4];

float64_t Logic_LKS_transition_flag;                           // LKS level transition active flag
float64_t Logic_LKS_weight_rate;                               // LKS level transition weight rate
float64_t Logic_LKS_sum_weight;                                // LKS level transition weight
float64_t Logic_LKS_prev_level;                                // Previous LKS level
float64_t Logic_LKS_target_level;                              // Target LKS level
float64_t LKS_level_init_flag;                           // LKS level transition state initialization flag

float64_t Logic_LKS_Smooth_LQR_K[4];                     // Smoothed LKS LQR gain

static float64_t SelectLKSLevel(const float64_t level)
{
    if (level <= 1.5)
    {
        return 1.0;
    }
    else if (level <= 2.5)
    {
        return 2.0;
    }
    else
    {
        return 3.0;
    }
}

static float64_t CalcLKSSWAOutput(const float64_t lqr_gain[4])
{
    return -(lqr_gain[0]*Logic_Lateral_x[0] +
             lqr_gain[1]*Logic_Lateral_x[1] +
             lqr_gain[2]*Logic_Lateral_x[2] +
             lqr_gain[3]*Logic_Lateral_x[3])*STEER_N;
}

void LaneKeepingController(void)
{
    float64_t LKS_current_level;
    float64_t LKS_level_LQR_K[12];

    // Reset LKS state when the LKS command is off.
    if (Logic_LKS_ON_OFF_SEND <= 0.5)
    {
        Logic_ref_SWA = 0.0;
        Logic_LKS_transition_flag = 0.0;
        Logic_LKS_sum_weight = 0.0;
        Logic_LKS_prev_level = 0.0;
        Logic_LKS_target_level = 0.0;
        Logic_LKS_Smooth_LQR_K[0] = 0.0;
        Logic_LKS_Smooth_LQR_K[1] = 0.0;
        Logic_LKS_Smooth_LQR_K[2] = 0.0;
        Logic_LKS_Smooth_LQR_K[3] = 0.0;
        LKS_level_init_flag = 0.0;
        return;
    }

    LKS_current_level = SelectLKSLevel(Logic_LKS_Level);
    Logic_LKS_Level = LKS_current_level;

    /* Smooth LQR gains by LKS level. */
    LKS_level_LQR_K[0] = Logic_LKS_L1_LQR_K[0];
    LKS_level_LQR_K[1] = Logic_LKS_L1_LQR_K[1];
    LKS_level_LQR_K[2] = Logic_LKS_L1_LQR_K[2];
    LKS_level_LQR_K[3] = Logic_LKS_L1_LQR_K[3];
    LKS_level_LQR_K[4] = Logic_LKS_L2_LQR_K[0];
    LKS_level_LQR_K[5] = Logic_LKS_L2_LQR_K[1];
    LKS_level_LQR_K[6] = Logic_LKS_L2_LQR_K[2];
    LKS_level_LQR_K[7] = Logic_LKS_L2_LQR_K[3];
    LKS_level_LQR_K[8] = Logic_LKS_L3_LQR_K[0];
    LKS_level_LQR_K[9] = Logic_LKS_L3_LQR_K[1];
    LKS_level_LQR_K[10] = Logic_LKS_L3_LQR_K[2];
    LKS_level_LQR_K[11] = Logic_LKS_L3_LQR_K[3];

    LevelTransitionArraySmoothing(LKS_current_level,
                                  LKS_level_LQR_K,
                                  4,
                                  Logic_LKS_Smooth_LQR_K,
                                  &Logic_LKS_prev_level,
                                  &Logic_LKS_target_level,
                                  &Logic_LKS_transition_flag,
                                  &Logic_LKS_sum_weight,
                                  &Logic_LKS_weight_rate,
                                  &LKS_level_init_flag);

    Logic_ref_SWA = CalcLKSSWAOutput(Logic_LKS_Smooth_LQR_K) * PI / 180.0;
}
