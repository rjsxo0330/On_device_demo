#include "../../../Parameter/car.par"
#include "../../../Parameter/Type_def.h"
#include "../../../Parameter/code_var.h"
#include "../../../Parameter/NumericalOperator.h"
#include "../../../Perception/Input_Mapping/Input_Mapping.h"
#include "../Longi_Con.h"
#include "../Transition_Logic/Longi_Transition.h"
#include "../Crusie_Controller/CruiseControl.h"
#include "AdaptiveCruiseControl.h"
#include "../Safety_Logic/Longi_safe_function.h"

/* Reference */
float64_t Logic_ACC_CTG_out;
float64_t Logic_SCC_ObjRelSpd;
float64_t Logic_SCC_ObjDst;
float64_t Logic_ACC_LEVEL_SEND;
float64_t Logic_ACC_EGO_SPD;

/* ACC parameter */
float64_t Logic_ACC_Time_headway;
float64_t Logic_ACC_Kp;
float64_t Logic_ACC_Kd;
float64_t Logic_ACC_Smooth_Time_headway[1];
float64_t Logic_ACC_THW_transition_flag;
float64_t Logic_ACC_THW_weight_rate;
float64_t Logic_ACC_THW_sum_weight;
float64_t Logic_ACC_THW_prev_level;
float64_t Logic_ACC_THW_target_level;
float64_t Logic_ACC_THW_level_init_flag;
float64_t Logic_ACC_L1_Kp = 0.400000;
float64_t Logic_ACC_L1_Kd = 0.032000;
float64_t Logic_ACC_L2_Kp = 0.285714;
float64_t Logic_ACC_L2_Kd = 0.028571;
float64_t Logic_ACC_L3_Kp = 0.222222;
float64_t Logic_ACC_L3_Kd = 0.026667;
float64_t Logic_ACC_Smooth_gain[2];
float64_t Logic_ACC_transition_flag;
float64_t Logic_ACC_weight_rate;
float64_t Logic_ACC_sum_weight;
float64_t Logic_ACC_prev_level;
float64_t Logic_ACC_target_level;
float64_t Logic_ACC_level_init_flag;
float64_t Logic_ACC_Range;
float64_t Logic_ACC_Range_dot;
float64_t Logic_ACC_Spacing_err;

static float64_t SelectACCLevel(const float64_t level)
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

void AdaptiveCruiseControl(void)
{
    float64_t ACC_time_headway_level;
    float64_t ACC_level_time_headway[3];
    float64_t ACC_current_level;
    float64_t ACC_level_gain[6];

    /* Select ACC Time headway from ACC level */
    if (Logic_ACC_LEVEL_SEND <= 1.5)
    {
        Logic_ACC_Time_headway     = 2.5;
    }
    else if (Logic_ACC_LEVEL_SEND <= 2.5)
    {
        Logic_ACC_Time_headway     = 3.5;
    }
    else
    {
        Logic_ACC_Time_headway     = 4.5;
    }

    ACC_time_headway_level         = SelectACCLevel(Logic_ACC_LEVEL_SEND);
    ACC_level_time_headway[0]      = 2.5;
    ACC_level_time_headway[1]      = 3.5;
    ACC_level_time_headway[2]      = 4.5;

    LevelTransitionArraySmoothing(ACC_time_headway_level,
                                  ACC_level_time_headway,
                                  1,
                                  Logic_ACC_Smooth_Time_headway,
                                  &Logic_ACC_THW_prev_level,
                                  &Logic_ACC_THW_target_level,
                                  &Logic_ACC_THW_transition_flag,
                                  &Logic_ACC_THW_sum_weight,
                                  &Logic_ACC_THW_weight_rate,
                                  &Logic_ACC_THW_level_init_flag);

    Logic_ACC_Time_headway         = Logic_ACC_Smooth_Time_headway[0];

    /*Define ACC variable*/
    Logic_ACC_EGO_SPD              = Logic_wheel_velocity;
    Logic_ACC_Range                = Logic_SCC_ObjDst - EGO_LENGTH;
    Logic_ACC_Range_dot            = Logic_SCC_ObjRelSpd;

    /*CTG policy code*/
    Logic_ACC_Spacing_err          = Logic_ACC_Range - Logic_ACC_Time_headway * Logic_ACC_EGO_SPD;

    ACC_current_level              = SelectACCLevel(Logic_ACC_RESP_LEVEL_SEND);
    Logic_ACC_RESP_LEVEL_SEND      = ACC_current_level;

    ACC_level_gain[0]              = Logic_ACC_L1_Kp;
    ACC_level_gain[1]              = Logic_ACC_L1_Kd;
    ACC_level_gain[2]              = Logic_ACC_L2_Kp;
    ACC_level_gain[3]              = Logic_ACC_L2_Kd;
    ACC_level_gain[4]              = Logic_ACC_L3_Kp;
    ACC_level_gain[5]              = Logic_ACC_L3_Kd;

    LevelTransitionArraySmoothing(ACC_current_level,
                                  ACC_level_gain,
                                  2,
                                  Logic_ACC_Smooth_gain,
                                  &Logic_ACC_prev_level,
                                  &Logic_ACC_target_level,
                                  &Logic_ACC_transition_flag,
                                  &Logic_ACC_sum_weight,
                                  &Logic_ACC_weight_rate,
                                  &Logic_ACC_level_init_flag);

    Logic_ACC_Kp                   = Logic_ACC_Smooth_gain[0];
    Logic_ACC_Kd                   = Logic_ACC_Smooth_gain[1];
    Logic_ACC_CTG_out              = (Logic_ACC_Kp*Logic_ACC_Range_dot + Logic_ACC_Kd * Logic_ACC_Spacing_err);

    Logic_accel_OUT                = Logic_ACC_CTG_out;
}
