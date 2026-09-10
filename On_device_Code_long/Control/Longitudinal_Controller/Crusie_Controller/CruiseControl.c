#include "../../../Parameter/car.par"  
#include "../../../Parameter/Type_def.h"
#include "../../../Parameter/code_var.h"
#include "../../../Parameter/NumericalOperator.h"
#include "../../../Perception/Input_Mapping/Input_Mapping.h"
#include "../Longi_Con.h"
#include "../Transition_Logic/Longi_Transition.h"
#include "CruiseControl.h"
#include "../Adaptive_Crusie_Controller/AdaptiveCruiseControl.h"
#include "../Safety_Logic/Longi_safe_function.h"

/* Reference */
float64_t Logic_V_Ref_vel;
float64_t Logic_ACC_TARG_SPD_SEND;
float64_t Logic_CC_LEVEL_SEND;
float64_t Logic_accel_OUT;
float64_t Logic_CC_PID_out;
float64_t Logic_CC_Smooth_PI_gain[2];

/* PID Controller gains */
float64_t Logic_CC_L1_P = 4.0;
float64_t Logic_CC_L1_I = 0.01;
//float64_t Logic_CC_L1_D = 0;
float64_t Logic_CC_L2_P = 4.0;
float64_t Logic_CC_L2_I = 0.01;
//float64_t Logic_CC_L2_D = 0;
float64_t Logic_CC_L3_P = 4.0;
float64_t Logic_CC_L3_I = 0.01;
//float64_t Logic_CC_L3_D = 0;



/* PID error define */
float64_t Logic_CC_p_error;
float64_t Logic_CC_i_error;
//float64_t Logic_CC_d_error;
float64_t Logic_CC_V_error; 
float64_t Logic_CC_V_ref;
float64_t Logic_CC_Vego;

float64_t Logic_CC_transition_flag;
float64_t Logic_CC_weight_rate;
float64_t Logic_CC_sum_weight;
float64_t Logic_CC_prev_level;
float64_t Logic_CC_target_level;
float64_t Logic_CC_level_init_flag;

static float64_t SelectCCLevel(const float64_t level)
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

static float64_t CalcCCPIOutput(const float64_t p_gain, const float64_t i_gain)
{
    return p_gain*Logic_CC_p_error + i_gain*Logic_CC_i_error;
}
 
void CruiseController(void) {
    float64_t CC_current_level;
    float64_t CC_level_PI_gain[6];

    /* PID Longitudinal Control*/
    Logic_CC_V_ref                = Logic_ACC_TARG_SPD_SEND;
    Logic_CC_Vego                 = Logic_wheel_velocity;

    // Error signal
    Logic_CC_V_error = Logic_CC_V_ref - Logic_CC_Vego;

    Logic_CC_p_error              = Logic_CC_V_error;
    Logic_CC_i_error              +=  SYS_SAMPLETIME * (Logic_CC_V_error);
    //Logic_CC_d_error = (Logic_CC_Vego - Logic_CC_V_prev)/SYS_SAMPLETIME;

    /* Anti-wind-up */
    if (Logic_CC_i_error > Logic_CC_INTEGRAL_MAX) {
        Logic_CC_i_error          = Logic_CC_INTEGRAL_MAX;
    } 
    else if (Logic_CC_i_error < Logic_CC_INTEGRAL_MIN) {
        Logic_CC_i_error          = Logic_CC_INTEGRAL_MIN;
    }

    CC_current_level = SelectCCLevel(Logic_CC_LEVEL_SEND);
    Logic_CC_LEVEL_SEND           = CC_current_level;

    /* Smooth PI gain by CC level */
    CC_level_PI_gain[0]           = Logic_CC_L1_P;
    CC_level_PI_gain[1]           = Logic_CC_L1_I;
    CC_level_PI_gain[2]           = Logic_CC_L2_P;
    CC_level_PI_gain[3]           = Logic_CC_L2_I;
    CC_level_PI_gain[4]           = Logic_CC_L3_P;
    CC_level_PI_gain[5]           = Logic_CC_L3_I;

    LevelTransitionArraySmoothing(CC_current_level,
                                  CC_level_PI_gain,
                                  2,
                                  Logic_CC_Smooth_PI_gain,
                                  &Logic_CC_prev_level,
                                  &Logic_CC_target_level,
                                  &Logic_CC_transition_flag,
                                  &Logic_CC_sum_weight,
                                  &Logic_CC_weight_rate,
                                  &Logic_CC_level_init_flag);

    Logic_CC_PID_out              = CalcCCPIOutput(Logic_CC_Smooth_PI_gain[0], Logic_CC_Smooth_PI_gain[1]);

    /* Store error and measurement for D gain use */
    //Logic_CC_V_prev_Error = Logic_CC_V_error;
    //Logic_CC_V_prev = Logic_CC_Vego;

    Logic_accel_OUT               = Logic_CC_PID_out;
}
