#include "../../Parameter/car.par"  
#include "../../Parameter/Type_def.h"
#include "../../Parameter/code_var.h"
#include "../../Parameter/NumericalOperator.h"
#include "Longi_Con.h"
#include "Transition_Logic/Longi_Transition.h"
#include "Crusie_Controller/CruiseControl.h"
#include "Adaptive_Crusie_Controller/AdaptiveCruiseControl.h"
#include "Safety_Logic/Longi_safe_function.h"

float64_t Logic_ACC_RESP_LEVEL_SEND;
float64_t Logic_Longi_current_mode;
float64_t Logic_Longi_target_mode;
float64_t Logic_Longi_transition_flag;
float64_t Logic_Longi_sum_weight;
float64_t Logic_Longi_weight_rate;
float64_t Logic_Longi_start_accel;
float64_t Logic_Longi_Smooth_accel;
float64_t Logic_Longi_mode_init_flag;

static void Longi_Mode_Transition_Smoothing(void)
{
    if (Logic_Longi_weight_rate <= 0.0)
    {
        Logic_Longi_weight_rate = SYS_SAMPLETIME / LONGI_MODE_SMOOTH_TIME;
    }

    if (Logic_Longi_mode_init_flag < 0.5)
    {
        Logic_Longi_target_mode = Logic_Longi_current_mode;
        Logic_Longi_Smooth_accel = Logic_accel_OUT;
        Logic_Longi_transition_flag = 0.0;
        Logic_Longi_sum_weight = 0.0;
        Logic_Longi_mode_init_flag = 1.0;
        return;
    }

    if (Logic_Longi_current_mode != Logic_Longi_target_mode)
    {
        Logic_Longi_target_mode = Logic_Longi_current_mode;
        Logic_Longi_start_accel = Logic_Longi_Smooth_accel;
        Logic_Longi_sum_weight = 0.0;
        Logic_Longi_transition_flag = 1.0;
    }

    if (Logic_Longi_transition_flag > 0.5)
    {
        Logic_Longi_sum_weight += Logic_Longi_weight_rate;

        if (Logic_Longi_sum_weight >= 1.0)
        {
            Logic_Longi_sum_weight = 1.0;
            Logic_Longi_transition_flag = 0.0;
        }

        Logic_Longi_Smooth_accel = (1.0 - Logic_Longi_sum_weight) * Logic_Longi_start_accel
                                 + Logic_Longi_sum_weight * Logic_accel_OUT;
        Logic_accel_OUT = Logic_Longi_Smooth_accel;
    }
    else
    {
        Logic_Longi_Smooth_accel = Logic_accel_OUT;
    }
}

void ALL_LONGI_CON(void) {
    Longi_Transition();
    Logic_Longi_current_mode = Logic_TR_FLAG;

    if (Logic_TR_FLAG == 1) {
        AdaptiveCruiseControl();
        Logic_CC_i_error            = 0.0;
    } 
    else {
        CruiseController();
    };

    Longi_Mode_Transition_Smoothing();
    Longi_Safty_Function();

}
