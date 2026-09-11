#include "../Parameter/Type_def.h"
#include "../Perception/Input_Mapping/Input_Mapping.h"
#include "Lateral_Controller/Lane_Keeping_Control/LKSControl.h"
#include "Control.h"

float64_t Logic_Lateral_Enable;

void Control_Gating(void)
{
    Logic_Lateral_Enable = 1.0;
}

void Control(void)
{
    Control_Gating();

    if (Logic_Lateral_Enable > 0.5)
    {
        LaneKeepingController();
    }
    else
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
    }

}
