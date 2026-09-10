#include "../../../Parameter/car.par"
#include "../../../Parameter/Type_def.h"
#include "../../../Parameter/code_var.h"
#include "../../../Parameter/NumericalOperator.h"
#include "../../../Perception/Input_Mapping/Input_Mapping.h"
#include "../Longi_Con.h"
#include "Longi_Transition.h"
#include "../Crusie_Controller/CruiseControl.h"
#include "../Adaptive_Crusie_Controller/AdaptiveCruiseControl.h"
#include "../Safety_Logic/Longi_safe_function.h"

float64_t Logic_TR_FLAG         = 0;
float64_t Logic_TR_Range        = 0;
float64_t Logic_TR_Range_Dot    = 0;
float64_t Logic_TR_Range_Final  = 0;
float64_t Logic_TR_EGO_SPD      = 0;
float64_t Logic_TR_Flag_value   = 0;
float64_t Logic_TR_Time_headway = 0;

void Longi_Transition(void)
{
    if (Logic_ACC_LEVEL_SEND <= 1.5)
    {
        Logic_TR_Time_headway = 2.5;
    }
    else if (Logic_ACC_LEVEL_SEND <= 2.5)
    {
        Logic_TR_Time_headway = 3.5;
    }
    else
    {
        Logic_TR_Time_headway = 4.5;
    }

    Logic_TR_EGO_SPD = Logic_wheel_velocity;
    Logic_TR_Range = Logic_SCC_ObjDst - EGO_LENGTH;
    Logic_TR_Range_Dot = Logic_SCC_ObjRelSpd;
    Logic_TR_Range_Final = Logic_TR_Time_headway * Logic_TR_EGO_SPD;

    Logic_TR_Flag_value = Logic_TR_Range_Final - TR_T * Logic_TR_Range_Dot;

    if (Logic_TR_FLAG == 1)
    {
        if (Logic_TR_Range > Logic_TR_Flag_value + TR_HYSTERESIS_RANGE)
        {
            Logic_TR_FLAG = 0;
        }
    }
    else if (Logic_TR_Range <= Logic_TR_Flag_value)
    {
        Logic_TR_FLAG = 1;
    }
}
