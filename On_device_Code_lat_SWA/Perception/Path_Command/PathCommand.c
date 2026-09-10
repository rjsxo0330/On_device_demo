/* Includes ********************************************************/
#include "../../Parameter/car.par"  
#include "../../Parameter/Type_def.h"
#include "../../Parameter/code_var.h"
#include "../../Parameter/NumericalOperator.h"
#include "PathCommand.h"
#include "../Input_Mapping/Input_Mapping.h"
#include "../Make_Vehicle_stat/Make_lane/Make_lane.h"
#include "../../Control/Lateral_Controller/Lane_Keeping_Control/LKSControl.h"

//float64_t Logic_slipangle;
float64_t Logic_IMU_YawRt;
float64_t Logic_LKS_LOOKAHEAD_DISTANCE;

void PathCommand(void)
{   
    Logic_Lateral_x[0] = (Logic_C0) + (Logic_C1*Logic_LKS_LOOKAHEAD_DISTANCE)+(Logic_C2*Logic_LKS_LOOKAHEAD_DISTANCE*Logic_LKS_LOOKAHEAD_DISTANCE)+(Logic_C3*Logic_LKS_LOOKAHEAD_DISTANCE*Logic_LKS_LOOKAHEAD_DISTANCE*Logic_LKS_LOOKAHEAD_DISTANCE);
    Logic_Lateral_x[1] = (/*Logic_slipangle + */Logic_C1)*Logic_wheel_velocity;
    Logic_Lateral_x[2] = Logic_C1;
    Logic_Lateral_x[3] = Logic_IMU_YawRt;
}
