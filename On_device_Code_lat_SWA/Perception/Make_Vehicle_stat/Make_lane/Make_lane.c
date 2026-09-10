#include "../../../Parameter/Type_def.h"
#include "../../Input_Mapping/Input_Mapping.h"
#include "Make_lane.h"

float64_t Logic_C0;
float64_t Logic_C1;
float64_t Logic_C2;
float64_t Logic_C3;
float64_t Logic_CMR_Right_C[4];
float64_t Logic_CMR_Left_C[4];

void Make_lane(void)
{
    Logic_C0 = (Logic_CMR_Left_C[0] + Logic_CMR_Right_C[0]) * 0.5;   /*lane position*/
    Logic_C1 = (Logic_CMR_Left_C[1] + Logic_CMR_Right_C[1]) * 0.5;   /*lane heading angle*/
    Logic_C2 = (Logic_CMR_Left_C[2] + Logic_CMR_Right_C[2]) * 0.5;   /*lane curvature*/
    Logic_C3 = (Logic_CMR_Left_C[3] + Logic_CMR_Right_C[3]) * 0.5;   /*lane curvature derivative*/
}
