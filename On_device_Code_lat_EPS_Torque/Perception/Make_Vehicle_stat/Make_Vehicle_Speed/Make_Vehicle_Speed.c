#include "../../../Parameter/Type_def.h"
#include "../../Input_Mapping/Input_Mapping.h"
#include "Make_Vehicle_Speed.h"

float64_t Logic_WHL_SpdRR;
float64_t Logic_WHL_SpdRL;
float64_t Logic_wheel_velocity;

void Make_Vehicle_Speed(void)
{
    Logic_wheel_velocity = (Logic_WHL_SpdRR + Logic_WHL_SpdRL) * 0.5;
}
