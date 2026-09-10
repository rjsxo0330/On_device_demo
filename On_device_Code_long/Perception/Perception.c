#include "Perception.h"
#include "../Parameter/Type_def.h"
#include "Input_Mapping/Input_Mapping.h"
#include "Make_Vehicle_stat/Make_Vehicle_Speed/Make_Vehicle_Speed.h"

void Perception(void)
{
    Input_Mapping();
    Make_Vehicle_Speed();
}
