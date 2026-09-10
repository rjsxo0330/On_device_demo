#include "../../../Parameter/car.par"  
#include "../../../Parameter/Type_def.h"
#include "../../../Parameter/code_var.h"
#include "../../../Parameter/NumericalOperator.h"
#include "../Longi_Con.h"
#include "../Transition_Logic/Longi_Transition.h"
#include "../Crusie_Controller/CruiseControl.h"
#include "../Adaptive_Crusie_Controller/AdaptiveCruiseControl.h"
#include "Longi_safe_function.h"
 
void Longi_Safty_Function(void) {
    
    if (Logic_accel_OUT > ACCEL_LIM_MAX) {
        Logic_accel_OUT = ACCEL_LIM_MAX;
    } 
    else if (Logic_accel_OUT < ACCEL_LIM_MIN) {
        Logic_accel_OUT = ACCEL_LIM_MIN;
    }
    else{
        Logic_accel_OUT = Logic_accel_OUT;
    }
}
