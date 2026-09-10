#include "../../Parameter/Type_def.h"
#include "../../Parameter/car.par"
#include "../../Parameter/code_var.h"
#include "../../Control/Longitudinal_Controller/Crusie_Controller/CruiseControl.h"
#include "../../Control/Longitudinal_Controller/Adaptive_Crusie_Controller/AdaptiveCruiseControl.h"
#include "Input_Mapping.h"

float64_t ECAN_WHL_SpdRR;
float64_t ECAN_WHL_SpdRL;
float64_t ECAN_SCC_ObjRelSpd;
float64_t ECAN_SCC_ObjDst;

float64_t MAB_CC_L1_P;
float64_t MAB_CC_L1_I;
float64_t MAB_CC_L2_P;
float64_t MAB_CC_L2_I;
float64_t MAB_CC_L3_P;
float64_t MAB_CC_L3_I;
float64_t MAB_ACC_L1_Kp;
float64_t MAB_ACC_L1_Kd;
float64_t MAB_ACC_L2_Kp;
float64_t MAB_ACC_L2_Kd;
float64_t MAB_ACC_L3_Kp;
float64_t MAB_ACC_L3_Kd;

float64_t MAB_CC_Level;
float64_t MAB_ACC_Level;
float64_t MAB_ACC_RESP_LEVEL;
float64_t MAB_Longi_CC_TARG_SPD;

void Input_Mapping(void)
{
    /* E-CAN interface -> Logic interface */
    Logic_WHL_SpdRR               = ECAN_WHL_SpdRR;
    Logic_WHL_SpdRL               = ECAN_WHL_SpdRL;
    Logic_SCC_ObjRelSpd           = ECAN_SCC_ObjRelSpd;
    Logic_SCC_ObjDst              = ECAN_SCC_ObjDst;

    /* MAB longitudinal controller gain interface -> Logic interface */
    Logic_CC_L1_P                 = MAB_CC_L1_P;
    Logic_CC_L1_I                 = MAB_CC_L1_I;
    Logic_CC_L2_P                 = MAB_CC_L2_P;
    Logic_CC_L2_I                 = MAB_CC_L2_I;
    Logic_CC_L3_P                 = MAB_CC_L3_P;
    Logic_CC_L3_I                 = MAB_CC_L3_I;
    Logic_ACC_L1_Kp               = MAB_ACC_L1_Kp;
    Logic_ACC_L1_Kd               = MAB_ACC_L1_Kd;
    Logic_ACC_L2_Kp               = MAB_ACC_L2_Kp;
    Logic_ACC_L2_Kd               = MAB_ACC_L2_Kd;
    Logic_ACC_L3_Kp               = MAB_ACC_L3_Kp;
    Logic_ACC_L3_Kd               = MAB_ACC_L3_Kd;

    /* MAB longitudinal command interface -> Logic interface */
    Logic_CC_LEVEL_SEND           = MAB_CC_Level;
    Logic_ACC_LEVEL_SEND          = MAB_ACC_Level;
    Logic_ACC_RESP_LEVEL_SEND     = MAB_ACC_RESP_LEVEL;
    Logic_ACC_TARG_SPD_SEND       = MAB_Longi_CC_TARG_SPD;
}
