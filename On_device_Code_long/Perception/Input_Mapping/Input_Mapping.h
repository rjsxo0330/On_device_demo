#ifndef __INPUT_MAPPING_H__
#define __INPUT_MAPPING_H__

extern float64_t ECAN_WHL_SpdRR;
extern float64_t ECAN_WHL_SpdRL;
extern float64_t ECAN_SCC_ObjRelSpd;
extern float64_t ECAN_SCC_ObjDst;

extern float64_t MAB_CC_L1_P;
extern float64_t MAB_CC_L1_I;
extern float64_t MAB_CC_L2_P;
extern float64_t MAB_CC_L2_I;
extern float64_t MAB_CC_L3_P;
extern float64_t MAB_CC_L3_I;
extern float64_t MAB_ACC_L1_Kp;
extern float64_t MAB_ACC_L1_Kd;
extern float64_t MAB_ACC_L2_Kp;
extern float64_t MAB_ACC_L2_Kd;
extern float64_t MAB_ACC_L3_Kp;
extern float64_t MAB_ACC_L3_Kd;

extern float64_t MAB_CC_Level;
extern float64_t MAB_ACC_Level;
extern float64_t MAB_ACC_RESP_LEVEL;
extern float64_t MAB_Longi_CC_TARG_SPD;

extern float64_t Logic_WHL_SpdRR;
extern float64_t Logic_WHL_SpdRL;
extern float64_t Logic_SCC_ObjRelSpd;
extern float64_t Logic_SCC_ObjDst;

extern float64_t Logic_ACC_TARG_SPD_SEND;
extern float64_t Logic_ACC_RESP_LEVEL_SEND;
extern float64_t Logic_ACC_LEVEL_SEND;
extern float64_t Logic_CC_LEVEL_SEND;

extern float64_t Logic_CC_L1_P;
extern float64_t Logic_CC_L1_I;
extern float64_t Logic_CC_L2_P;
extern float64_t Logic_CC_L2_I;
extern float64_t Logic_CC_L3_P;
extern float64_t Logic_CC_L3_I;
extern float64_t Logic_ACC_L1_Kp;
extern float64_t Logic_ACC_L1_Kd;
extern float64_t Logic_ACC_L2_Kp;
extern float64_t Logic_ACC_L2_Kd;
extern float64_t Logic_ACC_L3_Kp;
extern float64_t Logic_ACC_L3_Kd;

extern float64_t Logic_wheel_velocity;
extern float64_t Logic_accel_OUT;

void Input_Mapping(void);

#endif
