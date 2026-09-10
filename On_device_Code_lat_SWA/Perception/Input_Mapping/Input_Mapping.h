#ifndef __INPUT_MAPPING_H__
#define __INPUT_MAPPING_H__

extern float64_t ECAN_IMU_LatAccel;
extern float64_t ECAN_IMU_YawRt;
extern float64_t ECAN_SWA_Angle;
extern float64_t ECAN_SWA_Spd;
extern float64_t ECAN_WHL_SpdRR;
extern float64_t ECAN_WHL_SpdRL;
extern float64_t ECAN_CMR_Right_C[4];
extern float64_t ECAN_CMR_Left_C[4];

extern float64_t ScenarioCAN_Driver_ID;
extern float64_t ScenarioCAN_LKS_ON_OFF_SEND;
extern float64_t ScenarioCAN_LKS_LEVEL_SEND;

extern float64_t MAB_LKS_L1_LQR_K[4];
extern float64_t MAB_LKS_L2_LQR_K[4];
extern float64_t MAB_LKS_L3_LQR_K[4];
extern float64_t MAB_LKS_LOOKAHEAD_DISTANCE;

extern float64_t MAB_Lateral_flag;
extern float64_t MAB_Lateral_Level;

extern float64_t Logic_IMU_LatAccel;
extern float64_t Logic_IMU_YawRt;
extern float64_t Logic_SWA_Angle;
extern float64_t Logic_SWA_Spd;
extern float64_t Logic_WHL_SpdRR;
extern float64_t Logic_WHL_SpdRL;
extern float64_t Logic_CMR_Right_C[4];
extern float64_t Logic_CMR_Left_C[4];

extern float64_t Logic_Driver_ID;
extern float64_t Logic_LKS_ON_OFF_SEND;
extern float64_t Logic_LKS_Level;

extern float64_t Logic_LKS_L1_LQR_K[4];
extern float64_t Logic_LKS_L2_LQR_K[4];
extern float64_t Logic_LKS_L3_LQR_K[4];
extern float64_t Logic_LKS_LOOKAHEAD_DISTANCE;

extern float64_t Logic_Lateral_flag;
extern float64_t Logic_wheel_velocity;

void Input_Mapping(void);

#endif
