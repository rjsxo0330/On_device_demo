#ifndef __LKSCONTROL_H__
#define __LKSCONTROL_H__


extern float64_t Logic_ref_SWA;

extern float64_t Logic_Lateral_x[4];
extern float64_t Logic_LKS_L1_LQR_K[4];
extern float64_t Logic_LKS_L2_LQR_K[4];
extern float64_t Logic_LKS_L3_LQR_K[4];

extern float64_t Logic_LKS_Level;
extern float64_t Logic_LKS_ON_OFF_SEND;
extern float64_t Logic_LKS_transition_flag;
extern float64_t Logic_LKS_weight_rate;
extern float64_t Logic_LKS_sum_weight;
extern float64_t Logic_LKS_prev_level;
extern float64_t Logic_LKS_target_level;
extern float64_t LKS_level_init_flag;

extern float64_t Logic_LKS_Smooth_LQR_K[4];

extern void LaneKeepingController(void);

#endif
