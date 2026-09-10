#ifndef _ADAPTIVECRUISECONTROLLER_H__
#define _ADAPTIVECRUISECONTROLLER_H__

// Definition for ACC update
extern float64_t Logic_ACC_Time_headway;
extern float64_t Logic_ACC_Kp;
extern float64_t Logic_ACC_Kd;
extern float64_t Logic_ACC_Smooth_Time_headway[1];
extern float64_t Logic_ACC_THW_transition_flag;
extern float64_t Logic_ACC_THW_weight_rate;
extern float64_t Logic_ACC_THW_sum_weight;
extern float64_t Logic_ACC_THW_prev_level;
extern float64_t Logic_ACC_THW_target_level;
extern float64_t Logic_ACC_THW_level_init_flag;
extern float64_t Logic_ACC_L1_Kp;
extern float64_t Logic_ACC_L1_Kd;
extern float64_t Logic_ACC_L2_Kp;
extern float64_t Logic_ACC_L2_Kd;
extern float64_t Logic_ACC_L3_Kp;
extern float64_t Logic_ACC_L3_Kd;
extern float64_t Logic_ACC_Smooth_gain[2];
extern float64_t Logic_ACC_transition_flag;
extern float64_t Logic_ACC_weight_rate;
extern float64_t Logic_ACC_sum_weight;
extern float64_t Logic_ACC_prev_level;
extern float64_t Logic_ACC_target_level;
extern float64_t Logic_ACC_level_init_flag;
extern float64_t Logic_ACC_Range;
extern float64_t Logic_ACC_Range_dot;
extern float64_t Logic_ACC_Spacing_err;
extern float64_t Logic_ACC_EGO_SPD;

//extern void Initialize(void);
extern void AdaptiveCruiseControl(void);

#endif
