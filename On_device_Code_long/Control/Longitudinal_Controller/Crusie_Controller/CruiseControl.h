#ifndef _CRUISECONTROLLER_H__
#define _CRUISECONTROLLER_H__

extern float64_t Logic_V_Ref_vel;
extern float64_t Logic_CC_LEVEL_SEND;
extern float64_t Logic_CC_PID_out;
extern float64_t Logic_CC_Smooth_PI_gain[2];
extern float64_t Logic_accel_OUT;
extern float64_t Logic_CC_L1_P;
extern float64_t Logic_CC_L1_I;
//extern float64_t CC_L1_D;
extern float64_t Logic_CC_L2_P;
extern float64_t Logic_CC_L2_I;
//extern float64_t CC_L2_D;
extern float64_t Logic_CC_L3_P;
extern float64_t Logic_CC_L3_I;
//extern float64_t CC_L3_D;

extern float64_t Logic_CC_p_error;
extern float64_t Logic_CC_i_error;
//extern float64_t Logic_CC_d_error;
extern float64_t Logic_CC_V_error;
extern float64_t Logic_CC_V_ref;
extern float64_t Logic_CC_Vego;  

extern float64_t Logic_CC_transition_flag;
extern float64_t Logic_CC_weight_rate;
extern float64_t Logic_CC_sum_weight;
extern float64_t Logic_CC_prev_level;
extern float64_t Logic_CC_target_level;
extern float64_t Logic_CC_level_init_flag;

//extern void Initialize(void);
extern void CruiseController(void);

#endif
