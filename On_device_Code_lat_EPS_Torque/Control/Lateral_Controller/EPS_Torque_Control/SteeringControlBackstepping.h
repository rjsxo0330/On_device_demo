#ifndef __STEERINGCONTROLBACKSTEPPING_H__
#define __STEERINGCONTROLBACKSTEPPING_H__

extern float64_t Logic_EPS_Obs_xh[5];
extern float64_t Logic_EPS_Obs_L[5];
extern float64_t Logic_EPS_Obs_g0;

extern float64_t Logic_EPS_x_des[5];
extern float64_t Logic_EPS_x_d_old[4];
extern float64_t Logic_EPS_dx_d[4];
extern float64_t Logic_EPS_dx_d_old[4];
extern float64_t Logic_EPS_error_x[4];
extern float64_t Logic_EPS_BSP_K_back[4];
extern float64_t Logic_EPS_BSP_K_dam;
extern float64_t Logic_EPS_BSP_K_dam1;
extern float64_t Logic_EPS_BSP_v;
extern float64_t Logic_EPS_BSP_v1;
extern float64_t Logic_EPS_Torque_Out;
extern float64_t Lateral_Cont_Torque;
extern float64_t Logic_EPS_Prev_u;
extern float64_t Logic_EPS_u_rate;

extern float64_t Logic_ref_SWA;

extern void SteeringControlBackstepping(void);
extern void GetObserverState(float64_t *y,float64_t *u);
extern void Observer(float64_t* dxh, float64_t* xhat,float64_t *input,float64_t *disturbance);
extern float64_t BackstepControl(const float64_t* Logic_ref_SWA, const float64_t* xhat);

#endif
