/* Includes ********************************************************/
#include "../../../Parameter/car.par"
#include "../../../Parameter/Type_def.h"
#include "../../../Parameter/code_var.h"
#include "../../../Parameter/NumericalOperator.h"
#include "SteeringControlBackstepping.h"
#include "../Lane_Keeping_Control/LKSControl.h"
#include "../../../Perception/Path_Command/PathCommand.h"

float64_t Logic_SWA_Angle;
float64_t Logic_EPS_Obs_L[5];
float64_t Logic_EPS_Obs_g0 = 22122.0;
float64_t Logic_EPS_BSP_K_back[4];
float64_t Logic_EPS_BSP_K_dam;
float64_t Logic_EPS_BSP_K_dam1;
float64_t Logic_EPS_BSP_v;
float64_t Logic_EPS_BSP_v1;

float64_t Logic_EPS_Torque_In=0;

float64_t Logic_EPS_Obs_xh[5];

float64_t Logic_EPS_x_des[5];
float64_t Logic_EPS_x_d_old[4];
float64_t Logic_EPS_dx_d[4];
float64_t Logic_EPS_dx_d_old[4];
float64_t Logic_EPS_error_x[4];
float64_t Logic_EPS_Torque_Out;
float64_t Lateral_Cont_Torque;
float64_t Logic_EPS_Prev_u = 0;
float64_t Logic_EPS_u_rate = 0;

void SteeringControlBackstepping(void)
{
    if (Logic_LKS_ON_OFF_SEND <= 0.5)
    {
        Logic_EPS_Torque_Out = 0.0;
        Lateral_Cont_Torque = 0.0;
        Logic_EPS_Prev_u = 0.0;
        Logic_EPS_u_rate = 0.0;
        return;
    }

    GetObserverState(&Logic_SWA_Angle, &Logic_EPS_Torque_In);  // Update observer state xh.
    Logic_EPS_Torque_Out = BackstepControl(&Logic_ref_SWA,Logic_EPS_Obs_xh);// + 0.06/0.234;  // offset

    Logic_EPS_u_rate = Logic_EPS_Torque_Out - Logic_EPS_Prev_u;
    if(Logic_EPS_u_rate > 0.26)
    {Logic_EPS_Torque_Out = Logic_EPS_Prev_u + 0.26;}
    else if(Logic_EPS_u_rate < - 0.26)
    {Logic_EPS_Torque_Out = Logic_EPS_Prev_u - 0.26;}
    else
    {Logic_EPS_Torque_Out = Logic_EPS_Torque_Out;}


    if(Logic_EPS_Torque_Out > 20)
    {Logic_EPS_Torque_Out = 20;}
    else if(Logic_EPS_Torque_Out < -20)
    {Logic_EPS_Torque_Out = -20;}
    else
    {Logic_EPS_Torque_Out = Logic_EPS_Torque_Out;}

    Logic_EPS_Prev_u = Logic_EPS_Torque_Out;

    Logic_EPS_Torque_Out = 0.234*Logic_EPS_Torque_Out;
    Lateral_Cont_Torque = Logic_EPS_Torque_Out;
}

void GetObserverState(float64_t *y,float64_t *u)
{
    float64_t in[5];
    in[0]=y[0];	/* measure steering wheel angle */
    in[1]=u[0];	/* steering assist torque */

    ode4(&Observer,SYS_SAMPLETIME, Logic_EPS_Obs_xh, in, 0, 5);

}

void Observer(float64_t* dxh, float64_t* xhat, float64_t *input, float64_t *disturbance)
{
    float64_t err;

	err = input[0] - xhat[0];

    // Observer dynamics
    dxh[0] = xhat[1] + Logic_EPS_Obs_L[0]*err;
	dxh[1] = xhat[2] + Logic_EPS_Obs_L[1]*err;
	dxh[2] = xhat[3] + Logic_EPS_Obs_L[2]*err;
	dxh[3] = xhat[4] + Logic_EPS_Obs_L[3]*err + Logic_EPS_Obs_g0*input[1];
	dxh[4] = Logic_EPS_Obs_L[4]*err;
}

float64_t BackstepControl(const float64_t* Logic_ref_SWA, const float64_t* xhat)
{
    int i;
    float64_t pos_err;

    pos_err = Logic_SWA_Angle - Logic_ref_SWA[0];
    Logic_EPS_x_des[0] = Logic_ref_SWA[0];

    for(i=0;i<4;i++)
    {
        Euler_diff_lowpass(&Logic_EPS_dx_d[i],&Logic_EPS_dx_d_old[i],&Logic_EPS_x_des[i],&Logic_EPS_x_d_old[i],4);

        Logic_EPS_error_x[i] = xhat[i] - Logic_EPS_x_des[i];

        if(i < 3)
        {
            Logic_EPS_x_des[i+1] = -Logic_EPS_BSP_K_back[i]*Logic_EPS_error_x[i] + Logic_EPS_dx_d[i];
        }

    }

    Logic_EPS_x_d_old[0] = Logic_EPS_x_des[0];
    Logic_EPS_x_d_old[1] = Logic_EPS_x_des[1];
    Logic_EPS_x_d_old[2] = Logic_EPS_x_des[2];
    Logic_EPS_x_d_old[3] = Logic_EPS_x_des[3];

    Logic_EPS_dx_d_old[0] = Logic_EPS_dx_d[0];
    Logic_EPS_dx_d_old[1] = Logic_EPS_dx_d[1];
    Logic_EPS_dx_d_old[2] = Logic_EPS_dx_d[2];
    Logic_EPS_dx_d_old[3] = Logic_EPS_dx_d[3];

    return (-Logic_EPS_BSP_K_back[3]*Logic_EPS_error_x[3] + Logic_EPS_dx_d[3] - xhat[4] - Logic_EPS_BSP_K_dam*Logic_EPS_error_x[3]*sqrt(xhat[4]*xhat[4] + Logic_EPS_BSP_v) - Logic_EPS_BSP_K_dam1*Logic_EPS_error_x[3]*sqrt(pos_err*pos_err + Logic_EPS_BSP_v1))/Logic_EPS_Obs_g0;
}
