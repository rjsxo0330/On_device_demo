#include <math.h>
#include <float.h>
#include "Type_def.h"
#include "car.par"
#include "code_var.h"
#include "NumericalOperator.h"

#define EPSILON 1e-9
#define RELTOL 1e-9
#define MAXITER 12
#define BUFFER_SIZE 5

float64_t norm(float64_t * x,int n){
	int i;
	float64_t sum;
	sum=0.0;
	
	for(i=0;i <n;i++){
		sum+=fabs(x[i]);
	}
	return sum;
}

bool Inverse(float64_t *Ainv,float64_t *At,int n)
{
	int i;
	int pivot,p,row,col;
	float64_t factor;
	float64_t **Ar;
	float64_t **A;
	float64_t *temp;
	
	Ar=(float64_t**)malloc(sizeof(float64_t*)*n);
	A=(float64_t**)malloc(sizeof(float64_t*)*n);
	for(i=0;i < n;i++){
		Ar[i]=(float64_t*)malloc(sizeof(float64_t)*n*2);
	}


	//Matrix Constructor
	for(row=0;row < n;row++){
		A[row]=Ar[row];
		for(col=0;col < n;col++){
			Ar[row][col]=At[col*n+row];
		}
		memset(&(Ar[row][n]),0,sizeof(float64_t)*n);
		Ar[row][n+row]=1;
	}
	
	
	//Forward Substitution
	for(p=0;p <n-1;p++){
		//Find Pivot
		pivot=-1;
        
		for(row=p;row < n;row++) {
			if(A[row][p] != 0.0){
				pivot=row;
				break;
			}
		}
		//Singular!!!! Check]
		if (pivot==-1){
			for(i=0;i < n;i++){
				free(Ar[i]);
			}
			
			free(A);
			free(Ar);	

			return false;
		}else if (pivot!=p) {
		//swap row 
			temp=A[p];
			A[p]=A[pivot];
			A[pivot]=temp;
		}
		
		for(row=p+1;row < n;row++){
			factor=A[row][p]/A[p][p];
			for(col=0;col <n*2;col++){
				A[row][col]=A[row][col]-A[p][col]*factor;
			}
		}
	}

	//Backward Substitution
	for(p=n-1;p  > 0;p--){
		for (col=n;col < n*2;col++){
			A[p][col]=A[p][col]/A[p][p];
		}
		A[p][p]=1.0;
		
		for(row=p-1;row >= 0;row--){
			for(col=n;col <n*2;col++){
				A[row][col]=A[row][col]-A[p][col]*A[row][p];
			}
			A[row][p]=0.0;
		}
	}
	
	for(col=n;col <n*2;col++){
		A[0][col]=A[0][col]/A[0][0];
	}
	A[0][0]=1.0;
	
	for(row=0;row < n;row++){
		for(col=0;col < n;col++){
			Ainv[col*n+row]=A[row][col+n];
		}
	}
	for(i=0;i < n;i++){
		free(Ar[i]);
	}
	
	free(A);
	free(Ar);	

	return true;
}

float64_t MAX(float64_t a, float64_t b)
{
    if(a < b)
        return b;
    else 
        return a;
}

float64_t MIN(float64_t a, float64_t b)
{
    if(a < b)
        return a;
    else 
        return b;
}

int sgn(float64_t value){
	
	if(value == 0){
		return 0;
	}
	else if(value < 0){
		return -1;
	}
	else{
		return 1;
	}
}

float64_t sat(float64_t value, float64_t layer){
	
	if(value < -layer){
		return -1;
	}
    else if(value > layer){
		return 1;
	}
    else{
        return (value/layer);
    }
}

void feuler(deq eq,float64_t sampletime,float64_t *x,float64_t *input,float64_t* disturbance,unsigned char N)
{
    int i;
    float64_t xt[BUFFER_SIZE];
    float64_t dx[BUFFER_SIZE];
    
    for(i=0;i < N;i++){
        xt[i]=x[i];
    }   
    eq(dx,xt,input,disturbance);
    for(i=0;i < N;i++){
        x[i]=xt[i]+sampletime*dx[i];
    }    
    
}

void beuler(deq eq,float64_t sampletime,float64_t *x,float64_t *input,float64_t *disturbance,unsigned char N)
{   
    int iter;
    int i,j;
    float64_t xr[BUFFER_SIZE];
    float64_t dx1[BUFFER_SIZE];
    float64_t dx2[BUFFER_SIZE];
    float64_t J[BUFFER_SIZE][BUFFER_SIZE];
    float64_t Jinv[BUFFER_SIZE][BUFFER_SIZE];
    float64_t xt[BUFFER_SIZE];
    float64_t g[BUFFER_SIZE];
    //Setting Initial Value.
    for(j=0;j < N;j++){
        xr[j]=x[j];
    }
    //Iteration 
    for (iter=0;iter < MAXITER;iter++){
        
        eq(dx1,xr,input,disturbance);
        
        //Solving Equation x[n+1]-x[n]-h*f(x[n+1])=0 by Newton
        //Jacobian Calculate (I-hJ)
        for (i=0;i < N;i++){
            for(j=0;j < N;j++){
                xt[j]=xr[j];
            }
            xt[i]+=EPSILON;
            eq(dx2,xt,input,disturbance);
            for(j=0;j < N;j++){
                J[j][i]=-sampletime*(dx2[j]-dx1[j])/EPSILON;
            }
            J[i][i]+=1;
        }
        
        //Get Inverse
        if (Inverse(&Jinv[0][0],&J[0][0],N)){
            //Save Previous Root
            for(i=0;i < N; i++){
                //xr_old[i]=xr[i];
                //g=x(n+1)-x(n)-h*f(x(n+1));
                g[i]=xr[i]-x[i]-sampletime*dx1[i];
            }
			if (norm(g,N)< RELTOL ){
				break;
			}

            //Update Root
            for(i=0;i < N; i++){
                for(j=0;j < N;j++){
                    xr[i]-=Jinv[i][j]*g[j];
                }
            }
        }
        else{
            //����ó��
        }
        
    }
    for(i=0;i < N; i++){
        x[i]=xr[i];
    }

    
}

void ode4(deq eq,float64_t sampletime,float64_t *x,float64_t *input,float64_t *disturbance,unsigned char N)
{
    int i;
    float64_t dx[BUFFER_SIZE];
    float64_t k[4][BUFFER_SIZE];
    float64_t xt[BUFFER_SIZE];
    
    //1st
    for(i=0;i < N;i++){
        xt[i]=x[i];
    }
    eq(dx,xt,input,disturbance);

    for(i=0;i < N;i++){
        k[0][i]=sampletime*dx[i];
    }
    
    //2nd
    for(i=0;i < N;i++){
        xt[i]=x[i]+k[0][i]*0.5;
    }
    eq(dx,xt,input,disturbance);

    for(i=0;i < N;i++){
        k[1][i]=sampletime*dx[i];
    }
    //3rd
    for(i=0;i < N;i++){
        xt[i]=x[i]+k[1][i]*0.5;
    }
    eq(dx,xt,input,disturbance);

    for(i=0;i < N;i++){
        k[2][i]=sampletime*dx[i];
    }
    //4th
    for(i=0;i < N;i++){
        xt[i]=x[i]+k[2][i];
    }
    eq(dx,xt,input,disturbance);

    for(i=0;i < N;i++){
        k[3][i]=sampletime*dx[i];
    }
    //final value
    
    for(i=0;i < N;i++){
        x[i]=x[i]+(k[0][i]+2*(k[1][i]+k[2][i])+k[3][i])*0.1667;
    }
}

void Euler_diff(float64_t* dx, float64_t* x, const float64_t* x_old)
{
    *dx = (*x - *x_old)/SYS_SAMPLETIME;
}

void Euler_diff_lowpass(float64_t* dx, const float64_t* dx_old, const float64_t* x, const float64_t* x_old, float64_t tau)
{
    *dx = ((*x - *x_old) + (*dx_old)*tau)/(SYS_SAMPLETIME + tau);
}

void Euler_diff_highpass(float64_t* dx, const float64_t* dx_old, const float64_t* x, const float64_t* x_old, float64_t tau)
{
    *dx = ((*x - *x_old)*tau + *dx_old)/(SYS_SAMPLETIME*tau + 1);
}

void Tustin_diff(float64_t* dx, const float64_t* dx_old, const float64_t* x, const float64_t* x_old)
{
    *dx = (*x - *x_old)*2/SYS_SAMPLETIME - *dx_old;
}

static int LevelOutputIndex(float64_t level)
{
    if (level == 1.0)
    {
        return 0;
    }
    else if (level == 2.0)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

void LevelTransitionArraySmoothing(float64_t current_level,
                                   const float64_t *level_value,
                                   int value_count,
                                   float64_t *smoothed_value,
                                   float64_t *prev_level,
                                   float64_t *target_level,
                                   float64_t *transition_flag,
                                   float64_t *sum_weight,
                                   float64_t *weight_rate,
                                   float64_t *init_flag)
{
    int i;
    int current_index;
    int prev_index;
    int target_index;

    if (*weight_rate <= 0.0)
    {
        *weight_rate = SYS_SAMPLETIME/5;
    }

    if (*init_flag < 0.5)
    {
        *prev_level = current_level;
        *target_level = current_level;
        *sum_weight = 0.0;
        *transition_flag = 0.0;
        *init_flag = 1.0;
    }
    else if (current_level != *target_level)
    {
        *target_level = current_level;
        *sum_weight = 0.0;
        *transition_flag = 1.0;
    }

    if (*transition_flag > 0.5)
    {
        *sum_weight = *sum_weight + *weight_rate;

        if (*sum_weight > 1.0)
        {
            *sum_weight = 1.0;
        }

        prev_index = LevelOutputIndex(*prev_level);
        target_index = LevelOutputIndex(*target_level);

        for (i = 0; i < value_count; i++)
        {
            smoothed_value[i] = (1.0 - *sum_weight) * level_value[prev_index*value_count + i]
                              + (*sum_weight) * level_value[target_index*value_count + i];
        }

        if (*sum_weight >= 1.0)
        {
            *sum_weight = 0.0;
            *transition_flag = 0.0;
            *prev_level = *target_level;
        }
    }
    else
    {
        current_index = LevelOutputIndex(current_level);

        for (i = 0; i < value_count; i++)
        {
            smoothed_value[i] = level_value[current_index*value_count + i];
        }
    }
}

