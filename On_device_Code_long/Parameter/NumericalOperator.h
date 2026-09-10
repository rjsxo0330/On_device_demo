#ifndef _ODE4_
#define _ODE4_

typedef void(*deq)(float64_t*, float64_t*,float64_t*,float64_t*);

extern float64_t norm(float64_t * x,int n);
extern bool Inverse(float64_t *Ainv,float64_t *At,int n);
extern int sgn(float64_t value);
extern float64_t MAX(float64_t a, float64_t b);
extern float64_t MIN(float64_t a, float64_t b);
extern float64_t sat(float64_t value, float64_t layer);
extern void ode4(deq eq,float64_t sampletime,float64_t *x,float64_t *input,float64_t* disturbance,unsigned char N);
extern void beuler(deq eq,float64_t sampletime,float64_t *x,float64_t *input,float64_t* disturbance,unsigned char N);
extern void feuler(deq eq,float64_t sampletime,float64_t *x,float64_t *input,float64_t* disturbance,unsigned char N);
extern void Euler_diff(float64_t* dx, float64_t* x, const float64_t* x_old);
extern void Euler_diff_lowpass(float64_t* dx, const float64_t* dx_old, const float64_t* x, const float64_t* x_old, float64_t tau);
extern void Euler_diff_highpass(float64_t* dx, const float64_t* dx_old, const float64_t* x, const float64_t* x_old, float64_t tau);
extern void Tustin_diff(float64_t* dx, const float64_t* dx_old, const float64_t* x, const float64_t* x_old);
extern void LevelTransitionArraySmoothing(float64_t current_level,
                                          const float64_t *level_value,
                                          int value_count,
                                          float64_t *smoothed_value,
                                          float64_t *prev_level,
                                          float64_t *target_level,
                                          float64_t *transition_flag,
                                          float64_t *sum_weight,
                                          float64_t *weight_rate,
                                          float64_t *init_flag);
#endif
