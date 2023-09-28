#ifndef PID_H
#define PID_H

typedef struct pid Pid;

struct pid{
    float kp /*! Proportional constant */;
    float ki /*! Integrator constant */;
    float kd /*! Differential constant */;
    float max /*! Max manipulated value */;
    float min /*! Miniumum manipulated value */;
    float error /*! Error value */;
    float integralValue /*! Integrator value */;
    float lastError /*Diferential value*/;
};

void pidInit(Pid* p,float minimalPwm, float maximumPwm, float kp, float ki, float kd);

float computePwmValue(float setPoint, float feedBackValue, Pid* p);




#endif

