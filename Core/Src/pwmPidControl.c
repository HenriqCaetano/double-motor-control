#include "pwmPidControl.h"


//the min and max values should consider the duty cycle %
//example: if the minValue is 10% duty cycle, the minimalPwm must be a value that reflects it
void pidInit(Pid* p, float minimalPwm, float maximumPwm, float kp, float ki, float kd){
    p->max = maximumPwm;
    p->min = minimalPwm;
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->lastError = 0;
}


/// @brief given current and target speds, computes PWM increment
/// @param setPoint target speed in encoder pulses
/// @param feedBackValue current speeed in encoder pulses
/// @param p PID controller
/// @return increment to PWM
float computePwmValue(float setPoint, float feedBackValue, Pid* p){

    if(setPoint < 0) setPoint *= -1;
    if (feedBackValue < 0) feedBackValue *= -1;

    p->error = setPoint - feedBackValue; //get current error
    p->integralValue += p->error; //update integralError

    //calcula valor do registrador PWM
    //Componente diferencial pode ser adicionada com "+ p->kd * (p->error - p->lastError)"
    float result = p->kp * p->error + p->ki * p->integralValue;

    p->lastError = p->error;

    //deals with limit values
    if(result > p->max) result = p->max;
    else if(result < p->min) result = p->min;
    return result;
}

