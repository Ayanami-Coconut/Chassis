#include "pid.h"
/**
 * @brief 
 * 
 * @param pid 
 * @param kp 
 * @param ki 
 * @param kd 
 * @param set 
 * @return int32_t 返回pid输出量
 */
void PID_Init(PIDType *pid,float kp,float ki,float kd,int32_t set)
{
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
    pid->SetVal =set;
    pid->CurVal = 0;
    pid->err = 0;
    pid->err_prv = 0;
    pid->err_last = 0;
    pid->delta = 0;
}
int32_t PID_Caculate(PIDType *pid)
{

    pid->err = pid->SetVal - pid->CurVal;
    pid->delta = pid->KP*(pid->err - pid->err_last) + pid->KI *pid->err + pid->KD *(pid->err -2*pid->err_last +pid->err_prv);
    
    pid->err_prv = pid->err_last;
		pid->err_last = pid->err;//顺序不能乱
    return pid->delta;
}
