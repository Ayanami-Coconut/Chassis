#pragma once

#include "stm32f405xx.h"
typedef struct {
  float KP;
  float KI;
  float KD;
  volatile int32_t SetVal;   // 设置值
  volatile int32_t CurVal;   // 当前值
  volatile int32_t err;      // 误差
  volatile int32_t err_last; // 上次误差
  volatile int32_t err_prv;  // 前次误差
  volatile int32_t delta;    // 输出量
} PIDType;
void PID_Init(PIDType *pid, float kp, float ki, float kd, int32_t set);//初始化
int32_t PID_Caculate(PIDType *pid);//PID计算
