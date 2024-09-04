#ifndef __ZDRIVE_H__
#define __ZDRIVE_H__

#include "main.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"

#define USE_ZDRIVE_NUM 4

typedef enum
{
    Zdrive_Disable = 0,                  // 0失能
    Zdrive_Current,                  // 1电流模式
    Zdrive_Speed,                    // 2速度模式
    Zdrive_Postion,                  // 3位置模式
    Zdrive_Test,                     // 4测试模式
    Zdrive_RVCalibration,            // 5电阻电感校准
    Zdrive_EncoderLineCalibration,   // 6编码器线性补偿
    Zdrive_EncoudeOffsetCalibration, // 7编码器偏移校准
    Zdrive_VKCalibration,            // 8VK校准
    Zdrive_SaveSetting,              // 9保存配置
    Zdrive_EraseSetting,             // 10擦除配置
    Zdrive_ClearErr,                 // 11擦除错误
    Zdrive_Brake                     // 12刹车
} ZdriveMode;

typedef enum
{
    Zdrive_Well = 0,                // 0无
    Zdrive_InsufficientVoltage, // 1低电压
    Zdrive_OverVoltage,         // 2过电压
    Zdrive_InstabilityCurrent,  // 3电流不稳
    Zdrive_OverCurrent,         // 4过电流
    Zdrive_OverSpeed,           // 5超速
    Zdrive_ExcessiveR,          // 6电阻过大
    Zdrive_ExcessiveInductence, // 7电感过大
    Zdrive_LoseEncoder1,        // 8编码器错误
    Zdrive_PolesErr,            // 9极对数不匹配
    Zdrive_VKCalibrationErr,    // 10 KV校准失败
    Zdrive_ModeErr,             // 11模式不合法
    Zdrive_ParameterErr,        // 12参数错误
    Zdrive_Hot                  // 13过热
} ZdriveErr;

typedef struct
{
    float speed;
    float angle;
    float posIn;
    float current;
}ZdriveValue;

typedef struct
{
    uint16_t GapCnt;
    uint32_t lastRxtime;
    uint32_t timeoutTicks;
    uint32_t stuckCnt;
    float lockAngle;
}ZdriveArgum;
typedef struct  
{   
    float GearRatio;
    float ReductionRatio;
    /* data */
}ZdriveParam;

typedef struct
{
    bool timeoutCheck;
    bool stuckCheck;
    bool Arriveflag;
    ZdriveErr err;
}ZdriveStatus;

typedef struct
{
    
    ZdriveMode mode,modeRead;
    ZdriveParam param;
    ZdriveArgum argum;
    ZdriveValue valReal,valSetPre,valSetNow;
    ZdriveStatus statusflag;
    bool Enable;
    bool Begin;
}Zdrive;

extern Zdrive Zmotor[USE_ZDRIVE_NUM];

void ZdriveInit(void);
void ZdriveFunc(void);
void ZdriveReceive(CAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data);
void ZdriveSetSpeed(float speed,uint8_t id);
void ZdriveSetPosition(float angle,uint8_t id);
void ZdriveSetMode(float mode,uint8_t id);
void ZdriveSetPresentPos(float angle,uint8_t id);
void ZdriveAskPosition(uint8_t id);
void ZdriveAskSpeed(uint8_t id);
void ZdriveAskMode(uint8_t id);
void ZdriveAskPin(uint8_t id);
void ZdriveAskErr(uint8_t id);
void ZdriveAskCurrent(uint8_t id);
void ZdriveSetPVT(float speed,float angle,uint8_t id);


#endif /* __ZDRIVE_H__ */

