#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "mathFunc.h"
#include "VescMotor.h"
#include "ZDrive.h"
#include "can.h"
#include "CANQUEUE.h"

void Master_Response(uint8_t DLC,uint32_t ID,uint8_t Data0,uint8_t Data1,uint8_t Data2);
void Motor_Enable(bool EnableFlag);
void Drive_SetPos(uint8_t *Rx_data);
void Drive_SetRPM(uint8_t *Rx_data);
void Rotate_SetPos(uint8_t *Rx_data);
void Rotate_SetRPM(uint8_t *Rx_data);
void Drive_Position_Answer(void);
void Drive_RPM_Answer(void);
void Drive_Current_Answer(void);
void Rotate_Position_Answer(void);
void Rotate_RPM_Answer(void);
void Rotate_Current_Answer(void);
void ChassisFunc(CAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data);

#endif /* __CHASSIS_H__ */

