#include "ZDrive.h"
#include "stdbool.h"
#include "stdlib.h"
#include "CANQUEUE.h"
#include "mathFunc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "Chassis.h"
Zdrive Zmotor[USE_ZDRIVE_NUM];
CAN_Queue ZDrive_queue;
void ZdriveInit()
{
    for(int i = 0;i < USE_ZDRIVE_NUM ;i++)
    {
        Zmotor[i].param.GearRatio = 1.f;
        Zmotor[i].param.ReductionRatio = 1.f;
        Zmotor[i].Enable = false;
        Zmotor[i].Begin = false;
        Zmotor[i].mode = Zdrive_Disable;
        Zmotor[i].valSetNow.speed = 0;
        Zmotor[i].valSetNow.angle = 0;
        Zmotor[i].statusflag.Arriveflag =false;
        Zmotor[i].argum.GapCnt = 0;
		Zmotor[i].valReal.angle = 0;
        ZdriveSetPresentPos(0,i+1);
    }
}
void ZdriveSetPosition(float angle,uint8_t id) 
{
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    angle /= 360.f;//角度转换
    angle *= 5.6f; //减速比
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] = 0x47;
    memcpy(&ZDrive_queue.messages[ZDrive_queue.tail].data[1],&angle,sizeof(float));
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 5;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;

}
void ZdriveSetSpeed(float speed,uint8_t id) 
{
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    speed /= 60.f;
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] = 0x45;
    memcpy(&ZDrive_queue.messages[ZDrive_queue.tail].data[1],&speed,sizeof(float));
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 5;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;

}
void ZdriveSetMode(float mode,uint8_t id) 
{
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    Zmotor[id-1].mode = mode;//
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] = 0x3D;
    memcpy(&ZDrive_queue.messages[ZDrive_queue.tail].data[1],&mode,sizeof(float));
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 5;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;

}
void ZdriveSetPVT(float speed,float angle,uint8_t id)
{
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    angle /= 360.f;
    speed /= 60.f;

    double temp;
    temp = cvtFloat2Double(speed,angle);

    memcpy(ZDrive_queue.messages[ZDrive_queue.tail].data,&temp,sizeof(double));
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 8;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;
}
void ZdriveSetPresentPos(float angle,uint8_t id)
{
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    angle /= 360.f; //设置角度以圈为单位
    angle *= 5.6f; 
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] = 0x5f;
    memcpy(&ZDrive_queue.messages[ZDrive_queue.tail].data[1],&angle,sizeof(float));
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 5;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;
}

void ZdriveReceive(CAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data)
{
    uint8_t controlID =Rxheader.StdId;
    uint8_t operationID = Rx_data[0];

    if(controlID > 4) return;
    Zmotor[controlID - 1].statusflag.err = Zdrive_Well;//正常设置well
    switch (operationID)
    {
    case 0x5E: //position
    {   
        memcpy(&Zmotor[controlID - 1].valReal.angle,&Rx_data[1],sizeof(float));
        Zmotor[controlID -1].valReal.angle *= (360.f / Zmotor[controlID - 1].param.ReductionRatio/5.6f);
        break;
    }
    case 0x52: //current
    {   
        memcpy(&Zmotor[controlID - 1].valReal.current,&Rx_data[1],sizeof(float));
        break;
    }
    case 0x5C: //speed
    {
        memcpy(&Zmotor[controlID -1].valReal.speed,&Rx_data[1],sizeof(float));
        break;
    }
    case 0x3C://mode
    {
        float tempMode;
        memcpy(&tempMode,&Rx_data[1],sizeof(float));
        Zmotor[controlID -1].modeRead = (ZdriveMode)tempMode;
        break;
    }
    case 0x40: //error
    {
        float tempErr;
        memcpy(&tempErr,&Rx_data[1],sizeof(float));
        Zmotor[controlID - 1].statusflag.err = (ZdriveErr)tempErr;
        break;
    }
    case 0x46://pos_in
    {
        float tempPos_in;
        memcpy(&tempPos_in,&Rx_data[1],sizeof(float));
        Zmotor[controlID - 1].valReal.posIn = tempPos_in*360.f/5.6f;
        break;
    }
    default:
        break;
    }
}

void ZdriveAskPosition(uint8_t id){
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] = 0x5E;
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 1;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;
}
void ZdriveAskSpeed(uint8_t id){
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] =0x5C;
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 1;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;
}
void ZdriveAskCurrent(uint8_t id){
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] =0x52;
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 1;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;
}
void ZdriveAskMode(uint8_t id){
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] =0x3C;
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 1;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;
}
void ZdriveAskPin(uint8_t id){
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] =0x46;
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 1;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;
}
void ZdriveAskErr(uint8_t id)
{
    if(CAN_Queue_IsFull(&ZDrive_queue)){
        CAN2SendQueueFULL++;
        return;
    }
    ZDrive_queue.messages[ZDrive_queue.tail].data[0] =0x40;
    ZDrive_queue.messages[ZDrive_queue.tail].id = id;
    ZDrive_queue.messages[ZDrive_queue.tail].DLC = 1;
    ZDrive_queue.tail = (ZDrive_queue.tail + 1) % Queue_MAXN;

}
void ZdriveFunc()
{
    for(int i = 0;i < USE_ZDRIVE_NUM ;i++)
    {
        if(Zmotor[i].Enable)
        {
            if(Zmotor[i].Begin)
            {
                switch (Zmotor[i].mode)
                {
                    case Zdrive_Speed:
                    {
                        if(Zmotor[i].modeRead != Zdrive_Speed)
                            ZdriveSetMode((float)Zdrive_Speed,i+1);
                        else
                            ZdriveSetSpeed(Zmotor[i].valSetNow.speed,i+1);
                        break;   
                    }
                    case Zdrive_Current:
                    {
                        if(Zmotor[i].modeRead != Zdrive_Current)
                            ZdriveSetMode((float)Zdrive_Current,i+1);
                        break;
                    }
                    case Zdrive_Disable:
                    {
                        if(Zmotor[i].modeRead != Zdrive_Disable)
                            ZdriveSetMode((float)Zdrive_Disable,i+1);
                        break;
                    }
                    case Zdrive_Postion:
                    {
                        if(Zmotor[i].modeRead != Zdrive_Postion)
                            ZdriveSetMode((float)Zdrive_Postion,i+1);
                        else
                        {
                            if(Zmotor[i].valSetNow.angle != 0)
                            {
                                if(ABS(ABS(Zmotor[i].valSetNow.angle/Zmotor[i].valReal.posIn)-1) > 1e-4f)
                                {
                                    //当角度的偏差过大
                                    ZdriveSetPosition(Zmotor[i].valSetNow.angle,i+1);
                                }
															
                            }
                            else if(Zmotor[i].valReal.posIn > 0.2f)
                                    ZdriveSetPosition(Zmotor[i].valSetNow.angle,i+1);
                            
                        }
                        break;
                    }

                    default:
                        break;
                }
            }
            
        }
        else
        {
            if(Zmotor[i].modeRead != Zdrive_Disable)
                ZdriveSetMode((float)Zdrive_Disable,i+1);
        }
        ZdriveAskCurrent(i+1);
        ZdriveAskMode(i+1);
        ZdriveAskPin(i+1);
        ZdriveAskPosition(i+1);
        ZdriveAskSpeed(i+1);
        ZdriveAskErr(i+1);
        //问了才有数据反馈
    }
}


