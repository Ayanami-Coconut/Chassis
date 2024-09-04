#include "Chassis.h"
#include "CANQUEUE.h"
// 1——FL 2——FR 3——BL ——BR
CAN_Queue Answer_queue;
void Master_Response(uint8_t DLC,uint32_t ID,uint8_t Data0,uint8_t Data1,uint8_t Data2){
    if(CAN_Queue_IsFull(&Answer_queue))
    {
        CAN2SendQueueFULL++;
        return;
    }
    Answer_queue.messages[Answer_queue.tail].id = ID;
    Answer_queue.messages[Answer_queue.tail].DLC = DLC;
    Answer_queue.messages[Answer_queue.tail].data[0] = Data0;
    Answer_queue.messages[Answer_queue.tail].data[1] = Data1;
    Answer_queue.messages[Answer_queue.tail].data[2] = Data2;
    Answer_queue.tail = (Answer_queue.tail + 1 ) % Queue_MAXN;

}
void Motor_Enable(bool EnableFlag)
{
    if(EnableFlag == 1)
    {
        for(int i = 0 ; i < USE_VESCNUM ; i++) // 驱动电机
        {
            Vescmotor[i].Enable = true;
            Vescmotor[i].mode = VESC_RPM;
            if(!Vescmotor[i].Begin)
                Vescmotor[i].valSet.speed = 0;
        }
        for(int i= 0 ; i < USE_ZDRIVE_NUM ; i++) // 转向电机
        {
            Zmotor[i].Enable = true;
            Zmotor[i].Begin = true;
            Zmotor[i].mode = Zdrive_Postion;
						ZdriveSetPresentPos(0,i+1);
        }
    }
    else if(EnableFlag == 0)
    {
        for(int i = 0 ; i < USE_VESCNUM ; i++) 
        {    
            Vescmotor[i].valSet.speed = 0;
            Vescmotor[i].Enable = false;
            Vescmotor[i].Begin = false;       
        }
        for(int i= 0 ; i < USE_ZDRIVE_NUM ; i++)
        {
            Zmotor[i].Enable = false;
            Zmotor[i].Begin = false;
        }
    }
    Master_Response(2,0x02020101,'M',EnableFlag,0);
}

void Drive_SetPos(uint8_t *Rx_data)
{
    int16_t tempFL,tempFR,tempBL,tempBR;
            tempFL = (int16_t)(Rx_data[1] << 8 | Rx_data[0]);
            tempFR = (int16_t)(Rx_data[3] << 8 | Rx_data[2]);
            tempBL = (int16_t)(Rx_data[5] << 8 | Rx_data[4]);
            tempBR = (int16_t)(Rx_data[7] << 8 | Rx_data[6]);

    Vescmotor[0].valSet.angle = (float)(tempFL);
    Vescmotor[1].valSet.angle = (float)(tempFR);
    Vescmotor[2].valSet.angle = (float)(tempBL);
    Vescmotor[3].valSet.angle = (float)(tempBR);
            
    for(int i=0; i < USE_VESCNUM ; i++)
    {
        Vescmotor[i].mode = VESC_POSITION;
        Vescmotor[i].Begin = true;
    }
    Master_Response(4,0x02020102,'G','D','A'); 
}

void Drive_SetRPM(uint8_t *Rx_data)
{
    int16_t tempFL,tempFR,tempBL,tempBR;
            tempFL = (int16_t)(Rx_data[1] << 8 | Rx_data[0]);
            tempFR = (int16_t)(Rx_data[3] << 8 | Rx_data[2]);
            tempBL = (int16_t)(Rx_data[5] << 8 | Rx_data[4]);
            tempBR = (int16_t)(Rx_data[7] << 8 | Rx_data[6]);

    Vescmotor[0].valSet.speed = (float)(tempFL/5);
    Vescmotor[1].valSet.speed = (float)(tempFR/5);
    Vescmotor[2].valSet.speed = (float)(tempBL/5);
    Vescmotor[3].valSet.speed = (float)(tempBR/5);
            
    for(int i=0; i < USE_VESCNUM ; i++)
    {
        Vescmotor[i].mode = VESC_RPM;
        Vescmotor[i].Begin = true;
    }
    Master_Response(4,0x02020103,'G','D','V'); 
}

void Rotate_SetPos(uint8_t *Rx_data)
{
    int16_t tempFL,tempFR,tempBL,tempBR;
            tempFL = (int16_t)(Rx_data[1] << 8 | Rx_data[0]);
            tempFR = (int16_t)(Rx_data[3] << 8 | Rx_data[2]);
            tempBL = (int16_t)(Rx_data[5] << 8 | Rx_data[4]);
            tempBR = (int16_t)(Rx_data[7] << 8 | Rx_data[6]);

    Zmotor[0].valSetNow.angle = (float)(tempFL);
    Zmotor[1].valSetNow.angle = (float)(tempFR);
    Zmotor[2].valSetNow.angle = (float)(tempBL);
    Zmotor[3].valSetNow.angle = (float)(tempBR);
    for(int i = 0; i < USE_ZDRIVE_NUM ; i++)
        Zmotor[i].mode = Zdrive_Postion;
    Master_Response(4,0x02020105,'G','T','A');
}

void Rotate_SetRPM(uint8_t *Rx_data)
{
    int16_t tempFL,tempFR,tempBL,tempBR;
            tempFL = (int16_t)(Rx_data[1] << 8 | Rx_data[0]);
            tempFR = (int16_t)(Rx_data[3] << 8 | Rx_data[2]);
            tempBL = (int16_t)(Rx_data[5] << 8 | Rx_data[4]);
            tempBR = (int16_t)(Rx_data[7] << 8 | Rx_data[6]);

    Zmotor[0].valSetNow.speed = (float)(tempFL/5);
    Zmotor[1].valSetNow.speed = (float)(tempFR/5);
    Zmotor[2].valSetNow.speed = (float)(tempBL/5);
    Zmotor[3].valSetNow.speed = (float)(tempBR/5);
    for(int i = 0; i < USE_ZDRIVE_NUM ; i++)
        Zmotor[i].mode = Zdrive_Speed;
    Master_Response(4,0x02020106,'G','T','V');
}
void Drive_Position_Answer()
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    buffer_append_int16(Data,(int16_t)(Vescmotor[0].valNow.angle),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[1].valNow.angle),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[2].valNow.angle),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[3].valNow.angle),&index);

    if(CAN_Queue_IsFull(&Answer_queue))
    {
        CAN2SendQueueFULL++;
        return;
    }
    Answer_queue.messages[Answer_queue.tail].DLC = 8;
    Answer_queue.messages[Answer_queue.tail].id = 0x02020112;
    memcpy(Answer_queue.messages[Answer_queue.tail].data,Data,8);
    Answer_queue.tail = (Answer_queue.tail + 1) % Queue_MAXN;

}
void Drive_RPM_Answer()
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    buffer_append_int16(Data,(int16_t)(Vescmotor[0].valNow.speed*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[1].valNow.speed*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[2].valNow.speed*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[3].valNow.speed*5.f),&index);

    if(CAN_Queue_IsFull(&Answer_queue))
    {
        CAN2SendQueueFULL++;
        return;
    }
    Answer_queue.messages[Answer_queue.tail].DLC = 8;
    Answer_queue.messages[Answer_queue.tail].id = 0x02020113;
    memcpy(Answer_queue.messages[Answer_queue.tail].data,Data,8);
    Answer_queue.tail = (Answer_queue.tail + 1) % Queue_MAXN;
}
void Drive_Current_Answer()
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    buffer_append_int16(Data,(int16_t)(Vescmotor[0].valNow.current*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[1].valNow.current*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[2].valNow.current*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Vescmotor[3].valNow.current*5.f),&index);
    // memcpy(Data,(int16_t)Vescmotor[0].valNow.current,sizeof(int16_t));
    // ChangeDataByte(Data[0],Data[1]);
    // memcpy(Data,(int16_t)Vescmotor[1].valNow.current,sizeof(int16_t));
    // ChangeDataByte(Data[2],Data[3]);
    // memcpy(Data,(int16_t)Vescmotor[2].valNow.current,sizeof(int16_t));
    // ChangeDataByte(Data[4],Data[5]);
    // memcpy(Data,(int16_t)Vescmotor[3].valNow.current,sizeof(int16_t));
    // ChangeDataByte(Data[6],Data[7]);
    if(CAN_Queue_IsFull(&Answer_queue))
    {
        CAN2SendQueueFULL++;
        return;
    }
    Answer_queue.messages[Answer_queue.tail].DLC = 8;
    Answer_queue.messages[Answer_queue.tail].id = 0x02020114;
    memcpy(Answer_queue.messages[Answer_queue.tail].data,Data,8);
    Answer_queue.tail = (Answer_queue.tail + 1) % Queue_MAXN;
}
void Rotate_Position_Answer()
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    buffer_append_int16(Data,(int16_t)(Zmotor[0].valReal.angle),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[1].valReal.angle),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[2].valReal.angle),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[3].valReal.angle),&index);

    if(CAN_Queue_IsFull(&Answer_queue))
    {
        CAN2SendQueueFULL++;
        return;
    }
    Answer_queue.messages[Answer_queue.tail].DLC = 8;
    Answer_queue.messages[Answer_queue.tail].id = 0x02020115;
    memcpy(Answer_queue.messages[Answer_queue.tail].data,Data,8);
    Answer_queue.tail = (Answer_queue.tail + 1) % Queue_MAXN;
}

void Rotate_RPM_Answer()
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    buffer_append_int16(Data,(int16_t)(Zmotor[0].valReal.speed*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[1].valReal.speed*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[2].valReal.speed*5.f),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[3].valReal.speed*5.f),&index);

    if(CAN_Queue_IsFull(&Answer_queue))
    {
        CAN2SendQueueFULL++;
        return;
    }
    Answer_queue.messages[Answer_queue.tail].DLC = 8;
    Answer_queue.messages[Answer_queue.tail].id = 0x02020116;
    memcpy(Answer_queue.messages[Answer_queue.tail].data,Data,8);
    Answer_queue.tail = (Answer_queue.tail + 1) % Queue_MAXN;
}

void Rotate_Current_Answer()
{
    uint8_t Data[8] = {0};
    int32_t index = 0;
    buffer_append_int16(Data,(int16_t)(Zmotor[0].valReal.current),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[1].valReal.current),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[2].valReal.current),&index);
    buffer_append_int16(Data,(int16_t)(Zmotor[3].valReal.current),&index);

    if(CAN_Queue_IsFull(&Answer_queue))
    {
        CAN2SendQueueFULL++;
        return;
    }
    Answer_queue.messages[Answer_queue.tail].DLC = 8;
    Answer_queue.messages[Answer_queue.tail].id = 0x02020117;
    memcpy(Answer_queue.messages[Answer_queue.tail].data,Data,8);
    Answer_queue.tail = (Answer_queue.tail + 1) % Queue_MAXN;  
}
void ChassisFunc(CAN_RxHeaderTypeDef Rxheader,uint8_t *Rx_data)
{
        switch (Rxheader.ExtId)
        {
        case 0x01020201: //使能失能
        {
            Motor_Enable(Rx_data[1]);
            break;
        }       
        case 0x01020202: // 驱动电机速度设置
        {
            Drive_SetPos(Rx_data);
            break;
        }   
        case 0x01020203: //驱动电机角度设置
        {
            Drive_SetRPM(Rx_data);
            break;
        }
        case 0x01020205: //转向电机角度设置
        {
            Rotate_SetPos(Rx_data);
            break;
        }
        case 0x01020206: //转向电机速度设置
        {
            Rotate_SetRPM(Rx_data);
            break;
        }
        case 0x01020212: //驱动电机角度查询
        {
            Drive_Position_Answer();
            break;
        }
        case 0x01020213: //驱动电机速度查询
        {
            Drive_RPM_Answer();
            break;
        }
        case 0x01020214: //驱动电机电流查询
        {
            Drive_Current_Answer();
            break;
        }
        case 0x01020215: //转向电机位置查询
        {
            Rotate_Position_Answer();
            break;
        }
        case 0x01020216: //转向电机位置查询
        {
            Rotate_RPM_Answer();
            break;
        }
        case 0x01020217: //转向电机位置查询
        {
            Rotate_Current_Answer();
            break;
        }
        case 0x010202FF: //板子复位
        {
            __disable_irq();
            NVIC_SystemReset();
            break;
        }
        case 0x01020200:
        {
            bool Errflag = 0;
            for(int i = 0;i < 4;i++){
                if(Zmotor[i].statusflag.err > 0 || Vescmotor[i].statusflag.stuckflag || Vescmotor[i].statusflag.timeoutflag)
                {
                    Errflag = 1;
                }
            }
            if(Errflag)
                Master_Response(2,0x02020100,'C','E',0);
            else
                Master_Response(2,0x02020100,'C','R',0);
        }
        default:
            break;
        }
    
}
