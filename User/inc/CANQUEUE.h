#ifndef __CANQUEUE_H__
#define __CANQUEUE_H__

#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include "stdlib.h"

#define Queue_MAXN 50

typedef struct CAN_Message
{
    uint8_t data[8];
    uint32_t id;
    uint8_t DLC;
}CAN_Message;

typedef struct CAN_Queue
{
    CAN_Message messages[Queue_MAXN];
    int head;
    int tail;
    uint8_t IDE;
//    int count;
}CAN_Queue;

void CAN_Queue_Init(CAN_Queue *queue,bool STD_or_EXT);
bool CAN_Queue_Isempty(CAN_Queue *queue);
bool CAN_Queue_IsFull(CAN_Queue *queue);
//bool CAN_Queue_Enqueue(CAN_Queue *queue,CAN_Message point);
void CAN_Queue_Dequeue(CAN_Queue *queue,CAN_TypeDef *CANx);
//bool CAN_Queue_Front(CAN_Queue *queue,CAN_Message *point);


extern CAN_Queue Vesc_queue;
extern CAN_Queue ZDrive_queue;
extern CAN_Queue Answer_queue;

#endif /* __CANQUEUE_H__ */


