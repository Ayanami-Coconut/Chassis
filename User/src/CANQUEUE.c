#include "CANQUEUE.h"
#include "can.h"
#include "string.h"
#include "main.h"

void CAN_Queue_Init(CAN_Queue *queue,bool STD_or_EXT) {
  //queue->count = 0;
  queue->head = 0;
  queue->tail = 0;
  if(STD_or_EXT)
    queue->IDE = CAN_ID_EXT;//队列初始化
  else
    queue->IDE = CAN_ID_STD;
}
bool CAN_Queue_Isempty(CAN_Queue *queue) {
  return (queue->head == queue->tail);
}
bool CAN_Queue_IsFull(CAN_Queue *queue) {
  return ((queue->tail+1) % Queue_MAXN == queue->head);
}
// bool CAN_Queue_Enqueue(CAN_Queue *queue, CAN_Message message) {
//     if(CAN_Queue_IsFull(queue))
//         return 0;
//     queue->messages[queue->tail++] = message;
//     queue->tail %= Queue_MAXN;
// 		return 1;
// }
void CAN_Queue_Dequeue(CAN_Queue *queue,CAN_TypeDef *CANx) {
    if (CAN_Queue_Isempty(queue)) 
        return;
    uint8_t Tx_data[8] = {0};
    uint32_t pTxmailbox = 0;
    CAN_TxHeaderTypeDef Txheader;
    if(CANx == CAN2)
    {
      //注意以下message皆为messages[queue->head]
      Txheader.IDE = queue->IDE;
      if(queue->IDE == CAN_ID_EXT){
        if((queue->messages[queue->head].id & 0xf0000000) == 0xf0000000)  //注意运算顺序
          queue->messages[queue->head].id &= 0x0fffffff;//必做的一步，区分VESC和其他通信
        
        Txheader.ExtId = queue->messages[queue->head].id;
      }
      else if(queue->IDE == CAN_ID_STD)
      {
        Txheader.StdId = queue->messages[queue->head].id;   
      }
      Txheader.DLC = queue->messages[queue->head].DLC;
      Txheader.RTR = CAN_RTR_DATA;
      memcpy(Tx_data,queue->messages[queue->head].data,sizeof(uint8_t)*Txheader.DLC);
			queue->head++;
			queue->head %= Queue_MAXN;
			HAL_CAN_AddTxMessage(&hcan2,&Txheader,Tx_data,&pTxmailbox);
    }

}
// bool CAN_Queue_Front(CAN_Queue *queue, CAN_Message *message) {
//     if (CAN_Queue_Isempty(queue))
//         return 0;
//     *message=queue->messages[queue->head];
//     return 1;
// }
