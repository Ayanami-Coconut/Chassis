#include "usart_commnuicate.h"

void uart_CMInit(uart_communicate *uc,UART_HandleTypeDef *huart)
{
    uc->write = 0;
    uc->get_head = 0;
    uc->huart = huart;
}
void uart_CMReceive(uart_communicate *uc,unsigned char txc){
    if(uc->get_head)
    {
        uc->tail[0] =uc->tail[1];
        uc->tail[1] = txc;
        if(uc->tail[1] == TAIL_CODE2 && uc->tail[0] == TAIL_CODE1){
            if(uart_CMRxDetect(uc))
                uart_CMRxDeal(uc);
            uc->write = 0;
            uc->get_head = 0;
            return; 
        }
        uc->rx_data[uc->write++] = txc;
    }
    else{
        uc->head[0] = uc->head[1];
        uc->head[1] = txc;
        if(uc->head[1] == HEAD_CODE2 && uc->head[0] == HEAD_CODE1)
            uc->get_head = true;
    }
    if(uc->write >= 50){
        uc->write = 0;
        uc->get_head = 0;
    }
}
bool uart_CMRxDetect(uart_communicate *uc){
    unsigned short sum = 0;
    int i;
    for(i=0;i<uc->write-2;i++){
        sum+=uc->rx_data[i];
        sum&=0x00ff;
    }
    if(sum!=uc->rx_data[i])
        return 0;
    return 1;
}
void uart_CMRxDeal(uart_communicate *uc)
{
    if(uc->rx_data[0]==0) return;
    if(uc->rx_data[0]==1){
        if(uc->rx_data[1]==1)
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    }
    uart_CMTransmit(uc,0,&uc->rx_data[1],1);
}
void uart_CMTransmit(uart_communicate *uc,int id,unsigned char *data,int size){
    uc->tx_data[0] = HEAD_CODE1;
    uc->tx_data[1] = HEAD_CODE2;
    uc->tx_data[2] = id;
    int i;unsigned short sum=0;
    for(i=3;i<size+3;i++){
        uc->tx_data[i] = data[i];
        sum+= uc->tx_data[i];
        sum&= 0x00ff;
    }
    uc->tx_data[i++] = sum;
    uc->tx_data[i++] = TAIL_CODE1;
    uc->tx_data[i++] = TAIL_CODE2;
    HAL_UART_Transmit_DMA(uc->huart,uc->tx_data,i);
}
