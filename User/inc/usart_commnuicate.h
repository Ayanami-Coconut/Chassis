#pragma once

#include <stdbool.h>
#include <stdio.h>
#include "stdlib.h"
#include "main.h"
#define HEAD_CODE1 0xFF
#define HEAD_CODE2 0xfe
#define TAIL_CODE1 0x0a
#define TAIL_CODE2 0x0d
typedef struct uart_communicate
{
    char head[2];
    char tail[2];
    char rx_data[50];
    char tx_data[50];
    char write;
    char read;
    bool get_head;
    UART_HandleTypeDef *huart;
}uart_communicate;

void uart_CMInit(uart_communicate *uc,UART_HandleTypeDef *huart);
void uart_CMReceive(uart_communicate *uc,unsigned char txc);
void uart_CMTransmit(uart_communicate *uc,int id,unsigned char *data,int size);
bool uart_CMRxDetect(uart_communicate *uc);
void uart_CMRxDeal(uart_communicate *uc);

extern uart_communicate uc2;
