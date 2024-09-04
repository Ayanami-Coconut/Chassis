#include "main.h"
#include "Valve.h"
//unsigned char CYL_table[8] = {0x01,0x01,0x01,0x01,0x00,0x01,0x00 ,0x01} ;
//利用GPIO口模拟I2C通信 
void Valve_Control(uint8_t data)
{
    //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
    if(data > 0xff)
        data = 0xff;
    for(int i = 0; i < 8;i++)
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // 相当于产生下降沿 开始传输
        if(data & 0x01)
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET); // 按位与传输数据
        else
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
        data = data >> 1;
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); // 相当于产生上升沿 结束传输
    }
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
}   
