#include "LED.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
void LED_Flow(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
    osDelay(200);
}
