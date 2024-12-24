#include "n32g45x.h"
/*本文件用于XIP测试与存放*/

void blink_init(void)
{
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB,ENABLE);
    GPIO_InitType Blink_IO;
    GPIO_InitStruct(&Blink_IO);
    
    Blink_IO.GPIO_Mode  = GPIO_Mode_Out_PP;
    Blink_IO.Pin        =GPIO_PIN_5;
    Blink_IO.GPIO_Speed =GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB,&Blink_IO);
}

void blink(uint8_t sta)
{
    if(sta)
        GPIO_SetBits(GPIOB,GPIO_PIN_5);
    else
        GPIO_ResetBits(GPIOB,GPIO_PIN_5);
}










