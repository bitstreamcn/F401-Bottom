/*
 * gpiox.cpp
 *
 *  Created on: Apr 15, 2026
 *      Author: bitstream
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include "../Device/PMW3901/PMW3901.h"


extern void ( *Dio1IrqHandler )( void );

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case MOTION_Pin:
        	//GPIO_PIN_14
        	PMWMotionHandler();
        	break;
        case INT_Pin:
        	//GPIO_PIN_8
        	//下视雷达
        	break;

    }
}


#ifdef __cplusplus
}
#endif
