/**
  ******************************************************************************
  * @file    LA_LHA.c
  * @author  
  * @version V1.1.1
  * @date    2018-1-2
  * @brief   读取称重传感器数据
  *         
  @verbatim
  ==============================================================================
*/


/* Includes ------------------------------------------------------------------*/
#include "LZ_LHA.h"

static float GetVoltage(void);



/**
  * @brief  
  * @param  空
  * @retval 传感器测量的重量值
  */
  
uint32_t get_weight(void)
{
    /* 电压转化为重量的系数，与电压放大倍数有关*/
    float k = 500.f;
    
    /* 测量得到的重量值，单位：kg*/
    uint32_t weight = 0.f;
    
    weight = (uint32_t)(k * GetVoltage());
    
    return weight;
}
  

static float GetVoltage(void)
{
    float v = 0.f;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,50);    
    v = (float)(int32_t)(HAL_ADC_GetValue(&hadc1)) * 3.31f / 4096;
    HAL_ADC_Stop(&hadc1);
    return v;
}


