/**
  ******************************************************************************
  * @file    LA_LHA.c
  * @author  
  * @version V1.1.1
  * @date    2018-1-2
  * @brief   ��ȡ���ش���������
  *         
  @verbatim
  ==============================================================================
*/


/* Includes ------------------------------------------------------------------*/
#include "LZ_LHA.h"

static float GetVoltage(void);



/**
  * @brief  
  * @param  ��
  * @retval ����������������ֵ
  */
  
uint32_t get_weight(void)
{
    /* ��ѹת��Ϊ������ϵ�������ѹ�Ŵ����й�*/
    float k = 500.f;
    
    /* �����õ�������ֵ����λ��kg*/
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


