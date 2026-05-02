#ifndef __BSP_AD7606_H
#define __BSP_AD7606_H

#include "header.h"
//=======================================
// 1.引脚宏定义
//=======================================
// OS 过采样控制引脚
#define AD7606OS0_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET)
#define AD7606OS0_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)
#define AD7606OS1_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET)
#define AD7606OS1_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET)
#define AD7606OS2_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)
#define AD7606OS2_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET)
// 转换启动引脚
#define AD7606_CONVST_A_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET)
#define AD7606_CONVST_A_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET)
#define AD7606_CONVST_B_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET)
#define AD7606_CONVST_B_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET)
//
// #define AD7606_SCLK_H HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
// #define AD7606_SCLK_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
// 置位标志位
#define AD7606_RESET_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET)
#define AD7606_RESET_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET)
// 数字片选
#define AD7606_CS_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET)
#define AD7606_CS_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET)
//
// #define AD7606_BUSY HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)  // GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)
// #define AD7606_DOUTA HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) // GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)
// #define AD7606_DOUTB HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) // GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)

//=======================================
// 2.AD7606驱动函数
//=======================================
void AD7606_Init(void);
void AD7606_RESET(void);
void AD7606_SETOS(uint8_t osv);

//=======================================
// 3. 外部变量声明
//=======================================
#define FFT_POINT_NUM 1024
// extern volatile uint8_t FFT_Data_Ready;
extern volatile int16_t AD7606_Channel_Data[3]; // AD7606 采样值数组
extern volatile uint8_t AD7606_Data_Ready;      // AD7606 数据处理标志位
// extern int16_t Sample_CH1_Buffer[FFT_POINT_NUM];
// extern int16_t Sample_CH2_Buffer[FFT_POINT_NUM];
// extern int16_t Sample_CH3_Buffer[FFT_POINT_NUM];

extern volatile uint32_t exti_cnt;
extern volatile uint32_t dma_cnt;
#endif /* __BSP_AD7606_H */
