#include "bsp_AD7606.h"
#include "app_process.h"
#include "spi.h"

// 内部单次缓冲区，依次对应通道1到3
volatile int16_t AD7606_DMA_RxBuffer[3];

// // 存放FFT分析 三通道
// int16_t Sample_CH1_Buffer[FFT_POINT_NUM];
// int16_t Sample_CH2_Buffer[FFT_POINT_NUM];
// int16_t Sample_CH3_Buffer[FFT_POINT_NUM];

// 存放三路直流数组值
volatile int16_t AD7606_Channel_Data[3];
volatile uint8_t AD7606_Data_Ready = 0;
volatile uint8_t FFT_Data_Ready = 0;
volatile uint16_t sample_idx = 0;

// 调试探针
volatile uint32_t exti_cnt = 0;
volatile uint32_t dma_cnt = 0;
volatile uint8_t is_converting = 0;
//=======================================
// 驱动函数
//=======================================

/// @brief AD7606初始化函数
/// @param  null
void AD7606_Init(void)
{
    AD7606_CONVST_A_H;
    AD7606_CONVST_B_H;

    AD7606_CS_H;

    AD7606_RESET();
    AD7606_SETOS(0);
}

/// @brief 复位函数
/// @param  null
void AD7606_RESET(void)
{
    AD7606_RESET_H;
    for (volatile int i = 0; i < 1000; i++)
        ;
    AD7606_RESET_L;
}

/// @brief 设置过采样倍率
/// @param osv 过采样率设置：0~6（B）--> 1、2、4、8、16、32倍采样率
void AD7606_SETOS(uint8_t osv)
{
    switch (osv)
    {
    case 0: // 000
        AD7606OS0_L;
        AD7606OS1_L;
        AD7606OS2_L;
        break;
    case 1: // 001
        AD7606OS0_H;
        AD7606OS1_L;
        AD7606OS2_L;
        break;
    case 2: // 010
        AD7606OS0_L;
        AD7606OS1_H;
        AD7606OS2_L;
        break;
    case 3: // 011
        AD7606OS0_H;
        AD7606OS1_H;
        AD7606OS2_L;
        break;
    case 4: // 100
        AD7606OS0_L;
        AD7606OS1_L;
        AD7606OS2_H;
        break;
    case 5: // 101
        AD7606OS0_H;
        AD7606OS1_L;
        AD7606OS2_H;
        break;
    case 6: // 110
        AD7606OS0_L;
        AD7606OS1_H;
        AD7606OS2_H;
        break;
    }
}

//=======================================
// 中断服务
//=======================================
// 1.定时器触发采样转换

/// @brief FFT_Data_Ready位 定时器触发
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        static uint32_t led_cnt = 0;
        led_cnt++;
        // TIM3 是 10kHz，50000次就是 0.5秒翻转一次
        if (led_cnt >= 5000)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            led_cnt = 0;
        }
        // Debug_printf("TIM3 Fired!\r\n");
        //   if (FFT_Data_Ready == 1)
        //       return;
        if (AD7606_Data_Ready == 1)
            return;

        // 探针
        is_converting = 1;
        AD7606_CONVST_A_L;
        AD7606_CONVST_B_L;
        // 保证延时足够
        // __NOP();
        // __NOP();
        // __NOP();
        // __NOP();
        // __NOP();
        // __NOP();
        for (volatile int nop_delay = 0; nop_delay < 20; nop_delay++)
            ;
        AD7606_CONVST_A_H;
        AD7606_CONVST_B_H;
    }
}
// ---------------------------------------------------------
// 核心流水线 2：BUSY 下降沿外部中断，拉低 CS 并启动 DMA
// ---------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3) // PB3
    {
        if (is_converting == 0)
            return;
        is_converting = 0;
        if (AD7606_Data_Ready == 1)
            return; // 数据没读走，拒绝覆盖

        exti_cnt++;
        AD7606_CS_L;
        HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)AD7606_DMA_RxBuffer, 3);
    }
}

// ---------------------------------------------------------
// 核心流水线 3：DMA 接收完成回调，整理数据入列
// ---------------------------------------------------------
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        AD7606_CS_H;
        dma_cnt++;

        AD7606_Channel_Data[0] = AD7606_DMA_RxBuffer[0];
        AD7606_Channel_Data[1] = AD7606_DMA_RxBuffer[1];
        AD7606_Channel_Data[2] = AD7606_DMA_RxBuffer[2];

        AD7606_Data_Ready = 1;
    }
}