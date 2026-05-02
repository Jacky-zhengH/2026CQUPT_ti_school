#include "bsp_AD7606.h"
#include "tim.h"
//=======================================
// 变量声明
//=======================================
volatile int16_t AD7606_Channel_Data[3] = {0}; // 1~3通道 数据缓存区
volatile uint8_t AD7606_Data_Ready = 0;        // 读取标志位
//*********************************************************************************************************

/**
 * @brief   微秒级/纳秒级延时 (软件阻塞)
 * @param   nCount 延时长度
 * @note    无
 */
static void Delay_u32(uint32_t nCount)
{
    while (nCount--)
    {
        __NOP();
    }
}

/**
 * @brief   转换启动函数
 * @param   无
 * @note    产生低电平触发一次同步转
 */
// void AD7606_STARTCONV(void)
// {
//     AD7606_CONVST_A_L;
//     AD7606_CONVST_B_L;
//     Delay_u32(50);
//     AD7606_CONVST_A_H;
//     AD7606_CONVST_A_H;
// }

/**
 * @brief   AD7606硬件复位函数
 * @param   无
 * @note    增加延时时间保证AD7606复位成功
 */
void AD7606_RESET(void)
{
    AD7606_RESET_L;
    Delay_u32(10); // 确保一开始是低
    AD7606_RESET_H;
    Delay_u32(100); // 维持高电平一段时间
    AD7606_RESET_L;
    Delay_u32(100); // 等待复位恢复
}

/**
 * @brief   设置过采样率
 * @param   osv 范围[0,6]
 * @note    ~过采样倍数[1,2,4,8,16,32,64]
 */
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

    default:
        AD7606OS0_L;
        AD7606OS1_L;
        AD7606OS2_L;
        break;
    }
}

/**
 * @brief   AD7606 初始化函数
 * @param   无
 * @note    拉高片选、SPI时钟、同步转换。硬件复位
 */
void AD7606_Init(void)
{
    // 引脚默认状态
    AD7606_CS_H;
    AD7606_SCLK_H;
    // AD7606_CONVST_A_H;
    // AD7606_CONVST_B_H;

    // 初始化复位，无过采样
    AD7606_SETOS(0);
    AD7606_RESET();

    // 启动CONV_A和CONV_B PWM触发
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief   AD7606单通道数据读取函数
 * @param   无
 * @note    读取单个通道的16位数据
 */
static int16_t AD7606_OneChanel_ReadBytes(void)
{
    uint16_t usData = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        AD7606_SCLK_L;
        Delay_u32(5); // 等待稳定
        usData = usData << 1;
        if (AD7606_DOUTA == GPIO_PIN_SET)
        {
            usData |= 0x0001;
        }
        AD7606_SCLK_H;
        Delay_u32(5);
    }
    return (int16_t)usData;
}

//*********************************************************************************************************
// GPIO EXTI 中断服务函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3)
    {
        AD7606_CS_L;

        AD7606_Channel_Data[0] = AD7606_OneChanel_ReadBytes();
        AD7606_Channel_Data[1] = AD7606_OneChanel_ReadBytes();
        AD7606_Channel_Data[2] = AD7606_OneChanel_ReadBytes();
        AD7606_CS_H; // 传输完毕拉高CS片选
        // 4. 标记数据准备完成
        AD7606_Data_Ready = 1;
    }
}
//*********************************************************************************************************
/**
 * @brief   多通道轮询函数
 * @param   无
 * @note    只读取前 3 个通道 (CH1, CH2, CH3)
 */
// void AD7606_Sample_Task(void)
// {
//     // 如果数据还没被处理完，防止覆盖
//     if (AD7606_Data_Ready == 1)
//         return;

//     // 1. 发送触发脉冲
//     AD7606_STARTCONV();

//     // 2. 等待转换完成 (BUSY从高变低)
//     uint32_t timeout = 10000;
//     while (AD7606_BUSY == GPIO_PIN_SET && timeout--)
//         ;
//     // 超时防止死机

//     if (timeout > 0)
//     {
//         // 3. 开始串行读取
//         AD7606_CS_L;

//         // 顺序读取 CH1, CH2, CH3
//         for (uint8_t i = 0; i < 3; i++)
//         {
//             AD7606_Channel_Data[i] = AD7606_OneChanel_ReadBytes();
//         }
//         AD7606_CS_H; // 传输完毕拉高CS片选
//         // 4. 标记数据准备完成
//         AD7606_Data_Ready = 1;
//     }
// }