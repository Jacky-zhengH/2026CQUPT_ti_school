#include "app_process.h"
#include "header.h"
#include "bsp_AD7606.h"

#include "header.h"
//*********************************************************************************************************
extern UART_HandleTypeDef huart1;               // HMI 控制串口
extern UART_HandleTypeDef huart3;               // printf 调试串口
extern volatile int16_t AD7606_Channel_Data[3]; // AD7606 采样值数组
extern volatile uint8_t AD7606_Data_Ready;      // AD7606 数据处理标志位
//*********************************************************************************************************
static uint8_t hmi_rx_buffer[50]; // HMI 接收缓冲区
static char debug_buffer[128];    // 电脑串口调试 接收缓冲区
// static uint8_t hmi_cmd_flag = 0;  // 新指令标志位 (0 = false)
// static uint16_t hmi_cmd_size = 0; // 新指令长度
//*********************************************************************************************************
#define AD7606_VOLTAGE_LSB (5.0f / 32768.0f); // 电压转换参数
//*********************************************************************************************************
/**
 * @name    HMI_Process_Init()
 * @brief   启动HMI的处理逻辑
 * @note    启动第一次DMA接收
 * @param   无
 */
void HMI_Process_Init(void)
{
    // 启动HMI串口(USART1)的空闲中断DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, hmi_rx_buffer, sizeof(hmi_rx_buffer));
}

/**
 * @name    HMI_Send_Cmd()
 * @brief   向 HMI 串口屏 (USART1) 发送原始指令 (带 0xFF 结尾)
 * @note    可以用于比如切换页面（page pagenam）
 * @param   *cmd_string：指令（类型为char*）
 */
void HMI_Send_Cmd(const char *cmd_string)
{
    char cmd_buffer[100]; // 缓冲区
    int len = snprintf(cmd_buffer, sizeof(cmd_buffer), "%s\xff\xff\xff", cmd_string);

    // 使用阻塞式发送
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd_buffer, len, HAL_MAX_DELAY);
}

/**
 * @name    Debug_prinf()
 * @brief   自定义串口发送文本
 * @note    用于向电脑发送串口信息，用于调试
 * @param   *text:文本
 * @param   ...:可变变量
 */
void Debug_printf(const char *text, ...)
{
    va_list args;
    va_start(args, text);
    int len = vsnprintf(debug_buffer, sizeof(debug_buffer), text, args);
    va_end(args);
    if (len > 0)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)debug_buffer, len, 100);
    }
}

void Task_Debug_Sample_value(void)
{
    if (AD7606_Data_Ready == 1) // 数据转换-接收完毕（标志位待添加）
    {
        /**
         * 数据处理
         * 比如将采样值转成数据值
         */
        float v_in = (float)AD7606_Channel_Data[0] * AD7606_VOLTAGE_LSB;  // ch1
        float v_s = (float)AD7606_Channel_Data[1] * AD7606_VOLTAGE_LSB;   // ch2
        float v_out = (float)AD7606_Channel_Data[2] * AD7606_VOLTAGE_LSB; // ch3

        Debug_printf("[ADC Raw] CH1:%6d | CH2:%6d | CH3:%6d\r\n",
                     AD7606_Channel_Data[0], AD7606_Channel_Data[1], AD7606_Channel_Data[2]);
        Debug_printf("[Voltage] Vin: %7.3f V | Vs: %7.3f V | Vout: %7.3f V\r\n",
                     v_in, v_s, v_out);
        Debug_printf("---------------------------------------------------\r\n");
        // 清除标志位，允许数据更新(待添加)
        AD7606_Data_Ready = 0;
    }
}

//*********************************************************************************************************
// void Task_DC_Measurement_And_Debug(void)
// {
//     // 如果底层 DMA 已经准备好了一组新的 DC 数据
//     if (AD7606_Data_Ready == 1)
//     {
//         // 1. 及时清除标志位，允许底层继续更新数据
//         AD7606_Data_Ready = 0;

//         // 2. 将补码转化为实际电压浮点数 (单位：V)
//         float v_in = (float)AD7606_Channel_Data[0] * AD7606_VOLTAGE_LSB;  // CH1
//         float v_s = (float)AD7606_Channel_Data[1] * AD7606_VOLTAGE_LSB;   // CH2
//         float v_out = (float)AD7606_Channel_Data[2] * AD7606_VOLTAGE_LSB; // CH3

//         // 3. 计算电路特性参数 (根据你的拓扑图逻辑)
//         // 注意：计算分母时要加极小值防除零错误，或者做个限幅判断
//         float R_in = 0.0f;
//         if ((v_s - v_in) > 0.001f || (v_s - v_in) < -0.001f)
//         {
//             // 输入电阻 = (Vin / (Vs - Vin)) * 1kΩ (你的新拓扑改成了1k分压电阻)
//             R_in = (v_in / (v_s - v_in)) * 1000.0f;
//         }

//         float Gain = 0.0f;
//         if (v_in > 0.001f || v_in < -0.001f)
//         {
//             Gain = v_out / v_in; // 直流电压增益
//         }

//         // 4. 调用你的 Debug 串口打印函数输出日志
//         // 这里会以定时器触发的频率 (比如 10Hz) 实时在串口助手打印
//         Debug_printf("[ADC Raw] CH1:%6d | CH2:%6d | CH3:%6d\r\n",
//                      AD7606_Channel_Data[0], AD7606_Channel_Data[1], AD7606_Channel_Data[2]);

//         Debug_printf("[Voltage] Vin: %7.3f V | Vs: %7.3f V | Vout: %7.3f V\r\n",
//                      v_in, v_s, v_out);

//         Debug_printf("[Circuit] Rin: %8.1f Ω | DC Gain: %5.2f\r\n",
//                      R_in, Gain);

//         Debug_printf("---------------------------------------------------\r\n");

//         // 5. 这里可以把算好的 v_in, v_out, R_in 等数据发给串口屏去显示
//         // Send_To_Screen(v_in, v_out, R_in, ...);
//     }
// }