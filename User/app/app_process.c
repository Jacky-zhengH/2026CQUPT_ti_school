#include "app_process.h"
#include "header.h"
#include "bsp_AD7606.h"
#include "alog.h"
#include "header.h"
//*********************************************************************************************************
extern UART_HandleTypeDef huart1;               // HMI 控制串口
extern UART_HandleTypeDef huart3;               // printf 调试串口
extern volatile int16_t AD7606_Channel_Data[3]; // AD7606 采样值数组
extern volatile uint8_t AD7606_Data_Ready;      // AD7606 数据处理标志位
//*********************************************************************************************************
static uint8_t hmi_rx_buffer[50]; // HMI 接收缓冲区
static char debug_buffer[128];    // 电脑串口调试 接收缓冲区
static uint8_t hmi_cmd_flag = 0;  // 新指令标志位 (0 = false)
static uint16_t hmi_cmd_size = 0; // 新指令长度

//*********************************************************************************************************
float global_v1 = 0.0f, global_v2 = 0.0f, global_v3 = 0.0f;      // 实时采集的三路电压
float global_gain = 0.0f, global_rin = 0.0f, global_rout = 0.0f; // 第二问测量参数

#define AD7606_VOLTAGE_LSB (10.0f / 32768.0f) // 电压转换参数
// 继电器控制宏 (PE7)
#define RELAY_ON() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET)
#define RELAY_OFF() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET)
//*********************************************************************************************************
/**
 * @name    HMI_Process_Init()
 * @brief   启动HMI的处理逻辑
 * @note    启动第一次DMA接收
 * @param   无
 */
void HMI_Process_Init(void)
{
    RELAY_OFF();
    // 启动HMI串口(USART1)的空闲中断DMA接收
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, hmi_rx_buffer, sizeof(hmi_rx_buffer));
    HAL_UART_Receive_IT(&huart1, hmi_rx_buffer, 1);
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

// 封装：向串口屏特定文本控件发送浮点数
static void HMI_Update_FloatText(const char *obj_name, float value, const char *unit)
{
    char buf[64];
    // 陶晶驰/Nextion 格式: t0.txt="1.23V"
    snprintf(buf, sizeof(buf), "%s.txt=\"%.5f %s\"", obj_name, value, unit);
    HMI_Send_Cmd(buf);
}

// 封装：向串口屏特定文本控件发送字符串
static void HMI_Update_StringText(const char *obj_name, const char *str)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj_name, str);
    HMI_Send_Cmd(buf);
}
/**
 * @brief AD7606采集调试函数
 */
void Task_Debug_Sample_value(void)
{
    if (AD7606_Data_Ready == 1) // 数据转换-接收完毕（标志位待添加）
    {
        /**
         * 数据处理
         * 比如将采样值转成数据值
         */
        // 数据抓取
        int16_t raw_ch1 = AD7606_Channel_Data[0];
        int16_t raw_ch2 = AD7606_Channel_Data[1];
        int16_t raw_ch3 = AD7606_Channel_Data[2];

        // 清除标志位，允许数据更新(待添加)
        AD7606_Data_Ready = 0;

        float v_in = (float)raw_ch1 * AD7606_VOLTAGE_LSB;  // ch1
        float v_s = (float)raw_ch2 * AD7606_VOLTAGE_LSB;   // ch2
        float v_out = (float)raw_ch3 * AD7606_VOLTAGE_LSB; // ch3

        static uint16_t print_cnt = 0;
        if (++print_cnt >= 10000)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            print_cnt = 0;
            Debug_printf("[ADC Raw] CH1:%6d | CH2:%6d | CH3:%6d\r\n",
                         raw_ch1, raw_ch2, raw_ch3);
            Debug_printf("[Voltage] Vin: %7.3f V | Vs: %7.3f V | Vout: %7.3f V\r\n",
                         v_in, v_s, v_out);
            Debug_printf("---------------------------------------------------\r\n");
        }
    }
}

#define FILTER_N 20 // 滑动滤波窗口大小，越大越平滑，但响应越慢

/**
 * @name   Task_ADC_Data_Update
 * @brief  任务一：实时抓取底层ADC数据并更新全局电压变量
 */
static void Task_ADC_Data_Update(void)
{
    static float buf_v1[FILTER_N], buf_v2[FILTER_N], buf_v3[FILTER_N];
    static uint8_t filter_idx = 0;
    static uint8_t filter_cnt = 0;
    if (AD7606_Data_Ready == 1)
    {
        // 抓取并转换数据
        // global_v1 = (float)AD7606_Channel_Data[0] * AD7606_VOLTAGE_LSB;
        // global_v2 = (float)AD7606_Channel_Data[1] * AD7606_VOLTAGE_LSB;
        // global_v3 = (float)AD7606_Channel_Data[2] * AD7606_VOLTAGE_LSB;

        buf_v1[filter_idx] = fabs((float)AD7606_Channel_Data[0] * AD7606_VOLTAGE_LSB);
        buf_v2[filter_idx] = fabs((float)AD7606_Channel_Data[1] * AD7606_VOLTAGE_LSB);
        buf_v3[filter_idx] = fabs((float)AD7606_Channel_Data[2] * AD7606_VOLTAGE_LSB);

        filter_idx = (filter_idx + 1) % FILTER_N;
        if (filter_cnt < FILTER_N)
            filter_cnt++;

        // 2. 计算滑动平均值
        float sum1 = 0, sum2 = 0, sum3 = 0;
        for (int i = 0; i < filter_cnt; i++)
        {
            sum1 += buf_v1[i];
            sum2 += buf_v2[i];
            sum3 += buf_v3[i];
        }

        // 3. 输出极度平滑的最终电压
        global_v1 = sum1 / filter_cnt;
        global_v2 = sum2 / filter_cnt;
        global_v3 = sum3 / filter_cnt;

        AD7606_Data_Ready = 0; // 释放标志位，允许底层继续采集
    }
}

static void Task_Measure_StateMachine(void)
{
    typedef enum
    {
        STATE_OPEN = 0,
        STATE_RELAY_WAIT,
        STATE_LOAD,
        STATE_IDLE_DELAY
    } MeasState_t;

    static MeasState_t current_state = STATE_OPEN;
    static uint32_t state_timer = 0;
    static float v3_open_temp = 0.0f;

    switch (current_state)
    {
    case STATE_OPEN:
        v3_open_temp = global_v3;
        // 调用 algo 层：计算空载增益和输入电阻
        global_gain = Algo_Measure_Gain(global_v2, v3_open_temp);
        global_rin = Algo_Measure_Rin(global_v1, global_v2);

        // 闭合继电器，准备测输出电阻
        RELAY_ON();
        state_timer = HAL_GetTick();
        current_state = STATE_RELAY_WAIT;
        break;

    case STATE_RELAY_WAIT:
        if (HAL_GetTick() - state_timer > 200) // 等 50ms 机械稳定
        {
            current_state = STATE_LOAD;
        }
        break;

    case STATE_LOAD:
        // 调用 algo 层：计算输出电阻
        global_rout = Algo_Measure_Rout(v3_open_temp, global_v3);

        RELAY_OFF(); // 立马断开，保护硬件

        state_timer = HAL_GetTick();
        current_state = STATE_IDLE_DELAY;
        break;

    case STATE_IDLE_DELAY:
        if (HAL_GetTick() - state_timer > 1500) // 休息 1.5 秒再开启下一轮测算
        {
            current_state = STATE_OPEN;
        }
        break;
    }
}

/**
 * @name   Task_HMI_Display_Update
 * @brief  任务三：以固定频率刷新 HMI 屏幕显示 (避免频繁通信卡死串口)
 */
static void Task_HMI_Display_Update(void)
{
    static uint32_t display_timer = 0;

    // 每 200 毫秒刷新一次屏幕 (人眼看着很流畅，且节约资源)
    if (HAL_GetTick() - display_timer > 200)
    {
        display_timer = HAL_GetTick();

        // 刷新实时电压 (对应屏幕 t3, t4, t5)
        HMI_Update_FloatText("t3", Algo_Round_3(global_v1), "V");
        HMI_Update_FloatText("t4", Algo_Round_3(global_v2), "V");
        HMI_Update_FloatText("t5", Algo_Round_3(global_v3), "V");

        Debug_printf("[Voltage] Vin: %7.3f V | Vs: %7.3f V | Vout: %7.3f V\r\n",
                     global_v1, global_v2, global_v3);
        // 刷新第二问参数 (对应屏幕 t10, t11, t12)
        HMI_Update_FloatText("t10", global_gain, "");
        Debug_printf("[second] Gain: %7.3f ", global_gain);
    }
}

static void Task_HMI_Command_Process(void)
{
    Debug_printf("flag = %d\r\n", hmi_cmd_flag);
    if (hmi_cmd_flag == 1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 指示灯闪烁
        Debug_printf("=> RX Trig! Size:%d, Data[0]:%c (Hex:%02X)\r\n",
                     hmi_cmd_size, hmi_rx_buffer[0], hmi_rx_buffer[0]);

        // if (hmi_rx_buffer[0] == 'A') // 诉求：按下测量按钮，刷新第二问 HMI
        // {
        //     Debug_printf("=> Button A Pressed: Update Display\r\n");

        //     // 将后台算好的平滑电压上屏
        //     HMI_Update_FloatText("t3", global_v1, "V");
        //     HMI_Update_FloatText("t4", global_v2, "V");
        //     HMI_Update_FloatText("t5", global_v3, "V");

        //     // 将第二问参数上屏
        //     HMI_Update_FloatText("t10", global_gain, "");
        //     HMI_Update_FloatText("t11", global_rin, "R");
        //     HMI_Update_FloatText("t12", global_rout, "R");

        //     // 顺便在电脑串口打印，方便你们排查
        //     Debug_printf("[Meas] Gain:%.2f | Rin:%.1f | Rout:%.1f\r\n", global_gain, global_rin, global_rout);
        // }
        if (hmi_rx_buffer[0] == 'E')
        {
            HMI_Update_StringText("t9", "wait...");
            uint8_t fault_code = Algo_Diagnosis_Fault(global_gain, global_rin, global_rout, global_v3);

            switch (fault_code)
            {
            case 0:
                HMI_Update_StringText("t9", "normal");
                break;
            case 1:
                HMI_Update_StringText("t9", "R1");
                break;
            case 2:
                HMI_Update_StringText("t9", "R2");
                break;
            case 3:
                HMI_Update_StringText("t9", "R3");
                break;
            case 4:
                HMI_Update_StringText("t9", "R4");
                break;
            case 5:
                HMI_Update_StringText("t9", "R5");
                break;
            case 6:
                HMI_Update_StringText("t9", "RL");
                break;
            default:
                HMI_Update_StringText("t9", "UNknown");
                break;
            }
        }
        else if (hmi_rx_buffer[0] == 'B')
        {
            Algo_Set_Baseline(global_gain, global_rin, global_rout);
            HMI_Update_StringText("t9", "base");
        }
        else if (hmi_rx_buffer[0] == 'A')
        {
            HMI_Update_FloatText("t11", global_rin, "R");
            HMI_Update_FloatText("t12", global_rout, "R");
            Debug_printf("[third] 2: %7.3f V | 3: %7.3f V\r\n", global_rin, global_rout);
        }
        hmi_cmd_flag = 0;
    }
}
//=========================================================================================================
// 3. 主轮询整合与中断回调
//=========================================================================================================

/**
 * @name   App_Main_Process_Poll
 * @brief  放置于 main.c 的 while(1) 中，作为大管家统筹调度所有任务
 */
void App_Main_Process_Poll(void)
{
    // HMI_Update_FloatText("t3",0.0f,"V");
    Task_ADC_Data_Update();      // 极速更新实时数据
    Task_Measure_StateMachine(); // 维持继电器状态机运转
    Task_HMI_Display_Update();   // 周期性刷新屏幕 UI
    Task_HMI_Command_Process();  // 随时响应用户触摸指令
}

//*********************************************************************************************************
/**
 * @brief 重新定义USART中断回调函数
 */
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     if (huart->Instance == USART1) // 检测目前串口是否是USART1，即串口屏
//     {
//         if (hmi_cmd_flag == 0) // 上一轮的接收是否已经完成（完成后flag会置零）
//         {
//             hmi_cmd_flag = 1;    // 重新开始接收下一轮指令（flag重新置一，说明正在接收）
//             hmi_cmd_size = Size; // 保存指令长度（用于解析指令）
//         }
//         //
//         HAL_UARTEx_ReceiveToIdle_DMA(huart, hmi_rx_buffer, sizeof(hmi_rx_buffer));
//     }
// }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 因为只收1个字节，所以只要进中断，必定收到了新指令
        hmi_cmd_flag = 1;

        // 重新开启下一次 1字节 中断接收
        HAL_UART_Receive_IT(huart, hmi_rx_buffer, 1);
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