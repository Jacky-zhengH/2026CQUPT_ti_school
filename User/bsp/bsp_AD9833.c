#include "bsp_AD9833.h"

#include "header.h"
//***************************
//		Pin assign
//			STM32			AD9833
//		GPIOB_Pin_15 		---> FSYNC
//		GPIOB_Pin_14 		---> SCK
//		GPIOB_Pin_13 		---> DAT
//		GPIOB_Pin_12		---> CS
//***************************

/*端口宏定义 */
#define PORT_FSYNC GPIOB
#define PIN_FSYNC GPIO_PIN_15

#define PORT_SCK GPIOB
#define PIN_SCK GPIO_PIN_14

#define PORT_DAT GPIOB
#define PIN_DAT GPIO_PIN_13

#define PORT_CS GPIOB
#define PIN_CS GPIO_PIN_12 //
//****************************************************************

#define FSYNC_0() HAL_GPIO_WritePin(PORT_FSYNC, PIN_FSYNC, GPIO_PIN_RESET)
#define FSYNC_1() HAL_GPIO_WritePin(PORT_FSYNC, PIN_FSYNC, GPIO_PIN_SET)

#define SCK_0() HAL_GPIO_WritePin(PORT_SCK, PIN_SCK, GPIO_PIN_RESET)
#define SCK_1() HAL_GPIO_WritePin(PORT_SCK, PIN_SCK, GPIO_PIN_SET)

#define DAT_0() HAL_GPIO_WritePin(PORT_DAT, PIN_DAT, GPIO_PIN_RESET)
#define DAT_1() HAL_GPIO_WritePin(PORT_DAT, PIN_DAT, GPIO_PIN_SET)

#define CS_0() HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_RESET)
#define CS_1() HAL_GPIO_WritePin(PORT_CS, PIN_CS, GPIO_PIN_SET)

//****************************************************************

/*
*********************************************************************************************************
*	函数名称: AD9833_Delay
*	����˵��: ʱ����ʱ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AD9833_Delay(void)
{
    uint16_t i;
    for (i = 0; i < 1; i++)
        ;
}

/*
*********************************************************************************************************
*	�� �� ��: AD9833_Write
*	����˵��: ��SPI���߷���16��bit����
*	��    ��: TxData : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD9833_Write(unsigned int TxData)
{
    unsigned char i;

    SCK_1();
    // AD9833_Delay();
    FSYNC_1();
    // AD9833_Delay();
    FSYNC_0();
    // AD9833_Delay();
    for (i = 0; i < 16; i++)
    {
        if (TxData & 0x8000)
            DAT_1();
        else
            DAT_0();

        AD9833_Delay();
        SCK_0();
        AD9833_Delay();
        SCK_1();

        TxData <<= 1;
    }
    FSYNC_1();
}

/*
*********************************************************************************************************
*	�� �� ��: AD9833_AmpSet
*	����˵��: �ı�����źŷ���ֵ
*	��    ��: 1.amp ������ֵ  0- 255
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void AD9833_AmpSet(unsigned char amp)
{
    unsigned char i;
    unsigned int temp;

    CS_0();
    temp = 0x1100 | amp;
    for (i = 0; i < 16; i++)
    {
        SCK_0();
        if (temp & 0x8000)
            DAT_1();
        else
            DAT_0();
        temp <<= 1;
        SCK_1();
        AD9833_Delay();
    }

    CS_1();
}

/*
*********************************************************************************************************
*	�� �� ��: AD9833_WaveSeting
*	����˵��: ��SPI���߷���16��bit����
*	��    ��: 1.Freq: Ƶ��ֵ, 0.1 hz - 12Mhz
              2.Freq_SFR: 0 �� 1
              3.WaveMode: TRI_WAVE(���ǲ�),SIN_WAVE(���Ҳ�),SQU_WAVE(����)
              4.Phase : ���εĳ���λ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD9833_WaveSeting(double Freq, unsigned int Freq_SFR, unsigned int WaveMode, unsigned int Phase)
{

    int frequence_LSB, frequence_MSB, Phs_data;
    double frequence_mid, frequence_DATA;
    long int frequence_hex;

    /*********************************����Ƶ�ʵ�16����ֵ***********************************/
    frequence_mid = 268435456 / 25; // �ʺ�25M����
    // ���ʱ��Ƶ�ʲ�Ϊ25MHZ���޸ĸô���Ƶ��ֵ����λMHz ��AD9833���֧��25MHz
    frequence_DATA = Freq;
    frequence_DATA = frequence_DATA / 1000000;
    frequence_DATA = frequence_DATA * frequence_mid;
    frequence_hex = frequence_DATA;         // ���frequence_hex��ֵ��32λ��һ���ܴ�����֣���Ҫ��ֳ�����14λ���д�����
    frequence_LSB = frequence_hex;          // frequence_hex��16λ�͸�frequence_LSB
    frequence_LSB = frequence_LSB & 0x3fff; // ȥ�������λ��16λ����ȥ����λ������14λ
    frequence_MSB = frequence_hex >> 14;    // frequence_hex��16λ�͸�frequence_HSB
    frequence_MSB = frequence_MSB & 0x3fff; // ȥ�������λ��16λ����ȥ����λ������14λ

    Phs_data = Phase | 0xC000; // ��λֵ
    AD9833_Write(0x0100);      // ��λAD9833,��RESETλΪ1
    AD9833_Write(0x2100);      // ѡ������һ��д�룬B28λ��RESETλΪ1

    if (Freq_SFR == 0) // ���������õ�����Ƶ�ʼĴ���0
    {
        frequence_LSB = frequence_LSB | 0x4000;
        frequence_MSB = frequence_MSB | 0x4000;
        // ʹ��Ƶ�ʼĴ���0�������
        AD9833_Write(frequence_LSB); // L14��ѡ��Ƶ�ʼĴ���0�ĵ�14λ��������
        AD9833_Write(frequence_MSB); // H14 Ƶ�ʼĴ����ĸ�14λ��������
        AD9833_Write(Phs_data);      // ������λ
        // AD9833_Write(0x2000); /**����FSELECTλΪ0��оƬ���빤��״̬,Ƶ�ʼĴ���0�������**/
    }
    if (Freq_SFR == 1) // ���������õ�����Ƶ�ʼĴ���1
    {
        frequence_LSB = frequence_LSB | 0x8000;
        frequence_MSB = frequence_MSB | 0x8000;
        // ʹ��Ƶ�ʼĴ���1�������
        AD9833_Write(frequence_LSB); // L14��ѡ��Ƶ�ʼĴ���1�ĵ�14λ����
        AD9833_Write(frequence_MSB); // H14 Ƶ�ʼĴ���1Ϊ
        AD9833_Write(Phs_data);      // ������λ
        // AD9833_Write(0x2800); /**����FSELECTλΪ0������FSELECTλΪ1����ʹ��Ƶ�ʼĴ���1��ֵ��оƬ���빤��״̬,Ƶ�ʼĴ���1�������**/
    }

    if (WaveMode == TRI_WAVE) // ������ǲ�����
        AD9833_Write(0x2002);
    if (WaveMode == SQU_WAVE) // �����������
        AD9833_Write(0x2028);
    if (WaveMode == SIN_WAVE) // ������Ҳ���
        AD9833_Write(0x2000);
}
