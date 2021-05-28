#include "stm32f10x.h"

uc8 music[4] = {0,255,0,255};
//volatile vu16 CurrDataCounterEnd = 0;

void RCC_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);	//ʹ��GPIOEʱ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����72/6=12MHz������ܳ���14MHz
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	//ʹ��DACʱ��
	
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	//ʹ��TIM2����
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
}

void DMA_Config(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	DMA_DeInit(DMA2_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&DAC->DHR8R1;//DAC_DHR8R1_ADDR;//(uint32_t)DAC_BASE + 8 + 8;//(u32)&ADC1->DR;//((u32)0x40012400+0x4c);
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)music;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	//����Ϊ����Ŀ�ĵ�
	DMA_InitStructure.DMA_BufferSize = 4;	//����һ��DMA����Ĵ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//�����ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�ڴ��ַ����

	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel3,&DMA_InitStructure);

	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	DMA_ClearITPendingBit(DMA2_IT_TC3);
	DMA_ITConfig(DMA2_Channel3,DMA2_IT_TC3,ENABLE);
	/* Get Current Data Counter value before transfer begins */
	//CurrDataCounterEnd = DMA_GetCurrDataCounter(DMA1_Channel1);
	/* Enable DMA1 Channel1 transfer */
	DMA_Cmd(DMA2_Channel3, ENABLE);
}

/*
void EXTI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//���������PA0���ư���
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//SPEED:50MHz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //���ù̼����е�GPIO_EXTILineConfig������
  //�������������ֱ����жϿں��жϿڶ�Ӧ�����ź�
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  //���ж�ӳ�䵽�ж�/�¼�ԴLine0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //�ж�ģʽ
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  //����Ϊ�½����ж�
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  //�ж�ʹ�ܣ������ж�
  EXTI_Init(&EXTI_InitStructure);	
  //����EXTI_Init�̼��⺯�������ṹ��д��EXTI��ؼĴ�����
	EXTI_ClearITPendingBit(EXTI_Line0);
}
*/

/*
void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	//PA1 ģ������ģʽ                      
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_DeInit(ADC1);  //��λADC1

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;							//AD����ģʽ ����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;										//��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							//����ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//�������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;							//�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 3;													//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);																//��ʼ��
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_239Cycles5);	//��������ADC1��Ch16�����¶ȴ�����,��������239.5���ڡ�������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 3, ADC_SampleTime_239Cycles5); //��������ADC1��Ch17�����ڲ��ο���ѹ��������
	ADC_TempSensorVrefintCmd(ENABLE); //�������������ڲ��¶ȴ�������������
	
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);										//ʹ��ADC1
	
	ADC_ResetCalibration(ADC1);									//ʹ�ܸ�λУ׼
	while (ADC_GetResetCalibrationStatus(ADC1));		//�ȴ���λУ׼����
	ADC_StartCalibration(ADC1);									 //����ADУ׼
	while (ADC_GetCalibrationStatus(ADC1));				 //�ȴ�У׼����
 
	//ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);	//�ж�ʹ��
}
*/

void DAC_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//PA4��ΪDAC1ģ��ͨ��DAC1_OUT1�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	//ģ����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	DAC_InitTypeDef DAC_InitStructure; //��ʼ���ṹ�����
	DAC_DeInit();  			//��λDAC�Ĵ���
	//DAC_InitStructure.DAC_Trigger=DAC_Trigger_None;	//����Ҫ����
	DAC_InitStructure.DAC_Trigger=DAC_Trigger_Software;	//�������
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None;   //���ò��η���
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Disable;	//������
	DAC_Init(DAC_Channel_1,&DAC_InitStructure); //��ʼ��ͨ��1
	
	DAC_DMACmd(DAC_Channel_1, ENABLE); //ʹ��DACͨ��1��DMA
	DAC_Cmd(DAC_Channel_1, ENABLE);  //ʹ��ͨ��1
	
	DAC_SetChannel1Data(DAC_Align_8b_R,255);
}

void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	//��ʼ������

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;					//�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);							//��LED��GPIOE�ڳ�ʼ��

	GPIO_SetBits(GPIOE,GPIO_Pin_All);	//�ر����е�LED
}

void TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//����TIM�ṹ�����
	
	TIM_DeInit(TIM2);//��λʱ��TIM2���ָ�����ʼ״̬
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Period=9999;
	TIM_TimeBaseStructure.TIM_Prescaler=3599;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //TIM2ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //������ʽ
	//��ʱ���Ϊ��T=(�Զ���װ�ؼĴ�����ֵ+1)*(Ԥ��Ƶֵ+1)/ϵͳʱ��Ƶ��
	//�������õ�proteus��ϵͳʱ��Ϊ4MHz(����̫�ߵĻ������������ռ��̫�ߣ���ʱ��׼ȷ)���˼�Ϊ΢��������proteus��������ʱ��ϵͳƵ�ʡ�
	//������������9999��3599������ʱ����Ϊ0.5s��
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure); //��ʼ��
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);   //�����־
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //ʹ���ж�Դ
	
	TIM_Cmd(TIM2,ENABLE); //ʹ��TIM2
}

void USART_Config(void)
{
	//GPIO���ã�PA9--TX, PA10--RX
	GPIO_InitTypeDef stGpioInitTypeDef;
	//TX
	stGpioInitTypeDef.GPIO_Pin = GPIO_Pin_9;
	stGpioInitTypeDef.GPIO_Speed = GPIO_Speed_50MHz;
	stGpioInitTypeDef.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &stGpioInitTypeDef);
	//RX
	stGpioInitTypeDef.GPIO_Pin = GPIO_Pin_10;
	stGpioInitTypeDef.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &stGpioInitTypeDef);
	
	USART_InitTypeDef stUSART_InitTypeStruct;
	stUSART_InitTypeStruct.USART_BaudRate = 9600;
	stUSART_InitTypeStruct.USART_WordLength = USART_WordLength_8b;
	stUSART_InitTypeStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	stUSART_InitTypeStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	stUSART_InitTypeStruct.USART_StopBits = USART_StopBits_1;
	stUSART_InitTypeStruct.USART_Parity = USART_Parity_No;
	USART_Init(USART1, &stUSART_InitTypeStruct);
	USART_Cmd(USART1,ENABLE);
}

void AllInit(void)
{
	#ifdef RTE_DEVICE_STDPERIPH_RCC
		RCC_Config();
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_ADC
		ADC_Config();
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_BKP
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_CAN
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_CEC
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_CRC
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_DAC
		DAC_Config();
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_DBGMCU
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_DMA
		DMA_Config();
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_EXTI
		EXTI_Config();
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_FLASH
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_FSMC
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_GPIO
		GPIO_Config();
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_I2C
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_IWDG
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_PWR
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_RTC
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_SDIO
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_SPI
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_TIM
		TIM_Config();
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_USART
		USART_Config();
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_WWDG
		
	#endif
}
