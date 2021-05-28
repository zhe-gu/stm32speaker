#include "stm32f10x.h"

void RCC_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);	//ʹ��GPIOEʱ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����72/6=12MHz������ܳ���14MHz
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	//ʹ��TIM2����
	
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
}

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
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;										//��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;							//����ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//�������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;							//�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;													//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);																//��ʼ��
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);	//��������ADC1��Ch16�����¶ȴ�����,��������239.5���ڡ�������
	//ADC_TempSensorVrefintCmd(ENABLE); //�������������ڲ��¶ȴ�������������
	
	ADC_Cmd(ADC1, ENABLE);										//ʹ��ADC1
	
	ADC_ResetCalibration(ADC1);									//ʹ�ܸ�λУ׼
	while (ADC_GetResetCalibrationStatus(ADC1));		//�ȴ���λУ׼����
	ADC_StartCalibration(ADC1);									 //����ADУ׼
	while (ADC_GetCalibrationStatus(ADC1));				 //�ȴ�У׼����
 
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);	//�ж�ʹ��
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

/*
void TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//����TIM�ṹ�����
	
	TIM_DeInit(TIM2);//��λʱ��TIM2���ָ�����ʼ״̬
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Period=1999;
	TIM_TimeBaseStructure.TIM_Prescaler=3599;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //TIM2ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //������ʽ
	//��ʱ���Ϊ��T=(�Զ���װ�ؼĴ�����ֵ+1)*(Ԥ��Ƶֵ+1)/ϵͳʱ��Ƶ��
	//�������õ�proteus��ϵͳʱ��Ϊ4MHz(����̫�ߵĻ������������ռ��̫�ߣ���ʱ��׼ȷ)���˼�Ϊ΢��������proteus��������ʱ��ϵͳƵ�ʡ�
	//������������1999��3599������ʱ����Ϊ0.1s��
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
*/

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
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_DBGMCU
		
	#endif
	#ifdef RTE_DEVICE_STDPERIPH_DMA
		
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
