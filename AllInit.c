#include "stm32f10x.h"

uc8 music[4] = {0,255,0,255};
//volatile vu16 CurrDataCounterEnd = 0;

void RCC_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);	//使能GPIOE时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子72/6=12MHz，最大不能超过14MHz
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	//使能DAC时钟
	
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	//使能TIM2外设
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
}

void DMA_Config(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	DMA_DeInit(DMA2_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&DAC->DHR8R1;//DAC_DHR8R1_ADDR;//(uint32_t)DAC_BASE + 8 + 8;//(u32)&ADC1->DR;//((u32)0x40012400+0x4c);
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)music;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	//外设为数据目的地
	DMA_InitStructure.DMA_BufferSize = 4;	//完整一轮DMA传输的次数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//内存地址递增

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
	//配置输入脚PA0控制按键
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//SPEED:50MHz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //调用固件库中的GPIO_EXTILineConfig函数，
  //其中两个参数分别是中断口和中断口对应的引脚号
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  //将中断映射到中断/事件源Line0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  //设置为下降沿中断
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  //中断使能，即开中断
  EXTI_Init(&EXTI_InitStructure);	
  //调用EXTI_Init固件库函数，将结构体写入EXTI相关寄存器中
	EXTI_ClearITPendingBit(EXTI_Line0);
}
*/

/*
void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	//PA1 模拟输入模式                      
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_DeInit(ADC1);  //复位ADC1

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;							//AD工作模式 独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;										//多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							//连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;							//数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 3;													//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);																//初始化
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_239Cycles5);	//※※※※ADC1的Ch16就是温度传感器,采样周期239.5周期※※※※
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 3, ADC_SampleTime_239Cycles5); //※※※※ADC1的Ch17就是内部参考电压※※※※
	ADC_TempSensorVrefintCmd(ENABLE); //※※※※开启内部温度传感器※※※※
	
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);										//使能ADC1
	
	ADC_ResetCalibration(ADC1);									//使能复位校准
	while (ADC_GetResetCalibrationStatus(ADC1));		//等待复位校准结束
	ADC_StartCalibration(ADC1);									 //开启AD校准
	while (ADC_GetCalibrationStatus(ADC1));				 //等待校准结束
 
	//ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);	//中断使能
}
*/

void DAC_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//PA4作为DAC1模拟通道DAC1_OUT1输出引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	//模拟输入引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	DAC_InitTypeDef DAC_InitStructure; //初始化结构体变量
	DAC_DeInit();  			//复位DAC寄存器
	//DAC_InitStructure.DAC_Trigger=DAC_Trigger_None;	//不需要触发
	DAC_InitStructure.DAC_Trigger=DAC_Trigger_Software;	//软件触发
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None;   //不用波形发生
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Disable;	//不缓冲
	DAC_Init(DAC_Channel_1,&DAC_InitStructure); //初始化通道1
	
	DAC_DMACmd(DAC_Channel_1, ENABLE); //使能DAC通道1的DMA
	DAC_Cmd(DAC_Channel_1, ENABLE);  //使能通道1
	
	DAC_SetChannel1Data(DAC_Align_8b_R,255);
}

void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	//初始化变量

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;					//推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);							//对LED的GPIOE口初始化

	GPIO_SetBits(GPIOE,GPIO_Pin_All);	//关闭所有的LED
}

void TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//定义TIM结构体变量
	
	TIM_DeInit(TIM2);//复位时钟TIM2，恢复到初始状态
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Period=9999;
	TIM_TimeBaseStructure.TIM_Prescaler=3599;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //TIM2时钟分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //计数方式
	//定时间隔为：T=(自动重装载寄存器的值+1)*(预分频值+1)/系统时钟频率
	//我们设置的proteus中系统时钟为4MHz(设置太高的话，计算机负荷占比太高，定时不准确)。此即为微控制器在proteus仿真运行时的系统频率。
	//所以这里设置9999和3599，我们时间间隔为0.5s。
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure); //初始化
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);   //清除标志
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //使能中断源
	
	TIM_Cmd(TIM2,ENABLE); //使能TIM2
}

void USART_Config(void)
{
	//GPIO设置，PA9--TX, PA10--RX
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
