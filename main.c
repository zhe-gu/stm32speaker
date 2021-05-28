#include <stm32f10x.h>

extern uc32 music1[15000];
extern uc32 music2[15000];
extern uc32 music3[15000];
extern uc32 music4[15000];
extern uc32 music5[6840];

void DMA_Config(uint32_t musicADD,u16 Size);

void DMA2_Channel3_IRQHandler(void)
{
	static uint32_t ADD[5] = {(uint32_t)music1,(uint32_t)music2,(uint32_t)music3,(uint32_t)music4,(uint32_t)music5};
	static u16 NUM[5] = {15000,15000,15000,15000,5100};
	u8 flag = 1;
  if(DMA_GetITStatus(DMA2_IT_TC3))
  {
		DMA_ClearITPendingBit(DMA2_IT_GL3);
    DMA_ClearFlag(DMA2_FLAG_TC3);//清除通道1的传输完成标志位
		DMA_Cmd(DMA2_Channel3,DISABLE);
		DMA_Config(ADD[flag],NUM[flag]*4);
		DMA_Cmd(DMA2_Channel3,ENABLE);
		flag++;
		if(flag >= 5)
		{
			flag = 0;
		}
  }
}

/****************初始化引脚******************/
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //开时钟
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;       //模拟输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //输出速率 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ; //选择引脚
    GPIO_SetBits(GPIOA,GPIO_Pin_4);   //拉高输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);      //初始化
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  //开时钟
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_All;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;					//推挽输出
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
		GPIO_Init(GPIOE,&GPIO_InitStructure);				//对LED的GPIOE口初始化
		GPIO_SetBits(GPIOE,GPIO_Pin_All);	//关闭所有的LED
}

/******************DAC初始化ˉ*************************/
void DAC_Config(void)
{
    DAC_InitTypeDef            DAC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//开DAC时钟

  /**************DAC结构初始化*******************/
    DAC_StructInit(&DAC_InitStructure);    
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//不产生波形
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable; //不使能输出缓存
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;//DAC触发为定时器2触发
    DAC_Init(DAC_Channel_1, &DAC_InitStructure);//初始化
    DAC_Cmd(DAC_Channel_1, ENABLE);    //使能DAC的通道1
    DAC_DMACmd(DAC_Channel_1, ENABLE); //使能DAC通道1的DMA
}

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;     /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);     
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;     
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          
  NVIC_Init(&NVIC_InitStructure);
}

/*********定时器初始化************/
void TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//开时钟
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;     //不预分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; //不分频
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
    TIM_TimeBaseStructure.TIM_Period = 7199;//设置输出频率10000Hz
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);//设置TIME输出触发为更新模式
}

/*********DMA配置***********/
void DMA_Config(uint32_t musicADD,u16 Size)
{
    DMA_InitTypeDef            DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);//开启DMA2时钟
		DMA_DeInit(DMA2_Channel3);
	
    DMA_StructInit(&DMA_InitStructure);        //DMA结构体初始化
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//从寄存器读数据
    DMA_InitStructure.DMA_BufferSize = Size;//寄存器大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//宽度为字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//宽度为字节
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//优先级非常高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//关闭内存到内存模式
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//循环发送模式DMA_Mode_Circular;

		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&DAC->DHR8R1;//外设地址为DAC通道1的地址
		DMA_InitStructure.DMA_MemoryBaseAddr = musicADD;//波形数据表内存地址
		DMA_Init(DMA2_Channel3, &DMA_InitStructure);//初始化
	
		DMA_ClearITPendingBit(DMA2_IT_TC3);
    DMA_ITConfig(DMA2_Channel3,DMA_IT_TC,ENABLE);
}

void SYSTICK_Init(void)
{
	RCC_HCLKConfig(RCC_SYSCLK_Div1);	//外部时钟源HCLK也为72MHz
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//SysTick选择为外部时钟HCLK/8
}

void Init(void)
{
    GPIO_Config();             //初始化io
		SYSTICK_Init();
    TIM_Config();            //初始化定时器
    DAC_Config();              //配置DAC
    DMA_Config((uint32_t)music1,15000*4);              //配置DMA
		NVIC_Configuration();
}

void tick_ms(u16 nms)
{	 		  	  
	u32 temp;
	SysTick->LOAD = (u32)nms*9000; 				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL = 0x00; 							//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 	//开始倒数
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp&(1 << 16)))
		;		//等待时间到达
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 	//关闭计数器
	SysTick->VAL = 0X00;        					//清空计数器
}

int main(void)
{
	Init();
	DMA_Cmd(DMA2_Channel3, ENABLE); //使能DMA通道3
	TIM_Cmd(TIM2, ENABLE);        //开启定时器
	//printf("Welcome to Miscellaneous Music Player\n");
	//printf("Input Z to play the piano\n");
	//printf("Enter x into the pomfret state\n");
	while(1)
	{
		u8 i = 0;
		i = DMA_GetCurrDataCounter(DMA2_Channel3)/4000 + 1;
		GPIO_Write(GPIOE,~((1<<i)-1));
		tick_ms(100);
	}
}
