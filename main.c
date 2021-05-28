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
    DMA_ClearFlag(DMA2_FLAG_TC3);//���ͨ��1�Ĵ�����ɱ�־λ
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

/****************��ʼ������******************/
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //��ʱ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;       //ģ������ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //������� 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ; //ѡ������
    GPIO_SetBits(GPIOA,GPIO_Pin_4);   //�������
    GPIO_Init(GPIOA, &GPIO_InitStructure);      //��ʼ��
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  //��ʱ��
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_All;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;					//�������
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
		GPIO_Init(GPIOE,&GPIO_InitStructure);				//��LED��GPIOE�ڳ�ʼ��
		GPIO_SetBits(GPIOE,GPIO_Pin_All);	//�ر����е�LED
}

/******************DAC��ʼ����*************************/
void DAC_Config(void)
{
    DAC_InitTypeDef            DAC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//��DACʱ��

  /**************DAC�ṹ��ʼ��*******************/
    DAC_StructInit(&DAC_InitStructure);    
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//����������
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable; //��ʹ���������
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;//DAC����Ϊ��ʱ��2����
    DAC_Init(DAC_Channel_1, &DAC_InitStructure);//��ʼ��
    DAC_Cmd(DAC_Channel_1, ENABLE);    //ʹ��DAC��ͨ��1
    DAC_DMACmd(DAC_Channel_1, ENABLE); //ʹ��DACͨ��1��DMA
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

/*********��ʱ����ʼ��************/
void TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//��ʱ��
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;     //��Ԥ��Ƶ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; //����Ƶ
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
    TIM_TimeBaseStructure.TIM_Period = 7199;//�������Ƶ��10000Hz
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);//����TIME�������Ϊ����ģʽ
}

/*********DMA����***********/
void DMA_Config(uint32_t musicADD,u16 Size)
{
    DMA_InitTypeDef            DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);//����DMA2ʱ��
		DMA_DeInit(DMA2_Channel3);
	
    DMA_StructInit(&DMA_InitStructure);        //DMA�ṹ���ʼ��
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//�ӼĴ���������
    DMA_InitStructure.DMA_BufferSize = Size;//�Ĵ�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//���Ϊ�ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//���Ϊ�ֽ�
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//���ȼ��ǳ���
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//�ر��ڴ浽�ڴ�ģʽ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//ѭ������ģʽDMA_Mode_Circular;

		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&DAC->DHR8R1;//�����ַΪDACͨ��1�ĵ�ַ
		DMA_InitStructure.DMA_MemoryBaseAddr = musicADD;//�������ݱ��ڴ��ַ
		DMA_Init(DMA2_Channel3, &DMA_InitStructure);//��ʼ��
	
		DMA_ClearITPendingBit(DMA2_IT_TC3);
    DMA_ITConfig(DMA2_Channel3,DMA_IT_TC,ENABLE);
}

void SYSTICK_Init(void)
{
	RCC_HCLKConfig(RCC_SYSCLK_Div1);	//�ⲿʱ��ԴHCLKҲΪ72MHz
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//SysTickѡ��Ϊ�ⲿʱ��HCLK/8
}

void Init(void)
{
    GPIO_Config();             //��ʼ��io
		SYSTICK_Init();
    TIM_Config();            //��ʼ����ʱ��
    DAC_Config();              //����DAC
    DMA_Config((uint32_t)music1,15000*4);              //����DMA
		NVIC_Configuration();
}

void tick_ms(u16 nms)
{	 		  	  
	u32 temp;
	SysTick->LOAD = (u32)nms*9000; 				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL = 0x00; 							//��ռ�����
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 	//��ʼ����
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp&(1 << 16)))
		;		//�ȴ�ʱ�䵽��
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 	//�رռ�����
	SysTick->VAL = 0X00;        					//��ռ�����
}

int main(void)
{
	Init();
	DMA_Cmd(DMA2_Channel3, ENABLE); //ʹ��DMAͨ��3
	TIM_Cmd(TIM2, ENABLE);        //������ʱ��
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
