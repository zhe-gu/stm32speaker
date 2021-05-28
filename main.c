#include "stm32f10x.h"

void AllInit(void);

void NVIC_Configuration(void);
void HSE_SetSysClock(uint32_t pllmul);
void SYSTICK_Init(void);
void tick_us(u32 nus);
void tick_ms(u16 nms);

u8 flag = 0;

int main(void)
{
	u16 nums = 0;
	
	AllInit();
	SYSTICK_Init();
	NVIC_Configuration();
	
	while(1)
	{
		nums++;
		if(flag)
		{
			//GPIO_Write(GPIOE,64);
			ADC_SoftwareStartConvCmd(ADC1,ENABLE);			//����ת��
		} else {
			GPIO_Write(GPIOE,~nums);
		}
		tick_ms(500);
	}
}

void NVIC_Configuration(void)	 //Ƕ�������жϿ���������
{
  NVIC_InitTypeDef NVIC_InitStructure;  
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //ѡ�����ȼ����
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	 
  //ѡ���ж�ͨ����EXTI��0�жϣ���Ϊ�������ӵ���PA0��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
  //0����ռʽ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  //0�������ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //ʹ��������Ϊ�ж�Դ
  NVIC_Init(&NVIC_InitStructure);   //����NVIC_Init�̼��⺯����������
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;//ADC1��ADC2 �ж�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//�ض���fputc����,�빴ѡUse MicroLIB
/*
#include <stdio.h>
int fputc(int ch, FILE *fp)
{
	USART_SendData(USART1, (u8)ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET );
	return ch;
}
*/

void SYSTICK_Init(void)
{
	RCC_HCLKConfig(RCC_SYSCLK_Div1);	//�ⲿʱ��ԴHCLKҲΪ72MHz
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//SysTickѡ��Ϊ�ⲿʱ��HCLK/8
}

//��ʱn us		    								   
void tick_us(u32 nus)
{		
	u32 temp;
	SysTick->LOAD = nus*9;  					//ʱ�����
	SysTick->VAL = 0x00;         					//��ռ�����
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 	//��ʼ����
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp&(1 << 16)))
		;		//�ȴ�ʱ�䵽��   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 	//�رռ�����
	SysTick->VAL = 0X00;       					 //��ռ�����
}

//��ʱn ms
//SysTick->LOADΪ24λ�Ĵ���
//��72M�����£�nms<=1864
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

/*ʱ�������ú���
 * ʹ��HSEʱ������ϵͳʱ�ӵĲ���
 * 1������HSE �����ȴ� HSE �ȶ�
 * 2������ AHB��APB2��APB1��Ԥ��Ƶ����
 * 3������PLL��ʱ����Դ����PLL�ı�Ƶ���ӣ����ø���Ƶ����Ҫ��������������
 * 4������PLL�����ȴ�PLL�ȶ�
 * 5����PLLCK�л�Ϊϵͳʱ��SYSCLK
 * 6����ȡʱ���л�״̬λ��ȷ��PLLCLK��ѡΪϵͳʱ�� */

/* ���� ϵͳʱ��:SYSCLK, AHB����ʱ��:HCLK, APB2����ʱ��:PCLK2, APB1����ʱ��:PCLK1
 * PCLK2 = HCLK = SYSCLK
 * PCLK1 = HCLK/2,���ֻ����36M
 * ����˵����pllmul��PLL�ı�Ƶ���ӣ��ڵ��õ�ʱ������ǣ�RCC_PLLMul_x , x:[2,3,...16]
 * ������User_SetSysClock(RCC_PLLMul_9);  ������ϵͳʱ��Ϊ��8MHZ * 9 = 72MHZ
 *       User_SetSysClock(RCC_PLLMul_16); ������ϵͳʱ��Ϊ��8MHZ * 16 = 128MHZ����Ƶ����
 *
 * HSE��Ϊʱ����Դ������PLL��Ƶ��Ϊϵͳʱ�ӣ�����ͨ�������� */
/*
void HSE_SetSysClock(uint32_t pllmul)
{
	__IO uint32_t StartUpCounter = 0, HSEStartUpStatus = 0;
	// ��RCC�����ʼ���ɸ�λ״̬������Ǳ����
  RCC_DeInit();
  //ʹ��HSE�������ⲿ����һ�㿪�����õ���8M��Proteus����Ĭ���ⲿ������8M��
  RCC_HSEConfig(RCC_HSE_ON);
  // �ȴ� HSE �����ȶ�
  HSEStartUpStatus = RCC_WaitForHSEStartUp();	
	// ֻ�� HSE �ȶ�֮�����������ִ��
  if (HSEStartUpStatus == SUCCESS)
  {
    // AHBԤ��Ƶ��������Ϊ1��Ƶ��HCLK = SYSCLK 
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   
    // APB2Ԥ��Ƶ��������Ϊ1��Ƶ��PCLK2 = HCLK
    RCC_PCLK2Config(RCC_HCLK_Div1); 
    // APB1Ԥ��Ƶ��������Ϊ1��Ƶ��PCLK1 = HCLK/2 
    RCC_PCLK1Config(RCC_HCLK_Div2);		
//-----------------���ø���Ƶ����Ҫ��������������-------------------//
    // ����PLLʱ����ԴΪHSE������PLL��Ƶ����
		// PLLCLK = 8MHz * pllmul
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, pllmul);
//------------------------------------------------------------------//
    // ����PLL 
    RCC_PLLCmd(ENABLE);
    // �ȴ� PLL�ȶ�
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    { }
    // ��PLL�ȶ�֮�󣬰�PLLʱ���л�Ϊϵͳʱ��SYSCLK
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    // ��ȡʱ���л�״̬λ��ȷ��PLLCLK��ѡΪϵͳʱ��
    while (RCC_GetSYSCLKSource() != 0x08)
    {}
  }
  else
  { // ���HSE����ʧ�ܣ���ô����ͻ���������û�����������ӳ���Ĵ��봦��
		// ��HSE����ʧ�ܻ��߹��ϵ�ʱ�򣬵�Ƭ�����Զ���HSI����Ϊϵͳʱ�ӣ�
		// HSI���ڲ��ĸ���ʱ�ӣ�8MHZ
    while (1)
    {}
  }
}
*/
