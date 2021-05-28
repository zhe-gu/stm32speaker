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
			ADC_SoftwareStartConvCmd(ADC1,ENABLE);			//启动转换
		} else {
			GPIO_Write(GPIOE,~nums);
		}
		tick_ms(500);
	}
}

void NVIC_Configuration(void)	 //嵌套向量中断控制器配置
{
  NVIC_InitTypeDef NVIC_InitStructure;  
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //选择优先级组别
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	 
  //选择中断通道：EXTI线0中断，因为按键连接的是PA0脚
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
  //0级抢占式优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  //0级副优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //使能引脚作为中断源
  NVIC_Init(&NVIC_InitStructure);   //调用NVIC_Init固件库函数进行设置
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;//ADC1和ADC2 中断向量
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//重定义fputc函数,请勾选Use MicroLIB
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
	RCC_HCLKConfig(RCC_SYSCLK_Div1);	//外部时钟源HCLK也为72MHz
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//SysTick选择为外部时钟HCLK/8
}

//延时n us		    								   
void tick_us(u32 nus)
{		
	u32 temp;
	SysTick->LOAD = nus*9;  					//时间加载
	SysTick->VAL = 0x00;         					//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 	//开始倒数
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp&(1 << 16)))
		;		//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 	//关闭计数器
	SysTick->VAL = 0X00;       					 //清空计数器
}

//延时n ms
//SysTick->LOAD为24位寄存器
//对72M条件下，nms<=1864
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

/*时钟树设置函数
 * 使用HSE时，设置系统时钟的步骤
 * 1、开启HSE ，并等待 HSE 稳定
 * 2、设置 AHB、APB2、APB1的预分频因子
 * 3、设置PLL的时钟来源，和PLL的倍频因子，设置各种频率主要就是在这里设置
 * 4、开启PLL，并等待PLL稳定
 * 5、把PLLCK切换为系统时钟SYSCLK
 * 6、读取时钟切换状态位，确保PLLCLK被选为系统时钟 */

/* 设置 系统时钟:SYSCLK, AHB总线时钟:HCLK, APB2总线时钟:PCLK2, APB1总线时钟:PCLK1
 * PCLK2 = HCLK = SYSCLK
 * PCLK1 = HCLK/2,最高只能是36M
 * 参数说明：pllmul是PLL的倍频因子，在调用的时候可以是：RCC_PLLMul_x , x:[2,3,...16]
 * 举例：User_SetSysClock(RCC_PLLMul_9);  则设置系统时钟为：8MHZ * 9 = 72MHZ
 *       User_SetSysClock(RCC_PLLMul_16); 则设置系统时钟为：8MHZ * 16 = 128MHZ，超频慎用
 *
 * HSE作为时钟来源，经过PLL倍频作为系统时钟，这是通常的做法 */
/*
void HSE_SetSysClock(uint32_t pllmul)
{
	__IO uint32_t StartUpCounter = 0, HSEStartUpStatus = 0;
	// 把RCC外设初始化成复位状态，这句是必须的
  RCC_DeInit();
  //使能HSE，开启外部晶振，一般开发板用的是8M。Proteus仿真默认外部晶振是8M。
  RCC_HSEConfig(RCC_HSE_ON);
  // 等待 HSE 启动稳定
  HSEStartUpStatus = RCC_WaitForHSEStartUp();	
	// 只有 HSE 稳定之后则继续往下执行
  if (HSEStartUpStatus == SUCCESS)
  {
    // AHB预分频因子设置为1分频，HCLK = SYSCLK 
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   
    // APB2预分频因子设置为1分频，PCLK2 = HCLK
    RCC_PCLK2Config(RCC_HCLK_Div1); 
    // APB1预分频因子设置为1分频，PCLK1 = HCLK/2 
    RCC_PCLK1Config(RCC_HCLK_Div2);		
//-----------------设置各种频率主要就是在这里设置-------------------//
    // 设置PLL时钟来源为HSE，设置PLL倍频因子
		// PLLCLK = 8MHz * pllmul
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, pllmul);
//------------------------------------------------------------------//
    // 开启PLL 
    RCC_PLLCmd(ENABLE);
    // 等待 PLL稳定
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    { }
    // 当PLL稳定之后，把PLL时钟切换为系统时钟SYSCLK
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    // 读取时钟切换状态位，确保PLLCLK被选为系统时钟
    while (RCC_GetSYSCLKSource() != 0x08)
    {}
  }
  else
  { // 如果HSE开启失败，那么程序就会来到这里，用户可在这里添加出错的代码处理
		// 当HSE开启失败或者故障的时候，单片机会自动把HSI设置为系统时钟，
		// HSI是内部的高速时钟，8MHZ
    while (1)
    {}
  }
}
*/
