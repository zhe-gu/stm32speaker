#include "Common.h"

void delay_ms(u32 ms)
{
	u16 i = 0;
	while(ms--)
	{
		i = 4050;
		while(i--);
	};
}

void delay_us(u32 us)
{
	u16 i = 0;
	while(us--)
	{
		i = 2;
		while(i--);
	}
}

void sort(u16 *a, int l)//a为数组地址，l为数组长度。
{
	int i, j;
	u16 v;
	//排序主体
	for(i = 0; i < l - 1; i ++)
	{
		for(j = i+1; j < l; j ++)
		{
			if(a[i] > a[j])//如前面的比后面的大，则交换。
			{
				v = a[i];
				a[i] = a[j];
				a[j] = v;
			}
		}
	}
}
