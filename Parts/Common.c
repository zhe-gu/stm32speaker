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

void sort(u16 *a, int l)//aΪ�����ַ��lΪ���鳤�ȡ�
{
	int i, j;
	u16 v;
	//��������
	for(i = 0; i < l - 1; i ++)
	{
		for(j = i+1; j < l; j ++)
		{
			if(a[i] > a[j])//��ǰ��ıȺ���Ĵ��򽻻���
			{
				v = a[i];
				a[i] = a[j];
				a[j] = v;
			}
		}
	}
}
