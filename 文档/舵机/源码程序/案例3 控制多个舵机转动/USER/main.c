#include "delay.h"
#include "uart.h"
#include "LobotServoController.h"
#include "bool.h"

void delay_s(int s)
{
	for(;s>0;s--)
		delay_ms(1000);
}

LobotServo servos[3];

 int main(void)
 {
 	SystemInit();//系统时钟等初始化
	delay_init(72);	     //延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uartInit(9600);//串口初始化为9600
	servos[0].ID = 1;			//设置舵机ID
	servos[1].ID = 2;
	servos[2].ID = 3;
	
	while(1)
	{
		servos[0].Position = 500;		//设置舵机位置为500
		servos[1].Position = 500;
		servos[2].Position = 500;
		
		moveServosByArray(servos, 3, 1200);		//让3个舵机转动到指定位置，运行时间为800ms
		delay_s(2);
		
		servos[0].Position = 2500;		//修改位置为2500
		servos[1].Position = 2500;
		servos[2].Position = 2500;
		
		moveServosByArray(servos, 3, 1200);	//让3个舵机转动到1000的位置，运行时间为800ms
		delay_s(2);
		
	}
}
