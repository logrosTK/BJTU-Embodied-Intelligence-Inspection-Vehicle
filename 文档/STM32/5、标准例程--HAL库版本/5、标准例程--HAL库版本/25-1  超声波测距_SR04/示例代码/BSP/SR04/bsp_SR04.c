#include "bsp_SR04.h"





// 本地变量声明, 本地宏定义
#define TRIG_1  (SR04_TRIG_GPIO->BSRR = SR04_TRIG_PIN)         // 高电平
#define TRIG_0  (SR04_TRIG_GPIO->BSRR = SR04_TRIG_PIN << 16)   // 低电平
#define ECHO    ((SR04_ECHO_GPIO->IDR & SR04_ECHO_PIN)? 1: 0)  // 判断ECHO引脚电平



// 本地US粗略延时函数，减少移植时对外部文件依赖；
static void delay_us(uint32_t times)
{
    times = times * 14;    //  10us内用7;
    while (--times)
        __nop();
}



static void SR04_Stop(void)
{
    TIM6->CR1 = 0;      // 停止TIM6
    TIM6->CNT = 0;      // 清0计数器
    TRIG_0 ;
}



/*****************************************************************************
 * 函  数： TIM6_Init
 * 功  能： 配置基本定时器TIM6
 * 参  数： uint16_t PSC :  预分频器数值,16位; 用于设置计数器1脉冲的时长(时钟频率);
 *          uint16_t ARR :  自动重装载数值,16位; 即多少个脉冲产生一个更新或中断(1周期);
 *          FunctionalState NewState ：定时器开关; 初始化后，是打开还是关闭
 * 返回值： 无
 * 备  注： 这里TIM6的初始化，为了方便移植及减少代码量，使用了寄存器操作
*****************************************************************************/
static void TIM6_Init(uint16_t PSC, uint16_t ARR, FunctionalState state)
{
    __HAL_RCC_TIM6_CLK_ENABLE();  // 使能TIM6时钟
    
    TIM6->CR1  = 0;               // 在配置前，先关闭计数器, 并清0 CR1的配置
    TIM6->CR2  = 0;               // 清0 CR2的配置
    TIM6->DIER = 0x01 ;           // 开启更新中断(只是开启功能，未工作）、 关闭DMA清求
    TIM6->SR   = 0;               // 清0 SR寄存器, 本寄存器只有一个功能: 中断标志位。
    TIM6->CNT  = 0;               // 清0 计数器的值
    TIM6->PSC  = PSC - 1;         // 预分频器数值,16位; 计数器的时钟频率CK_CNT等于f CK_PSC /(PSC[15:0]+1)。在每一次更新事件时，PSC的数值被传送到实际的预分频寄存器中。
    TIM6->ARR  = ARR;             // 自动重装载数值,16位; 即多少个脉冲产生一个更新或中断(1周期)。如果自动重装载数值为0，则计数器停止。
    if (state)                    // 是否使能计数器(使能了就会立即开始工作)
         TIM6->CR1 |= 0x01;  
    
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);    // 设置中断优先级
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);            // 使能中断线（允许CPU响应中断并调用相应的中断服务函数）
}



/*****************************************************************************
 * 函  数： TIM6_DAC_IRQHandler
 * 功  能： 超过0.03秒，则判断
 * 参  数： 无
 * 返回值： 无
 * 重  要： 1: TIM定时器，必须先清中断，后干其它事！！否则可能出现莫名错误
 *          2: 为了方便代码移植，中断服务函数的位置，放置于定时器配置函数下放
 *          3: 本函数的功能：当超过0.03ms时，Echo还没收到低电平结束信号，则停止测量
*****************************************************************************/
void  TIM6_DAC_IRQHandler(void)
{
    TIM6->SR &= (uint16_t)~0x01;   // 清0更新中断标志位
    SR04_Stop() ;                  // 超时处理，0.03s ;
}



/******************************************************************************
 * 函  数： SR04_Init ()
 * 功  能： 初始化SR04
 * 参  数： 无
 * 说  明： 初始化Trig、Echo两个引脚
 *          初始化Tim6
 *          开始由TIM6自动控制偱环工作，其结果值CM将不断地更新且存储于xSRO4.CM中
 *          如果想停止SR04的工作，可调用SR04_Stop()，重新开始，则调用SR04_Start();
 * 返回值： 无
 ******************************************************************************/
void SR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能SR04_TRIG所用引脚端口时钟；使用端口判断的方法使能时钟, 以使代码移植更方便
    if (SR04_TRIG_GPIO == GPIOA)  __HAL_RCC_GPIOA_CLK_ENABLE();
    if (SR04_TRIG_GPIO == GPIOB)  __HAL_RCC_GPIOB_CLK_ENABLE();
    if (SR04_TRIG_GPIO == GPIOC)  __HAL_RCC_GPIOC_CLK_ENABLE();
    if (SR04_TRIG_GPIO == GPIOD)  __HAL_RCC_GPIOD_CLK_ENABLE();
    if (SR04_TRIG_GPIO == GPIOE)  __HAL_RCC_GPIOE_CLK_ENABLE();
    if (SR04_TRIG_GPIO == GPIOF)  __HAL_RCC_GPIOF_CLK_ENABLE();
    if (SR04_TRIG_GPIO == GPIOG)  __HAL_RCC_GPIOG_CLK_ENABLE();
    // 使能SR04_ECHO所用引脚端口时钟；使用端口判断的方法使能时钟, 以使代码移植更方便
    if (SR04_ECHO_GPIO == GPIOA)  __HAL_RCC_GPIOA_CLK_ENABLE();
    if (SR04_ECHO_GPIO == GPIOB)  __HAL_RCC_GPIOB_CLK_ENABLE();
    if (SR04_ECHO_GPIO == GPIOC)  __HAL_RCC_GPIOC_CLK_ENABLE();
    if (SR04_ECHO_GPIO == GPIOD)  __HAL_RCC_GPIOD_CLK_ENABLE();
    if (SR04_ECHO_GPIO == GPIOE)  __HAL_RCC_GPIOE_CLK_ENABLE();
    if (SR04_ECHO_GPIO == GPIOF)  __HAL_RCC_GPIOF_CLK_ENABLE();
    if (SR04_ECHO_GPIO == GPIOG)  __HAL_RCC_GPIOG_CLK_ENABLE();

    // Trig引脚 配置
    GPIO_InitStruct.Pin = SR04_TRIG_PIN;                 // Trig引脚用于控制开启测量：闲时用单片机置低电平，开启测距时，置高电平10us以上，然后再置低电平，即可开始工作;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;          // 推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;         // 引脚翻转速度
    HAL_GPIO_Init(SR04_TRIG_GPIO, &GPIO_InitStruct);
    TRIG_0;                                              // 低电平

    // Echo引脚 配置
    GPIO_InitStruct.Pin = SR04_ECHO_PIN;                 // Echo引脚用于接收结果信号：高电平延长的时间，就是超声波发出到接收时长，因此，障碍物的距离计算公式为： 距离(m) = 高电平时长(s) x 声速(346M/2) ÷ 2
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SR04_ECHO_GPIO, &GPIO_InitStruct);

    // 配置TIM6, 用于精确计时
    TIM6_Init(84, 30000, DISABLE);                       // TIM6初始化，分频值:84, 重装载值：30000，即：每个脉冲0.0000001秒(1MHz: 84000000/84), 每30000次脉冲产生一次中断(0.03秒：30000*0.000001);

    SR04_Stop();                                         // 先调用一次停止函数，没特殊的意思，只为了清0各个值的数据，防止出错
}



/*****************************************************************************
 * 函  数： SR04_GetCM
 * 功  能： 获取目前的测量距离值; 单位：cm
 * 参  数： 无
 * 返回值： cm
*****************************************************************************/
float SR04_GetCM(void)
{
    while (ECHO == 1);            // 等待Echo引脚低电平
    /********1.启动信号:10US以上的脉冲**********/
    TRIG_1;
    delay_us(30);
    TRIG_0;
    /********2.启动TIM，记录ECHO高电平时长**********/
    TIM6->CNT = 0;                // 狗血的调试：调试过程无意中删了这一行，然后剧情他妈的精彩。排查一轮后，心力交瘁躺平发呆，刚眯上眼才来的灵感：是不是CNT计数器没清0！！
    while (ECHO == 0);            // 等待高电平开始(SR04工作后，ECHO引脚将返回声波来回的等长高电平时长）
    TIM6->CR1 |= 0x01;            // TIM开始工作
    while (ECHO == 1);            // 等待高电平结束
    TIM6->CR1 = 0x00;             // TIM停止工作
    /********3.计算距离,单位:CM**********/
    return (float)(TIM6->CNT * 0.0173);  // 摄氏25度时音速每秒346米，因为TIM初始化成1us脉冲，换算成：0.0346厘米/1us; Echo引脚的高电平时长，是声波从发出到接收的时间，所以要除以2
}


