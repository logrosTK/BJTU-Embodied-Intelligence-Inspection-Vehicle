/**==================================================================================================================
 **【文件名称】  bsp_BH1750.c
 **【功能测试】  BH1750-光强度传感器数据采集
 **==================================================================================================================
 **【实验平台】  STM32F407 + KEIL5.31 + BH1750
 **
 **【实验操作】  1-模块接线，VCC  接  3.3V
 **                          GND  接  GND
 **                          SCL  接  PC0; 没要求，在h文件里修改
 **                          SDA  接  PC1; 没要求，在h文件里修改
 **                          ADDR 接  空置或GND; 用于决定器件的地址，接GND或空置时地址为0x46，接3.3V时地址为0xB8
 **
 **【文件移植】  1-复制文件：可复制bsp_BH1750.c和bsp_BH1750.h两个文件，或复制BH1750文件夹，保存到所需工程目录文件夹下；
 **              2-添加文件：在keil工各程左侧文件管理器中，双击某文件夹，以添加bsp_BH1750.c文件;
 **              3-添加路径：点击魔术棒工具按钮，在“c/c++"选项页中，点击”Include Path"后面的按键，以添加文件存放所在路径(是文件夹，不是文件);
 **　　　　　    4-添加引用：在需要BH1750输出的代码文件头部，添加：#include "bsp_BH1750.h";
 **
 **
 **【函数使用】  引脚修改：在bsp_BH1750.h文件里修改
 **              初 始 化：BH1750_Init( );           // 初始化BH1750的IIC引脚; 注意：本代码采用软件IIC，引脚在bsp_BH1750.h中修改
 **              获取数据：BH1750_GetData( );        // 获取BH1750的光度值; 注意: 返回float类型数据
 **
 **
 **【备注说明】  代码版权归魔女科技所有，请勿商用，谢谢！
 **              https://demoboard.taobao.com
 ====================================================================================================================*/
#include "bsp_BH1750.h"





// IIC 操作函数
static void    start(void);				    // IIC开始信号
static void    stop(void);	  			    // IIC停止信号
static void    sendByte(uint8_t txd);		// IIC发送一个字节
static uint8_t readByte(uint8_t ack);       // IIC读取一个字节
static uint8_t waitAck(void); 				// IIC等待ACK信号
static void    Ack(void);					// IIC发送ACK信号
static void    NAck(void);				    // IIC不发送ACK信号



// 引脚电平控制 宏定义
#define SDA_1     (BH1750_SDA_GPIO -> BSRR = BH1750_SDA_PIN)         // SDA 置高
#define SDA_0     (BH1750_SDA_GPIO -> BSRR = BH1750_SDA_PIN <<16)    // SDA 置低
#define SCL_1     (BH1750_SCL_GPIO -> BSRR = BH1750_SCL_PIN)         // SCL 置高
#define SCL_0     (BH1750_SCL_GPIO -> BSRR = BH1750_SCL_PIN <<16)    // SCL 置低

// 引脚电平读取 宏定义
#define READ_SDA  (BH1750_SDA_GPIO->IDR & BH1750_SDA_PIN)            // 读SDA电平状态; 低电平时返回0，高电平返回非0值

// BH1750 指令 宏定义
#define BH1750_ON       0x01                // 启动
#define BH1750_CON      0x10                // 连续读取
#define BH1750_ONE      0x20                // 一次读取
#define BH1750_RSET     0x07                // 重置,重置数据寄存器值在POWERON模式下有效
                                            


/******************************************************************************
 * 函  数： delay_ms
 * 功  能： ms 延时函数
 * 备  注： 1、系统时钟168MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
 *          3、编译优化级别：Level 3(-O3)
 *          4、操作volatile声明的外部变量，是为了防止变量、函数被编译器优化
 * 参  数： uint32_t  ms  毫秒值
 * 返回值： 无
 ******************************************************************************/
static volatile uint32_t ulTimesMS;
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;
}


// 通信时序中的延时类型
#define    DELAY_TIME6          0          // 0-不使用定时器、1-使用TIM6产生延时

// 这部分未启用，如需启用，请自行按需修改
#if DELAY_TIME6

// TIM6中断服务函数
void  TIM6_DAC_IRQHandler(void)                                 // 中断服务函数，可以写在任意文件中。建议直接在启动文件中复制名称，避免写错，而使NVIC无法找到中断函数，这种错误，编译时是不会报错的;
{
    TIM6->SR = 0;                                               // 清理中断标志
    TIM6->CR1 = 0;                                              // 停止TIM6
}

// TIM6 配置
void TIM6_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);        // 使能定时器时钟

    NVIC_InitTypeDef NVIC_InitStructure;                        // 声明优先级配置结构体
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn; 	    // 中断通道来源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	        // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // 使能通道
    NVIC_Init(&NVIC_InitStructure);                             // 配置写入寄存器

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	            // 声明结构体
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;	            // 分频; 把接口时钟分频后给计数器使用, 即多少个接口脉冲，才产生一次计数器脉冲; 简单理解：计算每一计数器脉冲的时长;
    TIM_TimeBaseStructure.TIM_Period = 1 - 1;                   // ARR, 自动重载值; 多少个计数器脉冲作为一周期;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);             // 初始化定时器，把配置写入寄存器

    TIM_ClearFlag(TIM6, TIM_FLAG_Update);                       // 清理中断标志位
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);                  // 开启定时器更新中断

    TIM_Cmd(TIM6, DISABLE);	                                    // 使能定时器
}

static void delay_us(uint16_t us)
{
    TIM6 -> ARR = us;
    TIM6 -> CNT = 0x0;
    TIM6 -> CR1 = 0x1 ;
    while (TIM6->CR1 & 0x01) ;
}
#else
/******************************************************************************
 * 函  数： delay_us
 * 功  能： us 延时函数
 * 备  注： 1、系统时钟168MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
 *          3、编译优化级别：Level 3(-O3)
 *          4、操作volatile声明的外部变量，是为了防止变量、函数被编译器优化
 * 参  数： uint32_t  us  微秒值
 * 返回值： 无
 ******************************************************************************/
static volatile uint32_t ulTimesUS;
static void delay_us(uint32_t us)
{
    ulTimesUS = us * 17;
    while (ulTimesUS)
        ulTimesUS--;
}
#endif



// SDA引脚配置：输出模式
static void setSdaOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};              // 引脚工作模式 初始化结构体
    GPIO_InitStruct.Pin = BH1750_SDA_PIN;                // 引脚编号
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;          // 引脚工作模式：推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;        // 引脚电平反转速度
    HAL_GPIO_Init(BH1750_SDA_GPIO, &GPIO_InitStruct);    // 初始化
}



// SDA引脚配置：输入模式
static void setSdaInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};              // 引脚工作模式 初始化结构体
    GPIO_InitStruct.Pin = BH1750_SDA_PIN;                // 引脚编号
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;              // 引脚工作模式：输入
    GPIO_InitStruct.Pull = GPIO_PULLUP;                  // 打开上拉，引脚闲时为高电平
    HAL_GPIO_Init(BH1750_SDA_GPIO, &GPIO_InitStruct);    // 初始化
}



// 产生IIC起始信号, SCL为高电平时，SDA由高电平向低电平跳变，起始信号开始，开始传送数据
void start(void)
{
    setSdaOutput();     // sda线输出

    SDA_1;
    SCL_1;
    delay_us(4);
    SDA_0;
    delay_us(4);

    SCL_0;              // 钳住I2C总线，准备发送或接收数据
}



// 产生IIC停止信号，SCL为高电平时，SDA由低电平向高电平跳变，结束传送数据
void stop(void)
{
    setSdaOutput();     // sda线输出

    SCL_0;
    SDA_0;
    delay_us(4);
    SCL_1;

    SDA_1;              // 发送I2C总线结束信号
    delay_us(4);
}



// 等待应答信号到来
// 接收数据的IC在接收到8bit数据后，向发送数据的IC发出特定的低电平脉冲，表示已经收到数据
// 返回值：1，接收应答失败
//         0，接收应答成功
uint8_t waitAck(void)
{
    uint16_t errTime = 0;

    SDA_1;
    delay_us(1);
    setSdaInput();            // SDA设置为输入
    SCL_1;
    delay_us(1);

    while (READ_SDA)          // 判断SDA电平
    {
        if (errTime++ > 1000) // 等待超过一定时间，就判断为没应答，退出
        {
            stop();
            return 1;
        }
    }
    SCL_0;                    // 时钟输出0
    return 0;
}



// 产生ACK应答
void Ack(void)
{
    SCL_0;
    setSdaOutput();
    SDA_0;
    delay_us(2);
    SCL_1;
    delay_us(2);
    SCL_0;
}



//  不产生ACK应答
void NAck(void)
{
    SCL_0;
    setSdaOutput();
    SDA_1;
    delay_us(2);
    SCL_1;
    delay_us(2);
    SCL_0;
}



//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void sendByte(uint8_t txd)
{
    uint8_t t;
    setSdaOutput();
    SCL_0;                   // 拉低时钟装备开始数据传输
    for (t = 0; t < 8; t++)
    {
        if (txd & 0x80)
            SDA_1;
        else
            SDA_0;

        txd <<= 1;
        delay_us(2);         // 这三个延时都是必须的
        SCL_1;
        delay_us(2);
        SCL_0;
        delay_us(2);
    }
}



// 读1个字节，ack=1时，发送ACK，ack=0，发送NACK
uint8_t readByte(uint8_t ack)
{
    uint8_t i, receive = 0;
    setSdaInput();             // SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        SCL_0;
        delay_us(2);
        SCL_1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        delay_us(1);
    }
    if (!ack)
        NAck();                // 发送nACK
    else
        Ack();                 // 发送ACK
    return receive;            // 返回数据
}



// 向BH1750发送指令
void sendOrder(uint8_t cmd)     // 按照BH1750数据手册编写
{
    start();                    // IIC起始信号

    sendByte(BH1750_ADDR + 0);  // 发送设备地址+写信号
    while (waitAck());

    sendByte(cmd);              // 指令值
    while (waitAck());

    stop();                     // 发送停止信号
    delay_ms(1);                // 稍停一下
}



/******************************************************************************
 * 函数名： BH1750_Init
 * 功  能： 初始化BH1750所用模拟IIC引脚
 *          发送初始化指令
 * 参  数： 无
 * 返  回： 无
 ******************************************************************************/
void BH1750_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};          // 引脚工作模式 初始化结构体

    // 使能SCL引脚端口时钟
    if (BH1750_SCL_GPIO == GPIOA)  __HAL_RCC_GPIOA_CLK_ENABLE();
    if (BH1750_SCL_GPIO == GPIOB)  __HAL_RCC_GPIOB_CLK_ENABLE();
    if (BH1750_SCL_GPIO == GPIOC)  __HAL_RCC_GPIOC_CLK_ENABLE();
    if (BH1750_SCL_GPIO == GPIOD)  __HAL_RCC_GPIOD_CLK_ENABLE();
    if (BH1750_SCL_GPIO == GPIOE)  __HAL_RCC_GPIOE_CLK_ENABLE();
    if (BH1750_SCL_GPIO == GPIOF)  __HAL_RCC_GPIOF_CLK_ENABLE();
    if (BH1750_SCL_GPIO == GPIOG)  __HAL_RCC_GPIOG_CLK_ENABLE();

    // SCL引脚 初始化设置
    GPIO_InitStruct.Pin = BH1750_SCL_PIN;                // 引脚编号
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;          // 引脚工作模式：推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;        // 引脚电平反转速度
    HAL_GPIO_Init(BH1750_SCL_GPIO, &GPIO_InitStruct);    // 初始化

    // 使能SDA引脚端口时钟
    if (BH1750_SDA_GPIO == GPIOA)  __HAL_RCC_GPIOA_CLK_ENABLE();
    if (BH1750_SDA_GPIO == GPIOB)  __HAL_RCC_GPIOB_CLK_ENABLE();
    if (BH1750_SDA_GPIO == GPIOC)  __HAL_RCC_GPIOC_CLK_ENABLE();
    if (BH1750_SDA_GPIO == GPIOD)  __HAL_RCC_GPIOD_CLK_ENABLE();
    if (BH1750_SDA_GPIO == GPIOE)  __HAL_RCC_GPIOE_CLK_ENABLE();
    if (BH1750_SDA_GPIO == GPIOF)  __HAL_RCC_GPIOF_CLK_ENABLE();
    if (BH1750_SDA_GPIO == GPIOG)  __HAL_RCC_GPIOG_CLK_ENABLE();
    
    // SDA引脚 初始化设置
    GPIO_InitStruct.Pin = BH1750_SDA_PIN;                // 引脚编号
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;          // 引脚工作模式：推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;        // 引脚电平反转速度
    HAL_GPIO_Init(BH1750_SDA_GPIO, &GPIO_InitStruct);    // 初始化

#if DELAY_TIM6
    TIM6_Init();
#endif

    SCL_1;
    SDA_1;

    sendOrder(BH1750_ON);	  // 发送启动命令
    sendOrder(BH1750_RSET);	  // 发送重置命令，清除寄存器内容
    sendOrder(BH1750_CON);    // 设置模式：连续读取指令
    delay_ms(180);            // 发送指令120ms后可读取，这里延时180ms左右
}



/******************************************************************************
 * 函数名： BH1750_GetData
 * 功  能： 获取数据
 * 参  数： 无
 * 返  回： float 光照度数据
 ******************************************************************************/
float BH1750_GetData(void)
{
    static uint8_t data[2];

    start();                                  // IIC起始信号
    sendByte(BH1750_ADDR + 1);                // 发送设备地址+读信号
    while (waitAck());
    data[0] = readByte(1);
    data[1] = readByte(0);
    stop();                                   // IIC停止信号

    return ((data[0] << 8) | data[1]) / 1.2f; // 合成光照强度
}
