/*****************************************************************************
 ** NRF24L01  使用重点说明！！
 ** 注意中断引脚的中断函数重写
 ** 思维导图： 配置： 读写通过【命令字+地址】，把各种参数值写入到寄存器，同时返回的第1个字节是状态字
 **            发送： TX_ADDR和RX_ADDR_P0=地址，【命令W_TX_PAYLOAD+数据】，写入自动重发次数，重发时间间隔
 **            接收： 【命令R_RX_PAYLOAD+数据】
 **            中断： 三种中断可致IRQ引脚被拉低： 发送接收到ack，接收到数据，达到最大重发次数
****************************************************************************/
#include "bsp_NRF24L01.h"





/*****************************************************************************
 ** 本地变量声明
 *****************************************************************************/
static char strTem[100];       // 临时数据存放;

struct _NRF24L01               // 结构体：用于保存NRF24L01的信息、状态、数据;
{
    uint8_t InitStatus;        // 设备初始化状态; 0-初始化失败、1-初始化成功;

    uint8_t TxAddr[5];         // 对面接收方的地址，5个字节;
    uint8_t SendStatus;        // 发送状态标志; 0-无效值、1-发送成功、2-发送失败、3-正在发送、4-字节数不在有效范围(1~31)、5-NRF24L01尚未初始化; 后面为本状态标志解释，无需人手操作: 本标志，在NRF24L01初始化函数中=0; 在发送函数中=3; 在中断服务函数中:发送成功=1，发送失败=2; 每次发送数据后，稍作延时后可以调用 NRF24L01_GetSendStatus()返回本变量值, 注意此函数会对本标志自动=0, 意思是读取一次有效;

    uint8_t Addr_EN[6];        // 接收通道0~5的使能标志
    uint8_t Addr[6][5];        // 接收通道0~5的地址; 地址值为5个字节; 通道0和1能自由取值，通道2~5的前4个字节值必须与通道1地址前4字节值一样，因此只能自由设置最后一个字节值;

    uint8_t RxNum;             // 接收标志; 值范围：0~31，单位：字节; 0-无接收、非0-接收到新一帧数据字节数
    uint8_t RxData [33];       // 接收到的数据; 数据接收存放，接收到数据后，从设备读到此缓冲区，等待外部函数处理; NRF24L01最包数据最大32字节，这里开辟33个字节，是为了第33字节中存放一个'\0'，以作为字符串结束标志
    uint8_t RxChannel;         // 接收通道值; NRF24L01有6个接收通道，此值用记录最后一包接收的数据，是被哪个通道接收到的;
} xNRF24L01 = {0};             // 声明并初始化结构体所有变量值=0;



/*****************************************************************************
 ** 本地函数声明
 ****************************************************************************/
// spi底层通信部分
static  void     spiConfig(void);                                           // SPI通信参数配置
static  uint8_t  spiSendByte(uint8_t Data);                                 // SPI读写一个字节
// 模块数据读写
static  uint8_t  regReadByte(uint8_t reg);					                // 向寄存器写入一个字节
static  void     regWriteReg(uint8_t reg, uint8_t value);	                // 向寄存器读出一个字节
static  uint8_t  regWriteBuffer(uint8_t reg, uint8_t *pBuf, uint8_t len);   // 向指定寄存器写入N个字节
static  uint8_t  regReadBuffer(uint8_t reg, uint8_t *pBuf, uint8_t len);    // 向指定寄存器读出N个字节
// 其它功能
static  uint8_t  CheckConnection(void);                                     // 连接测试，用于检查SPI参数配置、接线这两项是否正确
static  void     NRF24L01_RxMode(void);                                     // 进入接收模式; 不用手动调用，在初始化、发送函数中，已有调用



/*****************************************************************************
 ** 设备参数宏定义: 模块内部固定的寄存器命令值、地址; 移植不用修改
 ****************************************************************************/
// 命令字
#define R_REGISTER          0X00    // 读配置寄存器, 0x000AAAAA， 000是命令前辍， AAAAA是要读操作的寄存器地址
#define W_REGISTER          0x20    // 写配置寄存器, 0x001AAAAA， 001是命令前辍， AAAAA是要读操作的寄存器地址
#define R_RX_PAYLOAD        0X61    // 读RX有效数据
#define W_TX_PAYLOAD        0XA0    // 写TX有效数据
#define FLUSH_TX            0XE1    // 清除TX FIFO寄存器 应用于发射模式
#define FLUSH_RX            0XE2    // 清除RX FIFO寄存器 应用于接收模式
#define REUSE_TX_PL         0XE3    // 从新使用上一数据包
#define NOP                 0XFF    // 空操作
// 寄存器地址
#define CONFIG              0X00    // bit0(PRIM_RX ):1接收模式 0发射模式 bit1(pwr_up):1上电0掉电bit3：1 16位CRC校验 0 八位CRC校验   //bit3:CRC使能 bit4:可屏蔽中断MAX_RT bit5:可屏蔽中断TX_DS bit6可屏蔽中断RX_RD
#define EN_AA               0X01    // 使能0——5通道的自动应答功能
#define EN_RXADDR           0X02    // 接收地址允许 0-5 通道 ，默认通道0通道1启动
#define SETUP_AW            0X03    // 设置地址宽度 00：无效 01：3字节 10：字节 11：5字节
#define SETUP_RETR          0X04    // 建立自动重发 3:0 自动重发计数 7:4 自动重发延时
#define RF_CH               0X05    // 射频通道 6:0 设置SI24R1工作频率
#define RF_SETUP            0X06    // 射频寄存器 0:低噪声放大器增益 2:1 发射功率 3:传输效率
#define STATUS              0X07    // 状态寄存器 0:TX FIFO寄存器满标志 3:1 接收数据通道号 4:达到最大重发中断 5：数据发送完成中断 6：数据接收中断
#define OBSERVE_TX          0X08    // 3:0 重发计数器(发送新数据包时复位) 7:4 数据包丢失计数器(写RF_CH时复位)
#define CD                  0X09    // 载波检测
#define RX_ADDR_P0          0X0A    // 数据通道0接收地址，最大长度:5字节(先写低字节，所写字节数量覵ETUP_AW设定)
#define RX_ADDR_P1          0X0B    // 数据通道1接收地址，最大长度:5字节(先写低字节，所写字节数量覵ETUP_AW设定)
#define RX_ADDR_P2          0X0C    // 数据通道2接收地址，最低字节可设置。高字节部分必须与RX_ADDR_P1[39:8]相等
#define RX_ADDR_P3          0X0D    // 数据通道3接收地址，最低字节可设置。高字节部分必须与RX_ADDR_P1[39:8]相等
#define RX_ADDR_P4          0X0E    // 数据通道4接收地址，最低字节可设置。高字节部分必须与RX_ADDR_P1[39:8]相等
#define RX_ADDR_P5          0X0F    // 数据通道5接收地址，最低字节可设置。高字节部分必须与RX_ADDR_P1[39:8]相等
#define TX_ADDR             0X10    // 发送地址 39:0
#define RX_PW_P0            0X11    // 接收数据通道0有效数据宽度(从1到32字节)
#define RX_PW_P1            0X12    // 接收数据通道1有效数据宽度(从1到32字节)
#define RX_PW_P2            0X13    // 接收数据通道2有效数据宽度(从1到32字节)
#define RX_PW_P3            0X14    // 接收数据通道3有效数据宽度(从1到32字节)
#define RX_PW_P4            0X15    // 接收数据通道4有效数据宽度(从1到32字节)
#define RX_PW_P5            0X16    // 接收数据通道5有效数据宽度(从1到32字节)
#define FIFO_STATUS         0X17    // FIFO状态寄存器 0:RX FIFO寄存器空标志 1:RX FIFO寄存器满标志 4:TX FIFO寄存器空标志
// 0x07寄存器STATUS
#define STATUS_RX           0x40    // 接收完成
#define STATUS_TX           0x20    // 发送完成
#define STATUS_MAX          0x10    // 最大重发次数
// 通信相关
#define RX_DR                  6    // 数据接收完成中断标志位
#define TX_DR                  5    // 数据发送完成中断标志位 (状态寄存器位置)
#define IT_TX               0x0E    // 发送模式
#define IT_RX               0x0F    // 接收模式
// 自定义变量
#define NRF24L01_MAX_LEN      32    // 每帧数据最大32字节(此值是模块固定的，不用修改)


/** 通信引脚控制 *******************/
// CS引脚控制; SPI通信所用; 置高:不与本模块通信、置低:选取本模块通信
#define CS_LOW     NRF24L01_CS_GPIO->BSRR = (uint32_t)(NRF24L01_CS_PIN <<16)      // 置低电平
#define CS_HIGH    NRF24L01_CS_GPIO->BSRR = NRF24L01_CS_PIN                       // 置高电平
// CE引脚控制; NRF24L01的模式控制; 置低:配置寄存器、置高:进入寄存器设定的工作模式
#define CE_LOW     NRF24L01_CE_GPIO->BSRR = (uint32_t)(NRF24L01_CE_PIN << 16)     // 置低电平
#define CE_HIGH    NRF24L01_CE_GPIO->BSRR = NRF24L01_CE_PIN                       // 置高电平


/*****************************************************************************
 ** 变量
 *****************************************************************************/






// us延时函数
static void delay_us(uint32_t us)               // 定义一个us延时函数，减少移植时对外部文件依赖;
{
    while (us--)                                // 注意，本函数是简易延时，非精准延时
        for (uint32_t i = 0; i < 7; i++);       // 系统时钟下，大约多少个空循环耗时1us
}


#if 0
// ms延时函数
static void delay_ms(uint32_t ms)               // 定义一个ms延时函数，减少移植时对外部文件依赖; 本函数仅作粗略延时使用，而并非精准延时;
{
    ms = ms * 42001;                            // 注意：此参考值运行条件：打勾 Options/ c++ / One ELF Section per Function
    for (uint32_t i = 0; i < ms; i++);          // 注意系统时钟下，大约多少个空循环耗时1ms
}
#endif



/*****************************************************************************
 * 函  数： gpioEN
 * 功  能： 使能 GPIO端口时钟
 * 参  数： GPIO_TypeDef  *GPIOx  要使用的GPIO端口，参数范围：GPIOA~GPIOG
 * 返回值： 无
*****************************************************************************/
// 使能 GPIO端口时钟
static void gpioEN(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if (GPIOx == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if (GPIOx == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if (GPIOx == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if (GPIOx == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    if (GPIOx == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    if (GPIOx == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
}



/*****************************************************************************
 * 函  数： spiConfig
 * 功  能： SPI 配置
 *          外部函数每次被调用，先用本函数重新配置1次SPI, 避免多个设备共用同一SPI配置
 * 参  数： 无
 * 返回值： 无
*****************************************************************************/
static void spiConfig(void)
{
    CS_HIGH;                            // 拉高NRF24L01的CS引脚，即失能NRF24L01
    NRF24L01_SPI->CR1  = 0;             // 清0
    NRF24L01_SPI->CR1 |= 0 << 0;        // 采样沿数, NRF要求上升沿采样   CPHA:时钟相位,0x1=在第2个时钟边沿进行数据采样，
    NRF24L01_SPI->CR1 |= 0 << 1;        // 时钟线闲时极性,   CPOL:时钟极性,0x1=空闲状态时，SCK保持高电平
    NRF24L01_SPI->CR1 |= 1 << 2;        // 主从模式,         0=从，1=主
    NRF24L01_SPI->CR1 |= 3 << 3;        // 波特率控制[5:3],  0=fPCLK/2,  1=/4倍  2=/8  3/16
    NRF24L01_SPI->CR1 |= 0 << 7;        // LSB先行，         0=MSB,  1=LSB
    NRF24L01_SPI->CR1 |= 1 << 8;        // 内部从器件选择,根据9位设置(失能内部NSS)
    NRF24L01_SPI->CR1 |= 1 << 9;        // 软件从器件管理 :  0=禁止软件管理从设备， 1=使能软件从器件管理(软件NSS)
    NRF24L01_SPI->CR1 |= 0 << 11;       // 数据帧格式,       0=8位,  1=16位
    NRF24L01_SPI->CR1 |= 1 << 6;        // 使能
    //NRF24L01_SPI->I2SCFGR&=~(1<<11);  //选择SPI模式
}



/*****************************************************************************
 * 函  数： uint8_t SPI_RW(uint8_t Data)
 * 功  能： SPI写入一个字节，并返回一个字节
 * 参  数： 要写入的一字节
 * 返回值： 返回一字节数据
*****************************************************************************/
static uint8_t spiSendByte(uint8_t Data)
{
    static uint16_t retry = 0;
    while ((NRF24L01_SPI ->SR & 0x02) == 0)    // SR状态寄存器[1]: 0-正常发送、1-发送完成
    {
        if (retry++ > 500)                     // 超时处理
            return 0;                          // 返回
    }
    NRF24L01_SPI ->DR = Data;                  // 填入发送的数据

    retry = 0;
    while ((NRF24L01_SPI->SR & 0x01) == 0)     // SR状态寄存器[0]: 0-接收缓冲为空、1-接收缓冲非空
    {
        if (retry > 500)                       // 超时处理
            return 0;                          // 返回
    }
    return NRF24L01_SPI->DR ;                  // 返回读取到的数据
}



/*****************************************************************************
 * 函  数：uint8_t regWriteReg(uint8_t reg,uint8_t value)
 * 功  能：向指定寄存器地址，写一个字节数据
 * 参  数：reg：  寄存器地址
 *         val:   要写入的值
 * 返回值：无
*****************************************************************************/
static void regWriteReg(uint8_t reg, uint8_t value)
{
    CS_LOW;                   // 置低SPI的片选线
    spiSendByte(reg) ;        // 写寄存器地址值
    spiSendByte(value);       // 写数据值
    CS_HIGH;                  // 置高SPI的片选线
}



/*****************************************************************************
 * 函  数：uint8_t NRF24L01read_reg(uint8_t reg)
 * 功  能：向指定寄存器地址，读出一字节数据
 * 参  数：reg： 寄存器地址
 * 返回值：reg_val(第二个读取到的字节)
*****************************************************************************/
static uint8_t regReadByte(uint8_t reg)
{
    static uint8_t val;       // 定义一个临时变量
    CS_LOW;                   // 置低SPI的片选线
    spiSendByte(reg);         // 写寄存器地址值
    val = spiSendByte(0xFF);  // 读数据值（发送一个无用的数据，同时读回期待值）
    CS_HIGH;                  // 置高SPI的片选线
    return val;               // 返回数据
}



/*****************************************************************************
 * 函  数：uint8_t regWriteBuffer(uint8_t reg, uint8_t *pBuf, uint8_t len)
 * 功  能：写一组数据到寄存器
 * 参  数：reg：   寄存器地址
 *         pBuf：  要写入数据的地址
 *         len:    要写入的数据长度
 * 返回值：status
*****************************************************************************/
static uint8_t regWriteBuffer(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    static uint8_t status;
    CS_LOW;
    status = spiSendByte(reg);
    for (uint8_t i = 0; i < len; i++)
    {
        spiSendByte(pBuf[i]);
    }
    CS_HIGH;
    return status;
}



/*****************************************************************************
 * 函  数： uint8_t NRF24L01_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
 * 功  能： 向指定寄存器地址，读出指定长度的数据
 * 参  数： reg :    寄存器地址
 *          pBuf:    数据存放缓冲区
 *          len :    读取的字节数量
 * 返回值： status:  设备状态字
*****************************************************************************/
static uint8_t regReadBuffer(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    static uint8_t status;
    CS_LOW;
    status = spiSendByte(reg);
    for (uint8_t i = 0; i < len ; i++)
    {
        pBuf[i] = spiSendByte(0xFF);
    }
    CS_HIGH;
    return status;
}



/*****************************************************************************
 * 函  数： NRF24L01_RxMode
 * 功  能： 切换NRF2401为接收状态
 * 参  数： 无
 * 返回值： 无
*****************************************************************************/
void NRF24L01_RxMode(void)
{
    if (xNRF24L01.InitStatus == 0)
    {
        printf("\r无线通信设备：NRF24L01/SI24R1　不可用！\r");
        return;
    }

    spiConfig();                                                                              // 每次调用SPI前都重新设置一次，以适应多设备共用1个SPI

    CE_LOW;                                                                                   // CE置低，才能配置寄存器

    // 这一段，放在初始化中，能更省时间。放在这里，是为了方便对参数设置的理解
    regWriteReg(W_REGISTER + SETUP_AW,  0x03);                                                // 设置
    regWriteReg(W_REGISTER + EN_AA,      0x3F);                                               // 设置接收通道0~5，自动ACK应答
    regWriteReg(W_REGISTER + EN_RXADDR,  0x3F);                                               // 设置接收通道0~5，全部可用
    regWriteReg(W_REGISTER + RX_PW_P0, NRF24L01_MAX_LEN);                                     // 设置通道0的有效数据包大小(字节数)，当接收到正确的数据包(正确的地址、CRC校验码，NRF2401自动把字头、地址和CRC校验位移去), STATUS寄存器的RX_DR置位
    regWriteReg(W_REGISTER + RX_PW_P1, NRF24L01_MAX_LEN);                                     // 设置通道1的有效数据包大小(字节数)，当接收到正确的数据包(正确的地址、CRC校验码，NRF2401自动把字头、地址和CRC校验位移去), STATUS寄存器的RX_DR置位
    regWriteReg(W_REGISTER + RX_PW_P2, NRF24L01_MAX_LEN);                                     // 设置通道2的有效数据包大小(字节数)，当接收到正确的数据包(正确的地址、CRC校验码，NRF2401自动把字头、地址和CRC校验位移去), STATUS寄存器的RX_DR置位
    regWriteReg(W_REGISTER + RX_PW_P3, NRF24L01_MAX_LEN);                                     // 设置通道3的有效数据包大小(字节数)，当接收到正确的数据包(正确的地址、CRC校验码，NRF2401自动把字头、地址和CRC校验位移去), STATUS寄存器的RX_DR置位
    regWriteReg(W_REGISTER + RX_PW_P4, NRF24L01_MAX_LEN);                                     // 设置通道4的有效数据包大小(字节数)，当接收到正确的数据包(正确的地址、CRC校验码，NRF2401自动把字头、地址和CRC校验位移去), STATUS寄存器的RX_DR置位
    regWriteReg(W_REGISTER + RX_PW_P5, NRF24L01_MAX_LEN);                                     // 设置通道5的有效数据包大小(字节数)，当接收到正确的数据包(正确的地址、CRC校验码，NRF2401自动把字头、地址和CRC校验位移去), STATUS寄存器的RX_DR置位

    // 几个重要的参数。放在初始化中能更时间。
    regWriteReg(W_REGISTER + RF_CH,      NRF24L01CONFIG_CH);                                  // 射频通道，即频率(0-125)
    regWriteReg(W_REGISTER + RF_SETUP, (NRF24L01CONFIG_RF_DR << 3) | NRF24L01CONFIG_RF_PWR);  // 空中传输速率, 发射功率
    regWriteReg(W_REGISTER + CONFIG, 0x03 | ((NRF24L01CONFIG_CRC == 16 ? 1 : 0) << 2));       // 接收模式、CRC16位、使能中断

    regWriteBuffer(W_REGISTER + RX_ADDR_P0, xNRF24L01.Addr[0], 5);                            // 写入接收通道0的接收地址，地址宽度：5字节
    regWriteBuffer(W_REGISTER + RX_ADDR_P1, xNRF24L01.Addr[1], 5);                            // 写入接收通道1的接收地址，地址宽度：5字节
    regWriteBuffer(W_REGISTER + RX_ADDR_P2, xNRF24L01.Addr[2], 1);                            // 写入接收通道2的接收地址，1字节
    regWriteBuffer(W_REGISTER + RX_ADDR_P3, xNRF24L01.Addr[3], 1);                            // 写入接收通道3的接收地址，1字节
    regWriteBuffer(W_REGISTER + RX_ADDR_P4, xNRF24L01.Addr[4], 1);                            // 写入接收通道4的接收地址，1字节
    regWriteBuffer(W_REGISTER + RX_ADDR_P5, xNRF24L01.Addr[5], 1);                            // 写入接收通道5的接收地址，1字节

    regWriteReg(FLUSH_RX, 0xFF);                                                              // 清除RX_FIFO
    regWriteReg(W_REGISTER + STATUS, 0X70);                                                   // 清除所有中断,防止一进去接收模式就触发中断
    CE_HIGH;   			                                                                      // CE置高10us后，进入状态
}



/*****************************************************************************
 * 函  数： CheckConnection
 * 功  能： 检查NRF2401与MCU的SPI总线是否通信正常
 *          写入发送地址，再读回对比，用于测试能否正常通信
 * 参  数： 无
 * 返回值： 0-失败、1-成功
*****************************************************************************/
static uint8_t CheckConnection(void)
{
    spiConfig();                                           // 重新配置spi, 避免其它设备使用过同一spi

    uint16_t errNum = 0;                                   // 连接失败次数，防止死循环
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};       // 测试用发送地址

    while (1)                                              // 开始测试
    {
        delay_us(200);                                     // 每次测试延时间隔
        regWriteBuffer(W_REGISTER + TX_ADDR, buf, 5);      // 写入地址值(5个字节)
        memset(buf, 0, 5);                                 // 清0数组
        regReadBuffer(TX_ADDR, buf, 5);                    // 读出地址值(5个字节)
        for (uint8_t i = 0; i < 5; i++)                    // 偱环对比5个字节
        {
            if (buf[i] == 0XA5)                            // 和原值对比
            {
                if (i == 4)                                // 5个值都相同了
                    return 1;                              // 测试成功，返回1; 0-测试失败、1-测试成功
            }
            else
            {
                break;                                     // 退出for
            }
        }
        if (errNum++ >= 5)                                 // 只测试5次，防止出错时一直死循环
            return 0;                                      // 测试失败，返回0; 0-测试失败、1-测试成功
    }
}



/*****************************************************************************
 * 函  数：NRF24L01_IRQ_IRQHANDLER
 * 功  能：NR24L01中断处理函数
 * 参  数：无
 * 返回值：无
 * 备  注: 注意如果和其它中断引脚使用了相同的中断号，编译时会出现错误提示
*****************************************************************************/
void NRF24L01_IRQ_IRQHANDLER(void)
{
    if ((EXTI->PR & NRF24L01_IRQ_PIN))                                       // 判断是不NRF24L01的中断
    {
        static uint8_t  status = 0 ;                                         // 用于保存状态寄存器的值
        CE_LOW;                                                              // 拉低CE，以便读取NRF24L01中STATUS中的数据
        spiConfig();                                                         // 重新配置spi, 避免其它设备使用过同一spi
        status = regReadByte(R_REGISTER + STATUS);                           // 读取STATUS中的数据，以便判断是由什么中断源触发的IRQ中断
        // 接收完成
        if (status & STATUS_RX)
        {
            memset(xNRF24L01.RxData, 0, NRF24L01_MAX_LEN);
            regReadBuffer(R_RX_PAYLOAD, xNRF24L01.RxData, NRF24L01_MAX_LEN); // 把模块FIFO数据，读到数组; 为了方便调试，不使用RX_PAYLO_WIDTH，而统一使用最大字节数操作
            regWriteReg(FLUSH_RX, 0xFF);                                     // 清除RX FIFO(注意：这句话很必要)
            xNRF24L01.RxNum = xNRF24L01.RxData[0];                           // 接收字节数; NRF24L01限定每包最大32字节; 发送函数内部利用[0]存放字节数，[1]起存放有效数据，因此每包最大发送31字节;
            xNRF24L01.RxChannel = (status & 0x0E) >> 1;                      // 记录接收通道; NRF24L01有6个接收通道，接收到数据时，接收的通道值位被记录于状态寄存器的3:1;
        }
        // 发送完成
        if (status & STATUS_TX)
        {
            regWriteReg(FLUSH_TX, 0xff);                                     // 清发送缓冲区：TX_FIFO
            xNRF24L01.SendStatus = 1;                                        // 发送状态; 0-无效值、1-发送成功、2-发送失败、3-正在发送;

        }
        // 最大重发次数
        if (status & STATUS_MAX)
        {
            regWriteReg(FLUSH_TX, 0xFF);                                     // 清除TX_FIFO
            xNRF24L01.SendStatus = 2;                                        // 发送状态; 0-无效值、1-发送成功、2-发送失败、3-正在发送;

        }
        regWriteReg(W_REGISTER + STATUS, 0x70);                              // 清理NRF24L01的中断标志
        NRF24L01_RxMode();                                                   // 回到接收模式
    }
    EXTI->PR |= 0xFFFF;                                                      // 清理STM32的外部中断(引脚产生的中断)
}



/*****************************************************************************
 * 函  数： NRF24L01_Init
 * 功  能： NRF24L01初始化
 *          初始化后，模块处于接收状态，监听空中的信号
 * 参  数： 无
 * 返回值： 0-初始化失败、1-初始化成功
*****************************************************************************/
uint8_t NRF24L01_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // 声明初始化要用到的结构体

    xNRF24L01.InitStatus = 0;                                       // 初始化状态标志复位;０-失败、1-成功

    // 使能 SPI时钟
    if (NRF24L01_SPI == SPI1)  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // 使能SPI1时钟
    if (NRF24L01_SPI == SPI2)  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;  // 使能SPI2时钟
    if (NRF24L01_SPI == SPI3)  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;  // 使能SPI3时钟

    // 使能 CS引脚时钟
    gpioEN(NRF24L01_CS_GPIO);                                       // 使能CS  引脚端口时钟
    gpioEN(NRF24L01_SCK_GPIO);                                      // 使能SCK 引脚端口时钟
    gpioEN(NRF24L01_MISO_GPIO);                                     // 使能MISO引脚端口时钟
    gpioEN(NRF24L01_MOSI_GPIO);                                     // 使能MOSI引脚端口时钟
    gpioEN(NRF24L01_CE_GPIO);                                       // 使能CE  引脚端口时钟
    gpioEN(NRF24L01_IRQ_GPIO);                                      // 使能IRQ 引脚端口时钟

    // 配置引脚: CS
    GPIO_InitStruct.Pin   = NRF24L01_CS_PIN;                        // 引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    HAL_GPIO_Init(NRF24L01_CS_GPIO, &GPIO_InitStruct);              // 初始化引脚工作模式

    CS_HIGH  ;                                                      // 初始化后, 先拉高, 再初始化其它通信引脚

    // 配置 SPI引脚: SCK、MISO、MOSI
    GPIO_InitStruct.Pin   = NRF24L01_SCK_PIN | NRF24L01_MISO_PIN | NRF24L01_MOSI_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    GPIO_InitStruct.Alternate = NRF24L01_SPI_AF;                    // 引脚复用功能
    HAL_GPIO_Init(NRF24L01_SCK_GPIO, &GPIO_InitStruct);             // 初始化引脚工作模式

    // 配置引脚：CE
    GPIO_InitStruct.Pin   = NRF24L01_CE_PIN;                        // 引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    HAL_GPIO_Init(NRF24L01_CE_GPIO, &GPIO_InitStruct);              // 初始化引脚工作模式

    CE_LOW;                                                         // 拉低CE引脚

    // 配置引脚：IRQ
    GPIO_InitStruct.Pin   = NRF24L01_IRQ_PIN;                       // 引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;                        // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    HAL_GPIO_Init(NRF24L01_IRQ_GPIO, &GPIO_InitStruct);             // 初始化引脚工作模式

    // 配置 SPI通信参数，并尝试读取参数
    spiConfig();                                                    // 配置SPI; 把其封装成一个独立函数，可以方便多次重复调用。因为SPI可以控制多个不同设备，怕a设备初始化后，又被b设备初始化成其它配置。
    regWriteReg(W_REGISTER + STATUS, 0x70);                         // 清理一次NRF24L01的中断标志，以减少莫名错误的发生
    printf("NRF24L01连接测试...   ");                               // 输出提示
    if (CheckConnection())                                          // 连接测试, 如果连接失败，则退出本功能初始化
    {
        printf("设备读写成功\r\n");                                 // 输出提示
    }
    else
    {
        printf("连接失败, 无线通信不可用!!!\r\n");                  // 输出提示
        return 0;
    }

    // 配置 IRQ引脚
    GPIO_InitStruct.Pin = NRF24L01_IRQ_PIN;                         // 引脚编号
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;                    // 下降沿触发
    GPIO_InitStruct.Pull = GPIO_PULLUP;                             // 打开内部上拉，即闲时为高电平，NRF24L01发生中断事件时被拉置低电平
    HAL_GPIO_Init(NRF24L01_IRQ_GPIO, &GPIO_InitStruct);             // 初始化

    // 配置 IRQ引脚的中断优化级
    HAL_NVIC_SetPriority(NRF24L01_EXTI_IRQn, 0, 0);                 // 配置优先级;
    HAL_NVIC_EnableIRQ(NRF24L01_EXTI_IRQn);                         // 使能中断线

    xNRF24L01.InitStatus = 1;                                       // 初始化完成，标志置1
    xNRF24L01.SendStatus = 0;                                       // 发送状态; 0-无效值、1-发送成功、2-发送失败、3-正在发送;

    return 1;                                                       // 初始化完成，返回1; 0-失败、1-成功
}



/*****************************************************************************
*函  数：NRF24L01_ReadConfig
*功  能：读取NRF24L01的配置，并通过串口打印输出
*参  数：
*备  注：前提条件：1：已配置好USART调试printf可用， 2：NRF24L01已测试线路可连接
*****************************************************************************/
void NRF24L01_ReadConfig(void)
{
    printf("\r>>>>读取NRF24L01配置：\r");
    printf("=寄存器名称=地址===值========备注======\r");
    spiConfig();                                            // 重新配置spi, 避免其它设备使用过同一spi

    uint8_t status = 0;
    // 00 config
    status = regReadByte(R_REGISTER + CONFIG);
    sprintf((char *)strTem, "CONFIG      0x%02X = 0x%02X  %d位CRC校验  Power_UP=%d  Prim_RX=%d", CONFIG, status, ((status & (1 << 2)) == (1 << 2)) ? 16 : 8, ((status & (1 << 1)) == (1 << 1)) ? 1 : 0, ((status & 1) == 1) ? 1 : 0);
    printf("%s\r", strTem);
    // 01 EN_AA
    status = regReadByte(R_REGISTER + EN_AA);
    sprintf((char *)strTem, "EN_AA       0x%02X = 0x%02X  自动应答通道x使能", EN_AA, status);
    printf("%s\r\n", strTem);
    // 02 EN_RXADDR
    status = regReadByte(R_REGISTER + EN_RXADDR);
    sprintf((char *)strTem, "EN_RXADDR   0x%02X = 0x%02X  接收通道x使能", EN_RXADDR, status);
    printf("%s\r\n", strTem);
    // 03 SETUP_AW
    status = regReadByte(R_REGISTER + SETUP_AW);
    sprintf((char *)strTem, "SETUP_AW    0x%02X = 0x%02X  地址字节宽度", SETUP_AW, status);
    printf("%s\r\n", strTem);
    // 04 SETUP_RETR
    status = regReadByte(R_REGISTER + SETUP_RETR);
    sprintf((char *)strTem, "SETUP_RETR  0x%02X = 0x%02X  重发间隔：%dUS  重发次数：%d", SETUP_RETR, status, ((status >> 4) + 1) * 250, status & 0x0F);
    printf("%s\r\n", strTem);
    // 05 RF_CH
    status = regReadByte(R_REGISTER + RF_CH);
    sprintf((char *)strTem, "RF_CH       0x%02X = 0x%02X  频率：24%d MHz", RF_CH, status, status);
    printf("%s\r\n", strTem);
    // 06 RF_SETUP
    status = regReadByte(R_REGISTER + RF_SETUP);
    uint8_t rate = (status >> 3) & 0x05;
    char *sRate = "000000000";
    char *sdBm = "000000000";
    sRate = rate == 4 ? "250Kbps" : (rate == 1 ? "2Mbps" : "1Mpbs");
    rate = status & 0x07;
    sdBm = rate == 7 ? "7" : (rate == 6 ? "4" : (rate == 5 ? "3" : (rate == 4 ? "1" : (rate == 3 ? "0" : (rate == 2 ? "-4" : (rate == 1 ? "-6" : "-12"))))));
    sprintf((char *)strTem, "RF_SETUP    0x%02X = 0x%02X  空中传输速率:%s  发射功率：%sdBm  ", RF_SETUP, status, sRate, sdBm);
    printf("%s\r\n", strTem);
    // 07 STATUS
    status = regReadByte(R_REGISTER + STATUS);
    sprintf((char *)strTem, "STATUS      0x%02X = 0x%02X  收到数据=%d  发送成功=%d  重发溢出=%d", STATUS, status, (status & (1 << 6)) ? 1 : 0, (status & (1 << 5)) ? 1 : 0, (status & (1 << 4)) ? 1 : 0);
    printf("%s\r\n", strTem);
    // 0A RX_ADDR_P0
    regReadBuffer(R_REGISTER + RX_ADDR_P0, (uint8_t *)strTem, 5);
    sprintf((char *)strTem, "RX_ADDR_P0  0x%02X         0x%02X  0x%02X  0x%02X  0x%02X  0x%02X", RX_ADDR_P0, strTem[0], strTem[1], strTem[2], strTem[3], strTem[4]);
    printf("%s\r\n", strTem);
    // 10 TX_ADDR
    regReadBuffer(R_REGISTER + TX_ADDR, (uint8_t *)strTem, 5);
    sprintf((char *)strTem, "TX_ADDR     0x%02X         0x%02X  0x%02X  0x%02X  0x%02X  0x%02X", TX_ADDR, strTem[0], strTem[1], strTem[2], strTem[3], strTem[4]);
    printf("%s\r\n", strTem);
    // 11RX_PW_P0
    status = regReadByte(R_REGISTER + RX_PW_P0);
    sprintf((char *)strTem, "RX_PW_P0    0x%02X = 0x%02X  接收数据长度：%d 字节", RX_PW_P0, status, status);
    printf("%s\r\n", strTem);
    // 17 FIFO_STATUS
    status = regReadByte(R_REGISTER + FIFO_STATUS);
    sprintf((char *)strTem, "FIFO_STATUS 0x%02X = 0x%02X", FIFO_STATUS, status);
    printf("%s\r\n", strTem);

    // 发送通道的地址值
    static uint8_t buf[5] = {0};
    printf("\r发达地址    Byte4  Byte3  Byte2  Byte1  Byte0   \r");
    memset(buf, 8, 5);
    regReadBuffer(TX_ADDR, buf, 5);    
    printf("             %X %c    %X %c   %X %c   %X %c   %X %c \r", buf[4], buf[4], buf[3], buf[3], buf[2], buf[2], buf[1], buf[1], buf[0], buf[0]);
    
    // 接收通道的地址值
    printf("\r接收通道    Byte4  Byte3  Byte2  Byte1  Byte0 \r");
    memset(buf, 8, 5);
    regReadBuffer(RX_ADDR_P0, buf, 5);    
    printf("通道0地址    %X %c    %X %c   %X %c   %X %c   %X %c \r", buf[4], buf[4], buf[3], buf[3], buf[2], buf[2], buf[1], buf[1], buf[0], buf[0]);
    // 通道 1
    memset(buf, 8, 5);
    regReadBuffer(RX_ADDR_P1, buf, 5);
    printf("通道1地址    %X %c    %X %c   %X %c   %X %c   %X %c \r", buf[4], buf[4], buf[3], buf[3], buf[2], buf[2], buf[1], buf[1], buf[0], buf[0]);
    // 通道 2
    memset(buf, 8, 5);
    regReadBuffer(RX_ADDR_P2, buf, 5);
    printf("通道2地址    ----    ----   ----   ----   %X %c \r",  buf[0], buf[0]);
    // 通道 3
    memset(buf, 8, 5);
    regReadBuffer(RX_ADDR_P3, buf, 5);
    printf("通道3地址    ----    ----   ----   ----   %X %c \r",  buf[0], buf[0]);
    // 通道 4
    memset(buf, 8, 5);
    regReadBuffer(RX_ADDR_P4, buf, 5);
    printf("通道4地址    ----    ----   ----   ----   %X %c \r",  buf[0], buf[0]);
    // 通道 5
    memset(buf, 8, 5);
    regReadBuffer(RX_ADDR_P5, buf, 5);
    printf("通道5地址    ----    ----   ----   ----   %X %c \r",  buf[0], buf[0]);
}



/*****************************************************************************
 * 函  数： NRF24L01_SetMyAddr
 * 功  能： 设置接收通道x的地址，即模块自己的地址
 * 参  数： uint8_t*  pAddr      地址值; 5个字节;
 *                               可以用数组传入，如uint8_t addr[5] = {0x03,0x08,0xFF,0x22,0x11}, 但这样不方便区分;
 *                               建议使用字符串的方式进行地址命名，如"Room1"、"Dev-1"、"DHT11"等等;
 *          uint8_t   channel    通道值; 有效范围:0~5 
 *
 * 返回值： 无
 * 备  注：         Byte4  Byte3  Byte2  Byte1  Byte0
 *          通道0    自由   自由   自由   自由   自由    // 地址的5个字节值，都能自由设置
 *          通道1    自由   自由   自由   自由   自由    // 地址的5个字节值，都能自由设置
 *          通道2   须同1  须同1  须同1  须同1   自由    // 地址的高4字节，必须与通道1相同，最低1个字节，能自由设置
 *          通道3   须同1  须同1  须同1  须同1   自由    // 地址的高4字节，必须与通道1相同，最低1个字节，能自由设置
 *          通道4   须同1  须同1  须同1  须同1   自由    // 地址的高4字节，必须与通道1相同，最低1个字节，能自由设置
 *          通道5   须同1  须同1  须同1  须同1   自由    // 地址的高4字节，必须与通道1相同，最低1个字节，能自由设置
 *          
 *          模块共有6个独立的接收通道;
 *          接收通道0、1能设置自由的地址，通道2~5的前4个字节必须与通道1相同，因此在设置通道2~5时，函数的操作是：复制通道1的前4字节+参数第5字节，对通道地址进行赋值;
 *          对接收通道2~5设置地址时，会更新通道1的前4字节, 注意通道1的地址值是否会被修改;
 *          特别地：对通道1~5进行设置时，通道1地址值的高位4字节，以最后一次对通道1~5的地址设置值高4字节为准。
*****************************************************************************/
void NRF24L01_SetMyAddr(uint8_t *pAddr, uint8_t channel)
{
    if (channel == 0 || channel == 1)                                 // 通道0、1能设置自由的地址;
    {
        xNRF24L01.Addr[channel][4] = pAddr[0];                        // 为什么要反取？
        xNRF24L01.Addr[channel][3] = pAddr[1];                        // 是为同时适配：数组值或使用字符串命名
        xNRF24L01.Addr[channel][2] = pAddr[2];
        xNRF24L01.Addr[channel][1] = pAddr[3];
        xNRF24L01.Addr[channel][0] = pAddr[4];
    }

    if (channel == 2 || channel == 3 || channel == 4 || channel == 5) // 通道2~5的前4个字节必须与通道1相同
    {
        xNRF24L01.Addr[1][4] = pAddr[0];                              // 把字符串的前4个字节，反取给通道地址的高4位
        xNRF24L01.Addr[1][3] = pAddr[1];
        xNRF24L01.Addr[1][2] = pAddr[2];
        xNRF24L01.Addr[1][1] = pAddr[3];
        xNRF24L01.Addr[channel][0] = pAddr[4];                        // 地址值的最低位，用字符串(5个字节)的最后一个字符
        xNRF24L01.Addr_EN[1] = 1;                                     // 使能通道1，使NRF24L01_RxMode()能对通道1的地址更新
    }

    xNRF24L01.Addr_EN[channel] = 1;                                   // 接收通道使能标志置1

    if (xNRF24L01.Addr_EN[0] == 0)                                    // 逐一判断，如果通道x没有使能，就给它填充一个测试值，好处是能方便地测试观察，但不会影响其它通道的正常收发
        memcpy(xNRF24L01.Addr[0], (uint8_t *)"Test0", 5);
    if (xNRF24L01.Addr_EN[1] == 0)
        memcpy(xNRF24L01.Addr[1], (uint8_t *)"Test1", 5);
    if (xNRF24L01.Addr_EN[2] == 0)
        memcpy(xNRF24L01.Addr[2], (uint8_t *)"2", 1);
    if (xNRF24L01.Addr_EN[3] == 0)
        memcpy(xNRF24L01.Addr[3], (uint8_t *)"3", 1);
    if (xNRF24L01.Addr_EN[4] == 0)
        memcpy(xNRF24L01.Addr[4], (uint8_t *)"4", 1);
    if (xNRF24L01.Addr_EN[5] == 0)
        memcpy(xNRF24L01.Addr[5], (uint8_t *)"5", 1);
   
    NRF24L01_RxMode();	                                              // 设置为接收模式
}



/*****************************************************************************
 * 函  数： NRF24L01_SetTxAddr
 * 功  能： 设置发送的地址，5个字节
 *          即对面接收方的地址
 * 参  数： uint8_t *pAddr   5个字节的地址值
 *                           可以用数组传入，如uint8_t addr[5] = {0x03,0x08,0xFF,0x22,0x11}, 但这样不方便区分;
 *                           建议使用字符串的方式进行地址命名，如"Room1"、"Dev-1"、"DHT11"等等;
 * 返回值： 无
 * 备  注： 本函数在发送前调用; 如果每次的发送目标是一样的，则只需调用一次即可;
*****************************************************************************/
void NRF24L01_SetTxAddr(uint8_t *pAddr)
{
    xNRF24L01.TxAddr[4] = pAddr[0];                        // 为什么要反取？
    xNRF24L01.TxAddr[3] = pAddr[1];                        // 是为了方便适配字符串命名的地址值
    xNRF24L01.TxAddr[2] = pAddr[2];
    xNRF24L01.TxAddr[1] = pAddr[3];
    xNRF24L01.TxAddr[0] = pAddr[4];
}



/*****************************************************************************
 * 函  数： NRF24L01_SendData(uint8_t *pData,)
 * 功  能： 发送数据
 * 参  数： uint8_t *pData   要发送的数据(数据地址)
 *          uint8_t num      字节数; 范围：1~31
 * 返回值： 无
 * 备  注:  1：NRF24L01限定每包最大32字节; 发送函数内部利用[0]存放字节数，[1]起存放有效数据，因此每包最大发送31字节;
 *          2: 特别地注意：用户无需外部处理上条操作，只需传入原始数据、原始的字节数，即可; 函数内部会自动处理成：[0]-字节数、[1]起-有效数据;
 *          3: 本函数所设置的重发间隔为：586us; 重发次数：10次; 如果尝试10次对方都无法接收成功，则产生重发溢出中断;
 *          4：接收方处理，记得第[0]字节为字节数，第[1]字节起，才是有效数据;
*****************************************************************************/
void NRF24L01_SendData(uint8_t *pData, uint8_t num)
{
    static uint8_t temData[32] = {0};                                                         // 定义一个数组存放发送数据：[0]-字节数、[1]起-有效数据

    while (xNRF24L01.SendStatus == 3);                                                        // 如果上一包还在发送，就等待

    if (xNRF24L01.InitStatus == 0)                                                            // 尚未初始化，放弃发送
    {
        xNRF24L01.SendStatus = 5;                                                             // 发送状态; 0-无效值、1-发送成功、2-发送失败、3-正在发送、4-放弃发送、5-未初始化;
        return;
    }

    if (num == 0 || num > 31)                                                                 // 放弃发送; 字节数不在有效范围内
    {
        xNRF24L01.SendStatus = 4;                                                             // 发送状态; 0-无效值、1-发送成功、2-发送失败、3-正在发送、4-放弃发送;
        return ;
    }

    temData[0] = num;                                                                         // [0] 字节数
    memset(temData + 1, 0, 31);                                                               // 清0临时缓存
    memcpy(temData + 1, pData, num);                                                          // [1]起写入数数据

    spiConfig();                                                                              // 每次调用SPI前都重新设置一次，以适应多设备共用1个SPI

    CE_LOW;	                                                                                  // 只有在 CE=0 时才能配置寄存器，否则易出现问题
    regWriteReg(FLUSH_TX, 0XFF);                                                              // 清除TX FIFO寄存器
    regWriteReg(FLUSH_RX, 0xff);                                                              // 清除RX_FIFO
    regWriteReg(W_REGISTER + STATUS, 0X70);                                                   // 清除所有中断,防止一进去接收模式就触发中断

    regWriteBuffer(W_TX_PAYLOAD, temData, NRF24L01_MAX_LEN);                                  // 写数据到TX_BUFF; 为了方便调试、修改，统一发送最大的32字节，有效数据后段为0值
    regWriteBuffer(W_REGISTER + TX_ADDR, xNRF24L01.TxAddr,  5);                               // 写入要发送的目标地址
    regWriteBuffer(W_REGISTER + RX_ADDR_P0, xNRF24L01.TxAddr,  5);                            // 通道0的地址设为和目标地址一致，以接收自动回复auto_ack信号
    //regWriteReg(W_REGISTER + EN_AA,      0x01);                                             // 使能通道0的自动应答; 为了简化代码，本参数在接收模式中设置;
    regWriteReg(W_REGISTER + SETUP_RETR, 0x1A);                                               // 设置自动重发间隔时间:586us;最大自动重发次数:10次 0x1A
    regWriteReg(W_REGISTER + RF_CH,      NRF24L01CONFIG_CH);                                  // 射频通道，即频率(0-125)
    regWriteReg(W_REGISTER + RF_SETUP, (NRF24L01CONFIG_RF_DR << 3) | NRF24L01CONFIG_RF_PWR);  // 空中传输速率, 发射功率
    regWriteReg(W_REGISTER + CONFIG,     0x02 | ((NRF24L01CONFIG_CRC == 8 ? 0 : 1) << 2));    // 设为发送模式,开启所有中断
    CE_HIGH;	                                                                              // 引脚拉高后10us工作状态

    xNRF24L01.SendStatus = 3;                                                                 // 发送状态; 0-无效值、1-发送成功、2-发送失败、3-正在发送;
}



/******************************************************************************
 * 函  数： NRF24L01_GetSendStatus
 * 功  能： 获取发送状态 (辅助函数，非必要的)
 * 参  数： 无
 * 返回值： 0-无效值
 *          1-发送成功
 *          2-发送失败，超出最大发送次数，还没收到ack应答信号
 *          3-正在发送
 *          4-字节数不在有效范围(1~31)
 *          5-NRF24L01尚未初始化
 ******************************************************************************/
uint8_t NRF24L01_GetSendStatus(void)
{
    static uint8_t sendStatus = 0;
    sendStatus = xNRF24L01.SendStatus;

    if (sendStatus == 0)                    // 保留的无效值; 返回：0
    {
        return 0;
    }

    if (sendStatus == 1)                    // 发送成功; 返回：1
    {
        xNRF24L01.SendStatus = 0;
        return 1;
    }

    if (sendStatus == 2)                    // 发送失败，超出最大发送次数，还没收到ack应答信号; 返回：2
    {
        xNRF24L01.SendStatus = 0;
        return 2;
    }

    if (sendStatus == 3)                    // 正在发送中; 返回：3
    {
        return 3;
    }

    if (sendStatus == 4)                    // 不在有效字节数范围内(1~31); 返回：4
    {
        xNRF24L01.SendStatus = 0;
        return 4;
    }

    if (sendStatus == 5)                    // NRF24L01尚未初始化; 返回：5
    {
        xNRF24L01.SendStatus = 0;
        return 5;
    }

    return 0;                               // 返回
}


/******************************************************************************
 * 函  数： NRF24L01_GetRxNum
 * 功  能： 获取接收的字节数
 *          范围:0~31, 单位:字节; 0-无接收、非0-接收到新一帧数据字节数;
 *          利用返回值判断接收：返回值>0，即为接收到新一帧数据;
 * 参  数： 无
 * 返回值： 0-无接收、非0-接收到新一帧数据字节数;
 ******************************************************************************/
uint8_t NRF24L01_GetRxNum(void)
{
    return xNRF24L01.RxNum;
}



/******************************************************************************
 * 函  数： NRF24L01_GetRxData
 * 功  能： 获取接收到的数据(地址)
 *          NRF24L01限定每包最大32字节; 发送函数内部利用[0]存放字节数，[1]起存放有效数据，因此每包最大发送31字节;
 *          因此，有效数据应从[1]处起读
 * 参  数： 无
 * 返回值： 接收到的数据缓存地址
 ******************************************************************************/
uint8_t *NRF24L01_GetRxData(void)
{
    return xNRF24L01.RxData;
}



/******************************************************************************
 * 函  数： NRF24L01_GetRxChannel
 * 功  能： 获取接收到数据的通道;
 *          RF24L01有6个接收通道:0~5;
 * 参  数： 无
 * 返回值： 接收通道值：0~5
 ******************************************************************************/
uint8_t   NRF24L01_GetRxChannel(void)
{
    return xNRF24L01.RxChannel;
}



/******************************************************************************
 * 函  数： NRF24L01_ClearRx
 * 功  能： 清0接收;
 *          即清0接收标志、数据;
 *          每次处理完一帧数据后，记得调用本函数清0
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void NRF24L01_ClearRx(void)
{
    xNRF24L01.RxNum = 0;
    xNRF24L01.RxChannel = 0;
    memset(xNRF24L01.RxData, 0, sizeof(xNRF24L01.RxData));
}







