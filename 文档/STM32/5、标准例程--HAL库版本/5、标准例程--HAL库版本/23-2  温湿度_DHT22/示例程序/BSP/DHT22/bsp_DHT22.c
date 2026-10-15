#include "bsp_DHT22.h"
#include <stdint.h>
#include <stdbool.h>






#define  DHT22_BUS_HIGH     (HAL_GPIO_WritePin(DHT22_GPIO, DHT22_PIN, GPIO_PIN_SET))    // DAT置高
#define  DHT22_BUS_LOW      (HAL_GPIO_WritePin(DHT22_GPIO, DHT22_PIN, GPIO_PIN_RESET))  // DAT置低  
#define  DHT22_BUS_READ     (HAL_GPIO_ReadPin(DHT22_GPIO, DHT22_PIN))                   // 读取电平


static GPIO_TypeDef  *DHT22_GPIO = 0;     // DHT22所用引脚：端口
static uint32_t       DHT22_PIN  = 0;     // DHT22所用引脚：引脚号





// 本地US粗略延时函数，减少移植时对外部文件依赖
// 注意：此函数精度受编译优化和主频影响，建议根据实际主频校准或使用TIM定时器
static void delay_us(uint32_t us)
{
    uint16_t i = 0;
    while (us--)
    {
        i = 30;
        while (i--);
    }
}



// 引脚工作模式配置：输入模式
static void inputMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};          // 引脚工作模式 初始化结构体
    GPIO_InitStruct.Pin = DHT22_PIN;                 // 引脚编号
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;          // 引脚工作模式：输入
    GPIO_InitStruct.Pull = GPIO_PULLUP;              // 打开内部上拉
    HAL_GPIO_Init(DHT22_GPIO, &GPIO_InitStruct);     // 初始化
}



// 引脚工作模式配置：输出模式
static void outputMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};          // 引脚工作模式 初始化结构体
    GPIO_InitStruct.Pin = DHT22_PIN;                 // 引脚编号
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;      // 引脚工作模式：推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // 引脚电平反转速度
    HAL_GPIO_Init(DHT22_GPIO, &GPIO_InitStruct);     // 初始化
}



// 发送起始信号（唤醒传感器）
// 时序：MCU拉低 ≥800us → 释放总线20~40us
static void rst(void)
{
    outputMode(); 	// 输出模式
    DHT22_BUS_LOW; 	        // 拉低
    HAL_Delay(1);    	    // 拉低至少800us，典型1ms
    DHT22_BUS_HIGH; 	    // 拉高
    delay_us(30);     	    // 释放总线20~40us
}



// 检测传感器应答信号
// 时序：传感器拉低80us → 拉高80us
// 0 - 应答正常，1 - 无响应或超时
static uint8_t check(void)
{
    uint8_t retry = 0;
    
    inputMode();                          // 输入模式
    
    // 等待总线从高变低（传感器拉低）
    while (DHT22_BUS_READ && retry < 100)    
    {        
        delay_us(1);
        retry++;
    }
    if (retry >= 100)    return 1;        // 超时，无低电平
 
    retry = 0;
    // 等待总线从低变高（传感器释放）
    while (!DHT22_BUS_READ && retry < 100) 
    {
        delay_us(1);
        retry++;
    }
    if (retry >= 100)    return 1;        // 超时，无高电平
    
    return 0;
}



// 读取一个数据位
// 时序：50us低电平起始 + 26~28us高电平(0) 或 70us高电平(1)
// 采样点：高电平开始后40us
// 返回：0-低电平、1-高电平、0xFF-超时错误
static uint8_t readBit(void)
{
    uint8_t retry = 0;
   
    // 等待低电平开始（50us低电平起始信号）
    while (DHT22_BUS_READ && retry < 100) 
    {
        retry++;
        delay_us(1);
    }
    if (retry >= 100)   return 0xFF;   // 超时，无起始低电平

    retry = 0;
    // 等待高电平到来（数据位高电平）
    while (!DHT22_BUS_READ && retry < 100) 
    {
        retry++;
        delay_us(1);
    }
    if (retry >= 100) return 0xFF;    // 超时，无高电平
    
    delay_us(40);                     // 延时40us后采样
    return DHT22_BUS_READ;            // 高电平=1，低电平=0
}



// 读取一个字节（高位先出）
// 读取到的字节，若某位超时则返回0xFF
// 返回：0-低电平、1-高电平、0xFF-超时错误
static uint8_t readByte(void)
{
    uint8_t i, dat = 0;
    for (i = 0; i < 8; i++)
    {
        uint8_t bit = readBit();
        if (bit == 0xFF)   return 0xFF;   // 位读取超时，向上层报告
        dat = (dat << 1) | bit;
    }
    return dat;
}



/******************************************************************************
 * 函  数： DHT22_Init
 * 功  能： 初始化DHT22（AM2302）
 * 参  数： GPIO_TypeDef  *GPIOx:  GPIO端口号，取值范围：GPIOA ~ GPIOG
 *          uint32_t      PINx  :  引脚编号，取值范围：GPIO_PIN_0 ~ GPIO_PIN_15
 * 返回值： 0-成功，1-传感器无响应
 * 说  明： 上电后需等待2S越过不稳定状态，此函数不自动延时
 ******************************************************************************/
uint8_t DHT22_Init(GPIO_TypeDef *GPIOx, uint32_t PINx)
{    
    DHT22_GPIO = GPIOx;
    DHT22_PIN = PINx;
    
    // 时钟使能：引脚端口;用判断端口的方式使能时钟线, 减少移植时的工作
    if (GPIOx == GPIOA)  __HAL_RCC_GPIOA_CLK_ENABLE();
    if (GPIOx == GPIOB)  __HAL_RCC_GPIOB_CLK_ENABLE();
    if (GPIOx == GPIOC)  __HAL_RCC_GPIOC_CLK_ENABLE();
    if (GPIOx == GPIOD)  __HAL_RCC_GPIOD_CLK_ENABLE();
    if (GPIOx == GPIOE)  __HAL_RCC_GPIOE_CLK_ENABLE();
    if (GPIOx == GPIOF)  __HAL_RCC_GPIOF_CLK_ENABLE();
    if (GPIOx == GPIOG)  __HAL_RCC_GPIOG_CLK_ENABLE();

    // 配置引脚为输出，默认高电平
    outputMode();
    DHT22_BUS_HIGH;

    // 尝试通信，验证传感器是否存在
    uint8_t retry = 3;
    while (retry--)
    {
        rst();
        if (check() == 0) 
        {   
            printf("DHT22 初始化：成功！\r\n");    // 检测到设备
            return 0;                              // 返回：0-成功
        }
        HAL_Delay(10);
    }
    // 设备无响应，处理
    printf("DHT22 初始化：无响应！\r\n");
    return 1;                                      // 返回：1-失败、无响应
}



/******************************************************************************
 * 函  数： DHT22_GetData
 * 功  能： 读取温湿度数据
 * 数据格式：湿度高位(8bit) + 湿度低位(8bit) + 温度高位(8bit) + 温度低位(8bit) + 校验和(8bit)
 *          共40位，高位先出
 * 温度格式：16位，最高位Bit15=1表示负温度，Bit15=0表示正温度
 * 数据值：  实际值 = 传输值 / 10
 * 返回值： 0-成功，1-传感器无响应，2-校验和错误，3-位读取超时
 * 说  明： 两次读取间隔必须≥2秒，防止自热效应
 ******************************************************************************/
uint8_t DHT22_GetData(float *temperature, float *humidity)
{
    uint8_t buf[5] = {0};
    uint8_t sum = 0;    
    uint8_t retry = 3;                     // 增加重试机制            
    uint8_t success = 0;
    
    while (retry--)
    {
        rst();
        if (check() == 0) 
        {
            success = 1;
            break;
        }
        HAL_Delay(10);
    }
    if (!success)   return 1;              // 传感器无响应
    
    // 读取40位数据（5字节）
    for (uint8_t i = 0; i < 5; i++)
    {
        buf[i] = readByte();
        if (buf[i] == 0xFF)    return 3;   // 返回; 3-位读取超时
    }
    
    // 和校验：湿度高位+湿度低位+温度高位+温度低位 = 校验和
    sum = buf[0] + buf[1] + buf[2] + buf[3];    
    if (sum != buf[4])    return 2;        // 校验错误
    
    // 湿度 = (湿度高8位<<8 | 湿度低8位) / 10
    *humidity = (float)((buf[0] << 8) + buf[1]) / 10;
    // 温度 = 16位有符号整数（最高位为符号位） / 10
    int16_t temp_raw = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    *temperature = (float)temp_raw / 10.0f;
 
    return 0;
}









