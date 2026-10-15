/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  数据存储_I2C_外部EEPROM_24C02
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库 + 24C02
 **
 ** 【功能概述】  本程序通过软件模拟 I2C 实现对 AT24C02 EEPROM 的读写操作，演示不同数据类型的存取方法。
 **               1. 初始化系统时钟、GPIO、UART1 及 AT24C02
 **               2. 串口输出系统运行时钟频率，确认时钟配置正常
 **               3. 向 AT24C02 指定地址（0x58）写入并读取多种数据类型：
 **                  - char 字符型
 **                  - short 短整型
 **                  - uint32_t 无符号32位整型
 **                  - float 浮点型
 **                  - 字符串（字符数组）
 **               4. 读取结果通过串口打印，验证读写正确性
 **               5. 主循环中实现 LED 周期性闪烁，并支持串口数据接收（UART1 中断转发）
 **               6. 按键中断（KEY1/KEY2/KEY3）控制 LED 状态反转并输出提示
 **
 ** 【硬件接线】  本示例使用软件模拟 I2C，无需硬件 I2C 引脚。引脚定义在 bsp_I2CSoft.h 中修改：
 **               VCC --- 3.3V
 **               GND --- GND
 **               SCK --- PB6  （可修改）
 **               SDA --- PB7  （可修改）
 **               注意：若使用开发板自带的 24C02 芯片，请勿占用 PB6/PB7 外接其他模块。
 **
 ** 【文件说明】  本示例依赖以下 4 个文件，均已封装好底层通信接口：
 **               - bsp_24C02.c / bsp_24C02.h   ：24C02 驱动核心
 **               - bsp_I2CSoft.c / bsp_I2CSoft.h ：软件 I2C 模拟
 **
 ** 【代码使用】  用户只需调用以下三个函数即可完成读写操作：
 **               1. 初始化：AT24C02_Init()                // 返回 0-失败，1-成功
 **               2. 写数据：AT24C02_WriteBytes(addr, buf, len)  // 返回 0-失败，1-成功
 **               3. 读数据：AT24C02_ReadBytes(addr, buf, len)   // 返回 0-失败，1-成功
 **
 ** 【实验现象】  1. 串口助手显示系统运行时钟频率
 **               2. 依次输出 5 组测试结果，每组包含写入值和读取值，验证数据一致性
 **               3. 绿色 LED 每 500ms 闪烁一次，指示程序正常运行
 **               4. 按下 KEY1/KEY2/KEY3 可控制蓝色 LED 状态反转，串口输出对应按键提示
 **               5. 通过串口助手向开发板发送数据，UART1 会接收并回显接收内容
 **
 ** 【划重点】    1. 本示例仅适用于 AT24C02 芯片（256 字节，32 页，每页 8 字节）
 **               2. 写操作必须遵守页限制：单次连续写入不能超过 8 字节（跨页需重新发送起始信号）
 **               3. 读操作无页限制，可连续读取任意长度
 **               4. 写入后需延时 5ms 等待 EEPROM 内部编程完成
 **               5. 设备地址（0xA0）已在 bsp_24C02.h 中定义，如需修改请参考芯片引脚 A2/A1/A0 的接法
 **               6. 测试地址范围 0~255，本例使用地址 88（0x58）
 **
 ** 【移植说明】  1. 将 NRF24L01 文件夹复制到目标工程目录，并添加头文件路径
 **               2. 在需要使用的文件中包含 #include "bsp_24C02.h"
 **               3. 如需修改 I2C 引脚，请在 bsp_I2CSoft.h 中修改宏定义
 **               4. 确保未使用与 I2C 引脚冲突的其他外设
 **
 ** 【调试建议】  1. 测试前确保 24C02 供电正常，引脚连接正确
 **               2. 若读写失败，检查 I2C 时序延时是否适合当前主频（可微调延时）
 **               3. 使用逻辑分析仪抓取 I2C 波形可辅助排查通信问题
 **               4. 注意写入后必须延时，否则后续操作可能失败
 **
 ** 【更新记录】  2025-03-16  HAL 工程整理，优化注释
 **               2023-11-21  新建工程，更新网盘
 **
 ** 【备注说明】  版权归魔女科技所有，仅供学习使用，请勿用于商业用途。
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

#include "bsp_UART.h"               // UART 串口通信底层驱动文件，已编写好6个串口的初始化和收发, 调用函数即可使用;
#include "bsp_24C02.h"              // 24C02 底层通信
#include "bsp_I2CSoft.h"            // I2C 软件模拟支持


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* 说明：
     * 1. 不要删除CubeMX生成的注释行（形如 USER CODE BEGIN/END），这些注释用于代码重新生成时的定位
     * 2. 所有用户自定义代码必须写在对应的BEGIN与END注释行之间，否则重新生成时会被覆盖
     * 3. 主要初始化函数说明：
     *    - HAL_Init()            初始化HAL库，配置中断优先级、系统时基等
     *    - SystemClock_Config()  配置系统时钟
     *    - MX_GPIO_Init()        初始化GPIO引脚（仅包含推挽输出、中断触发等配置）
     *    - 外设功能引脚（如UART、SPI等）的初始化在各自的功能文件中
     */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

    HAL_Delay(20);                                                                                // 上电后稍延时，让芯片、外部设备稳定上电后，再跑代码

    UART1_Init(115200);

    printf("\r\r\r=========== STM32F407 外设初始报告 ===========\r");                             // 注意，printf函数，必须重定向后才能使用，否则程序会卡死; 本示例已重写fputc, 在main.c的底部附近
    printf("系统运行时钟：           %d MHz\r", SystemCoreClock / 1000000);                       // 输出系统运行时钟的频率，以检查是否配置正常

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);                          // LED控制引脚置高电平，红亮灭

    AT24C02_Init();                                                                               // 设备24C02： 引脚初始化，测试连接

    printf("\r\n开始测试24C02......\r\n");
    printf("尝试不同地址的读写，及各数据类型的转换：\r\n");
    uint8_t addr   = 88;                                                                          // 注意地址值范围 0~255

    // 测试1：字符
    char writeChar = 'F';                                                                         // 新建变量：字符类型，用于保存写入的数据
    AT24C02_WriteBytes(addr, (uint8_t*)&writeChar, 1);                                            // 写入数据：地址，数据首地址，字节数
    HAL_Delay(5);                                                                                 // 写入后，稍作延时让芯片完成操作
    char readChar = 0;                                                                            // 新建变量：字符类型，用于保存读取的数据
    AT24C02_ReadBytes(addr, (uint8_t*)&readChar, 1);                                              // 读取数据：地址，数据首地址，字节数
    printf("测试char     地址:0x%X    写入:%c       读取:%c \r\n", addr, writeChar, readChar);    // 输出信息到串口助手

    // 测试2：带符号的16位
    short writeShort = -888;                                                                      // 新建变量，用于保存写入的数据
    AT24C02_WriteBytes(addr, (uint8_t*)&writeShort, sizeof(writeShort));                          // 写入数据：地址，数据首地址，字节数
    HAL_Delay(5);                                                                                 // 写入后，稍作延时让芯片完成操作
    short readShort = 0;                                                                          // 新建变量，用于保存读取的数据
    AT24C02_ReadBytes(addr, (uint8_t*)&readShort, sizeof(writeShort));                            // 读取数据：地址，缓存首地址，长度
    printf("测试short    地址:0x%X    写入:%d    读取:%d \r\n", addr, writeShort, readShort);     // 输出信息到串口助手

    // 测试3：无符号的32位
    uint32_t writeUint = 0x12345678;                                                              // 新建变量，用于保存写入的数据
    AT24C02_WriteBytes(addr, (uint8_t*)&writeUint, 4);                                            // 写入数据：地址，数据首地址，字节数
    HAL_Delay(5);                                                                                 // 写入后，稍作延时让芯片完成操作
    uint32_t readUint = 0;                                                                        // 新建变量，用于保存读取的数据
    AT24C02_ReadBytes(addr, (uint8_t*)&readUint, 4);                                              // 读取数据：地址，缓存首地址，长度
    printf("测试uint     地址:0x%X    写入:0x%X     读取:0x%X \r\n", addr, writeUint, readUint);  // 输出信息到串口助手

    // 测试4：浮点型
    float writeFloat = -123.456;                                                                  // 新建变量，用于保存写入的数据; 特别地：请自行百度浮点数有效位数
    AT24C02_WriteBytes(addr, (uint8_t*)&writeFloat, 4);                                           // 写入数据：地址，数据首地址，字节数
    HAL_Delay(5);                                                                                 // 写入后，稍作延时让芯片完成操作
    float readFloat;                                                                              // 新建变量，用于保存读取的数据
    AT24C02_ReadBytes(addr, (uint8_t*)&readFloat, 4);                                             // 读取数据：地址，缓存首地址，长度
    printf("测试float    地址:0x%X　  写入:%f    读取:%f \r\n", addr, writeFloat, readFloat);     // 输出信息到串口助手

    // 测试5：字符串, C没有字符串这个类型, 用字符数组
    char writeString[100] = "hi! 近来好吗？";                                                     // 新建变量，用于保存写入的数据
    AT24C02_WriteBytes(addr, (uint8_t*)writeString, sizeof(writeString));                         // 写入数据：地址，数据首地址，字节数
    HAL_Delay(5);                                                                                 // 写入后，稍作延时让芯片完成操作
    char readString[100];                                                                         // 新建变量，用于保存读取的数据
    AT24C02_ReadBytes(addr, (uint8_t*)readString, sizeof(readString));                            // 读取数据：地址，缓存首地址，长度
    printf("测试string   地址:0x%X　  写入:%s  读取:%s \r\n", addr, writeString, readString);     // 输出信息到串口助手

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

        /** 1ms延时 **/
        HAL_Delay(1 - 1);                                                  // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** 检查UART1是否收到数据 (串口助手) **/
        if (UART1_GetRxNum())                                              // 判断UART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            uint8_t *rxData = UART1_GetRxData();                           // 获取数据的地址
            uint16_t rxNum = UART1_GetRxNum();                             // 获取数据的字节数
            printf("\r<<<<< UART1 接收到一帧数据 \r");                     // 提示UART1接收到数据
            printf("字节数：%d \r", rxNum);                                // 显示字节数
            printf("ASCII : %s\r", (char *)rxData);                        // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                            // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < rxNum; i++)                           // 逐个字节输出
                printf("0x%02X ", rxData[i]);                              // 以16进制显示
            printf("\r\r");                                                // 显示换行
            UART1_ClearRx();                                               // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0
            // UART1_SendString("Hello World！\r");                        // 示范 UART1_SendString( )的使用; 可接受像pritnf的格式化参数
            // UART1_SendData((uint8_t*)"Hello World!\r", 13);             // 示范 UART1_SendData( )的使用, 可用于发送字符串、数组、结构体数据等
        }

        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                                // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                         // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {                                                                 
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);                         // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                              // 记录当前时间，作为新的基准点，为下一次判断做准备
        }

    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */



/******************************************************************************
 * 函  数： HAL_GPIO_EXTI_Callback
 * 功  能： EXTI中断回调函数
 * 参  数： uint16_t  GPIO_Pin    引脚编号，参数范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 返回值： 无
 * 备  注： CubeMX生成的代码在思维和风格上，与使用标准库编写时有很大的区别：
 *          1：这个是回调函数，不是中断服务函数。完整的调用：EXTIx_IRQHandler() > HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_x) > HAL_GPIO_EXTI_Callback(GPIO_PIN_x)
 *          2：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_gpio.c的底部); 不建议在原弱定义中增添代码，而是重写本函数
 *          3：中断条件触发后，本函数被自动调用，无需进行人工调用; 也无需进行中断标志的清理，因为HAL_GPIO_EXTI_IRQHandler()函数中已有操作;
 *          4：生成的所有EXTI中断服务函数，都统一调用这个函数，以引脚编号作参数
 *          5：判断参数传进来的引脚编号，即可知道是哪个外部中断线产生的中断信号
******************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_1_Pin)                                  // 按键1 产生的中断信号：KEY_1、PA0、闲时下拉、按下时高电平、上升沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯
        printf("KEY_1 按下\r");                                 // 通过串口助手输出提示
        //XPT2046_ReCalibration();                              // 触摸屏重新校准
    }

    if (GPIO_Pin == KEY_2_Pin)                                  // 按键2 产生的中断信号：KEY_2、PA1、闲时上拉、按下时低电平、下降沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯
        printf("KEY_2 按下\r");                                 // 通过串口助手输出提示
    }

    if (GPIO_Pin == KEY_3_Pin)                                  // 按键3 产生的中断信号：KEY_3、PA4、闲时上拉、按下低高电平、下降沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯
        printf("KEY_3 按下\r");                                 // 通过串口助手输出提示
    }
}




/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and linerxNumber,
       ex: printf("Wrong parameters value: file %s on line %d\r", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
