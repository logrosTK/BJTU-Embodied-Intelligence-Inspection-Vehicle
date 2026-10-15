/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  0.96寸 OLED 显示屏驱动（I2C接口，含W25Q128字库）
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库 + OLED（SSD1306）
 **
 ** 【功能概述】  本程序通过软件模拟 I2C 驱动 0.96 寸 OLED 显示屏，演示中英文字符及数值的显示方法。
 **               1. 初始化系统时钟、GPIO、UART1、W25Q128 外部 Flash 及 OLED 显示屏
 **               2. 演示两种汉字显示方式：
 **                  - 静态字模数组：显示“温湿度采集”五个汉字（字模存于代码区）
 **                  - 外部 Flash 字库：从 W25Q128 读取字模数据，显示中英文混合字符串
 **               3. 演示浮点数转换为字符串，并显示温湿度示例数据
 **               4. 支持 12/16/24/32 四种字号的中英文字符显示
 **               5. 绿色 LED 每 500ms 闪烁，指示系统运行状态
 **               6. 串口接收数据并回显（测试用）
 **               7. 按键中断（KEY1/KEY2/KEY3）控制蓝色 LED 反转并输出提示
 **
 ** 【硬件接线】  1. OLED 模块（4线 I2C）：
 **                  VCC —— 3.3V  （严禁接5V）
 **                  GND —— GND
 **                  SCL —— PB6   （模拟 I2C，引脚可在 bsp_OLED.h 中修改）
 **                  SDA —— PB7   （模拟 I2C，引脚可在 bsp_OLED.h 中修改）
 **               2. W25Q128 外部 Flash（SPI接口）：
 **                  - 用于存储汉字字库（约6MB），字库已预置，地址 0x00A00000 起
 **               3. UART1 调试串口：PA9(TX)、PA10(RX)，波特率 115200
 **               4. LED：绿色 PC5，蓝色 PB2（低电平点亮）
 **               5. 按键：KEY1(PA0)、KEY2(PA1)、KEY3(PA4) 已配置中断
 **
 ** 【移植说明】  1. 将本工程中的 bsp_OLED.c/h、bsp_W25Q128.c/h、bsp_UART.c/h 等文件复制到目标工程
 **               2. 在 Keil 中添加文件路径，并添加对应源文件
 **               3. 如需修改 OLED 的 I2C 引脚，请在 bsp_OLED.h 中修改宏定义
 **               4. 确保 W25Q128 的 SPI 接口及片选引脚配置正确（参考 bsp_W25Q128.h）
 **               5. 若使用外部字库，需确保 Flash 中已烧录字库数据（地址 0x00A00000 起）
 **
 ** 【函数使用】  1. OLED_Init()                           // 初始化 I2C 及 OLED 屏幕
 **               2. OLED_ShowChinese(x, y, index, size)   // 显示静态字模汉字（x,y为起点坐标，index为字模数组索引，size为字号）
 **               3. OLED_String(x, y, str, size)          // 显示中英文字符串（自动从 Flash 读取字模）
 **               4. sprintf() 转换数值为字符串后，再用 OLED_String() 显示
 **
 ** 【调试建议】  1. 上电后若 OLED 无显示，检查 3.3V 供电及 I2C 引脚连接
 **               2. 若中文字符显示异常，检查 W25Q128 是否已烧录字库数据
 **               3. 使用串口助手观察按键及串口接收数据，确认系统运行正常
 **               4. 可通过修改 OLED_String 的 size 参数切换字号（12/16/24/32）
 **
 ** 【更新记录】  2026-03-27  完善注释，规范格式，优化显示示例
 **               2024-02-19  增加外部字库支持
 **               2024-02-17  从标准库迁移至 HAL 库
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
/* 用户代码，必须写在配对的BEGIN与END之间，否则CubeMX重新生成后，会被删除掉 */

#include "bsp_UART.h"            // 串口通信底层驱动文件; 已重写好初始化、收发，调用函数即可使用串口
#include "bsp_W25Q128.h"         // 外部FLASH存储驱动; 芯片共16M字节存储空间，前10M可自由分配，后6M,即地址0x00A00000起，存储有字库数据
#include "bsp_OLED.h"            // OLED 显示屏驱动

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
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);   // 引脚置高电平，红亮灭

    UART1_Init(115200);                                                    // 初始化 串口1; 已写好底层，调用h中的函数即可使用; 引脚(PA9 +PA10)、波特率-None-8-1; 如果使用CubeMX配置，请使用前述引脚，但，不要在MX上进行中断及DMA配置，否则冲突

    W25Q128_Init();                                                        // 外部Flash 初始化，其内部存储有汉字字模数据

    OLED_Init();                                                           // OLED 初始化。引脚的修改在bsp_OLED.h中

    // 文字显示方式_1                                                      // 使用存储在font.h文件中的汉字字模数据, 需要提示自行制作汉字字模数据，然后存储于代码数组中; 这种方式，适合没有Flash字库且要显示少量汉字的方案
    OLED_ShowChinese(20, 0, 0, 16);                                        // 温, 在字模数组中的第0个字模
    OLED_ShowChinese(38, 0, 1, 16);                                        // 湿, 在字模数组中的第1个字模
    OLED_ShowChinese(56, 0, 2, 16);                                        // 度, 在字模数组中的第2个字模
    OLED_ShowChinese(74, 0, 3, 16);                                        // 采, 在字模数组中的第3个字模
    OLED_ShowChinese(92, 0, 4, 16);                                        // 集, 在字模数组中的第4个字模

    // 文字显示方式_2
    static char strTemp[30];
    OLED_String(5, 24, "温度:", 16);                                       // 本函数，可显示中英文字符串; 函数内自动选择font.h中的ASCII字模、Flash中的汉字字模; Flash中已存在放较为完成的汉字字模数据，四字字号，12号、16号、24号、32号，约6M大小
    float temperature = 18.5;                                              // 假设已采集的温度。用于展示如何显示小数等数据;
    sprintf(strTemp, "%2.1f℃", temperature);                              // 重要：很多人咨询, 如何把各种数据类型，转换为字符串，再显示到显示屏; 请百度 sprintf() 的具体用法，
    OLED_String(54, 24, strTemp, 16);                                      // 显示数值到显示屏上; 四字字号:12号、16号、24号、32号

    OLED_String(5, 46, "湿度:", 16);
    float humidity = 85.3;                                                 // 假设已采集的温度。用于展示如何显示小数等数据;
    sprintf(strTemp, "%2.1f %%", humidity);                                // 重要：很多人咨询, 如何把各种数据类型，转换为字符串，再显示到显示屏. 请百度 sprintf() 的具体用法，
    OLED_String(54, 46, strTemp, 16);                                      // 假设值，只作演示如何显示数值到显示屏上; 四字字号:12号、16号、24号、32号


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */

        /** 1ms延时 **/
        HAL_Delay(1 - 1);                                                  // 延时函数，参数：ms; 特别地：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** 检查UART1是否收到数据 (串口助手) **/
        if (UART1_GetRxNum())                                              // 判断USART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            uint16_t num = UART1_GetRxNum();                               // 获取接收到的字节数
            uint8_t *pData = UART1_GetRxData();                            // 获取接收到的数据(缓存的地址)
            printf("\r USART1 准备发 ESP8266 发送一帧数据 \r");            // 提示
            printf(" 字节数：%d \r", UART1_GetRxNum());                    // 显示字节数
            printf(" ASCII ：%s\r", (char *)UART1_GetRxData());            // 显示数据，以ASCII方式显示，即以字符串的方式显示
            UART1_ClearRx();                                               // 清0本次接收; 每一次处理接收的数据后，要清0，才能进行下一轮的判断接收;
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
/* 所有用户代码，必须写在配对的BEGIN与END之间 */

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
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
