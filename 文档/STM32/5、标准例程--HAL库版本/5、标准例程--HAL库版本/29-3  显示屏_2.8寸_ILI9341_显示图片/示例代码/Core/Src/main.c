/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  2.8寸屏_显示测试（图片 + 外部字库）
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库
 **
 ** 【功能概述】  本程序驱动 ILI9341 2.8 寸显示屏，演示图片显示、静态字模显示、外部 Flash 字库显示及温湿度数据格式化输出。
 **               1. 初始化系统时钟、UART1、W25Q128 外部 Flash、ILI9341 显示屏
 **               2. 设置屏幕方向为竖屏，背景填充白色
 **               3. 显示位图图片（128x128 叶子图案，数据存于 font.h）
 **               4. 显示静态字模汉字：“温湿度采集”（字模存于 font.h 数组）
 **               5. 使用外部 Flash 字库（W25Q128）显示“温度:”、“湿度:”及数值，并演示浮点数转字符串
 **               6. 主循环中：
 **                  - 绿色 LED 每 500ms 闪烁
 **                  - 串口接收数据并回显，同时将数据实时显示到屏幕底部
 **
 ** 【硬件接线】  1. LCD 显示屏（ILI9341）：
 **                  - 引脚定义见 bsp_LCD_ILI9341.h（SPI 接口 + 控制引脚）
 **               2. W25Q128 外部 Flash：
 **                  - 存储汉字字库（12/16/24/32 号字模），发货前已烧录
 **               3. UART1 调试串口：PA9(TX)、PA10(RX)，波特率 115200
 **               4. LED：绿色 PC5（低电平点亮），蓝色 PB2（低电平点亮）
 **               5. 按键：KEY1(PA0)、KEY2(PA1)、KEY3(PA4) 已配置中断
 **
 ** 【移植说明】  1. 复制本工程的 LCD_ILI9341 和 W25Q128 文件夹到目标工程目录
 **               2. 在 Keil 中添加头文件路径：Options → C/C++ → Include Paths
 **               3. 添加源文件到工程：bsp_LCD_ILI9341.c、bsp_W25Q128.c
 **               4. 在需要使用显示功能的文件中包含 #include "bsp_LCD_ILI9341.h"
 **               5. 如需修改显示引脚，请在 bsp_LCD_ILI9341.h 中修改宏定义
 **
 ** 【函数使用】  1. LCD_Init()                     // 初始化显示屏
 **               2. LCD_SetDir(0/1)               // 设置显示方向：0-竖屏，1-横屏
 **               3. LCD_ShowChinese()             // 显示静态字模汉字（需提前在 font.h 中制作字模）
 **               4. LCD_Image()                   // 显示位图图片（数据流）
 **               5. LCD_String()                  // 显示中英文字符串（支持外部字库）
 **               6. LCD_Line() / LCD_Fill()       // 画线/区域填充
 **               7. sprintf() 转换数值为字符串后，用 LCD_String() 显示
 **
 ** 【调试建议】  1. 若屏幕无显示，检查 LCD 供电（3.3V）及 SPI 引脚连接
 **               2. 若中文字符显示异常，确认 W25Q128 已烧录字库，且 font.h 中静态字模索引正确
 **               3. 图片显示若花屏，检查图片数据格式（RGB565）及数组定义是否正确
 **               4. 串口调试：通过串口助手发送任意数据，屏幕底部会显示接收内容，同时串口回显
 **
 ** 【更新记录】  2024-02-19  增加图片显示、温湿度数值格式化示例
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
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

#include "bsp_W25Q128.h"
#include "bsp_LCD_ILI9341.h"
#include "bsp_XPT2046.h"
#include "bsp_UART.h"

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

    UART1_Init(115200);

    printf("\r\r\r=========== STM32F407 外设初始报告 ===========\r");             // 注意，printf函数，必须重定向后才能使用，否则程序会卡死; 本示例已重写fputc, 在main.c的底部附近
    printf("系统运行时钟：           %d MHz\r", SystemCoreClock / 1000000);       // 输出系统运行时钟的频率，以检查是否配置正常

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);          // LED控制引脚置高电平，红亮灭

    W25Q128_Init();                                                               // 外部Flash 初始化，其内部存储有汉字字模数据

    LCD_Init();                                                                   // LCD ILI9341 初始化
    LCD_SetDir(0);                                                                // 设置显示方向：0-竖屏、1-横屏
    LCD_Fill(0, 0, 240, 320, WHITE);                                              // 整个背景填充白色

    // 图片显示示范
    extern const unsigned char aBmpLeaf_128x128[ ];                               // 图片数组声明; 存放于font.h文件中。这里仅作示范，不建议使用, 太吃资源了。
    LCD_Image(90,  200,  128, 128, aBmpLeaf_128x128);                             // 图片显示：x坐标、Y坐标、图片宽度、图片高度，图片数组地址; 注意，这里的图片尺寸是指真实尺寸，而不是缩放尺寸

    // 文本显示示范_方式1                                                         // 使用存储在font.h文件中的汉字字模数据; 这种方式，适合没有Flash字库且要显示少量汉字的方案; 前提条件：需要提示自行制作汉字字模数据，然后存储于代码数组中;
    LCD_ShowChinese(48,  8, 24, 0, BLACK, WHITE);                                 // 温, 在字模数组中的第0个字模
    LCD_ShowChinese(78,  8, 24, 1, BLACK, WHITE);                                 // 湿, 在字模数组中的第1个字模
    LCD_ShowChinese(108, 8, 24, 2, BLACK, WHITE);                                 // 度, 在字模数组中的第2个字模
    LCD_ShowChinese(138, 8, 24, 3, BLACK, WHITE);                                 // 采, 在字模数组中的第3个字模
    LCD_ShowChinese(168, 8, 24, 4, BLACK, WHITE);                                 // 集, 在字模数组中的第4个字模

    // 画线示范
    LCD_Line(0, 38, 240, 38, GRAY);                                               // 画个横线
    LCD_Line(0, 39, 240, 39, GRAY);                                               // 再画个横线，意思就是加粗一下
    LCD_Line(0, 40, 240, 40, GRAY);                                               // 再画个横线，意思就是加粗一下

    // 文本显示示范_方式2                                                         // 使用板上Flash中的汉字字模，发货前已烧录好四种大小字模：12号、16号、24号、32号
    static char strTemp[30];
    LCD_String(5, 50, " 温度:  ", 24, BLACK, GREEN);                              // 本函数，可显示中英文字符串; 函数内自动选择font.h中的ASCII字模、Flash中的汉字字模; Flash中已存在放较为完成的汉字字模数据，四字字号，12号、16号、24号、32号，约6M大小
    float temperature = 18.3f;                                                    // 假设已采集的温度。用于展示如何显示小数等数据;
    sprintf(strTemp, "%4.1f℃    ", temperature);                                 // 重要：很多人咨询, 如何把各种数据类型，转换为字符串，再显示到显示屏: sprintf函数有神效，请自行百度用法;
    LCD_String(85, 50, strTemp, 24, BLACK, GREEN);                                // 显示数值到显示屏上

    LCD_String(5, 85, " 湿度:  ", 24, BLACK, GBLUE);
    float humidity = 85.3f;                                                       // 假设已采集的温度。用于展示如何显示小数等数据;
    sprintf(strTemp, "%2.1f %%    ", humidity);                                   // 重要：很多人咨询, 如何把各种数据类型，转换为字符串，再显示到显示屏: sprintf函数有神效，请自行百度用法;
    LCD_String(85, 85, strTemp, 24, BLACK, GBLUE);                                // 显示数值到显示屏上


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

        /** 触摸检查 **/
        //XPT2046_TouchHandler();                                          // 触摸屏状态检测：1:检测是否有按下，2:如果有按下，就处理好按下的坐标

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

            LCD_Fill(0, 300, 239, 319, WHITE);                             // 清理旧数据，准备显示新数据
            LCD_String(5, 303, (char *)rxData, 16, BLACK, WHITE);          // 把串口助手发过来的数据，直接显示到LCD上
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
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
