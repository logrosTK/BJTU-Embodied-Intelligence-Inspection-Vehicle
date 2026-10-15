/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  2.8寸 ILI9341 显示屏综合测试（含 W25Q128 字库、XPT2046 触摸）
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库
 **
 ** 【功能概述】  本程序驱动 2.8 寸 ILI9341 显示屏，演示文本、数字、图形显示，并支持外部 Flash 字库及触摸屏。
 **               1. 初始化系统时钟、UART1、W25Q128 外部 Flash、ILI9341 显示屏及 XPT2046 触摸屏（可选）
 **               2. 设置屏幕方向（竖屏/横屏）
 **               3. 显示多种类型数据：
 **                  - 英文字符串（font.h 字模）
 **                  - 中文字符串（静态字模数组）
 **                  - 中文字符串（W25Q128 外部字库）
 **                  - 整型、十六进制、浮点数转换为字符串显示
 **               4. 图形绘制：画点、画线、画圆、区域填充
 **               5. 串口接收数据并实时显示到屏幕
 **               6. 绿色 LED 周期性闪烁，按键中断控制蓝色 LED 并输出串口信息
 **
 ** 【硬件接线】  1. LCD 显示屏（ILI9341）：
 **                  - 引脚定义见 bsp_LCD_ILI9341.h（SPI 接口 + 控制引脚）
 **               2. W25Q128 外部 Flash（SPI 接口）：
 **                  - 存储汉字字库，地址范围 0x00A00000 起
 **               3. XPT2046 触摸屏（SPI 接口）：
 **                  - 引脚定义见 bsp_XPT2046.h
 **               4. UART1 调试串口：PA9(TX)、PA10(RX)，波特率 115200
 **               5. LED：绿色 PC5，蓝色 PB2（低电平点亮）
 **               6. 按键：KEY1(PA0)、KEY2(PA1)、KEY3(PA4) 已配置中断
 **
 ** 【移植说明】  1. 复制本工程的 LCD_ILI9341 和 W25Q128 文件夹到目标工程目录
 **               2. 在 Keil 中添加头文件路径：Options → C/C++ → Include Paths
 **               3. 添加源文件到工程：bsp_LCD_ILI9341.c、bsp_W25Q128.c、bsp_XPT2046.c（如需触摸）
 **               4. 在需要使用显示功能的文件中包含 #include "bsp_LCD_ILI9341.h"
 **               5. 如需修改引脚，请修改对应的 bsp_xxx.h 文件中的宏定义
 **               6. 本工程不依赖 CubeMX 配置引脚，所有引脚均在 bsp 文件中定义
 **
 ** 【函数使用】  1. LCD_Init()                                         // 初始化显示屏
 **               2. LCD_SetDir(0/1)                                    // 设置显示方向：0-竖屏，1-横屏
 **               3. LCD_String(x, y, str, size, fColor, bColor)        // 显示中英文字符串（支持12/16/24/32字号）
 **               4. LCD_ShowChinese(x, y, size, index, fColor, bColor) // 显示静态字模汉字
 **               5. LCD_DrawPoint(x, y, color)                         // 画点
 **               6. LCD_Line(x0, y0, x1, y1, color)                    // 画线
 **               7. LCD_Circle(x0, y0, r, color)                       // 画圆
 **               8. LCD_Fill(x0, y0, x1, y1, color)                    // 区域填充
 **               9. sprintf() 转换数值为字符串后，调用 LCD_String() 显示
 **
 ** 【调试建议】  1. 上电后若屏幕无显示，检查 LCD 供电（3.3V）及 SPI 引脚连接
 **               2. 若中文字符显示异常，确认 W25Q128 已烧录字库数据（地址 0x00A00000 起）
 **               3. 若触摸无响应，检查 XPT2046 引脚及 SPI 通信，可取消注释 XPT2046_TouchHandler()
 **               4. 使用串口助手观察系统启动信息及按键状态，确认程序运行
 **               5. 通过串口发送数据，验证屏幕实时显示功能
 **               6. 图形绘制效果可根据需求修改坐标及颜色参数
 **
 ** 【更新记录】  2024-02-19  完善注释，增加更多显示示例
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

    W25Q128_Init();                                                               // W25Q128 初始化

    LCD_Init();                                                                   // LCD ILI9341 初始化
    LCD_SetDir(0);                                                                // 设置显示方向：0-竖屏、1-横屏

    // 测试1：输出字符串，使用font.h中的字模
    LCD_String(5, 5, "Hello!", 16, GREEN, BLACK);                                 // LCD_String函数，支持中英文，支持4种字号大小：12、16、24、32

    // 测试2：输出中文，使用font.h中的字模
    LCD_ShowChinese(5,   30, 24, 0, WHITE, BLACK);                                // 温, 在font.h文件，字模数组中的第0个字模
    LCD_ShowChinese(35,  30, 24, 1, WHITE, BLACK);                                // 湿, 在font.h文件，字模数组中的第1个字模
    LCD_ShowChinese(65,  30, 24, 2, WHITE, BLACK);                                // 度, 在font.h文件，字模数组中的第2个字模
    LCD_ShowChinese(95,  30, 24, 3, WHITE, BLACK);                                // 采, 在font.h文件，字模数组中的第3个字模
    LCD_ShowChinese(130, 30, 24, 4, WHITE, BLACK);                                // 集, 在font.h文件，字模数组中的第4个字模

    // 测试3：输出中文，使用外部Flash_W25Q128中的字模
    LCD_String(5, 65, "我们去图书馆好吗？", 24, BRRED, BLACK);                    // LCD_String字符串输出函数; 英文使用font.h的字模，汉字使用W25Q128的字模; 支持12\16\24\32字号大小

    // 测试4：输出整型数值
    uint32_t numInt = 35;
    char strTem[100];
    sprintf(strTem, "%d", numInt);                                                // 使用sprintf进行字符串格式化，输出整形数据
    LCD_String(5, 122, strTem, 24, RED, BLACK);                                   // LCD_String字符串输出函数; 英文使用font.h的字模，汉字使用W25Q128的字模; 支持12\16\24\32字号大小

    // 测试5：输出16进制值
    uint32_t numInt2 = 250;
    sprintf(strTem, "0x%02X", numInt2);                                           // 使用sprintf进行字符串格式化，把数据输出为16进制显示，并控制格式，如果不到2位，就在前面补0显示
    LCD_String(80, 122, strTem, 24, RED, BLACK);                                  // LCD_String字符串输出函数; 英文使用font.h的字模，汉字使用W25Q128的字模; 支持12\16\24\32字号大小

    // 测试6：输出float数值
    float numFloat = 0.25;
    sprintf(strTem, "%4.3f", numFloat);                                           // 使用sprintf进行字符串格式化，控制输出3位小数
    LCD_String(160, 122, strTem, 24, RED, BLACK);                                 // LCD_String字符串输出函数; 英文使用font.h的字模，汉字使用W25Q128的字模; 支持12\16\24\32字号大小

    // 测试6：输出混合型字符串：中英文+多个不同数值
    sprintf(strTem, "营收%d万元，增长:%d%%", numInt, (uint32_t)(numFloat * 100)); // 使用sprintf进行字符串格式化
    LCD_String(5, 160, strTem, 16, YELLOW, BLACK);                                // LCD_String字符串输出函数; 英文使用font.h的字模，汉字使用W25Q128的字模; 支持12\16\24\32字号大小

    // 测试7：画点
    for (uint16_t y = 20; y < 300; y++)
    {
        LCD_DrawPoint(230, y, MAGENTA);                                           // 画点：x坐标，y坐标，颜色
        LCD_DrawPoint(235, y, MAGENTA);
    }

    // 测试8：画线
    LCD_Line(0,  319, 239, 230, WHITE);                                           // 画线：起始x坐标，起始y坐标，结束x坐标，结束y坐标，颜色
    LCD_Line(0,  319, 239, 250, GREEN);

    // 测试9：画圆
    LCD_Circle(180, 224, 25, WHITE);                                              // 画圆：圆心x坐标，圆心y坐标，半径，颜色

    // 测试10：区域填充
    LCD_Fill(20, 210, 80, 250, GREEN);                                            // 区域填充：起始x坐标，起始y坐标，结束x坐标，结束y坐标，颜色
    LCD_Fill(70, 230, 120, 310, RED);


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
        //XPT2046_TouchHandler();                                            // 触摸屏状态检测：1:检测是否有按下，2:如果有按下，就处理好按下的坐标

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
