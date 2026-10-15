/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  2.8寸屏 触摸测试（XPT2046）
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库
 **
 ** 【功能概述】  本程序实现 XPT2046 电阻触摸屏的驱动，演示触摸检测、坐标获取及屏幕画点，支持校准功能。
 **               1. 初始化系统时钟、UART1、W25Q128 外部 Flash、ILI9341 显示屏及 XPT2046 触摸屏
 **               2. 在屏幕上显示操作提示及坐标信息
 **               3. 循环检测触摸屏状态：按下时在对应位置画点，并实时显示坐标
 **               4. 支持两种方式进入校准模式：
 **                  - 按下 KEY1 按键
 **                  - 通过串口发送字符串 "XPT2046"
 **               5. 校准数据存储于 W25Q128 外部 Flash 中，断电保存
 **               6. 绿色 LED 每 500ms 闪烁一次，指示系统运行状态
 **
 ** 【硬件接线】  1. LCD 显示屏（ILI9341）：引脚定义见 bsp_LCD_ILI9341.h
 **               2. XPT2046 触摸屏：引脚定义见 bsp_XPT2046.h（SPI 接口 + 中断引脚）
 **               3. W25Q128 外部 Flash：存储字库及触摸校准数据（SPI 接口）
 **               4. UART1 调试串口：PA9(TX)、PA10(RX)，波特率 115200
 **               5. LED：绿色 PC5（低电平点亮），蓝色 PB2（低电平点亮）
 **               6. 按键：KEY1(PA0)、KEY2(PA1)、KEY3(PA4)，已配置中断
 **
 ** 【移植说明】  1. 复制本工程的 LCD_ILI9341、W25Q128、XPT2046 文件夹到目标工程目录
 **               2. 在 Keil 中添加头文件路径：Options → C/C++ → Include Paths
 **               3. 添加源文件到工程：bsp_LCD_ILI9341.c、bsp_W25Q128.c、bsp_XPT2046.c
 **               4. 在需要使用触摸功能的文件中包含 #include "bsp_XPT2046.h"
 **               5. 如需修改触摸引脚，请在 bsp_XPT2046.h 中修改宏定义
 **               6. 本工程不依赖 CubeMX 配置引脚，所有引脚均在 bsp 文件中定义
 **
 ** 【函数使用】  触摸屏应用只需四步（适用于裸机、FreeRTOS、LVGL 等）：
 **               1. 初始化：XPT2046_Init(width, height, dir)  // 传入屏幕宽度、高度、方向
 **               2. 检测按下：XPT2046_IsPressed()            // 返回 0-未按下，1-按下
 **               3. 获取坐标：XPT2046_GetX() / XPT2046_GetY() // 获取按下位置的 X/Y 坐标
 **               4. 可选：XPT2046_ReCalibration()            // 重新校准触摸屏
 **
 ** 【调试建议】  1. 若触摸无响应，检查 XPT2046 的 SPI 通信及中断引脚连接
 **               2. 若触摸位置偏移，可执行校准（按下 KEY1 或串口发送 "XPT2046"）
 **               3. 校准数据存储于 W25Q128 中，若 Flash 未烧录字库，需确保校准数据存储区域未被占用
 **               4. 可在屏幕上观察画点位置与触摸笔位置是否一致，判断校准效果
 **
 ** 【更新记录】  2024-04-08  完善 font.h 数组命名，优化注释
 **               2024-02-19  增加串口校准命令，完善触摸演示界面
 **
 ** 【备注说明】  版权归魔女科技所有，仅供学习测试，请勿商用。
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

#include "bsp_UART.h"            // 串口驱动   
#include "bsp_LCD_ILI9341.h"     // LCD驱动
#include "bsp_XPT2046.h"         // 触屏驱动
#include "bsp_W25Q128.h"         // 外部Flash, 存有触屏的校准数据

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
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */
    /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

    UART1_Init(115200);                                                             // 初始化UART1

    printf("\r\r\r=========== STM32F407 外设初始报告 ===========\r");               // 注意，printf函数，必须重定向后才能使用，否则程序会卡死; 本示例已重写fputc, 在main.c的底部附近
    printf("系统运行时钟：           %d MHz\r", SystemCoreClock / 1000000);         // 输出系统运行时钟的频率，以检查是否配置正常

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);            // LED控制引脚置高电平，红亮灭

    W25Q128_Init();                                                                 // 外部Flash 初始化，其内部存储有汉字字模数据

    LCD_Init();                                                                     // LCD ILI9341 初始化
    LCD_SetDir(0);                                                                  // 设置显示方向：0-竖屏、1-横屏

    XPT2046_Init(xLCD.width, xLCD.height, xLCD.dir);                                // 触摸检测 初始化; 参数：屏宽、屏高、显示的方向

    // 显示提示，可以自行删除
    LCD_Fill(0, 0, 240, 320, BLACK);                                                // 整个背景填充颜色
    LCD_String(58, 8, "触 摸 检 测", 24, WHITE, BLACK);                             // 显示标题;
    LCD_Line(0, 38, 240, 38, GRAY);                                                 // 画个横线
    LCD_String(5, 240, "这是一个电阻屏测试程序", 12, WHITE, BLACK);                 // 显示文字;
    LCD_String(5, 255, "用于示范：触摸检测、坐标获取、画点", 12, WHITE, BLACK);     // 显示文字;
    LCD_String(5, 270, "如果发现坐标不对，按下左1按键进入校准", 12, WHITE, BLACK);  // 显示文字;
    LCD_String(5, 285, "电阻屏优点：适合油、潮湿、粉尘等环境", 12, WHITE, BLACK);   // 显示文字;
    LCD_String(5, 300, "电阻屏缺点：需要使用指尖等硬物触控", 12, WHITE, BLACK);     // 显示文字;


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

        /** 检查触摸屏是否按下 **/
        if (XPT2046_IsPressed())                                           // 检查触摸屏是否按下; 返回：0-未按下、1-按下
        {
            LCD_DrawPoint(XPT2046_GetX(), XPT2046_GetY(), GREEN);          // 在按下的位置，画点; 用于测试
            static char str[20] = {0};                                     // 新建数组，用于存放坐标字符串
            sprintf(str, "X:%3d  Y:%3d", XPT2046_GetX(), XPT2046_GetY());  // 格式化坐标字符串
            LCD_String(78, 40, str, 16, WHITE, BLACK);                     // 显示坐标字符串
        }

        /** UART1 是否接收到数据 **/
        if (UART1_GetRxNum())                                              // 如果接收字节数>0, 即为接收到新一帧数据
        {
            if (strstr((char *)UART1_GetRxData(), "XPT2046"))              // 判断是否收到"XPT2046", 调用触摸屏重新校准函数
            {
                XPT2046_ReCalibration();                                   // 进入触摸屏校准
            }
            UART1_ClearRx();                                               // 清0本次接收
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
        XPT2046_ReCalibration();                                // 触摸屏重新校准
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



/******************************************************************************
 * 函  数： HAL_TIM_PeriodElapsedCallback
 * 功  能： 周期更新回调函数
 * 备  注： 本函数是TIM的CNT溢出中断回调函数。
 *          当TIM的计数器CNT，完成1周期计数时触发(向上计数：CNT==ARR、向下计数：CNT==0);
 *          上述中断触发后，硬件自动调用相关中断服务函数，继而调用本函数。
 *          所有TIM的周期更新中断，都是调用本函数，因此需要在函数内判断是哪一个TIM触发的中断;
 * 参  数： TIM_HandleTypeDef   *htim
 * 返回值： 无
******************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)                                        // 判断是哪个TIM产生的中断
    {

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
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
