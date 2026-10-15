/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  红外避障传感器中断检测
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库
 **
 ** 【功能概述】  本程序通过外部中断检测红外避障传感器的输出信号，实现障碍物检测与指示。
 **               1. 初始化系统时钟、GPIO、UART1（printf输出）
 **               2. 配置 PC12 为下降沿触发的外部中断（传感器输出低电平时表示检测到障碍物）
 **               3. 中断响应时点亮蓝色 LED（PB2），打印“发现障碍物！”提示信息
 **               4. 主循环中绿色 LED（PC5）每 500ms 闪烁一次，指示系统运行状态
 **               5. 支持 printf 重定向到 UART1，便于调试输出
 **
 ** 【硬件接线】  1. 红外避障传感器（或类似输出低电平有效的传感器）信号线接 PC12
 **               2. 蓝色 LED 接 PB2（低电平点亮）
 **               3. 绿色 LED 接 PC5（低电平点亮）
 **               4. UART1 用于调试输出，默认引脚 PA9(TX)、PA10(RX)
 **               5. 传感器电源 VCC 接 3.3V 或 5V（视模块要求），GND 共地
 **
 ** 【移植说明】  1. 若传感器输出引脚不同，请在 MX_GPIO_Init() 中修改对应的引脚及中断配置
 **               2. 若需修改 LED 引脚或极性，可调整 HAL_GPIO_WritePin 中的 GPIO 端口及引脚
 **               3. printf 重定向已实现，无需额外配置
 **               4. 中断防抖延时（HAL_Delay(10)）可根据实际情况调整
 **
 ** 【使用说明】  1. 编译下载后，绿色 LED 周期性闪烁，表示系统运行正常
 **               2. 当传感器检测到障碍物时，蓝色 LED 瞬间点亮，串口输出“发现障碍物！”
 **               3. 打开串口助手（115200-8-N-1），可实时查看障碍物检测信息
 **
 ** 【调试建议】  1. 若中断无响应，检查传感器是否输出低电平，可用万用表测量 PC12 电平变化
 **               2. 若蓝色 LED 不亮，检查引脚连接及电平极性
 **               3. 若串口无输出，检查 UART 引脚连接及波特率设置
 **               4. 中断中的延时可根据实际需求调整，避免影响其他中断响应
 **
 ** 【更新记录】  2025-03-16  创建红外避障传感器中断检测程序
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


/******************************************************************************
 * 功能说明： printf函数支持代码
 *           【特别注意】加入以下代码, 使用printf函数时, 不再需要打勾use MicroLIB
 * 备    注：
 * 最后更新： 2024年06月07日
 ******************************************************************************/
#include "stdio.h"                         // 引用C语言标准库，它定义了标准输入输出函数; 如：printf、scanf、sprintf、fopen 等; 
#include "stdarg.h"                        // 引用C语言标准库，它定义了处理可变参数的宏; 如：va_start、va_arg、va_end、va_list 等;
#include "string.h"                        // 引用C语言标准库，它定义了操作字符串的函数; 如：strcpy、strcmp、strlen、memset、memcpy 等;
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};                                         // 标准库需要的支持函数
FILE __stdout;                             // FILE 在stdio.h文件
void _sys_exit(int x)
{
    x = x;                                 // 定义_sys_exit()以避免使用半主机模式
}

int fputc(int ch, FILE *f)                 // 重定向fputc函数，使printf的输出，由fputc输出到UART
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}



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
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);            // 熄灭蓝色LED; 引脚PB2, 高电平断路熄灭;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                    // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)             // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);             // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                  // 记录当前时间，作为新的基准点，为下一次判断做准备
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

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
 * 参  数： uint16_t  GPIO_Pin    引脚编号；参数范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 返回值： 无
 * 备  注： CubeMX生成的代码在思维和风格上，与使用标准库编写时有很大的区别：
 *          1：这个是回调函数，不是中断服务函数。它是由硬件触发中断函数继而被程序调用的，背后的流程：EXTIx_IRQHandler() > HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_x) > HAL_GPIO_EXTI_Callback(GPIO_PIN_x)
 *          2：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_gpio.c的底部); 不建议在原弱定义中增添代码，而是在适当的位置重写本函数;
 *          3：中断条件触发后，本函数被自动调用，无需进行人工调用; 也无需进行中断标志的清理，因为HAL_GPIO_EXTI_IRQHandler()函数中已有操作;
 *          4：生成的所有EXTI中断服务函数，都统一自动调用这个函数，传入的参数是引脚的编号;
 *          5：判断参数传进来的引脚编号，即可知道是哪个外部中断线产生的中断信号;
******************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12)                               // 由PC12触发
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // 点亮蓝色LED; 引脚PB2, 低电平通路点亮;
        printf("发现障碍物！\r\n");                            // 打印提示信息; printf需要预先使能UART1，并重定向到UART1输出
        HAL_Delay(10);
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
