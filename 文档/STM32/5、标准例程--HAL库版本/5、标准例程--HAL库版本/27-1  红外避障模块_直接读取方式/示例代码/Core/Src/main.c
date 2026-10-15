/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  障碍物检测（红外/避障传感器）
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库
 **
 ** 【功能概述】  本程序通过读取传感器输出引脚电平，判断是否有障碍物，并控制 LED 指示及串口输出。
 **               1. 初始化系统时钟、GPIO 及 UART1
 **               2. 主循环中每隔 500ms 读取一次传感器输出引脚（PC12）状态
 **               3. 若检测到低电平（障碍物），点亮蓝色 LED（PB2）并打印提示信息
 **               4. 若无障碍物，熄灭蓝色 LED
 **               5. 绿色 LED（PC5）周期性闪烁（500ms 周期），指示系统运行状态
 **               6. 支持 printf 输出，串口重定向至 UART1
 **
 ** 【硬件接线】  1. 传感器（如红外避障模块、超声波模块等）输出引脚接 PC12
 **               2. 蓝色 LED 接 PB2（低电平点亮）
 **               3. 绿色 LED 接 PC5（低电平点亮，由 HAL_GPIO_TogglePin 控制）
 **               4. UART1 用于调试输出，默认引脚 PA9(TX)、PA10(RX)
 **
 ** 【移植说明】  1. 若传感器引脚不同，可修改 HAL_GPIO_ReadPin 中的引脚参数
 **               2. 若需修改 LED 引脚或极性，可调整 HAL_GPIO_WritePin 的 GPIO 端口及引脚
 **               3. printf 重定向已实现，无需额外配置
 **               4. 如需修改检测间隔，可调整 main 循环中的延时时间
 **
 ** 【使用说明】  1. 编译下载后，开发板绿色 LED 开始周期性闪烁
 **               2. 当传感器检测到障碍物时，蓝色 LED 亮起，串口输出“发现障碍物！”
 **               3. 打开串口助手（115200-8-N-1），可看到障碍物提示信息
 **
 ** 【调试建议】  1. 若蓝色 LED 常亮或不亮，检查传感器输出电平是否正常
 **               2. 可使用万用表测量 PC12 引脚电压，确认传感器工作状态
 **               3. 若串口无输出，检查 UART 引脚连接及波特率设置
 **               4. 若绿色 LED 不闪烁，检查系统时钟及 GPIO 初始化
 **
 ** 【更新记录】  2025-03-16  创建障碍物检测测试程序
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
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_Delay(500);                                            // 延时间隔

        /** 当遇到障碍物：亮灯、printf输出提示信息 **/
        uint8_t state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);      // 读取OUT引脚电平; 只是读引脚电平，用哪个引脚都可以，无需提前初始化
        if (state == 0)                                            // 判断 OUT 引脚是否是低电平(闲时高电平，遇到障碍物时持续输出低电平)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // 点亮蓝色LED; 引脚PB2, 低电平通路点亮;
            printf("发现障碍物！\r\n");                            // 打印提示信息; printf需要预先使能UART1，并重定向到UART1输出
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // 熄灭蓝色LED; 引脚PB2, 高电平断路熄灭;
        }

        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                        // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                 // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);                 // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                      // 记录当前时间，作为新的基准点，为下一次判断做准备
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
