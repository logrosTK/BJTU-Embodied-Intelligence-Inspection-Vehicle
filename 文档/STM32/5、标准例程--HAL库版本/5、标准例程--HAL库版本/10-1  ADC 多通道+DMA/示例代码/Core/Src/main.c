/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  ADC1 多通道采集 + DMA 转运
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【实验目标】  通过ADC1配置多通道扫描，配合DMA实现自动转运;
 **               1-采集ADC1_IN0(PA0)、ADC1_IN1(PA1)、ADC1_IN2(PA2)三路电压;
 **               2-DMA自动搬运转换结果到数组，不占用CPU;
 **               3-500ms定时打印ADC原始值与实际电压值;
 **               4-LED闪烁指示系统正常运行;
 **
 ** 【引脚接线】  ADC1_IN0 → PA0
 **               ADC1_IN1 → PA1
 **               ADC1_IN2 → PA2
 **               输入电压范围：0 ~ 3.3V
 **
 ** 【 CubeMX 】  打开Analog → ADC1;
 **               IN0 / IN1 / IN2 ：Single-ended
 **               Scan Conversion ：Enable
 **               Continuous Conversion ：Enable
 **               DMA Continuous Requests ：Enable
 **               Resolution ：12 Bit
 **               外设设置DMA：Normal→Circular，数据宽度HalfWord
 **
 ** 【初 始 化】  1-ADC1与DMA初始化由CubeMX自动生成;
 **               2-调用HAL_ADC_Start_DMA()启动连续转换+DMA搬运;
 **               3-主循环非阻塞读取adcValue数组即可;
 **
 ** 【ADC重点】   1-分辨率12位：数值范围0~4095，对应电压0~3.3V;
 **               2-电压计算公式：电压 = (ADC值 / 4095) × 3.3;
 **               3-DMA循环模式：自动不断刷新转换结果，无需手动重启;
 **               4-多通道扫描：硬件自动切换通道，无需软件干预;
 **
 ** 【移植说明】  1-增减采集通道只需修改CubeMX与DMA缓存长度;
 **               2-修改打印间隔只需调整500ms定时值;
 **               3-电压计算公式可根据参考电压灵活修改;
 **
 ** 【更新记录】  2026-03-26  整理注释
 **
 ** 【备注说明】  版权归魔女科技所有，请勿商用，谢谢！
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;
#include "bsp_UART.h"         // UART通信


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
// 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;
uint16_t adcValue[3] = {0};      // 缓存数组


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
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */
    // 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;
    UART1_Init(115200);                                       // UART1初始化

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcValue, 3);       // 使能ADC1+DMA，开始不断地转换


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /** 1ms延时间隔 **/
        HAL_Delay(1 - 1);                                              // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** 每500ms执行一次的任务 (读取转换结果)**/
        static uint32_t lastTime_500ms = 0;                            // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                     // 检查当前时间距离上次执行是否已经过了500毫秒; HAL_GetTick()是获取启动后经过的毫秒数
        {
            printf("IN0(PA0) = %04d     ", adcValue[0]);               // printf 通道0的ADC转换结果
            printf("IN1(PA1) = %04d     ", adcValue[1]);               // printf 通道1的ADC转换结果
            printf("IN2(PA2) = %04d   \r", adcValue[2]);               // printf 通道2的ADC转换结果

            float vol_0 = ((float)adcValue[0] / 4095) * 3.3f;          // 把通道0的ADC值，按比例计算成电压值
            float vol_1 = ((float)adcValue[1] / 4095) * 3.3f;          // 把通道1的ADC值，按比例计算成电压值
            float vol_2 = ((float)adcValue[2] / 4095) * 3.3f;          // 把通道2的ADC值，按比例计算成电压值

            printf("IN0(PA0) = %3.2fV    ",  vol_0);                   // printf 通道0 所采集到的电压值
            printf("IN1(PA1) = %3.2fV    ",  vol_1);                   // printf 通道1 所采集到的电压值
            printf("IN2(PA2) = %3.2fV \r\r", vol_2);                   // printf 通道2 所采集到的电压值

            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);    // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行

            lastTime_500ms =   HAL_GetTick();                          // 记录当前时间，作为新的基准点，为下一次判断做准备
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
