/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  DAC+TIM6+DMA 双通道正弦波输出
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【实验目标】  通过TIM6触发、DMA转运、DAC输出，生成高精度正弦波;
 **               1-实现PA4、PA5双通道同步输出正弦波;
 **               2-128点波形表，波形平滑无失真;
 **               3-硬件自动运行，CPU零占用;
 **               4-LED闪烁指示系统正常运行;
 **
 ** 【引脚接线】  DAC_OUT1 → PA4
 **               DAC_OUT2 → PA5
 **               输出波形：正弦波 0~3.3V
 **
 ** 【 CubeMX 】  打开Analog → DAC、Timers → TIM6、Middleware → DMA;
 **               DAC：双通道使能，触发源TIM6_TRGO，DMA使能
 **               TIM6：配置触发频率，控制正弦波输出频率
 **               DMA：循环模式，数据宽度半字，存储器到外设
 **
 ** 【初 始 化】  1-DAC/TIM6/DMA初始化由CubeMX自动生成;
 **               2-调用HAL_TIM_Base_Start()启动定时器触发;
 **               3-调用HAL_DAC_Start_DMA()启动波形连续输出;
 **
 ** 【重点说明】  1_波形数据：128点12位正弦波表，范围0~4095，偏置2048;
 **               2_硬件协同：TIM6定时触发 → DMA自动搬数据 → DAC输出电压;
 **               3_无CPU占用：一旦启动，全程硬件自动运行，不占用主循环资源;
 **               4_双通道同步：两路DAC使用同一套波形表，同步输出相同波形;
 **
 ** 【移植说明】  1-修改波形频率只需调整TIM6分频/重载值;
 **               2-更换波形只需修改sineData数组内容;
 **               3-增减通道只需开启对应DAC通道DMA;
 **
 ** 【更新记录】  2026-03-26  新建DAC+TIM+DMA正弦波工程
 **
 ** 【备注说明】  版权归魔女科技所有，请勿商用，谢谢！
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
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

#define WAVE_POINTS 128       // 正弦波一个周期的采样点数

/* 1、正弦波数据表 (12位DAC，数值范围0~4095)
 * 使用数组法的优点：减少每次启动时的计算时间，保证波形精度
 * 注意事项：若将数组定义在函数内部，建议使用static修饰，避免栈溢出
 */
static const uint16_t sineData[WAVE_POINTS] =     
{
    /* 0°~90° 上升段 (n=0~31) */
    2048, 2148, 2248, 2348, 2447, 2545, 2642, 2737,
    2831, 2923, 3013, 3100, 3185, 3267, 3346, 3423,
    3495, 3565, 3630, 3692, 3750, 3804, 3853, 3898,
    3939, 3975, 4007, 4034, 4056, 4073, 4085, 4093,

    /* 90°~180° 下降段 (n=32~63) */
    4095, 4093, 4085, 4073, 4056, 4034, 4007, 3975,
    3939, 3898, 3853, 3804, 3750, 3692, 3630, 3565,
    3495, 3423, 3346, 3267, 3185, 3100, 3013, 2923,
    2831, 2737, 2642, 2545, 2447, 2348, 2248, 2148,

    /* 180°~270° 下降段 (n=64~95) */
    2048, 1947, 1847, 1747, 1648, 1550, 1453, 1358,
    1264, 1172, 1082,  995,  910,  828,  749,  672,
     600,  530,  465,  403,  345,  291,  242,  197,
     156,  120,   88,   61,   39,   22,   10,    2,

    /* 270°~360° 上升段 (n=96~127) */
       0,    2,   10,   22,   39,   61,   88,  120,
     156,  197,  242,  291,  345,  403,  465,  530,
     600,  672,  749,  828,  910,  995, 1082, 1172,
    1264, 1358, 1453, 1550, 1648, 1747, 1847, 1947
};



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
    MX_DAC_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */
    // 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;

    UART1_Init(115200);                                                                           // UART1初始化

    /* 2、启动定时器TIM6-产生精确的触发脉冲 */
    HAL_TIM_Base_Start(&htim6);                                                                   // 启动TIM。每个更新事件(计数达到一周期)，由硬件发出脉冲信号触发DAC转换一次

    /* 3、启动DAC通道1的DMA传输 (输出至PA4) */
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)sineData, WAVE_POINTS, DAC_ALIGN_12B_R);  // 启动DAC通道1的DMA传输，数据输出至PA4引脚
   
    /* 启动DAC通道2的DMA传输 (输出至PA5) */
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t *)sineData, WAVE_POINTS, DAC_ALIGN_12B_R);  // 启动DAC通道2的DMA传输，数据输出至PA5引脚



    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        /** 每500ms规律地闪烁绿色LED, 便于观察程序是否运行、卡死 **/
        static uint32_t lastTime_500ms = 0;                                     // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                              // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);                              // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                                   // 记录当前时间，作为新的基准点，为下一次判断做准备
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
