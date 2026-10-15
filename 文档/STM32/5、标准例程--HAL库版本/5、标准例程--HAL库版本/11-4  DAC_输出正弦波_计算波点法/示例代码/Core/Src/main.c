/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  DAC + TIM6 + DMA 双通道正弦波输出
 **
 ** 【适用平台】  STM32F407 + CubeIDE + HAL库
 **
 ** 【实验目标】  1. 实现 PA4、PA5 双通道同步输出高精度正弦波
 **               2. 采用 168 点动态计算波形表，输出平滑无失真
 **               3. TIM6 硬件触发 + DMA 循环搬运，CPU 零占用
 **               4. 系统运行时 LED 周期性闪烁，指示工作状态
 **
 ** 【引脚接线】  DAC_OUT1 → PA4
 **               DAC_OUT2 → PA5
 **               系统运行指示灯 → PC5
 **
 ** 【 CubeMX 】  1. 开启 DAC 双通道，触发源选择 TIM6_TRGO
 **               2. 开启 TIM6 定时器，配置触发频率
 **               3. 开启 DMA 循环模式，配置存储器到外设传输
 **               4. 配置 UART1 用于调试打印
 **
 ** 【初 始 化】  1. 外设初始化由 CubeMX 自动生成
 **               2. 上电调用 DAC_SineStart() 启动波形输出
 **               3. 全程硬件自动运行，无需主循环干预
 **
 ** 【重点说明】  1. 正弦波数据动态生成，可灵活修改点数、幅值、偏置
 **               2. 输出范围 0~3.3V，12 位精度（0~4095）
 **               3. 波形频率由 TIM6 分频/重载值决定
 **               4. 双通道同步输出相同波形
 **
 ** 【移植说明】  1. 修改频率、波形 → 修改 DAC_SineStart里面的参数组合: WAVE_POINTS、MIN/MAX_VALUE 等
 **               2. 修改通道 → 开启对应 DAC 通道 DMA
 **
 ** 【更新记录】  2026-03-26  完善注释
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






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;



/******************************************************************************
 * 函数名 : DAC_SineStart
 * 功  能 : 启动DAC正弦波输出 (动态计算波形表) 
 * 备  注 : 1. 波点数可通过宏定义WAVE_POINTS灵活调整; 
 *          2. 波点数的取值，建议根据 TIM时钟频率即值，如84MHz时钟下，取值21、42、84、168等，能令公式整除，以达到精准的波形频率
 *          2. 输出幅值：[MIN_VALUE, MAX_VALUE]（默认 0-4095）
 *          3. 直流偏置：OFFSET（默认 0）
 *          4. TIM6 更新事件触发，DMA 循环模式
 *          5. 上电重新计算表，调试方便；若需提速，可在调试成型后改用 const 静态表
 * 参  数 : 无 (由函数内宏定义控制)
 * 返回值 : 无
 ******************************************************************************/
#include "math.h"

void DAC_SineStart(void)
{
    /* 1、定义参数 */
    #define WAVE_POINTS     168                      // 正弦数据点数; 取值建议根据 TIM时钟频率即值，如84MHz时钟下，取值21、42、84、168等，能令公式整除，以达到精准的波形频率
    #define MIN_VALUE         0                      // 正弦数据最小值; 即，波底的DAC值; 取值范围：0~4095 (对应0V~3.3V); 用于适配不同设备的工作安全范围，减少硬件电路的设计修改;
    #define MAX_VALUE      4095                      // 正弦数据最大值; 即，波顶的DAC值; 取值范围：0~4095 (对应0V~3.3V); 用于适配不同设备的工作安全范围，减少硬件电路的设计修改;
    #define OFFSET            0                      // 偏移量; 控制波形的直流偏置 (垂直位置)
    #define SCALE ((MAX_VALUE - MIN_VALUE) / 2.0)    // 缩放系数; 控制波形的幅度 (振幅大小)
    
    /* 2、开辟缓存 */
    static uint16_t sineData[WAVE_POINTS] = { 0 };   // 存储正弦波点值

    /* 3、生成无缝正弦表
          计算法：上电实时算表
          优点：调试阶段可随时改 WAVE_POINTS / MIN / MAX，方便灵活调试出需要的波形、振幅
          缺点：每次上电运行都得跑一遍 sin()，启动时间随点数线性增加; 调试定型后建议改用 const 静态表，上电即用，零计算延时  
    */
    for (uint16_t i = 0; i < WAVE_POINTS; i++)
    {
        double x = (double)i * (2.0 * 3.14159265 / WAVE_POINTS);              // 周期内均匀采样; 将索引转换为0到2π之间的值
        double sinValue = sin(x);                                             // 计算正弦值
        sineData[i] = (uint16_t)(lround((sinValue + 1.0) * SCALE + OFFSET));  // 缩放和平移正弦值        
        
    }
 
    /* 4、启动 TIM */
    HAL_TIM_Base_Start(&htim6);                                               // 启动TIM。每个更新事件(计数达到一周期)，由硬件发出脉冲信号触发DAC转换一次

    /* 5、启动DAC的DMA传输 */
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)sineData, WAVE_POINTS, DAC_ALIGN_12B_R);    // 启动DAC通道1的DMA传输，数据输出至PA4引脚
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t *)sineData, WAVE_POINTS, DAC_ALIGN_12B_R);    // 启动DAC通道2的DMA传输，数据输出至PA5引脚
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
    MX_DMA_Init();
    MX_DAC_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */
    // 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;

    UART1_Init(115200);              // UART1初始化

    DAC_SineStart();                 // 启动波形输出



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
