/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  通用定时器 TIM4 PWM呼吸灯
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【实验目标】  通过TIM4配置PWM输出，实现呼吸灯效果;
 **               1-模拟真实呼吸节奏：吸气快、呼气慢、带停顿;
 **               2-支持LED反接设计：PWM引脚接LED负极;
 **               3-非阻塞实现：不影响系统其他任务;
 **
 ** 【引脚接线】  TIM4_CH1对应PD12引脚;
 **               LED正极接3.3V，负极接PB6;
 **               注意：此为LED反接方式，CCR=0时LED最亮，CCR=1000时LED熄灭;
 **
 ** 【 CubeMX 】  打开Timers，选择TIM4;
 **               Clock Source(时钟源): Internal Clock     内部时钟
 **               Prescaler(PSC)      ：84-1               分频值       时钟源84MHz分频为1MHz
 **               Counter Period(ARR) ：1000-1             周期值       PWM频率1kHz
 **               auto-reload preload : Enable             预装载       使能
 **               CH1 Mode            : PWM Generation CH1 PWM模式      通道1输出PWM
 **               CH1 Polarity        : High               高电平有效   CCR值越小，PWM低电平时间越长
 **
 ** 【初 始 化】  1-TIM4的PWM初始化代码，CubeMX会根据配置自动生成;
 **               2-在main函数中，调用HAL_TIM_PWM_Start()开启PWM输出;
 **               3-在while循环中，调用本呼吸灯函数 (共两个测试函数);
 **
 ** 【TIM 重点】  1_时钟频率：STM32F407默认系统频率168MHz; TIM4挂在APB1上，时钟频率84MHz;
 **               2_PSC预分频：控制计数器每一脉冲的时长; 本实验设置为84-1，将84MHz分频为1MHz;
 **               3_ARR自动重载值：控制PWM周期; 本实验设置为1000-1，PWM频率=1MHz/1000=1kHz;
 **               4_CCR捕获/比较值：控制PWM占空比; 本实验中动态调整CCR值实现呼吸效果;
 **               5_输出极性：LED反接时，CCR值越小，PWM低电平时间越长，LED越亮;
 **
 ** 【移植说明】  1-如需使用其他定时器，请修改CubeMX配置和代码中的htim4变量;
 **               2-如需调整呼吸节奏，可修改状态保持时间和速度系数;
 **               3-如需调整PWM范围，修改亮度转PWM的公式;
 **
 ** 【更新记录】  2025-12-31  创建呼吸灯函数
 **
 ** 【备注说明】  版权归魔女科技所有，请勿商用，谢谢！
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
/* 用户代码，必须写在配对的BEGIN与END之间，否则CubeMX重新生成后，会被删除掉 */

#include "bsp_UART.h"            // 串口通信底层驱动文件; 已重写好初始化、收发，调用函数即可使用串口



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
 * 函  数： BreathingLED_1
 * 功  能： 呼吸灯 (状态机+非线性控制)
 * 备  注： 1、稍占计算资源
 *          2、LED反接：CCR=0时最亮，CCR=1000时灭
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void BreathingLED_1(void)
{
    // 呼吸状态定义
    static enum { 
        IN,         // 吸气状态（快速变亮）
        HOLD_HI,    // 高亮保持状态
        OUT,        // 呼气状态（缓慢变暗）
        HOLD_LO     // 低亮保持状态（完全熄灭）
    } state = IN;
    
    static float bright = 0.0f;       // 亮度值：0.0（熄灭）~ 1.0（最亮）
    static uint32_t state_start = 0;  // 当前状态开始时间
    static uint32_t last = 0;         // 上次更新时间
    
    // 每10ms更新一次（非阻塞设计）
    if (HAL_GetTick() - last < 10) return;
    last = HAL_GetTick();
    
    // 状态机处理
    switch (state)
    {
        case IN:  // 吸气阶段：快速变亮
            // 非线性加速公式：速度随亮度增加
            bright += (0.5f + bright * 1.5f) * 0.012f;
            
            // 到达最亮，切换状态
            if (bright >= 1.0f)
            {
                bright = 1.0f;                // 限制亮度为最大值
                state = HOLD_HI;              // 切换到高亮保持
                state_start = HAL_GetTick();  // 记录状态开始时间
            }
            break;
            
        case HOLD_HI:  // 高亮保持阶段：保持200ms
            // 保持200ms后切换到呼气
            if (HAL_GetTick() - state_start >= 200)
            {
                state = OUT;  // 切换到呼气状态
            }
            break;
            
        case OUT:  // 呼气阶段：缓慢变暗
            // 非线性减速公式：速度随亮度减少
            // 修改系数：从0.006f改为0.004f，使呼气更慢
            bright -= (2.0f - bright * 1.5f) * 0.004f;
            
            // 到达完全熄灭，切换状态
            if (bright <= 0.0f)
            {
                bright = 0.0f;                // 完全熄灭
                state = HOLD_LO;              // 切换到低亮保持（实际完全熄灭）
                state_start = HAL_GetTick();  // 记录状态开始时间
            }
            break;
            
        case HOLD_LO:  // 低亮保持阶段：完全熄灭，保持300ms
            // 保持300ms后重新开始呼吸
            if (HAL_GetTick() - state_start >= 300)
            {
                state = IN;  // 重新开始吸气
            }
            break;
    }
    
    // LED反接处理：亮度值转换为PWM值
    // 公式：PWM = (1.0 - bright) * 1000
    // 当bright=1.0（最亮）时，PWM=0
    // 当bright=0.0（熄灭）时，PWM=1000
    uint16_t pwm = (uint16_t)((1.0f - bright) * 1000.0f);
    
    // 更新PWM输出
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
}



/******************************************************************************
 * 函  数： BreathingLED_2
 * 功  能： 呼吸灯_查表法
 * 备  注： 1、省计算资源的查表实现
 *          2、LED反接：CCR=0时最亮，CCR=1000时灭
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void BreathingLED_2(void)
{
    static uint16_t idx = 0;
    static uint32_t lastTime = 0;

    // 呼吸表（110个点，吸气快，呼气慢）
    static const uint16_t breath_tab[] =
    {
        15, 18, 22, 27, 33, 40, 48, 57, 67, 78, 90, 103, 117, 132, 148, 165, 183, 202, 222, 243,
        265, 288, 312, 337, 363, 390, 418, 447, 477, 508, 540, 573, 607, 642, 678, 715, 753, 792,
        832, 873, 915, 958, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 985, 965, 945, 925, 905, 885, 865, 845,
        825, 805, 785, 765, 745, 725, 705, 685, 665, 645, 625, 605, 585, 565, 545, 525, 505, 485,
        465, 445, 425, 405, 385, 365, 345, 325, 305, 285, 265, 245, 225, 205, 185, 165, 145, 125,
        105, 85, 65, 45
    };
    
    // 自动计算数组大小
    #define TABLE_SIZE (sizeof(breath_tab) / sizeof(breath_tab[0]))

    // 30ms更新一次
    if (HAL_GetTick() - lastTime < 30) return;

    // 更新PWM输出 (PWM引脚连接的是LED负极，因此用1000减它以取反值)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000-breath_tab[idx]);

    // 更新索引（循环）
    idx = (idx + 1) % TABLE_SIZE;

    // 保存最后一次执行的时间
    lastTime = HAL_GetTick();
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
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);      // 引脚置高电平，绿亮灭

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 999);       // 设置TIM4通道1的CCR值，简单理解为高电平的脉冲个数; 是的，我们应该把它放在PWM_Start之前，以确定PWM_Start启动的瞬间输出预期的信号，而不是CubeMX里的默认值。
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);                // 启动TIM4_CH1的PWM输出;


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        /** 呼吸灯任务 **/
        BreathingLED_1();                                     // 注意，需要使用杜邦线，连接PD12到PB2; 本示例共提供了两个呼吸灯测试函数。
        
        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                   // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)            // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {                                                     
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);            // 1.执行动作。控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                 // 2.更新本任务最后执行的时间值
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
/* 所有用户代码，必须写在配对的BEGIN与END之间 */





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
