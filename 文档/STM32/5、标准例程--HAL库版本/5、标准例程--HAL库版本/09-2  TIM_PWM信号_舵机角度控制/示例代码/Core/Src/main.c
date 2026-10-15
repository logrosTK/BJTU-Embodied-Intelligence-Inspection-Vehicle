/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  通用定时器 TIM2 PWM 输出 —— 180°舵机角度控制
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【实验目标】  通过TIM2配置PWM输出，驱动180°模拟舵机;
 **               1-实现0°~180°精准角度控制;
 **               2-提供角度设置函数，使用更简洁;
 **               3-支持双通道PWM同时输出;
 **
 ** 【引脚接线】  TIM2_CH1对应PA0引脚，TIM2_CH2对应PA1引脚;
 **               舵机信号线接PA0/PA1，VCC接5V，GND共地; （注意，这里是指两个舵机的信号线，并非一个舵机有两个信号线)
 **               注意：舵机供电必须为5V，3.3V可能驱动无力或角度不准;
 **
 ** 【 CubeMX 】  打开Timers，选择TIM2;
 **               Clock Source(时钟源): Internal Clock      内部时钟
 **               Prescaler(PSC)       ：83                  分频值        时钟源84MHz分频为1MHz
 **               Counter Period(ARR)  ：19999               周期值        PWM周期20ms（50Hz）
 **               auto-reload preload  : Enable              预装载        使能
 **               CH1/CH2 Mode         : PWM Generation      PWM模式       通道1/2输出PWM
 **               CH1/CH2 Polarity     : High                高电平有效
 **
 ** 【初 始 化】  1-TIM2的PWM初始化代码，CubeMX会根据配置自动生成;
 **               2-在main函数中，调用HAL_TIM_PWM_Start()开启PWM输出;
 **               3-调用AnalogServo_1_SetAngle()设置舵机角度;
 **
 ** 【TIM 重点】  1_时钟频率：STM32F407默认系统频率168MHz; TIM2挂在APB1上，时钟频率84MHz;
 **               2_PSC预分频：控制计数器脉冲时长; 本实验设置为83，将84MHz分频为1MHz（1us）;
 **               3_ARR自动重载值：控制PWM周期; 本实验设置为19999，周期=20ms（标准舵机要求）;
 **               4_CCR捕获/比较值：控制高电平脉宽; 500us=0°，1500us=90°，2500us=180°;
 **               5_角度公式：CCR = 角度 × 11.11 + 500;
 **
 ** 【移植说明】  1-如需使用其他定时器通道，修改定时器句柄与引脚即可;
 **               2-如需控制270°舵机，需重新调整CCR计算公式;
 **               3-代码已做角度限幅，防止写入非法值;
 **
 ** 【更新记录】  2026-03-26  新建舵机PWM控制工程
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

/*********************************************************************************
 * 函  数：AnalogServo_1_SetAngle
 * 功  能：180°舵机角度控制
 * 参  数：float  angle ; 角度; 有效范围：0.0°~180.0°
 * 返回值：无
 ********************************************************************************/
void AnalogServo_1_SetAngle(float angle)
{
    uint16_t ccrValue = 0;

    if (angle > 180) angle = 180;     // 限制最大值，防止出错
    if (angle < 0) angle = 0;         // 限制最小值，防止出错

    ccrValue = angle * 11.11 + 500;   // 角度转为CCR值(脉宽), 单位us; 500us是0度，2500us是180度，因此每1度是11.11us
    TIM2->CCR1 = ccrValue;            // 设置比较器CCR的值(控制脉宽)

    //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccrValue); // 等同上面最后一句; 上面那行更简洁
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
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);      // 引脚置高电平，绿亮灭


    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);                // 启动TIM2的通道1 (PA0); 注意，启动后，它会默认输出CubeMX里设置的CCR值

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);                // 启动TIM2的通道2 (PA1); 注意，启动后，它会默认输出CubeMX里设置的CCR值


    //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);    // 设置CCR值，即改变脉宽
    AnalogServo_1_SetAngle(122.5);                           // 设置角度; 函数写在了main.c的上方
    
    

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */

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
