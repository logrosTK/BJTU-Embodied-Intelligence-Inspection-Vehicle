/* USER CODE BEGIN Header */
/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【实验名称】  STM32F407 GPIO 输入 —— 按键控制 LED 翻转
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【 CubeMX 】  1- 开启外部高速晶振 HSE，系统时钟配置为 168MHz；
 **               2- PB2、PC5 配置为 GPIO 推挽输出（蓝灯、绿灯）；
 **               3- PA0、PA1、PA4 配置为 GPIO 输入模式（KEY1、KEY2、KEY3）；
 **               4- PA0 下拉输入，PA1/PA4 上拉输入；
 **
 ** 【功能说明】  本工程实现 GPIO 输入检测 + 输出控制：
 **               1- 按键 KEY1（PA0）、KEY2（PA1）、KEY3（PA4）控制蓝灯翻转；
 **               2- 绿灯 500ms 无阻塞自动闪烁；
 **               3- 按键采用电平检测方式，松手生效，无阻塞运行；
 **               4- 适合新手学习：GPIO 输入、按键检测、无阻塞定时；
 **
 ** 【代码使用】  1- 上电：蓝灯亮、绿灯灭；
 **               2- 按任意键（KEY1/2/3）→ 翻转蓝灯亮灭；
 **               3- 绿灯 500ms 闪烁一次，指示系统运行；
 **
 ** 【划 重 点】  1- KEY1(PA0)：下拉输入 → 按下=高电平；
 **               2- KEY2/3(PA1/PA4)：上拉输入 → 按下=低电平；
 **               3- LED 低电平点亮，高电平熄灭；
 **               4- 无阻塞闪烁：使用 HAL_GetTick()，不卡住主循环；
 **               5- 按键松手检测：while 等待引脚恢复空闲电平；
 **
 *******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
    /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);    // 蓝灯，置低电平，点亮
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);    // 绿灯，置高电平，熄灭

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 所有用户代码，必须写在配对的BEGIN与END之间 */
      
        /** 1ms 延时 **/
        HAL_Delay(1 - 1);                                                  // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大
        
        /** 判断 KEY_1 是否按下 **/
        if (1 == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))                      // 判断KEY_1是否被按下：引脚PA0、闲时下拉、按下时高电平
        {
            while (1 == HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin));     // 等待按键释放；也可以使用常规的延时判断，这里这种写法是为了方便展示
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                         // 反转蓝色LED灯电平状态(PB2)
        }
        /** 判断 KEY_2 是否按下 **/
        if (0 == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))                      // 判断KEY_2是否被按下：引脚PA1、闲时上拉、按下时低电平
        {
            while (0 == HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin));     // 等待按键释放；也可以使用常规的延时判断，这里这种写法是为了方便展示
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                         // 反转蓝色LED灯电平状态(PB2)
        }
        /** 判断 KEY_3 是否按下 **/
        if (0 == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))                      // 判断KEY_3是否被按下：引脚PA4、闲时上拉、按下时低电平
        {
            while (0 == HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin));     // 等待按键释放；也可以使用常规的延时判断，这里这种写法是为了方便展示
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                         // 反转蓝色LED灯电平状态(PB2)
        }

        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                                // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                         // 检查当前时间距离上次执行是否已经过了500毫秒; HAL_GetTick()是获取启动后经过的毫秒数
        {                                                                 
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);        // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                              // 记录当前时间，作为新的基准点，为下一次判断做准备
        }

    } // end While

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
