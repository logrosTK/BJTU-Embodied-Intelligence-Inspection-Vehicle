/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  GPIO 全引脚电平翻转测试
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库
 **
 ** 【功能概述】  本程序用于测试开发板 GPIO 引脚的基本输出功能，通过周期性翻转所有 GPIO 引脚电平。
 **               1. 初始化系统时钟及 GPIO（所有已使能的 GPIO 端口）
 **               2. 主循环中每隔 1 秒翻转一次 GPIOA、GPIOB、GPIOC、GPIOD、GPIOE 的所有引脚电平
 **               3. 可用于验证硬件连接、观察引脚波形或快速测试开发板
 **
 ** 【硬件注意】  1. 本程序会同时翻转多个 GPIO 端口的全部引脚，请确保外接设备不会因电平变化而损坏
 **               2. 若某些引脚用于特殊功能（如 JTAG/SWD），需先禁用相应功能或注意不影响调试
 **               3. 建议使用示波器或逻辑分析仪观察引脚输出波形
 **
 ** 【移植说明】  1. 本工程无需额外外设驱动文件，仅需 HAL 库支持
 **               2. 如需测试特定引脚，可修改 HAL_GPIO_TogglePin 中的端口及引脚宏
 **               3. 延时时间可通过修改 HAL_Delay 参数调整
 **
 ** 【使用说明】  1. 编译下载后，开发板所有 GPIO 引脚将每秒同步翻转一次
 **               2. 可用万用表或示波器测量任意引脚，观察电平周期性变化
 **               3. 该程序常用于验证最小系统是否正常工作，或排查引脚虚焊问题
 **
 ** 【调试建议】  1. 若程序无法运行，检查系统时钟配置及 HAL 初始化
 **               2. 若某引脚无输出，检查该引脚对应的 GPIO 时钟是否已使能（CubeMX 已自动处理）
 **               3. 注意避免引脚冲突（如与调试接口共用）
 **
 ** 【更新记录】  2025-03-27  创建 GPIO 全引脚翻转测试程序
 **
 ** 【备注说明】  版权归魔女科技所有，仅供学习使用，请勿用于商业用途。
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 所用用户代码，必须写在配对的BEGIN与END注释之间，否则重新生成时会被自动删除


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
    // 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;



    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;
        
        HAL_Delay(1000);                            // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大
       
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_All);    // 反转 GPIOA 引脚的电平    
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_All);    // 反转 GPIOB 引脚的电平  
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_All);    // 反转 GPIOC 引脚的电平  
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_All);    // 反转 GPIOD 引脚的电平  
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_All);    // 反转 GPIOE 引脚的电平  

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
    RCC_OscInitStruct.PLL.PLLQ = 7;
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
