/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  DHT22 温湿度传感器数据获取
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库 + DHT22
 **
 ** 【功能概述】  本程序通过单总线协议读取 DHT22 温湿度传感器的数据，并通过串口输出。
 **               1. 初始化系统时钟、GPIO 及 UART1
 **               2. 初始化 DHT22 传感器，指定数据引脚（PC3）
 **               3. 主循环中每 2 秒读取一次温湿度值（满足 DHT22 采样间隔要求），通过串口打印
 **               4. 绿色 LED 每 500ms 闪烁一次，指示系统运行状态
 **               5. 按键中断（KEY1/KEY2/KEY3）控制蓝色 LED 状态反转并输出提示
 **
 ** 【硬件接线】  DHT22 接线（可根据需要修改引脚，在 DHT22_Init 调用时指定）：
 **               VCC   — 3.3V 或 5V（建议5V）
 **               DATA  — PC3（三脚的模块可以直接连接杜邦线; 而四脚的DHT22必须外接 4.7kΩ 上拉电阻至 VCC）
 **               GND   — GND
 **               注意：DHT22 数据线必须外接上拉电阻，否则通信失败；若使用模块（带底板）通常已集成上拉电阻。
 **
 ** 【划重点】    1. 供电范围：3.3V~6.0V，工作电流 1.5mA，待机电流 150μA
 **               2. 上电稳定时间：约 2 秒，建议首次读取前延时 2 秒以上
 **               3. 采样周期：两次读取间隔必须大于 2 秒，过快会导致传感器无响应或数据错误
 **               4. 时序要求：DHT22 通信时序为微秒级，强烈建议使用定时器精确延时，避免循环延时受编译优化影响
 **               5. 精度说明：
 **                  - 温度范围：-40℃ ~ +80℃，精度 ±0.5℃（典型），分辨率 0.1℃
 **                  - 湿度范围：0% ~ 100%RH，精度 ±2%RH（典型），分辨率 0.1%RH
 **                  - 支持负温度显示（如 -12.3℃）
 **               6. 数据输出：40位数据 = 16位湿度 + 16位温度 + 8位校验和，高位在前
 **
 ** 【移植说明】  1. 复制 bsp_DHT22.c 和 bsp_DHT22.h 到工程目录
 **               2. 在 Keil 中添加文件路径：Options → C/C++ → Include Paths
 **               3. 将 bsp_DHT22.c 添加到工程
 **               4. 在需要使用 DHT22 的文件中包含 #include "bsp_DHT22.h"
 **               5. 调用 DHT22_Init(GPIOx, GPIO_PIN_x) 初始化引脚
 **               6. 调用 DHT22_GetData(float*, float*) 获取温湿度
 **
 ** 【函数使用】  已封装好初始化及数据获取函数，使用简单：
 **               1. DHT22_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)    // 初始化，指定引脚
 **               2. uint8_t DHT22_GetData(float *temp, float *humi)       // 获取温度（℃）和湿度（%），返回状态码
 **
 ** 【实验现象】  1. 串口助手每 2 秒输出一次温湿度数据，格式：DHT22  温度:xx.x℃   湿度:xx.xRH%
 **               2. 绿色 LED 每 500ms 闪烁一次，指示程序运行
 **               3. 按下 KEY1/KEY2/KEY3，蓝色 LED 状态反转，串口输出相应提示
 **
 ** 【调试建议】  1. 若首次读取数据错误，可增加 DHT22_Init 前的延时（等待传感器稳定）
 **               2. 若通信失败，检查数据线连接及上拉电阻，并用示波器或逻辑分析仪抓取时序
 **               3. 若使用系统滴答延时（HAL_Delay）影响时序，可改用定时器实现微秒延时
 **               4. 不同编译优化等级可能影响延时精度，建议优化等级设为 0 或使用精确延时
 **               5. DHT22 采样间隔必须大于 2 秒，代码中已设置为 2000ms
 **
 ** 【更新记录】  2026-04-07  从 DHT11 版本修改为 DHT22 版本，修正采样间隔为 2 秒
 **               2024-02-19  优化代码结构，增加详细注释（原DHT11记录）
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
#include "bsp_UART.h"            // 串口通信
#include "bsp_DHT22.h"           // 温湿度 DHT22

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
    // 所用用户代码，必须写在配对的BEGIN与END注释之间，否则重新生成时会被自动删除

    UART1_Init(115200);                // 初始化 UART1; 用于printf调试信息到串口助手
   
    HAL_Delay(2500);                   // 延时2秒; 据文档建议，上电后等待2s以待稳定再读数据
    DHT22_Init(GPIOC, GPIO_PIN_3);     // 初始化 DHT22、指定数据引脚




    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        // 所用用户代码，必须写在配对的BEGIN与END注释之间，否则重新生成时会被自动删除


        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                    // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)             // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);             // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                  // 记录当前时间，作为新的基准点，为下一次判断做准备
        }

        /** 每2000ms执行一次的任务 **/
        /* 注意：DHT22要求采样间隔 >= 2 秒 */
        static uint32_t lastTime_2000ms = 0;                   // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_2000ms >= 2000)           // 每2000ms执行一次（DHT22建议≥2000ms，请根据需要修改）
        {                                                   
            static float temp = 0.0;                           // 用于存储温度值
            static float humi = 0.0;                           // 用于存储湿度值
            printf("DHT22  ");                           
            DHT22_GetData( &temp,    &humi);                   // 获取DHT22 温度、湿度值
            printf("温度:%4.1f℃   ", temp);                    // 把数据输出到电脑串口软件，方便观察
            printf("湿度:%4.1fRH%%\r\n", humi);                // 把数据输出到电脑串口软件，方便观察
            lastTime_2000ms =   HAL_GetTick();                 // 记录当前时间，作为新的基准点，为下一次判断做准备
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
// 所用用户代码，必须写在配对的BEGIN与END注释之间，否则重新生成时会被自动删除

/******************************************************************************
 * 函  数： HAL_GPIO_EXTI_Callback
 * 功  能： EXTI中断回调函数
 * 参  数： uint16_t  GPIO_Pin    引脚编号，参数范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 返回值： 无
 * 备  注： CubeMX生成的代码在思维和风格上，与使用标准库编写时有很大的区别。
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
        printf("KEY_1 按下 \r\n");                              // 通过串口助手输出提示
    }

    if (GPIO_Pin == KEY_2_Pin)                                  // 按键2 产生的中断信号：KEY_2、PA1、闲时上拉、按下时低电平、下降沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯
        printf("KEY_2 按下 \r\n");                              // 通过串口助手输出提示
    }

    if (GPIO_Pin == KEY_3_Pin)                                  // 按键3 产生的中断信号：KEY_3、PA4、闲时上拉、按下低高电平、下降沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯
        printf("KEY_3 按下 \r\n");                              // 通过串口助手输出提示
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
