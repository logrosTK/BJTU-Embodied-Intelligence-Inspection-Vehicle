/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  SR04 超声波测距模块数据采集
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL库 + SR04
 **
 ** 【功能概述】  本程序通过定时器捕获方式测量 SR04 超声波模块的回波脉冲宽度，计算距离并通过串口输出。
 **               1. 初始化系统时钟、GPIO 及 UART1
 **               2. 初始化 SR04 模块（配置 Trig 输出引脚、Echo 输入及定时器捕获）
 **               3. 主循环中每 1 秒触发一次测距，读取距离值并滤波输出
 **               4. 绿色 LED 每 500ms 闪烁一次，指示系统运行状态
 **               5. 按键中断（KEY1/KEY2/KEY3）控制蓝色 LED 状态反转并输出提示
 **               6. 支持串口接收数据并回显（测试用）
 **
 ** 【硬件接线】  SR04 模块接线（引脚可在 bsp_SR04.h 中修改）：
 **               VCC   — 5.0V  （SR04 必须使用 5V 供电）
 **               Trig  — PC0   （触发信号，输出）
 **               Echo  — PC2   （回响信号，输入捕获）
 **               GND   — GND
 **               注意：有效测量范围 2cm ~ 400cm，测量角度约 15°。
 **
 ** 【文件说明】  本示例依赖以下 2 个文件，均已封装好底层接口：
 **               - bsp_SR04.c / bsp_SR04.h   ：SR04 驱动核心（定时器捕获 + 引脚控制）
 **
 ** 【函数使用】  使用非常简单，只需两个函数：
 **               1. SR04_Init(void)          // 初始化引脚和定时器
 **               2. float SR04_GetCM(void)   // 触发一次测距，返回距离值（厘米）
 **
 ** 【移植说明】  1. 复制 bsp_SR04.c 和 bsp_SR04.h 到工程目录
 **               2. 在 Keil 中添加文件路径：Options → C/C++ → Include Paths
 **               3. 将 bsp_SR04.c 添加到工程
 **               4. 在需要使用 SR04 的文件中包含 #include "bsp_SR04.h"
 **               5. 如需修改 Trig/Echo 引脚，请修改 bsp_SR04.h 中的宏定义
 **               6. 注意确保定时器捕获通道与 Echo 引脚匹配（本示例使用通用定时器）
 **
 ** 【调试建议】  1. 若测量数据异常，检查模块供电是否为 5V（3.3V 可能无法工作）
 **               2. 测量距离时，模块前方不要有障碍物，避免多次反射干扰
 **               3. 建议测量间隔 ≥ 60ms，避免连续触发导致回波重叠
 **               4. 可使用示波器观察 Trig 和 Echo 波形，确认时序正常
 **               5. 本示例采用了 0.1*旧值 + 0.9*新值 的滤波，若需要快速响应可取消滤波
 **
 ** 【实验现象】  1. 串口助手每 1 秒输出一次距离值，格式：SR04测量距离值： 12.3CM
 **               2. 绿色 LED 每 500ms 闪烁一次，指示程序运行
 **               3. 按下 KEY1/KEY2/KEY3，蓝色 LED 状态反转，串口输出相应提示
 **               4. 通过串口向开发板发送任意数据，会回显接收内容
 **
 ** 【更新记录】  2024-02-19  优化代码结构，增加详细注释
 **               2024-02-17  从标准库迁移至 HAL 库
 **
 ** 【备注说明】  版权归魔女科技所有，仅供学习使用，请勿用于商业用途。
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 所用用户代码，必须写在配对的BEGIN与END注释之间，否则重新生成时会被自动删除

#include "bsp_UART.h"           // 串口通信
#include "bsp_SR04.h"

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
static void MX_GPIO_Init(void);
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
    // 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);    // 点亮LED_BLUE: PB2置低电平
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);    // 点亮LED_RED: PC5置低电平

    UART1_Init(115200);                                      // UART1 初始化; 已在bsp_UART.c中处理printf的重定向输出到UART1

    SR04_Init();                                             // SR04 初始化：引脚、TIM

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;

        /** 1ms延时 **/
        HAL_Delay(1 - 1);                                      // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /******  检查 USART1 是否收到数据 ******/
        if (UART1_GetRxNum())                                  // 如果USART1收到数据(从串口上位机中发过来的）
        {
            // 获取数据
            uint16_t rxNum  = UART1_GetRxNum();                // 获取最后一帧的字节数
            uint8_t *rxData = UART1_GetRxData();               // 获取最后一帧的数据(地址)
            // 把收到的数据，printf到串口助手观察
            printf("\r<<<<<< USART1 接收到新数据 <<<<<<");     // 准备输出
            printf("\r 长度(字节数)：%d", rxNum);              // printf 字节数
            printf("\r 数据(16进制)：");                       // printf 16进制方式显示数据，方便观察真实数据
            for (uint16_t i = 0; i < rxNum; i++)               // 输出每一字节数值
                printf("0x%X ", rxData[i]);
            printf("\r 数据(ASCII) ：%s\r", rxData);           // ASCII方式显示数据，方便观察字符串数据;
            // 清0本帧接收
            UART1_ClearRx();                                   // 清0接收(接收字节数); 重要: 处理完数据后，谨记把接收数量清０, 再进入下次判断;
        }

        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                    // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)             // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);             // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                  // 记录当前时间，作为新的基准点，为下一次判断做准备
        }

        /** 每1000ms执行一次的任务 **/
        static uint32_t lastTime_1000ms = 0;                   // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_1000ms >= 1000)           // 每1000ms执行一次
        {
            static double cmValue = 0.0;                       // SR04测得的距离值，单位：厘米; 注意，cmValue用static修饰，初始化一次，程序运行期间会保留最后一次更新的值
            cmValue = cmValue * 0.1f + SR04_GetCM() * 0.9f;    // 获取SR04的数据，并转化成厘米值; 注意，算式中的 旧值*0.1+新值*0.9，是为了输出结果的平稳
            printf("SR04测量距离值：%5.1fCM\r\n", cmValue);    // 结果输出到串口上位机中

            lastTime_1000ms =   HAL_GetTick();                 // 记录当前时间，作为新的基准点，为下一次判断做准备
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : KEY_1_Pin */
    GPIO_InitStruct.Pin = KEY_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(KEY_1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : KEY_2_Pin KEY_3_Pin */
    GPIO_InitStruct.Pin = KEY_2_Pin | KEY_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : LED_GREEN_Pin */
    GPIO_InitStruct.Pin = LED_GREEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LED_BLUE_Pin */
    GPIO_InitStruct.Pin = LED_BLUE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

/******************************************************************************
 * 函  数： HAL_GPIO_EXTI_Callback
 * 功  能： EXTI中断回调函数
 * 参  数： uint16_t  GPIO_Pin    引脚编号，参数范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 返回值： 无
 * 备  注： CubeMX生成的代码在思维和风格上，与使用标准库编写时有很大的区别：
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
        printf("KEY_1 按下\r");                                 // 通过串口助手输出提示
        //XPT2046_ReCalibration();                              // 触摸屏重新校准
    }

    if (GPIO_Pin == KEY_2_Pin)                                  // 按键2 产生的中断信号：KEY_2、PA1、闲时上拉、按下时低电平、下降沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯
        printf("KEY_2 按下\r");                                 // 通过串口助手输出提示
    }

    if (GPIO_Pin == KEY_3_Pin)                                  // 按键3 产生的中断信号：KEY_3、PA4、闲时上拉、按下低高电平、下降沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯
        printf("KEY_3 按下\r");                                 // 通过串口助手输出提示
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
