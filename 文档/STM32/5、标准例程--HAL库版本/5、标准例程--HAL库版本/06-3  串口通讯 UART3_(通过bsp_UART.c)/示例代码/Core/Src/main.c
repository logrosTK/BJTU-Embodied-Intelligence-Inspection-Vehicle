/* USER CODE BEGIN Header */
/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【实验名称】  UART3 通信（接收数据 + 解析控制LED + 定时闪烁）
 **
 ** 【 约  定 】  本示例所用串口通信，为异步通信。
 **               2024年起更新的示例，串口通信函数命名时，统一使用"UART"，而不使用旧文件中的"USART".
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【 CubeMX 】  无需通过CubeMX、CubeIDE开启、配置串口;
 **               bsp_UART.c中已重写好底层驱动文件，只需调用bsp_UART.h中声明的全局函数，即可初始化、发送、接收.
 **               如果还是想通过CubeMX开启串口，以方便标记使用哪些引脚，可以的，但是: 不能在CubeMX或CubeIDE中打勾使能"中断"、配置DMA, 否则冲突。
 **               即，无论有没有通过CubeMX开启UART，都能使用bsp_UART.h的函数，只要不打勾"中断”、"DMA"。
 **
 ** 【移植说明】  1- 复制本工程的UART文件夹，到目标工程文件夹中;
 **               2- 添加头文件路径： Keil > Option > C/C++ > Include Paths;
 **               3- 添加C文件到工程: Keil > 左侧工程管理器中双击目标文件夹 > 选择 bsp_UART.c;
 **               4- 添加文件引用:    #include "bsp_uart.h＂，即哪个文件要用串口功能，就在其代码顶部添加引用;
 **
 ** 【串口引脚】  六个串口的初始化函数UARTx_Init()，将按以下默认引脚进行初始化：
 **               1- UART1  TX_PA9   RX_PA10       特别说明：魔女开发板已板载虚拟串口，连接UART1，使用烧录USB即可与电脑通信
 **               2- UART2  TX_PA2   RX_PA3
 **               3- UART3  TX_PB10  RX_PB11
 **               4- UART4  TX_PC10  RX_PC11
 **               5- UART5  TX_PC12  RX_PD2
 **               6- UART6  TX_PC6   RX_PC7
 **
 ** 【代码功能】  1- UART1 / UART3 初始化（115200波特率）
 **               2- UART1 接收数据，自动解析 ASCII + 十六进制打印
 **               3- UART1 接收字符串包含"LED"时，翻转蓝色LED状态
 **               4- UART1 收到数据后，通过 UART3 转发发送
 **               5- UART3 收到数据后，通过 printf 输出到串口助手（UART1）方便调试
 **               6- 绿色LED 500ms 定时闪烁
 **               7- 按键中断可控制LED翻转
 **
 ** 【代码使用】  每组串口已封装7个函数，以UART1示范说明：
 **               初始化:             void      UART1_Init(uint32_t baudrate);                                        // 初始化串口; 配置GPIO引脚PA9+PA10、配置通信参数：波特率-None-8-1、配置中断
 **               发送字符串:         void      UART1_SendString(const char *string,...);                             // 发送字符串;   参数：字符串地址; 使用方法如同printf
 **               发送指定长度：      void      UART1_SendData(uint8_t *data, uint16_t num);                          // 发送指定数据; 参数：数据地址、字节数
 **               发送AT指令字符串：  uint8_t   UART1_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 针对AT指令设备，发送并等待应答; 返回0失败/1成功
 **               获取接收的字节数:   uint16_t  UART1_GetRxNum(void);                                                 // 获取接收到的字节数; 大于0表示接收到新一帧数据
 **               获取接收的数据：    uint8_t*  UART1_GetRxData(void);                                                // 获取接收缓冲区首地址
 **               清零接收标志：      void      UART1_ClearRx(void);                                                  // 清空接收计数，处理完数据必须调用
 **
 *******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp_UART.h"            // 串口底层驱动文件; 已重写好初始化、收发，调用函数即可使用串口

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
/* 用户代码，必须写在配对的BEGIN与END之间 */



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
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);                    // 引脚置高电平，绿亮灭

    UART1_Init(115200);                                                    // 初始化 串口1; 已写好底层，调用h中的函数即可使用; 引脚(TX-PA9 + RX-PA10)、波特率-None-8-1; 如果使用CubeMX配置，请使用前述引脚，但，不要在MX上进行中断及DMA配置，否则冲突
    UART3_Init(115200);                                                    // 初始化 串口3; 引脚 (TX-PB10、RX-PB11)


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */

        /** 1ms延时 **/
        HAL_Delay(1 - 1);                                                  // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** 检查UART1是否收到数据 (串口助手) **/
        if (UART1_GetRxNum())                                              // 判断UART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            uint8_t *rxData = UART1_GetRxData();                           // 获取数据的地址
            uint16_t rxNum = UART1_GetRxNum();                             // 获取数据的字节数
            printf("\r<<<<< UART1 接收到一帧数据 \r");                     // 提示接收到新一帧
            printf("字节数：%d \r", rxNum);                                // 显示字节数
            printf("ASCII : %s\r", (char *)rxData);                        // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                            // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < rxNum; i++)                           // 逐个字节输出
                printf("0x%02X ", rxData[i]);                              // 以16进制显示
            printf("\r\r");                                                // 显示换行
            if (strstr((char *)UART1_GetRxData(), "LED"))                  // 判断接收到的字符串，是否有"LED"字样，如果有，就执行
            {
                HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);      // 反转LED电平
            }
            UART3_SendData(rxData, rxNum);                                 // 把数据转发出去
            UART1_ClearRx();                                               // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0
            // UART1_SendString("Hello World！\r");                        // 示范 UART1_SendString( )的使用; 可接受像pritnf的格式化参数
            // UART1_SendData((uint8_t*)"Hello World!\r", 13);             // 示范 UART1_SendData( )的使用, 可用于发送字符串、数组、结构体数据等
        }

        /** 检查UART3是否收到数据 **/
        if (UART3_GetRxNum())                                              // 判断UART3是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            uint8_t *rxData = UART3_GetRxData();                           // 获取数据的地址
            uint16_t rxNum = UART3_GetRxNum();                             // 获取数据的字节数
            printf("\r<<<<< UART3 接收到一帧数据 \r");                     // 提示接收
            printf("字节数：%d \r", rxNum);                                // 显示字节数
            printf("ASCII : %s\r", (char *)rxData);                        // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                            // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < rxNum; i++)                           // 逐个字节输出
                printf("0x%02X ", rxData[i]);                              // 以16进制显示
            printf("\r\r");                                                // 显示换行
            UART3_ClearRx();                                               // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0
            // UART3_SendString("Hello World！\r");                        // 示范 UART3_SendString( )的使用; 可接受像pritnf的格式化参数
            // UART3_SendData((uint8_t*)"Hello World!\r", 13);             // 示范 UART3_SendData( )的使用, 可用于发送字符串、数组、结构体数据等
        }

        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                                // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                         // 检查当前时间距离上次执行是否已经过了500毫秒; HAL_GetTick()是获取启动后经过的毫秒数
        {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);        // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                              // 记录当前时间，作为新的基准点，为下一次判断做准备
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
/* 所有用户代码，必须写在配对的BEGIN与END之间 */

/******************************************************************************
 * 函  数： HAL_GPIO_EXTI_Callback
 * 功  能： EXTI中断回调函数
 * 参  数： uint16_t  GPIO_Pin    引脚编号，参数范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 返回值： 无
 * 备  注： CubeMX生成的代码在思维和风格上，与使用标准库编写时有很大的区别：
 *          1：这个是回调函数，不是中断服务函数。完整的调用：EXTIx_IRQHandler() > HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_x) > HAL_GPIO_EXTI_Callback(GPIO_PIN_x)
 *          2：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_gpio.c的底部); 不建议在原弱定义中增添代码，而应该重写函数
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
    /* User can add his own implementation to report the file name and linerxNumber,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
