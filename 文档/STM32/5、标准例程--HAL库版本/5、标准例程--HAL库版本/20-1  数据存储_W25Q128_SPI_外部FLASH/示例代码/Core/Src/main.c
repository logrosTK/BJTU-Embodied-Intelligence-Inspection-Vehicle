/* USER CODE BEGIN Header */
/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【实验名称】  数据存储-外部Flash存储-芯片W25Q128
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【 CubeMX 】  为了移植方便，本示例另写了 bsp_W25Q128.c 和 bsp_W25Q128.h 两个驱动文件
 **               无需使用CubeMX配置SPI引脚，直接调用 bsp_W25Q128.h 所声明的函数，即可实现：初始化、读取、存储
 **
 ** 【移植说明】  1- 复制本工程的W25Q128文件夹，到目标工程文件夹中;
 **               2- 添加头文件路径： Keil > Option > C/C++ > Include Paths, 选择刚才复制后粘贴的目标文件夹路径;
 **               3- 添加C文件到工程: Keil > 左侧工程管理器中双击目标文件夹 > 选择 bsp_W25Q128.c;
 **               4- 添加文件引用:    #include "bsp_W25Q128.h＂，即哪个文件中需要使用其函数功能，就在其代码文件顶部添加引用;
 **
 ** 【代码使用】  所用GPIO引脚和SPI端口，可以在 "bsp_W25Q128.h＂中修改;
 **               为简化整个W25Q128的读写操作，已封装成3个全局函数，只用这3个函数，即可完成对其存取操作
 **               初始化  ：  W25Q128_Init()　
 **               读取数据：  W25Q128_ReadData( uint32_t addr, uint8_t *pData, uint16_t num)　 // 数据地址，缓存指针，字节数
 **               写入数据：  W25Q128_WriteData(uint32_t addr, uint8_t *pData, uint16_t num)   // 数据地址，缓存指针，字节数
 **
 ** 【划 重 点】 1_ 本代码，适用W25Q16、W25Q32、W25Q65、W25Q128，不适用于W25Q256;
 **              2_ 地址的范围取值，如W25Q128，容量：128/8=16M字节，则字节地址范围：0~16777216，用16进制表示为：0x00~0x01000000
 **              3_ 如果使用的是魔女开发板上的W25Q128，存储数据时，请使用前10M空间地址0X0~0x9FFFFF; 芯片后6M空间,已存储汉字字模数据，地址为0x00A00000~0x01000000;
 **              4_ 代码已经多次优化，直接使用即可，而无需过于纠结SPI通信原理、芯片的存储原理，而浪费了时间;
 **
 *******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

#include "bsp_UART.h"               // UART 串口通信底层驱动文件，已编写好6个串口的初始化和收发, 调用函数即可使用;
#include "bsp_W25Q128.h"            // 外部Flash芯片-W25Q128的底层驱动文件，已编写好初始化和读写函数，调用函数即可使用;


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
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */



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
  /* USER CODE BEGIN 2 */
    /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

    HAL_Delay(20);                                                                      // 上电后稍延时，让芯片、外部设备稳定上电后，再跑代码

    UART1_Init(115200);

    printf("\r\r\r=========== STM32F407 外设初始报告 ===========\r");                   // 注意，printf函数，必须重定向后才能使用，否则程序会卡死; 本示例已重写fputc, 在main.c的底部附近
    printf("系统运行时钟：           %d MHz\r", SystemCoreClock / 1000000);             // 输出系统运行时钟的频率，以检查是否配置正常

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);                // LED控制引脚置高电平，绿亮灭

    W25Q128_Init();                                                                     // W25Q128 初始化

    // 读写测试1：字符
    printf("\r>>>> 读写测试 1：char\r");
    uint32_t addrChar   = 0x00001000;                                                   // 地址; 用户数据须安排在<0x00A00000的位置，避免覆盖了代码段数据
    char writeChar = 'A';
    W25Q128_WriteData(addrChar, (uint8_t *)&writeChar, 1);                              // 存入数据
    char readChar = 0;
    W25Q128_ReadData(addrChar, (uint8_t *)&readChar, 1);                                // 读取数据
    printf("地址:0X%08X    写入:%c       读取:%c \r", addrChar, writeChar, readChar);   // 输出测试操作结果

    // 读写测试2：带符号的16位
    printf("\r>>>> 读写测试 2：short \r");
    uint32_t addrShort  = 0x00002000;                                                    // 地址; 用户数据须安排在<0x00A00000的位置，避免覆盖了代码段数据
    short writeShort = -666;
    W25Q128_WriteData(addrShort, (uint8_t *)&writeShort, sizeof(writeShort));            // 存入数据
    short readShort = 0;
    W25Q128_ReadData(addrShort, (uint8_t *)&readShort, sizeof(writeShort));              // 读取数据
    printf("地址:0X%08X    写入:%d    读取:%d \r", addrShort, writeShort, readShort);    // 输出测试操作结果

    // 读写测试3：无符号的32位
    printf("\r>>>> 读写测试 3：uint32_t \r");
    uint32_t addrUint   = 0x00003000;                                                    // 地址; 用户数据须安排在<0x00A00000的位置，避免覆盖了代码段数据
    uint32_t writeUint = 0x12345678;
    W25Q128_WriteData(addrUint, (uint8_t *)&writeUint, 4);                               // 存入数据
    uint32_t readUint = 0;
    W25Q128_ReadData(addrUint, (uint8_t *)&readUint, 4);                                 // 读取数据
    printf("地址:0X%08X    写入:0x%X     读取:0x%X \r", addrUint, writeUint, readUint);  // 输出测试操作结果

    // 读写测试4：浮点型
    printf("\r>>>> 读写测试 4：float \r");
    uint32_t addrFloat  = 0x00004000;                                                    // 地址; 用户数据须安排在<0x00A00000的位置，避免覆盖了代码段数据
    float writeFloat = -123.456;
    W25Q128_WriteData(addrFloat, (uint8_t *)&writeFloat, 4);                             // 存入数据
    float readFloat;
    W25Q128_ReadData(addrFloat, (uint8_t *)&readFloat, 4);                               // 读取数据
    printf("地址:0X%08X　  写入:%f    读取:%f \r", addrFloat, writeFloat, readFloat);    // 输出测试操作结果

    // 读写测试5：数组
    printf("\r>>>> 读写测试 5：数组 \r");
    uint32_t addrString   = 0x00005000;                                                  // 地址; 用户数据须安排在<0x00A00000的位置，避免覆盖了代码段数据
    char writeString[100] = " abc 123 早!";
    W25Q128_WriteData(addrString, (uint8_t *)writeString, sizeof(writeString));          // 存入数据
    char readString[100];
    W25Q128_ReadData(addrString, (uint8_t *)readString, sizeof(readString));             // 读取数据
    printf("地址:0X%08X　  写入:%s   读取:%s \r", addrString, writeString, readString);  // 输出测试操作结果

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

        /** 1ms延时 **/
        HAL_Delay(1 - 1);                                                  // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** 检查UART1是否收到数据 (串口助手) **/
        if (UART1_GetRxNum())                                              // 判断UART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            uint8_t *rxData = UART1_GetRxData();                           // 获取数据的地址
            uint16_t rxNum = UART1_GetRxNum();                             // 获取数据的字节数
            printf("\r<<<<< UART1 接收到一帧数据 \r");                     // 提示UART1接收到数据
            printf("字节数：%d \r", rxNum);                                // 显示字节数
            printf("ASCII : %s\r", (char *)rxData);                        // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                            // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < rxNum; i++)                           // 逐个字节输出
                printf("0x%02X ", rxData[i]);                              // 以16进制显示
            printf("\r\r");                                                // 显示换行
            UART1_ClearRx();                                               // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0
            // UART1_SendString("Hello World！\r");                        // 示范 UART1_SendString( )的使用; 可接受像pritnf的格式化参数
            // UART1_SendData((uint8_t*)"Hello World!\r", 13);             // 示范 UART1_SendData( )的使用, 可用于发送字符串、数组、结构体数据等
        }

        /** 每500ms规律地闪烁绿色LED, 便于观察程序是否运行、卡死 **/
        static uint32_t lastTime_500ms = 0;                                // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                         // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {                                                                
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);                         // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
    /* User can add his own implementation to report the file name and linerxNumber,
       ex: printf("Wrong parameters value: file %s on line %d\r", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
