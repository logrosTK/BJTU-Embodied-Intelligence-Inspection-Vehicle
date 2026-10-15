/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  NRF24L01 (发送方）
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【实验操作】  本示例文件中，有发送方、接收方两个示例;
 **               需要使用两个开发板+NRF24L01模块, 接bsp_NRF24L01.h中引脚宏定义进行接线;
 **               分别烧录发送方示例、接收方示例;
 **               打开两个串口助手;
 **               发送方：已在while()中设置了每隔0.5秒发送一次数据, 发送成功会printf提示, 发送失败也会printf提示;
 **               接收方：已在while()里写好判断示范代码，当接收到一包数据后，会printf出详细的数据;
 **               发送方和接收方两者代码的区别：bsp_NRF24L01.c和bsp_NRF24L01.h代码，是一样的; 不同的地方有两处：其一是main中的地址设置不一样，简单的互调即可, 其二是while部分的收发不一样，查看main.c文件即可明白;
 **
 ** 【 CubeMX 】  在h文件中修改引脚、SPI端口号，即可;
 **               无需通过CubeMX、CubeIDE对本模块进行引脚、SPI的配置;
 **               这样做的目的：为了简化文件的复制、移植;
 **               bsp_NRF24L01.c中，已对初始化、通信的收发，都编写好了处理函数 ，调用h文件中声明的函数，即可完成初始化、收发通信等;
 **               注意冲突：IRQ引脚需要使用到中断，在本C文件中，已写好了中断服务函数，注意不要使用与CubeMX工程中相冲突的引脚、中断编号，否则编译时会提示中断服务函数的重复定义;
 **
 ** 【移植说明】  1- 复制本工程的NRF24L01文件夹，到目标工程文件夹中;
 **               2- 添加文件路径：  Keil > Option > C/C++ > Include Paths，选择NRF24L01的目录路径;
 **               3- 添加文件到工程: Keil > 左侧工程管理器中双击目标文件夹 > 选择 bsp_NRF24L01.c;
 **               4- 添加文件引用:   #include "bsp_NRF24L01.h＂，即哪个文件要用此功能，就在其代码文件顶部添加引用;
 **
 ** 【引脚解释】  VCC  -  3.3V; 千万别接子5V
 **               GND  -  GND
 **               CE   -  工作模式控制; 置低:配置寄存器、置高:10us后进入设定的工作模式(发送、接收等)
 **               IRQ  -  中断信号引脚，闲时是高电平，芯片发生中断时会把其拉至低电平; 发生中断时的3种情况：发送成功、重发次数溢出、接收到新一帧数据
 **               SCK  -  SPI通信，时钟信号线
 **               CS   -  SPI通信, 片选线; 闲时高电平，通信时手动置低电平;
 **               MISO -  SPI通信，主机输入从机输出
 **               MOSI -  SPI通信，主机输出从机输入
 **
 ** 【引脚修改】  1-在bsp_NRF24L01.h的宏定义中修改需要使用的引脚;
 **               2-IRQ引脚的修改，要特别留意, 别和CubeMX工程中的其它中断引脚重叠中断号，否则编译时会冲突; 中断号可以csdn刨文章;
 **               3-SPI端口，也在此文件的宏定义中修改;
 **
 ** 【参数修改】  1-在bsp_NRF24L01.h的宏定义中: 频率、CRC、波特率、发送功率;
 **               2-通信双方上述前3个参数，必须一样，才能通信;
 **               3-发射功率、波特率，这两个参数会影响通信距离，具体请按需另行实测;
 **
 ** 【初 始 化】  1-初始化，需要2个函数：①初始化、②设置接收通道地址;
 **               2-调用 NRF24L01_Init()，即可对h文件中宏定义的引脚、SPI端口进行初始化;
 **               3-调用 NRF24L01_SetAddr(uint8_t* 5字节地址, uint8_t 通道0~5)，设置模块自己接收通道x的地址值; NRF24L01其实有6个接收通道, 通道0和1能自由地设置5个字节的地址值，通道2~5的高4字节要和通道1的高4字节相同，只能自由设定最低1字节的值;
 **               4-设置自己的地址后，函数会配置为接收模式，模块开始自动对空中信号进行监听;
 **
 ** 【如何发送】  1-发送，需要2个函数：①设置发送地址、②发送数据
 **               2-调用 NRF24L01_SetTxAddr(uint8_t*  5字节地址值)，设置对面接收方的地址; 参数可以传入数组，也可以用传入字符串，建议使用后者，这样更便于命名和记忆。
 **               3-调用 NRF24L01_SendData(uint8_t* 数据, uint8_t 字节数)，函数将自动切换到发送模式，并按设定的频率、地址等参数，把数据发出;
 **               4-关于判断发送成功，当对方成功收到数据后，会发回ack信号 ; 调用 NRF24L01_GetSendStatus()，通过其返回值对比，可以知道发送的各个状态：0-无效值、1-发送成功、2-发送失败、3-正在发送、4-字节数不在有效范围(1~31)、5-NRF24L01尚未初始化
 **               5-NRF24L01限定每包最大32字节; 发送函数内部利用[0]存放字节数，[1]起存放有效数据，因此每包最大发送31字节; 用户无需外部处理上述操作，只需传入原始数据、原始的字节数即可，函数内部会自动处理成：[0]-字节数、[1]起-有效数据;
 **               6-每次发送数据后，发送成功、发送失败，都会自动切换回接收模式继续监听空中信号;
 **               7-每包发送间隔时间：建议间隔1ms以上; 因为spi传输、空中传输、工作模式这三项，都需要时间;
 **
 ** 【如何接收】  1-接收，需要3个函数：①获取接收字节数、②获取数据地址、③清零接收
 **               2-调用函数 NRF24L01_GetRxNum(void), 返回接收的字节数:0~31; 单位:字节; 0-无接收、非0-新一帧数据的字节数;
 **               3-利用返回值判断接收：返回值>0，即为接收到新一帧数据;
 **               4-返回值>0时，调用函数 NRF24L01_GetRxData(), 即可返回当前一帧数据的缓存地址;
 **               5-每次处理完一帧数据后，调用 NRF24L01_ClearRx(), 即可清0接收字节数、数据，以方便下一帧数据的接收判断工作;
 **
 ** 【调试经验】  1-不要和其它功能联合调试，以避免大量的调试时间消耗; 先调通了基本的收发，再一点点增加其它功能，如按键发送、协议等;
 **               2-调试时，先把两个开发板+模块，放在桌面上调试，不要间隔太远，以减少信号衰减掉包问题的发生;
 **               3-掉包问题，一般是这几种原因：双方空中参数不一致、间隔太远、发送间隔太短、地址错误、接收判断间隔太久;
 **               4-信号衰减，主要受限于收发间的障碍物; 空旷地方能有20米左右，室内实测，一堵墙10米内，两堵墙6米内;
 **
 ** 【更新记录】  2024-02-19  简化函数名称、优化接收处理
 **               2024-02-17  从标准库中更新至HAL库
 **
 ** 【备注说明】  版权归魔女科技所有，请勿商用，谢谢！
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 用户代码，必须写在配对的BEGIN与END之间，否则CubeMX重新生成后，会被删除掉 */

#include "bsp_UART.h"            // 串口通信底层驱动文件; 已重写好初始化、收发，调用函数即可使用串口
#include "bsp_NRF24l01.h"        // NRF24L01通信的自定义文件，包含：发送函数、接收函数等


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
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);       // 引脚置高电平，绿亮灭

    UART1_Init(115200);                                                    // 初始化 串口1; 已写好底层，调用h中的函数即可使用; 引脚(PA9 +PA10)、波特率-None-8-1; 如果使用CubeMX配置，请使用前述引脚，但，不要在MX上进行中断及DMA配置，否则冲突

    NRF24L01_Init();                                                       // NRF24L01初始化：引脚、SPI
    NRF24L01_SetMyAddr((uint8_t *)"Boss", 0);                              // 设置NRF24L01某接收通道的地址; 参数：地址值(5字节)、通道(0~5); NRF24L01有6个接收通道，均能独立接收数据; 通道0、1能设置自由的地址，通道2~5的前4个字节必须与通道1相同;
    NRF24L01_SetTxAddr((uint8_t *)"Dev-3");                                // 设置发送地址，即对面接收方的地址; 如果每次都是相同的接收方，只需设置一次; 如果每次的发送目标地址不一样，则发送前调用本函数进行设置;
    // NRF24L01_ReadConfig();                                              // 读取NRF24L01的参数，并printf出来，以方便观察和调试



    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */

        /** 1ms延时 **/
        HAL_Delay(1 - 1);                                                  // 延时函数，参数：ms; 特别地：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** 检查UART1是否收到数据 (串口助手) **/
        if (UART1_GetRxNum())                                              // 判断USART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            printf("\r>>>>> 准备发送一帧数据 \r");                         // 提示
            printf("字节数：%d \r", UART1_GetRxNum());                     // 显示字节数
            printf("ASCII ：%s\r", (char *)UART1_GetRxData());             // 显示数据，以ASCII方式显示，即以字符串的方式显示
            // 把串口助手发过来的数据，经NRF24L01发出去
            NRF24L01_SendData(UART1_GetRxData(), UART1_GetRxNum());        // 发送数据; 参数:数据、字节数(1~31); NRF24L01限定每包最大32字节; 发送函数内部利用[0]存放字节数，[1]起存放有效数据，因此每包最大发送31字节;
            UART1_ClearRx();                                               // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0
        }

        /** 检查NRF24L01是否收到数据 **/
        if (NRF24L01_GetRxNum())                                           // 获取接收字节数; 值范围:0~31; 0-无接收、非0-新一帧数据的字节数;
        {
            uint8_t  rxNum = NRF24L01_GetRxNum();                          // 获取接收的字节数; 值范围:0~31;
            uint8_t *rxData = NRF24L01_GetRxData();                        // 获取接收的数据(地址); 特别注意：[0]-字节数、[1]起-有效数据;
            printf("\r<<<<< NRF24L01 收到一帧数据 \r");                    // 输出提示;
            printf("字节数: %d\r", rxNum);                                 // 显示字节数;
            printf("ASCII : %s\r", (char *)rxData);                        // 显示数据，以ASCII方式显示，即以字符串的方式显示;
            printf("16进制: ");                                            // 显示数据，以16进制方式，显示每一个字节的值;
            for (uint8_t i = 0; i <= rxNum; i++)                           // 逐个字节输出; [0]-字节数、[1]起-有效数据;
                printf("%d ", *rxData++);                                  // 格式化;
            printf("\r\r");                                                // 显示换行;
            NRF24L01_ClearRx();                                            // 重要：清0接收标志、数据; 每次处理完一帧数据后，必须清0;
        }

        /** 每1000ms执行一次 **/
        static uint16_t ms = 0;
        if (++ms == 1000)                                                  // 每1000ms执行一次
        {
            ms = 0;                                                        // 计数清0
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);        // 规律地闪烁绿色LED，方便外部观察系统是否正常运行
            // 间隔发送数据，以方便测试
            uint8_t testData[] = "hello, 你好吗？";                        // 定义发送的测试数据
            NRF24L01_SendData(testData, strlen((char *)testData));         // 发送数据; 参数:数据、字节数(1~31); NRF24L01限定每包最大32字节; 发送函数内部利用[0]存放字节数，[1]起存放有效数据，因此每包最大发送31字节;
        }

        /** 获取发送状态 **/                                               // 获取发送状态，调试使用，非必需的功能
        uint8_t sendStatus = NRF24L01_GetSendStatus();
        switch (sendStatus)
        {
            case 0:
                break;
            case 1:                                                        // 发送成功; 可自行修改为需要的处理
                printf("NRF24L01: 已发送成功!\r\r");
                break;
            case 2:                                                        // 发送失败; 可自行修改为需要的处理
                printf("NRF24L01: 发送失败，达到最在发送次数\r\r");
                break;
            case 3:                                                        // 正在发送; 可自行修改为需要的处理
                break;
            case 4:
                printf("NRF24L01: 放弃发送，字节数不在有效范围(1~31)！\r");// 放弃发送; 字节数不在有效范围内
                break;
            case 5:
                printf("NRF24L01: 放弃发送，NRF24L01尚未初始化！\r");      // 放弃发送; NRF24L01尚未初始化成功
                break;
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
