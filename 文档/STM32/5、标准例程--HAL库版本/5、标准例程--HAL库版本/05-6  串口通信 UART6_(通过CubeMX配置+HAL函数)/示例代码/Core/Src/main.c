/* USER CODE BEGIN Header */
/***********************************************************************************************************************************
** 【代码编写】  魔女开发板团队
** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
***********************************************************************************************************************************
** 【实验名称】  STM32F407 双串口 —— USART1+USART6 DMA+空闲中断 数据透传/转发
**
** 【适用平台】  STM32F407 + keil5 + HAL库
**
** 【 CubeMX 】  1- 系统时钟配置为 168MHz；
**               2- 开启 USART1、USART6 异步通信；
**               3- 开启两路串口 RX DMA + 空闲中断；
**               4- 配置 LED、按键 GPIO 用于调试指示；
**
** 【功能说明】  本工程实现双串口高效率数据收发与转发：
**               1- USART1、USART6 均采用 DMA + 空闲中断接收，CPU 零占用；
**               2- 支持阻塞/中断/DMA 三种发送方式，完整示例；
**               3- printf 重定向至 USART1，无需 MicroLIB；
**               4- USART1 接收数据 → 自动 DMA 转发至 USART6；
**               5- 双串口数据 ASCII + 十六进制打印，调试直观；
**               6- 按键中断控制 LED，绿灯 500ms 无阻塞闪烁；
**
** 【代码使用】  1- 上电自动打印系统时钟；
**               2- USART1 发数据 → 自动转发到 USART6 并打印；
**               3- USART6 发数据 → 自动打印接收信息；
**               4- 按键触发 → LED 翻转 + 串口打印；
**
** 【划 重  点】  1- 双串口独立 DMA+IDLE 接收，互不干扰；
**               2- 回调函数：HAL_UARTEx_RxEventCallback；
**               3- 接收完成必须重新开启 DMA；
**               4- 连续发送需判断 gState 状态；
**               5- 所有用户代码必须放在 USER CODE 区间；
**
***********************************************************************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */
#include <stdio.h>              // 用于sprintf函数
#include <string.h>

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

// 定义串口的结构体; 为了方便使用，一次过全定义了
xUATR_TypeDef xUART1 = {0};          // 定义结构体，用于UART1。也可以不用结构体，用单独的变量
xUATR_TypeDef xUART2 = {0};          // 定义结构体，用于UART2。也可以不用结构体，用单独的变量
xUATR_TypeDef xUART3 = {0};          // 定义结构体，用于UART3。也可以不用结构体，用单独的变量
xUATR_TypeDef xUART4 = {0};          // 定义结构体，用于UART4。也可以不用结构体，用单独的变量
xUATR_TypeDef xUART5 = {0};          // 定义结构体，用于UART5。也可以不用结构体，用单独的变量
xUATR_TypeDef xUART6 = {0};          // 定义结构体，用于UART6。也可以不用结构体，用单独的变量

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
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    /* USER CODE BEGIN 2 */
    /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

    // 示范使用printf输出调试信息
    printf("\r\r\r===========  STM32F407VE 外设初始报告 ===========\r");                // 使用printf发送调试信息。它有灵活的格式化，很适合处理字符串; 注意：printf需要重定向fputc函数才能使用，否则程序会卡死; 本示例已重写fputc, 在main.c的底部附近
    printf("系统运行时钟：%d MHz\r", SystemCoreClock / 1000000);                        // 输出系统运行时钟频率，以检查系统时钟是否配置正常。注意：F407系列常用系统时钟=168MHz。

    // LED控制-绿色LED使用了PC5，置高电平-灭，置低电平-亮
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);                                 // 引脚置高电平，绿亮灭

    // 开启USART1的接收：DMA+空闲中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUART1.BuffTemp, sizeof(xUART1.BuffTemp));    // 开启DMA+空闲中断接收; 配合回调函数使用(在main.c的最下方：HAL_UARTEx_RxEventCallback());
    // 开启USART6的接收：DMA+空闲中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, xUART5.BuffTemp, sizeof(xUART5.BuffTemp));    // 开启DMA+空闲中断接收; 配合回调函数使用(在main.c的最下方：HAL_UARTEx_RxEventCallback());

    /** USART的3个发送函数，以USART1作示范，其它串口，是一样的方式 **/
    static char     strTem[100] = "Hello World! 你好吗？\r";                            // 定义一个数组，也可以是其它的数据，如结构体等
    // 发送方式1：阻塞发送，不建议使用。
    HAL_UART_Transmit(&huart1, (uint8_t *)strTem, strlen(strTem), 0xFFFF);              // 发送方式1, 不推荐使用; 阻塞式发送，当调用后，程序会一直死等，不干其它事了(中断除外)，直到发送完毕
    // 发送方式2: 利用中断发送，推荐使用
    while ((&huart1)->gState != HAL_UART_STATE_READY);                                  // 等待上条发送结束; 也可以用HAL_Delay延时法，但就要计算发送用时; 两种方法都是死等法，程序暂时卡死不会往下运行; 如果两次发送间隔时间大，如，大于100ms, 就不用判断语句了。
    HAL_UART_Transmit_IT(&huart1, (uint8_t *)strTem, strlen(strTem));                   // 发送方式2, 推荐使用; 利用中断发送，非阻塞式，大大减少资源占用; 注意：当上次的调用还没完成发送，下次的调用会直接返回(放弃)，所以，要想连接发送，两行调用间，要么判断串口结构体gState的值，要么调用延时HAL_Delay(ms), ms值要大于前一帧发送用时, 用时计算：1/(波特率*11*前一帧字节数)
    // 发送方式4: 利用DMA发送，最推荐使用
    while ((&huart1)->gState != HAL_UART_STATE_READY);                                  // 等待上条发送结束; 也可以用HAL_Delay延时法，但就要计算发送用时; 两种方法都是死等法，程序暂时卡死不会往下运行; 如果两次发送间隔时间大，如，大于100ms, 就不用判断语句了。
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)strTem, strlen(strTem));                  // 发送方式3，推荐使用; 利用DMA发送，非阻塞式，最大限度减少资源占用; 注意：当上次的调用还没完成发送，下次的调用会直接返回(放弃); 所以，要想连接发送，两行调用间，要么判断串口结构体gState的值，要么调用延时HAL_Delay(ms), ms值要大于前一帧发送用时，用时计算：1/(波特率*11*前一帧字节数)


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

        /** 1ms 延时 **/
        HAL_Delay(1 - 1);                                                             // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** USART1 的接收示范(通过串口助手) **/
        if (xUART1.ReceiveNum)
        {
            printf("\r>>>>> 将向串口6发送下面一帧数据 \r");                           // 提示发送, 即把串口助手发过来的数据(UART1接收到的), 发送到其它串口
            printf("字节数：%d \r", xUART1.ReceiveNum);                               // 显示字节数
            printf("ASCII : %s\r", (char *)xUART1.ReceiveData);                       // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                                       // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < xUART1.ReceiveNum; i++)                          // 逐个字节输出
                printf("0x%X ", xUART1.ReceiveData[i]);                               // 以16进制显示
            printf("\r\r");                                                           // 显示换行

            while ((&huart6)->gState != HAL_UART_STATE_READY);                        // 等待上条发送结束; 也可以用HAL_Delay延时法，但就要计算发送用时; 两种方法都是死等法，程序暂时卡死不会往下运行; 如果两次发送间隔时间大，如，大于100ms, 就不用判断语句了。
            HAL_UART_Transmit_DMA(&huart6, xUART1.ReceiveData, xUART1.ReceiveNum);    // 把串口1接收到的数据(串口助手发过来的), 通过串口6发送到模块

            xUART1.ReceiveNum = 0;                                                    // 清0接收标记
        }

        /** UART6 的接收示范(通过串口助手自发自收） **/
        if (xUART6.ReceiveNum)
        {
            printf("\r<<<<< UART6 接收到一帧数据 \r");                                // 提示
            printf("字节数：%d \r", xUART6.ReceiveNum);                               // 显示字节数
            printf("ASCII : %s\r", (char *)xUART5.ReceiveData);                       // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                                       // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < xUART6.ReceiveNum; i++)                          // 逐个字节输出
                printf("0x%X ", xUART6.ReceiveData[i]);                               // 以16进制显示
            printf("\r\r");                                                           // 显示换行

            xUART6.ReceiveNum = 0;                                                    // 清0接收标记
        }

        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                                           // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                                    // 检查当前时间距离上次执行是否已经过了500毫秒; HAL_GetTick()是获取启动后经过的毫秒数
        {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);                   // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                                         // 记录当前时间，作为新的基准点，为下一次判断做准备
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




/******************************************************************************
 * 功  能： printf函数支持代码
 * 说  明： 加入以下代码, 使用printf函数时, 不再需要选择use MicroLIB
 *          复制本函数到你的工程，即可使用
 * 参  数：
 * 返回值：
 * 备  注： 最后修改_2020年07月15日
 ******************************************************************************/
#include <stdio.h>
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};                                                         // 标准库需要的支持函数

FILE __stdout;                                             // FILE 在stdio.h文件
void _sys_exit(int x)
{
    x = x;                                                 // 定义_sys_exit()以避免使用半主机模式
}

int fputc(int ch, FILE *f)                                 // 重写fputc函数，使printf的输出由UART1实现,  这里使用USART1
{
    // 注意，不能使用HAL_UART_Transmit_IT(), 机制上会冲突; 因为调用中断发送函数后，如果上次发送还在进行，就会直接返回！它不会继续等待，也不会数据填入队列排队发送
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0x02);   // 使用HAL_UART_Transmit，相等于USART1->DR = ch, 函数内部加了简单的超时判断(ms)，防止卡死
    return ch;
}



/******************************************************************************
 * 函  数： HAL_UARTEx_RxEventCallback
 * 功  能： DMA+空闲中断回调函数
 * 参  数： UART_HandleTypeDef  *huart   // 触发的串口
 *          uint16_t             Size    // 接收字节
 * 返回值： 无
 * 备  注： 1：这个是回调函数，不是中断服务函数。技巧：使用CubeMX生成的工程中，中断服务函数已被CubeMX安排妥当，我们只管重写回调函数
 *          2：触发条件：当DMA接收到指定字节数时，或产生空闲中断时，硬件就会自动调用本回调函数，无需进行人工调用;
 *          2：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_uart.c的底部); 不要在原弱定义中增添代码，而是重写本函数
 *          3：无需进行中断标志的清理，它在被调用前，已有清中断的操作;
 *          4：生成的所有DMA+空闲中断服务函数，都会统一调用这个函数，以引脚编号作参数
 *          5：判断参数传进来的引脚编号，即可知道是哪个串口接收收了多少字节
******************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
#if 1
    // 串口UART1的DMA+空闲中断回调函数
    if (huart == &huart1)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART1.ReceiveNum  = Size;                                                           // 接收完成标志
        memset(xUART1.ReceiveData, 0, sizeof(xUART1.ReceiveData));                           // 清0前一帧的接收数据
        memcpy(xUART1.ReceiveData, xUART1.BuffTemp, Size);                                   // 把新数据，从临时缓存中，复制到xUART1.ReceivedData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, xUART1.BuffTemp, sizeof(xUART1.BuffTemp));     // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
    }
#endif

#if 0
    // 串口UART2的DMA+空闲中断回调函数
    if (huart == &huart2)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART2.ReceiveNum  = Size;                                                           // 接收完成标志
        memset(xUART2.ReceiveData, 0, sizeof(xUART2.ReceiveData));                           // 清0前一帧的接收数据
        memcpy(xUART2.ReceiveData, xUART2.BuffTemp, Size);                                   // 把新数据，从临时缓存中，复制到xUART2.ReceivedData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, xUART2.BuffTemp, sizeof(xUART2.BuffTemp));     // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
    }
#endif

#if 0
    // 串口UART3的DMA+空闲中断回调函数
    if (huart == &huart3)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART3.ReceiveNum  = Size;                                                           // 接收完成标志
        memset(xUART3.ReceiveData, 0, sizeof(xUART3.ReceiveData));                           // 清0前一帧的接收数据
        memcpy(xUART3.ReceiveData, xUART3.BuffTemp, Size);                                   // 把新数据，从临时缓存中，复制到xUART3.ReceivedData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, xUART3.BuffTemp, sizeof(xUART3.BuffTemp));     // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
    }
#endif

#if 0
    // 串口UART4的DMA+空闲中断回调函数
    if (huart == &huart4)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART4.ReceiveNum  = Size;                                                           // 接收完成标志
        memset(xUART4.ReceiveData, 0, sizeof(xUART4.ReceiveData));                           // 清0前一帧的接收数据
        memcpy(xUART4.ReceiveData, xUART4.BuffTemp, Size);                                   // 把新数据，从临时缓存中，复制到xUART4.ReceivedData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, xUART4.BuffTemp, sizeof(xUART4.BuffTemp));     // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
    }
#endif

#if 0
    // 串口UART5的DMA+空闲中断回调函数
    if (huart == &huart5)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART5.ReceiveNum  = Size;                                                           // 接收完成标志
        memset(xUART5.ReceiveData, 0, sizeof(xUART5.ReceiveData));                           // 清0前一帧的接收数据
        memcpy(xUART5.ReceiveData, xUART5.BuffTemp, Size);                                   // 把新数据，从临时缓存中，复制到xUART5.ReceivedData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, xUART5.BuffTemp, sizeof(xUART5.BuffTemp));     // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
    }
#endif

#if 1
    // 串口UART6的DMA+空闲中断回调函数
    if (huart == &huart6)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART6.ReceiveNum  = Size;                                                           // 接收完成标志
        memset(xUART6.ReceiveData, 0, sizeof(xUART6.ReceiveData));                           // 清0前一帧的接收数据
        memcpy(xUART6.ReceiveData, xUART6.BuffTemp, Size);                                   // 把新数据，从临时缓存中，复制到xUART6.ReceivedData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, xUART6.BuffTemp, sizeof(xUART6.BuffTemp));     // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
    }
#endif
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
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
