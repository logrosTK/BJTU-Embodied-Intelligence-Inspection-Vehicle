/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】  无线通信_蓝牙_ECHB02_主机模式_两个模块间主、从通信
 **
 ** 【适用平台】  STM32F407 + keil5
 **
 **【实验操作】  接线操作：
 **                  1-本示例，使用两个ECB02模块，两个STM32开发板，相同接线
 **                  2-USB线，从电脑端，连接至板上的CMSIS-DAP接口
 **                  3-ECB02接线：RXD--PC10, TXD--PC11, GND--GND, VCC--3.3V
 **              电脑端操作：
 **                  1-打开2个串口软件，115200-None-8-1, 并打开相应的串口号
 **              代码操作：
 **                  1-从机方，烧录上一示例代码：从机模式
 **                  2-主机方，烧录本示例代码：主机模式
 **              测试现象：
 **                  1-两模块未互相连接, 模块蓝灯两秒闪烁一次; 在电脑串口助手，可以输入并发送AT指令，对ECB02进行参数设置、查询
 **                  2-两模块相互连接后(主机连接从机)，模块蓝灯一秒闪烁一次; 电脑串口助手, 还可以继续使用AT指令，指令外的数据，会当作普通数据直接发送到小程序
 **                  3-使用其中一方的串口助手，发送数据，另一方在收到数据后会输出到关联的串口助手
 **
 **【划 重 点】  1-ECB02的波特率是115200
 **              2-AT指令，必大写，指令结尾+"\r\n"
 **              3-ECB02空中信号每帧数据最大244字节; 单片机发送数据到ECB02时，≤511字节ECB02自动分包并连续发送，≥512字节ECB02会发生数据错乱
 **              4-发送间隔：每包数据的发送，必须有时间的间隔，否则出错！！具体的间隔时间计算可翻查ECB02官方手册.
 **              5-从机模式：一个ECB02模块，想被其它设备连接，如手机、电脑，必须设置为从机模式，才能被主机搜索发现、被连接. 重点是：“被”，自己是不能主动连接其它设备的。
 **              6-主机模式：要主动连接其它设备，必须设置为主机模式，才能连接其它从机设备。注意区分哪些设备是从机设备。
 **              7-两个ECB02模块，想互相连接，必须用AT指令设置为一主、一从！然后主机通过可通过三种方式连接从机：搜索设备并连接设备号、蓝牙名称绑定、或MAC地址绑定，后两者在意外断开后会自动尝试连接。
 **              8-特别地，ECB02上电后，会根据最后一次保存的配置进行工作, (不经STM32代码控制); 即，主机上电后，会使用最后一次绑定的记录，搜索设备并尝试连接，而从机在上电后，也会间隔广播自己的名称，静待被搜索、连接;
 **              9-ECB02的设备名称、小程序，汉字显示需要URL编码，具体可以翻查官方pdf;
 **
 **【更新记录】  2024-06-08  优化UART通信
 **              2023-10-02  移植
 **
 **【备注说明】  版权归魔女科技所有，请勿商用，谢谢！
 **              https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
 * 函  数： ECB02_Init
 * 功  能： 初始化ECB02：主机模式
 *          配置串口、主从机模式、AT模式、蓝牙名称
 * 参  数： 无
 * 返回值： 0-失败、1-成功
 ******************************************************************************/
static uint8_t ECB02_Init(void)
{
    uint8_t status = 1;                                               // 执行状态标志; 0-失败、1-成功

    HAL_Delay(200);                                                   // 上电后稍作延时，等待ECB02进入工作状态

    UART4_Init(115200);                                               // 串口4初始化; 用于与ECB02通信; 波特率115200; 使用引脚PC10、PC11

    /** 1-恢复出厂设置**/
    status &= UART4_SendAT("AT+FACTORY\r\n", "OK", 1000);             // 恢复出厂时的默认配置
    if (status == 0)                                                  // 等待指令执行返回; AT指令执行成功返回"OK"、执行失败不返回任何数据;
        goto error_ECB02_Init;                                        // 错误：等待超时; 跳到错误处理
    printf("1- ECB02 默认配置        配置完成\r");                    // 输出提示：修改成功
    HAL_Delay(700);                                                   // 注意：执行恢复出厂配置指令后，要稍等700ms左右，才能操作其它指令

    /** 2-主机模式 **/
    status &= UART4_SendAT("AT+ROLE=1\r\n", "OK", 1000);              // 设置主从机模式：0-由ROLE引脚配置、1-主机模式、2-从机模式
    if (status == 0)                                                  // 等待指令执行返回; AT指令执行成功返回"OK"、执行失败不返回任何数据;
        goto error_ECB02_Init;                                        // 错误：等待超时; 跳到错误处理
    printf("2- ECB02 主机模式        配置完成\r");                    // 输出提示：修改成功
    HAL_Delay(500);                                                   // 注意：执行主从机模式指令后，要稍等500ms后，才能操作其它指令

    /** 3-连接后AT指令仍然有效 **/
    status &= UART4_SendAT("AT+MODE=1\r\n", "OK", 1000);              // 设置连接后仍可使用AT指令; 0-AT_EN引脚控制、1-连接后AT指令有效、2-连接后AT指令无效，所有数据数据透传;
    if (status == 0)                                                  // 等待指令执行返回; AT指令执行成功返回"OK"、执行失败不返回任何数据;
        goto error_ECB02_Init;                                        // 错误：等待超时; 跳到错误处理
    printf("3- ECB02 AT模式          配置完成\r");                    // 输出提示：修改成功

    /** 4-清除旧的绑定 **/
    status &= UART4_SendAT("AT+BONDC\r\n", "OK", 1000);               // 清除旧的绑定记录：如果最后一次连接的是设备A，那么，在连接设备B前，要清除旧的绑定记录
    if (status == 0)                                                  // 等待指令执行返回; AT指令执行成功返回"OK"、执行失败不返回任何数据;
        goto error_ECB02_Init;                                        // 错误：等待超时; 跳到错误处理
    printf("4- ECB02 清除绑定        配置完成\r");                    // 输出提示：修改成功
    HAL_Delay(500);                                                   // 注意：执行清除旧绑定记录指令后，要稍等500ms后，才能操作其它指令

    /** 5-连接指定设备 **/
    status &= UART4_SendAT("AT+BONDNAME=myECB02\r\n", "OK", 1000);    // 设置要绑定的从机设备名称，在设置后，ECB02会开始自动搜索，并尝试连接设备; 注意：如果是汉字的名称，要使用URL编码，具体可翻查官方pdf
    if (status == 0)                                                  // 等待指令执行返回; AT指令执行成功返回"OK"、执行失败不返回任何数据;
        goto error_ECB02_Init;                                        // 错误：等待超时; 跳到错误处理
    printf("5- ECB02 开始搜索指定从机设备...\r\n");                   // 输出提示：尝试连接设备

    return 1;                                                         // 配置顺利完成，函数返回：1

error_ECB02_Init:                                                     // 失败处理
    printf("ECB-02 配置失败！已跳过配置\r");                          // 输出提示：配置失败
    return 0;                                                         // 返回：0-失败、1-成功
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
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);   // 引脚置高电平，绿亮灭
    UART1_Init(115200);                                                    // 初始化 串口1; 已写好底层，调用h中的函数即可使用; 引脚(PA9 +PA10)、波特率-None-8-1; 如果使用CubeMX配置，请使用前述引脚，但，不要在MX上进行中断及DMA配置，否则冲突
    ECB02_Init();                                                          // 初始化ECB02：配置串口、从机模式、AT模式、蓝牙名称; 返回：0-失败、1-成功

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
            UART4_SendData(UART1_GetRxData(), UART1_GetRxNum());           // 把串口助手发过来的数据，经ECB02所用串口发送到ECB02模块
            UART1_ClearRx();                                               // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0
        }

        /** 检查ECB02所用串口是否收到数据 **/
        if (UART4_GetRxNum())                                              // 检查ECB02所连接的UART, 否收到数据; 如果收到，就输出到串口助手，并判断其它动作
        {
            uint16_t  usRxNum = UART4_GetRxNum();                          // 获取串口接收最后一帧的字节数
            uint8_t  *puRxData = UART4_GetRxData();                        // 获取串口接收最后一帧的数据
            printf("\r\nECB02收到 %d 字节<<<:\r", usRxNum);                // 字节数，printf到串口助手，方便观察
            printf("%s\r", puRxData);                                      // 数据，以字符串形式，printf到串口助手，方便观察
            if (strstr((char *)puRxData, "LED_ON"))                        // 判断APP发过来的数据包，是否包含字符串：LED_RED_ON
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);      // PB2置低电平，即点亮蓝色LED
            }
            if (strstr((char *)puRxData, "LED_OFF"))                       // 判断APP发过来的数据包，是否包含字符串：LED_RED_OFF
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);        // PB2置高电平，即熄灭蓝色LED
            }
            UART4_ClearRx()  ;                                             // 清空串口的接收
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
