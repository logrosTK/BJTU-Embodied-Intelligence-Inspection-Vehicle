/* USER CODE BEGIN Header */
/**==================================================================================================================
 **【文件名称】  main.c
 **【功能测试】  ESP8266通信
 **==================================================================================================================
 **【实验平台】  STM32F407 + KEIL5.31 + ESP8266(ESP-12F)
 **
 **【实验操作】  接线操作：
 **                  1_USB转TTL：已板载USB转TTL功能, 使用的是UART1(PA9+PA10); 用USB线，连接电脑与开发板的CMSIS-DAP接口，即可;
 **                  2_ESP8266 ：已板载ESP8266模组; 其RX、TX引脚，位于开发板右侧排针上方，使用跳线帽，可以与相邻的USART3排针位相连接，即PB10,PB11，无需其它接线;
 **
 **              电脑端操作：
 **                  1_打开串口助手，设置参数 115200-None-8-1, 打开端口;
 **
 **              代码操作：
 **                  1_编译、无错误后，烧录！
 **
 **              测试现象：
 **                  1_本示例，可通过串口助手，发送AT指令, 记得Enter换行，或者打勾发送新行; 留意:发出的字节数=AT指令字符串长度+2(\r\n)
 **                  2_当AT指令正确时，ESP8266会发回执行信息，并在串口助手上显示;
 **
 **【移植说明】  1_复制本工程的UART、ESP8266文件夹，到目标工程文件夹中;
 **              2_添加头文件路径： Keil > Option > C/C++ > Include Paths, 添加刚才两个粘贴后的文件夹目录
 **              3_添加C文件到工程: Keil > 左侧工程管理器中双击目标文件夹 > 选择 bsp_UART.c、bsp_ESP8266;
 **              4_添加文件引用:    #include "bsp_UART.h＂、#include "bsp_ESP8266.h＂，即哪个文件中需要使用其函数功能，就在其代码文件顶部添加引用;
 **
 **【划 重 点】  1_如果使用的是外接的ESP8266模块, 连线：VCC接3.3V, IO0、EN、RST这3引脚接3.3V，IO15、GND这2引脚接GND, TXD、RXD这2引脚可以通过排针右上角的中线帽连接接USART3;
 **              2_ESP8266信号问题: 因为模块使用的是板载天线，信号比较弱，不能离路由器过远，适合8米内操作;
 **              3_固件：ESP8266已烧录厂家默认的AT固件; 如果需要上云，则需要重新烧录MQTT-AT固件, 在本示例文件夹中，已收录MQTT-AT固件、烧录软件、图解参考;
 **              4_AT命令：大写、前后不带空格、"\r\n"作结尾;
 **
 **【更新记录】  2024-07-05  移植至HAL库
 **              2022-10-17  完善注释
 **              2021-12-26  完善代码结构、注释
 **              2021-12-03  创建
 **
 **【备注说明】  版权归魔女科技所有，请勿商用，谢谢！
 **              https://demoboard.taobao.com
====================================================================================================================*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 用户代码，必须写在配对的BEGIN与END之间，否则CubeMX重新生成后，会被删除掉 */

#include "bsp_UART.h"            // 串口通信底层驱动文件; 已重写好初始化、收发，调用函数即可使用串口
#include "bsp_ESP8266.h"         // ESP8266通信的自定义文件，包含：发送函数、接收函数等


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
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);                    // 引脚置高电平，绿亮灭

    UART1_Init(115200);                                                    // 初始化 串口1; 已写好底层，调用h中的函数即可使用; 引脚(PA9 +PA10)、波特率-None-8-1; 如果使用CubeMX配置，请使用前述引脚，但，不要在MX上进行中断及DMA配置，否则冲突

    UART3_Init(115200);                                                    // 初始化8266所用串口
    
    ESP8266_JoinAP("ZHOU", "55025865502586");                              // 加入到AP，即连接到Wifi; 注意：ESP8266支持2.4G信号、不支持5G信号、不支持中文WiFi名称 
   
    uint8_t *pData = ESP8266_GetIP();                                      // 获取公网IP; 获得8字节数据：前4字节-公网IP、后4字节-局域网IP; 返回值：0_获取失败、非0_数据地址
    if (pData)                                                             // 输出提示
    {
        printf("公  网IP地址:  ");
        printf("%d.%d.%d.%d\r", pData[0], pData[1], pData[2], pData[3]);   // 输出公网IP地址
        printf("局域网IP地址:  ");
        printf("%d.%d.%d.%d\r", pData[4], pData[5], pData[6], pData[7]);   // 输出局域网IP地址
    }


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */

        /** 1ms延时 **/
        //HAL_Delay(1 - 1);                                                // 延时函数，参数：ms; 特别地：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** 检查UART1是否收到数据 (串口助手) **/
        if (UART1_GetRxNum())                                              // 判断USART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            uint16_t num = UART1_GetRxNum();                               // 获取接收到的字节数
            uint8_t *pData = UART1_GetRxData();                            // 获取接收到的数据(缓存的地址)
            printf("\r>>>准备向 ESP8266 发送以下数据：\r");                // 提示
            printf(">>>长度：%d 字节\r", UART1_GetRxNum());                // 显示字节数
            printf(">>>内容：%s \r", (char *)UART1_GetRxData());           // 显示数据，以ASCII方式显示，即以字符串的方式显示
            UART3_SendString((char *)UART1_GetRxData());                   // 把上位机发过来的数据，发送到ESP8266
            UART1_ClearRx();                                               // 清0本次接收; 每一次处理接收的数据后，要清0，才能进行下一轮的判断接收;
        }

        /** 判断ESP8266是否接收到新数据，打印，并处理 **/
        if (UART3_GetRxNum())                                              // 检查ESP8266所用的串口是否收到数据
        {
            uint16_t num = UART3_GetRxNum();                               // 获取接收到的字节数
            uint8_t *pData = UART3_GetRxData();                            // 获取接收到的数据(缓存的地址)
            printf("\r<<<ESP8266 返回以下数据：\r");                       // 提示
            printf("<<<长度：%d 字节\r", num);                             // 显示字节数
            printf("<<<内容：%s \r", pData);                               // 显示数据，以ASCII方式显示，即以字符串的方式显示
            UART3_ClearRx();                                               // 清0本次接收; 每一次处理接收的数据后，要清0，才能进行下一轮的判断接收;
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
