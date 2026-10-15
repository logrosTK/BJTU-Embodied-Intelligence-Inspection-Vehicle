/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * 实验说明
  * 接线：开发板上带的RS485电路，在开发板右上角，可以通过跳线帽，很方便地连接UART3, 即PB10、PB11
  *       也可以使用杜邦，选择其它串口
  * 代码：本示例，使用了bsp_UART.c作串口通信的底层驱动。
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */
#include <stdio.h>              // 用于sprintf函数
#include <string.h>
#include "bsp_UART.h"

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
    /* USER CODE BEGIN 2 */
    /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

    UART1_Init(115200);                  // 初始化 UART1; 使用默认引脚: PA9 +PA10; 这俩引脚已并联了板载USB转TTL电路，可用于与电脑串口助手通信

    UART3_Init(3000000);                  // 初始化 UART3; 使用默认引脚: PB10+PB11;




    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

        HAL_Delay(1 - 1);                                                  // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /** USART1 的接收示范(通过串口助手) **/
        if (UART1_GetRxNum())                                              // 判断串口是否收到数据
        {
            uint16_t  RxNum = UART1_GetRxNum();                            // 获取串口最后一帧接收的字节数
            uint8_t  *RxData = UART1_GetRxData();                          // 获取串口最后一帧接收的数据(地址)
            printf("\r<<<<< UART1 接收到一帧数据 \r");                     // 提示

            printf("字节数：%d \r", RxNum);                                // 显示字节数
            printf("ASCII : %s\r", (char *)RxData);                        // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                            // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < RxNum; i++)                           // 逐个字节输出
                printf("0x%X ", RxData[i]);                                // 以16进制显示
            printf("\r");                                                  // 显示换行
            printf("\r>>>>> 将向串口3发送本帧的数据 \r");                  // 提示发送, 即把串口助手发过来的数据(UART1接收到的), 发送到其它串口
            UART3_SendData(RxData, RxNum);                                 // 把串口1接收到的数据(串口助手发过来的), 通过串口3发送到RS485总线
            printf("\r");                                                  // 显示换行
            UART1_ClearRx();                                               // 清0接收标记
        }

        /** RS485通过 UART3 的接收示范 **/
        if (UART3_GetRxNum())                                              // 判断串口是否收到数据
        {
            uint16_t  RxNum = UART3_GetRxNum();                            // 获取串口最后一帧接收的字节数
            uint8_t  *RxData = UART3_GetRxData();                          // 获取串口最后一帧接收的数据(地址)
            printf("\r<<<<< RS485(UART3) 接收到一帧数据 \r");              // 提示
            printf("字节数：%d \r", RxNum);                                // 显示字节数
            printf("ASCII : %s\r", (char *)RxData);                        // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                            // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < RxNum; i++)                           // 逐个字节输出
                printf("0x%X ", RxData[i]);                                // 以16进制显示
            printf("\r\r");                                                // 显示换行
            UART3_ClearRx();                                               // 清0接收标记

            if (strstr((char *)RxData, "RS485_Test"))                      // 判断是否测试工具发过来的数据
            {
                UART3_SendString("RS485_OK");                              // 发回一次报文，方便对方测试;
            }
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
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯状态
        printf("KEY_1 按下\r");                                 // 通过串口助手输出提示
    }

    if (GPIO_Pin == KEY_2_Pin)                                  // 按键2 产生的中断信号：KEY_2、PA1、闲时上拉、按下时低电平、下降沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯状态
        printf("KEY_2 按下\r");                                 // 通过串口助手输出提示
    }

    if (GPIO_Pin == KEY_3_Pin)                                  // 按键3 产生的中断信号：KEY_3、PA4、闲时上拉、按下低高电平、下降沿触发
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                  // 反转蓝色LED灯状态
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
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
