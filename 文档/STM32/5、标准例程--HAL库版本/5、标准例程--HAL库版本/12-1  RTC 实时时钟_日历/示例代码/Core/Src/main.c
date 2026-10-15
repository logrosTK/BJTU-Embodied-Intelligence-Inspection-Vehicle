/* USER CODE BEGIN Header */
/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【实验名称】  STM32F407 RTC实时时钟 —— 日历设置 + 闹钟配置 + 串口时间校准
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【 CubeMX 】  1- 开启RTC，启用LSE外部晶振（32.768kHz）；
 **               2- 开启USART1，用于调试打印与时间字符串接收；
 **
 ** 【功能说明】  本工程实现 RTC 完整功能：
 **               1- 支持手动设置年月日时分秒星期，带参数合法性校验；
 **               2- 支持串口发送固定格式时间字符串，自动解析并更新RTC；
 **               3- 每秒自动打印当前时间、日期、星期；
 **               4- 按键中断触发，可扩展闹钟、低功耗唤醒功能；
 **
 ** 【时间格式】  发送格式示例：2026年03月05日15时45分00秒星期四
 **               严格按照：年-月-日-时-分-秒-星期 顺序发送
 **
 ** 【代码使用】  1- 上电后自动打印当前时间；
 **               2- 串口助手发送时间字符串，自动校准RTC；
 **               3- LED(PC5) 500ms闪烁一次，表示系统正常运行；
 **               4- 按键KEY1/KEY2/KEY3触发中断，可自定义功能；
 **
 ** 【划 重 点】  1- RTC读写必须遵循：先写时间 → 后写日期；
 **               2- RTC读取必须遵循：先读时间 → 后读日期；
 **               3- CubeMX默认年份基准为2000年，使用时需偏移计算；
 **               4- 串口时间解析严格匹配格式，错一个字符都会解析失败；
 **               5- LSE晶振是RTC精准计时的关键，务必开启外部低速晶振；
 **
 *******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 * 函  数： RTC_SetNewData
 * 功  能： 设置RTC的时间和日期（严格遵循“先时间后日期”的操作顺序）
 * 说  明： 1. 操作前需解锁后备域；
 *          2. CubeMX默认以2000年为RTC年份基准，需做偏移处理；
 *          3. 中国不实行夏令时，相关参数固定为NONE；
 * 参  数： uint16_t  year    实际年份（2000-2127，超出范围则报错）
 *          uint8_t   month   月份（1-12）
 *          uint8_t   date    日期（1-31）
 *          uint8_t   hours   小时（0-23，24小时制）
 *          uint8_t   minutes 分钟（0-59）
 *          uint8_t   seconds 秒数（0-59）
 *          uint8_t   weekday 星期（1=周一，2=周二，…，7=周日）
 * 返回值： 无
 ******************************************************************************/
void RTC_SetNewData(uint16_t year, uint8_t month, uint8_t date, uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t weekday)
{
    /* 定义RTC时间、日期结构体 */
    RTC_TimeTypeDef sTime = {0};                                     // 时间结构体 (时/分/秒)
    RTC_DateTypeDef sDate = {0};                                     // 日期结构体 (年/月/日/星期)
    /* 检查参数合法性 */
    if ((year < 2000) || (year > 2127) || (month < 1) || (month > 12) || (date < 1) || (date > 31) ||
            (hours > 23) || (minutes > 59) || (seconds > 59) || (weekday < 1) || (weekday > 7))
    {
        printf("日历配置函数：参数非法，退出本次设置。\r\n");        // 打印 非法参数
        return;                                                      // 非法参数直接退出，不执行后续配置
    }
    /* 配置时间; RTC强制要求：先写时间，后写日期 */
    sTime.Hours = hours;                                             // 时; 注意小时的格式，检查MX_RTC_Init()函数里的HourFormat参数，有24小时、12小时两种格式
    sTime.Minutes = minutes;                                         // 分
    sTime.Seconds = seconds;                                         // 秒
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;                  // 关闭夏令时。中国不实行夏令时，因此固定设置为NONE
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;                 // 夏令时配套参数，固定为RESET
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)    // 设置时间; RTC要求：先设置时间（必须顺序）
    {
        printf("RTC时间写入失败! \n");
        return;                                                      // 直接退出，不执行后续配置
    }
    /* 配置日期; 必须在时间写入后执行 */
    sDate.WeekDay = weekday;                                         // 星期：1-一、2-二、3-三、4-四、5-五、6-六、7-日
    sDate.Month = month;                                             // 月
    sDate.Date = date;                                               // 日
    sDate.Year = year - 2000;                                        // 年份偏移; CubeMX默认2000为基准, 需减去2000
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)    // RTC要求：后设置日期（必须顺序）
    {
        printf("RTC日期写入失败! \n");
        return;                                                      // 直接退出，不执行后续配置
    }
    /* 写入标记值到备份寄存器，标记已配置过日历 */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x01);                   // 备份寄存器1写入标志值。用于下次上电时判断是否已设置日历; 网上很多示例会用类似0x5A5A的值，原理是一样的，但0x5A5A容易被误解有特殊作用
    /* 打印提示 */
    printf("\nRTC日历数据写入成功！\r\n");                           // 打印 提示
}



/******************************************************************************
 * 函  数： RTC_ParseTimeString
 * 功  能： 解析固定格式的时间字符串，提取有效数据并更新RTC时间
 * 格  式： "YYYY年MM月DD日HH时MM分SS秒星期X"（如：2026年02月04日19时11分30秒星期三）
 * 约  定： 1. 字符串无空格、无换行，字段顺序/单位不可修改；
 *          2. 年4位、月/日/时/分/秒各2位，星期为中文单字（一/二/…/日）；
 *          3. 星期映射：一=1、二=2、三=3、四=4、五=5、六=6、日=7；
 * 参  数： const char* timeStr  待解析的时间字符串（串口接收的原始数据）
 * 返回值： 无
 * 异常处理：空指针/格式错误/星期非法 → 触发提示并退出，不执行更新
 ******************************************************************************/
void RTC_ParseTimeString(const char* timeStr)
{
    /* 空指针校验，避免程序崩溃 */
    if (timeStr == NULL)
    {
        printf("解释失败：输入字符串为空！ \r\n");      // 空字符串直接退出
        return;
    }
    /* 定义变量存储解析结果 */
    uint16_t year = 0;        // 年（4位，如2026）
    uint8_t month = 0;        // 月（2位，01-12）
    uint8_t date = 0;         // 日（2位，01-31）
    uint8_t hours = 0;        // 时（2位，00-23）
    uint8_t minutes = 0;      // 分（2位，00-59）
    uint8_t seconds = 0;      // 秒（2位，00-59）
    uint8_t weekday = 0;      // 星期（1-7，对应一-日）
    char weekStr[2] = {0};    // 存储星期中文单字
    /* 按固定格式解析字符串 */
    // %4hu：匹配4位无符号短整型（对应uint16_t）
    // %2hhu：匹配2位无符号字符（对应uint8_t）
    // %2s：匹配星期中文单字（适配GBK/UTF-8编码）
    int parseRet = sscanf(timeStr, "%4hu年%2hhu月%2hhu日%2hhu时%2hhu分%2hhu秒星期%2s",
                          &year, &month, &date, &hours, &minutes, &seconds, weekStr);
    /* 解析结果校验（正常应提取7个字段，否则格式不匹配） */
    if (parseRet != 7)
    {
        printf("解释失败：字符串格式错误! 请按示例格式发送：\n");  // 格式不匹配，进入错误处理
        printf("2026年03月05日15时45分00秒星期四 \n\n");           // 字符串格式，必须严格匹配
        return;
    }
    /* 中文星期转换为数字(匹配RTC_SetNewData参数) */
    if      (strcmp(weekStr, "一") == 0)    weekday = 1;           // 中文“一”对应RTC星期1（周一）
    else if (strcmp(weekStr, "二") == 0)    weekday = 2;           // 中文“二”对应RTC星期2（周二）
    else if (strcmp(weekStr, "三") == 0)    weekday = 3;           // 中文“三”对应RTC星期3（周三）
    else if (strcmp(weekStr, "四") == 0)    weekday = 4;           // 中文“四”对应RTC星期4（周四）
    else if (strcmp(weekStr, "五") == 0)    weekday = 5;           // 中文“五”对应RTC星期5（周五）
    else if (strcmp(weekStr, "六") == 0)    weekday = 6;           // 中文“六”对应RTC星期6（周六）
    else if (strcmp(weekStr, "日") == 0)    weekday = 7;           // 中文“日”对应RTC星期7（周日）
    else                                                           // 未知星期字符，解析失败
    {
        printf("\n解释失败，星期格式错误！\r\n ");
        return;
    }
    /* 解析成功，调用函数更新RTC时间 */
    RTC_SetNewData(year, month, date, hours, minutes, seconds, weekday);
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
    MX_USART1_UART_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    UART1_Init(115200);                                  // 初始化 串口1; 已写好底层，调用h中的函数即可使用; 引脚(PA9 +PA10)、波特率-None-8-1; 如果使用CubeMX配置，请使用前述引脚，但，不要在MX上进行中断及DMA配置，否则冲突



    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */

        /** 检查UART1是否收到串口助手发来的数据，用于更新日期时间 **/
        if (UART1_GetRxNum())                                                            // 判断USART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            /* 获取数据 */
            uint8_t *rxData = UART1_GetRxData();                                         // 获取数据的地址
            uint16_t rxNum = UART1_GetRxNum();                                           // 获取数据的字节数
            /* 把收到的数据，printf到串口助手观察*/
            printf("\r<<<<< USART1 收到一帧数据 \r");                                    // 提示
            printf("字节数：%d \r", rxNum);                                              // 显示字节数
            printf("ASCII : %s\r", (char *)rxData);                                      // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("\r\r");                                                              // 显示换行
            /* 执行数据分析 */
            RTC_ParseTimeString((const char*) rxData);                                   // 解释日历时间，并更新保存
            /* 清零接收标志 */
            UART1_ClearRx();                                                             // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0
        }

        /** 每隔500ms，闪烁一次绿色LED **/
        static uint32_t lastTime_500ms = 0;                                              // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                                       // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {                                                                             
            /* 执行任务 */                                                            
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);                                       // 执行动作。控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            /* 更新任务时间戳 */                                                      
            lastTime_500ms =   HAL_GetTick();                                            // 更新本任务最后执行的时间值
        }
        
        /** 每隔1000ms，读取一次日历数据，并打印 **/
        static uint32_t lastTime_1000ms = 0;                                             // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_1000ms >= 1000)                                     // 检查当前时间距离上次执行是否已经过了1000ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            /* 读取数据; 重点, 必须先读时间、后读日期 否则数据错乱 */
            RTC_TimeTypeDef time = { 0 };                                                // 时间 时-分-秒
            RTC_DateTypeDef data = { 0 };                                                // 日期 年-月-日          
            HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);                               // 读取时间数据; 必须先读时间后读日期; 写操作时，也必须先写时间后写日期，否则数据可能错乱
            HAL_RTC_GetDate(&hrtc, &data, RTC_FORMAT_BIN);                               // 读取日期数据; 必须先读时间后读日期; 写操作时，也必须先写时间后写日期，否则数据可能错乱
            /* 打印数据 */
            printf("%04d年%02d月%02d日  ", 2000 + data.Year, data.Month, data.Date);     // 打印 日期
            printf("%02d:%02d:%02d  ", time.Hours, time.Minutes, time.Seconds);          // 打印 时间
            switch (data.WeekDay)                                                        // 打印 星期
            {
                case 1:
                    printf("星期一");
                    break;
                case 2:
                    printf("星期二");
                    break;
                case 3:
                    printf("星期三");
                    break;
                case 4:
                    printf("星期四");
                    break;
                case 5:
                    printf("星期五");
                    break;
                case 6:
                    printf("星期六");
                    break;
                case 7:
                    printf("星期日");
                    break;
                default:
                    printf("星期错误");
                    break;
            }
            printf("  \r\n");                                                            // 打印 换行
            /* 更新任务时间戳 */
            lastTime_1000ms = HAL_GetTick();                                             // 更新时间戳：将当前时间设为新基准，确保下一次周期准确从此时开始计算
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
