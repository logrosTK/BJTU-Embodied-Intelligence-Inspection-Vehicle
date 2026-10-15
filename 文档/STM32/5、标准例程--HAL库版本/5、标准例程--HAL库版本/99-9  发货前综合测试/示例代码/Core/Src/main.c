/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【工程说明】  开发板出厂综合测试程序
 **
 ** 【功能概述】  本示例为发货前综合测试程序，已完成以下外设初始化与功能验证：
 **               1. UART1 / UART3 串口通信（中断接收 + 数据转发）
 **               2. CAN 总线通信（收发测试 + 数据解析）
 **               3. LCD ILI9341 显示屏（显示 + GUI）
 **               4. XPT2046 触摸屏（坐标检测 + 校准）
 **               5. W25Q128 外部 FLASH（初始化 + 字库检测）
 **               6. 24C02 EEPROM（I2C 读写测试）
 **               7. LED 指示灯（状态闪烁）
 **               8. 按键中断（KEY1/KEY2/KEY3 触发测试）
 **
 ** 【适用平台】  STM32F407 + Keil5 + HAL 库
 **
 ** 【使用说明】  1. 所有用户代码必须写在配对的 USER CODE BEGIN / END 之间，
 **                  否则使用 CubeMX 重新生成代码时会被自动删除。
 **               2. 通过串口助手（115200-8-N-1）观察系统启动信息及外设状态。
 **               3. 可通过串口发送数据，测试 UART1/UART3 转发、CAN 转发及 LCD 显示。
 **               4. 触摸屏校准：串口发送 “XPT2046” 或按下 KEY1 按键。
 **
 ** 【移植说明】  1. 本示例使用软件 I2C（24C02）和 SPI（W25Q128、LCD、触摸屏），
 **                  引脚定义在对应的 bsp 头文件中，无需 CubeMX 配置。
 **               2. CAN 通信需外接 CAN 收发器（PB8、PB9）。
 **               3. UART3 可用于 RS485 或 ESP8266 通信，视跳线帽而定。
 **
 ** 【更新记录】  2024-02-19  完善注释，优化外设初始化流程
 **               2024-02-17  从标准库迁移至 HAL 库
 **
 ** 【备注说明】  版权归魔女科技所有，请勿商用，谢谢！
 **               https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 所用用户代码，必须写在配对的BEGIN与END注释之间，否则重新生成时会被自动删除

#include "bsp_UART.h"           // 串口通信
#include "bsp_W25Q128.h"        // 外部Flash, W25Q128
#include "bsp_lcd_ili9341.h"    // 2.8寸显示屏
#include "bsp_xpt2046.h"        // 触摸屏
#include "bsp_CAN.h"            // CAN
#include "bsp_I2CSoft.h"        // 软件模拟I2C
#include "bsp_24C02.h"          // 24C02

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
// 所用用户代码，必须写在配对的BEGIN与END注释之间，否则重新生成时会被自动删除



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
    MX_CAN1_Init();
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除 */

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);          // 点亮LED_BLUE:  PB2置低电平
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);          // 点亮LED_GREEN: PC5置低电平

    UART1_Init(115200);                                             // UART1 初始化; 已在bsp_UART.c中处理printf的重定向输出到UART1
    printf("芯片内部FLASH            ");                            // 读取本芯片FLASH大小，
    printf("%d Kbytes\n", *(uint16_t *)(0x1FFF7A22));               // 本地址段，存放着芯片出厂时编写的FLASH大小;　本行另一个作用：示例使用指针读取数据
    
    W25Q128_Init();                                                 // 设备W25Q128： 引脚初始化，SPI初始化，测试连接, 测试是否存在字库

    LCD_Init();                                                     // 初始化LCD
    LCD_SetDir(0);                                                  // 设置显示的方向; 0-竖屏、1-横屏
    LCD_GUI();                                                      // 示例显示

    XPT2046_Init(LCD_GetWidth(), LCD_GetHeight(), LCD_GetDir());    // 触摸检测 初始化; 参数：屏宽、屏高、显示的方向

    UART3_Init(115200);                                             // UART3初始化，进入接收状态; 用于RS485、ESP8266, 视跳线帽而定

    I2CSoft_Init();
    AT24C02_Init();                                                 // 初始化EEPROM芯片24C02：引脚初始化、读写测试

    CAN1_FilterInit();                                              // 配置CAN1的筛选器; 配置完后启动CAN1; 为方便调试，本函数配置为：接收所有数据帧，包括标准帧、扩展帧
    if (HAL_CAN_Start(&hcan1) == HAL_OK)                            // 启动CAN，并检查返回值：成功-0、错误-1、忙错误-2、超时-3
    {                                                             
        printf("CAN1 启动成功! \r");                                // 启动成功
    }                                                             
    else                                                          
    {                                                             
        printf("CAN1 启动失败! \r");                                // 启动失败
        printf("检查PB8、PB9是否已连接收发器！\r");               
        //Error_Handler();                                          // 错误处理
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // 用户代码，必须写在配对的BEGIN与END注释之间; 否则,重新生成时会被自动删除;

        /** 1ms延时 **/
        // HAL_Delay(1 - 1);                                     // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        /******  检查 USART1 是否收到数据 ******/
        static uint32_t lastTime_UART1Show = 0;                  // 定义静态变量，存储串口1数据显示到LCD的最后时间值，用于清屏计时
        if (UART1_GetRxNum())                                    // 如果USART1收到数据(从串口上位机中发过来的）
        {
            // 获取数据
            uint16_t rxNum  = UART1_GetRxNum();                  // 获取最后一帧的字节数
            uint8_t *rxData = UART1_GetRxData();                 // 获取最后一帧的数据(地址)
            // 把收到的数据，printf到串口助手观察
            printf("\r<<<<<< USART1 接收到新数据 <<<<<<");       // 准备输出
            printf("\r 长度(字节数)：%d", rxNum);                // printf 字节数
            printf("\r 数据(16进制)：");                         // printf 16进制方式显示数据，方便观察真实数据
            for (uint16_t i = 0; i < rxNum; i++)                 // 输出每一字节数值
                printf("0x%X ", rxData[i]);
            printf("\r 数据(ASCII) ：%s\r", rxData);             // ASCII方式显示数据，方便观察字符串数据;
            // 通过各个渠道转发，以方便测试
            printf("\r>>>>>> 准备发送下面数据 >>>>>>\r");        // printf 提示转发
            printf("%s", (char *)rxData);                        // printf 发送的具体内容
            UART3_SendData(rxData, rxNum);                       // 通过UART3转发; 视跳线帽而定 RS485或ESP8266
            printf("\r已通过 USART3 转发 \r");
            CAN1_SendData(CAN_ID_EXT, 0x123, rxData, rxNum);     // 通过CAN1转发
            printf("\r已通过 CAN1 转发 \r");
            // 判断数据是否包含某些指令
            if (strstr((char *)rxData, "XPT2046"))               // 判断数据是否包含某些指令
                XPT2046_ReCalibration();                         // 重新校准触摸屏
            // 清0本帧接收
            UART1_ClearRx();                                     // 清0接收(接收字节数); 重要: 处理完数据后，谨记把接收数量清０, 再进入下次判断;
            // 数据显示到 LCD
            lastTime_UART1Show = HAL_GetTick();                  // 记录UART1数据显示到LCD的最后时间值，用于清屏
            static char strUART1[100] = {0};
            memset(strUART1, 0, 100);
            LCD_Fill(120, 71, 239, 125, BLACK);                  // 清0区域显示
            for (uint8_t i = 0; i < 14; i++)
                strUART1[i] = rxData[i];
            LCD_String(125, 71, strUART1, 16, GREEN, BLACK);     // 显示字符串
            uint8_t x = 125;
            for (uint8_t i = 0; i < 6; i++)
            {
                sprintf(strUART1, "%02X ",  rxData[i]);
                LCD_String(x, 88, strUART1, 12, WHITE, BLACK);   // 显示16进制
                x = x + 18;
            }
            x = 125;
            for (uint8_t i = 6; i < 12; i++)
            {
                sprintf(strUART1, "%02X ",  rxData[i]);
                LCD_String(x, 100, strUART1, 12, WHITE, BLACK);  // 显示16进制
                x = x + 18;
            }
            x = 125;
            for (uint8_t i = 12; i < 18; i++)
            {
                sprintf(strUART1, "%02X ", rxData[i]);
                LCD_String(x, 112, strUART1, 12, WHITE, BLACK);  // 显示16进制
                x = x + 18;
            }
        }
        else
        {
            if (HAL_GetTick() - lastTime_UART1Show > 5000)       // 超过N秒没收到新数据，清空LCD上的显示数据
            {
                LCD_Fill(120, 71, 239, 125, BLACK);              // 清0区域显示
                lastTime_UART1Show = HAL_GetTick();              // 记录UART1数据显示到LCD的最后时间值，用于清屏
            }
        }

        /******  检查USART3是否收到数据 ******/
        static uint32_t lastTime_UART3Show = 0;                  // 定义静态变量，存储串口3数据显示到LCD的最后时间值，用于清屏计时
        if (UART3_GetRxNum())                                    // 检查USART3是否收到数据
        {
            // 获取数据
            uint16_t rxNum  = UART3_GetRxNum();                  // 获取最后一帧的字节数
            uint8_t *rxData = UART3_GetRxData();                 // 获取最后一帧的数据(地址)
            // 把收到的数据，printf到串口助手观察
            printf("\r<<<<<< USART3 接收到新数据 <<<<<<");       // 准备输出
            printf("\r 长度(字节数)：%d", rxNum);                // 长度; 单位:字节
            printf("\r 数据(16进制)：");                         // 16进制方式显示数据，方便观察真实数据
            for (uint16_t i = 0; i < rxNum; i++)                 // 输出每一字节数值
                printf("0x%X ", rxData[i]);
            printf("\r 数据(ASCII) ：%s\r", rxData);             // 显示数据， 输出到电脑上位机
            // 判断数据是否包含某些指令
            if (strstr((char *)rxData, "RS485_Test"))            // 判断是否测试工具发过来的数据
                UART3_SendString("RS485_OK");                    // 发回一次报文，方便对方测试;
            // 清0本帧接收
            UART3_ClearRx();                                     // 清0接收(接收字节数); 重要: 处理完数据后，谨记把接收数量清０, 再进入下次判断;
            // 数据显示到 LCD
            lastTime_UART3Show = HAL_GetTick();                  // 记录UART3数据显示到LCD的最后时间值，用于清屏
            static char strUART3[100];
            memset(strUART3, 0, 100);
            LCD_Fill(120, 146, 239, 205, BLACK);                 // 清空显示区域
            for (uint8_t i = 0; i < 14; i++)
                strUART3[i] = rxData[i];
            LCD_String(125, 147, strUART3, 16, GREEN, BLACK);    // 显示字符串
            uint8_t x = 125;
            for (uint8_t i = 0; i < 6; i++)
            {
                sprintf(strUART3, "%02X ",  rxData[i]);
                LCD_String(x, 166, strUART3, 12, WHITE, BLACK);  // 显示16进制
                x = x + 18;
            }
            x = 125;
            for (uint8_t i = 6; i < 12; i++)
            {
                sprintf(strUART3, "%02X ",  rxData[i]);
                LCD_String(x, 179, strUART3, 12, WHITE, BLACK);  // 显示16进制
                x = x + 18;
            }
            x = 125;
            for (uint8_t i = 12; i < 18; i++)
            {
                sprintf(strUART3, "%02X ", rxData[i]);
                LCD_String(x, 192, strUART3, 12, WHITE, BLACK);  // 显示16进制
                x = x + 18;
            }
        }
        else
        {
            if (HAL_GetTick() - lastTime_UART3Show > 5000)       // 超过N秒没收到新数据，清空LCD上的显示数据
            {
                LCD_Fill(120, 146, 239, 205, BLACK);             // 清空显示区域
                lastTime_UART3Show = HAL_GetTick();              // 记录UART3数据显示到LCD的最后时间值，用于清屏
            }
        }

        /** 检查CAN1是否收到数据 **/
        static uint32_t lastTime_CAN1Show = 0;                                          // 定义静态变量，存储 CAN 数据显示到LCD的最后时间值，用于清屏计时
        if (CAN1_GetRxNum())                                                            // 检查接收字节数是否>0，以判断是否接收到新一帧数据
        {
            static uint16_t  canRxCNT = 1;                                              // 用于计算已接收了多少帧数据; 非必要;
            // 1.获取报文 数据
            uint8_t  rxNum = CAN1_GetRxNum();                                           // 获取最后一帧的字节数
            uint8_t *rxData = CAN1_GetRxData();                                         // 获取最后一帧的数据(地址)
            CAN_RxHeaderTypeDef rxHeader = CAN1_GetRxHeader();                          // 获取最后一帧的头部信息
            // 2.解释并打印报文详情 (调试用)
            printf("\r****** CAN 接收到第 %d 帧报文 ******", canRxCNT++);               // 准备把CAN帧报文，详细地输出到串口软件，方便观察调试
            printf("\r 帧类型：%s",   rxHeader.RTR ? "遥控帧" : "数据帧");              // 帧类型
            printf("\r 帧格式：%s",   rxHeader.IDE ? "扩展帧" : "标准帧");              // 帧格式
            printf("\r 帧 ID ：0x%X", rxHeader.IDE ? rxHeader.ExtId : rxHeader.StdId);  // 帧ID
            printf("\r 字节数：%d",   rxHeader.DLC);                                    // 字节数
            printf("\r 筛选器匹配序号：%d", rxHeader.FilterMatchIndex);                 // 筛选器匹配序号; 和筛选器编号，是不一样的。大概：从筛选器0开始，每个16位宽筛选器+2, 32位宽+1, 没有被使用的筛选器，默认是16位宽，+2;
            // 3.打印数据字节（十六进制）
            printf("\r 显示数据(16进制)：");                                            // 16进制方式显示数据，方便观察真实数据
            for (uint8_t i = 0; i < rxNum; i++)                                         // 逐个字节显示CAN的数据值
                printf(" 0x%02X ", rxData[i]);                                          // 格式：16进制显示
            printf("\r");                                                               // 换行，以使每帧的输出更清晰
            // 4.报文是否包含预期执行的指令
            if (strstr((char *)rxData, "CAN_Test"))                                     // 判断是否测试工具发过来的指定数据
            {
                static char strTemp[8] = "CAN_OK";                                      // 测试工具约定好的返回数据
                CAN1_SendData(CAN_ID_EXT, 0x123, (uint8_t *)strTemp, 8);                // 发送CAN报文
            }
            // 5.清0本帧接收
            CAN1_ClearRx();                                                             // 清0接收(接收字节数); 重要: 处理完数据后，谨记把接收数量清０, 再进入下次判断;
            // 6.显示数据到LCD
            static char strCAN[100];
            lastTime_CAN1Show = HAL_GetTick();                                          // 记录CAN1数据显示到LCD的最后时间值，用于清屏
            LCD_Fill(120, 226, 239, 287, BLACK);                                        // 把数据显示到LCD上方便观察
            LCD_String(125, 228, (char *)rxData, 16, GREEN, BLACK);
            sprintf(strCAN, "%X %X %X %X", rxData[0], rxData[1], rxData[2], rxData[3]);
            LCD_String(125, 246, strCAN, 16, WHITE, BLACK);
            sprintf(strCAN, "%X %X %X %X", rxData[4], rxData[5], rxData[6], rxData[7]);
            LCD_String(125, 265, strCAN, 16, WHITE, BLACK);
        }
        else
        {
            if (HAL_GetTick() - lastTime_CAN1Show > 5000)                               // 超过N秒没收到新数据，清空LCD上的显示数据
            {
                LCD_Fill(120, 226, 239, 287, BLACK);
                lastTime_CAN1Show = HAL_GetTick();                                      // 记录 CAN1 数据显示到LCD的最后时间值，用于清屏
            }
        }

        /** 每5ms执行一次的任务 **/
        static uint32_t lastTime_5ms = 0;                                    // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_5ms >= 5)                               // 检查当前时间距离上次执行是否已经过了30ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            if (XPT2046_IsPressed())                                         // 1.执行动作：检查触摸屏此刻是否按下; 返回：0-未按下、1-按下
            {
                static char str[20] = {0};                                   //   新建数组，用于存放坐标字符串
                LCD_DrawPoint(XPT2046_GetX(), XPT2046_GetY(), GREEN);        //   在按下的位置，画点; 用于测试
                sprintf(str, "%3d  %3d", XPT2046_GetX(), XPT2046_GetY());    //   格式化坐标字符串
                LCD_String(185, 290, str, 12, BLACK, WHITE);                 //   在LCD上显示坐标字符串
            }
            lastTime_5ms =   HAL_GetTick();                                  // 2.更新本任务最后执行的时间值
        }

        /** 每隔500ms，闪烁一次绿色LED **/
        static uint32_t lastTime_500ms = 0;                                  // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                           // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            /* 执行任务 */
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);                           // 执行动作。控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            /* 更新任务时间戳 */
            lastTime_500ms =   HAL_GetTick();                                // 更新本任务最后执行的时间值
        }
        
        /** 每1000ms执行一次的任务 **/
        static uint32_t lastTime_1000ms = 0;                                 // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_1000ms >= 1000)                         // 检查当前时间距离上次执行是否已经过了1000ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {     
            static struct{
            uint8_t h,m,s;
            } time = { 0 };                                                  // 定义一个结构体存储时分秒
            static char timeDate[20];                                        // 定义字符串缓存
            if(++time.s>=60)                                                 // 秒值达到60
            {                                                                
                time.s=0;                                                    
                if(++time.m>=60)                                             // 分值达到60
                {                                                            
                    time.m=0;                                                
                    if(++time.h>=24)                                         // 时值达到24
                    {
                        time.h=0;
                    }
                }
            }
            sprintf(timeDate,"%02d:%02d:%02d", time.h, time.m, time.s);      // 把时分秒转换为字符串
            LCD_String(185, 306, timeDate, 12, BLACK, WHITE);                // 把时分秒显示到LCD
            lastTime_1000ms =   HAL_GetTick();                               // 更新本任务最后执行的时间值
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
