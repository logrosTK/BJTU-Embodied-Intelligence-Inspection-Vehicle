/* USER CODE BEGIN Header */
/**==================================================================================================================
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 **==================================================================================================================
 ** 【实验名称】   SD卡 基础读写测试（无USB/文件系统）
 **                
 ** 【适用平台】   STM32F407 + Keil5 + HAL库
 **                
 ** 【功能概述】   本程序实现对 SD 卡的基础块读写操作，不涉及文件系统，用于验证 SDIO 接口及 SD 卡底层通信。
 **                1. 初始化系统时钟、GPIO、SDIO 接口
 **                2. 获取 SD 卡信息（类型、版本、容量等）并通过串口打印
 **                3. 读取指定块区域的原数据，保存到缓冲区
 **                4. 向指定块区域写入测试数据，并回读验证
 **                5. 擦除指定块区域，读取验证擦除效果
 **                6. 恢复原数据，确保不破坏卡内原有内容
 **                7. 绿色 LED 周期性闪烁，指示系统运行状态
 **                
 ** 【硬件注意】   1. SD 卡消耗电流较大，建议使用独立电源或开发板主电源供电，避免使用 STLink、USB 转 TTL 等模块供电
 **                2. SDIO 通信占用 6 个引脚：PC8(CLK)、PC9(CMD)、PC10~12(D0~D3) 和 PD2(D4)，请确保这些引脚未被其他外设占用
 **
 ** 【CubeMX配置】 1. 在 Connectivity / SDIO 中选择 "SD 4 bits Wide bus"
 **                2. 对于 F1 系列，需将 SDIOCLK clock divide factor 设置为 6
 **                3. 注意 CubeMX 代码生成 bug：sdio.c 中的 MX_SDIO_SD_Init() 函数中，hsd.Init.BusWide 会被设置为 SDIO_BUS_WIDE_4B，
 **                   必须手动修改为 SDIO_BUS_WIDE_1B，否则 SD 卡无法正常工作
 **                
 ** 【移植说明】   1. 确保目标工程已正确配置 SDIO 外设及 GPIO
 **                2. 在 main.c 中包含 "bsp_UART.h" 以支持串口调试输出
 **                3. 若使用其他串口，请修改 UART1_Init() 对应的波特率和引脚
 **
 ** 【测试流程】   本程序按顺序执行以下步骤，结果通过串口输出：
 **                1. 获取卡信息（类型、版本、块数量、块大小、容量）
 **                2. 读取测试块（起始块 7，长度 2 块）的原数据并打印
 **                3. 写入测试数据（0x88）到测试块，并回读验证
 **                4. 擦除测试块（块 7~8），回读验证擦除后数据为 0x00
 **                5. 将原数据写回测试块，回读验证恢复成功
 **                
 ** 【调试建议】   1. 测试前请备份 SD 卡重要数据，程序会修改块 7 和块 8 的内容
 **                2. 若通信失败，检查 SDIO 引脚连接及供电是否充足
 **                3. 使用串口助手观察输出，确认每一步执行结果
 **                4. 注意 CubeMX 生成的 SDIO 初始化代码中的总线宽度错误，必须手动修正
 **                
 ** 【更新记录】   2024-02-19  优化代码结构，增加详细注释
 **                2024-02-17  从标准库迁移至 HAL 库
 **                
 ** 【备注说明】   版权归魔女科技所有，仅供学习使用，请勿用于商业用途。
 **                https://demoboard.taobao.com
 **
==================================================================================================================**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sdio.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
    MX_SDIO_SD_Init();
    /* USER CODE BEGIN 2 */
    // 所用用户代码，必须写在配对的BEGIN与END注释之间，否则重新生成时会被自动删除

    UART1_Init(115200);

    /***************** SD卡读写通信测试 *****************/
    /* 1、获取卡信息，打印到串口助手                    */
    /* 2、读测试：读出测试位置原数据，保存在 aOldData[] */
    /* 3、写测试：在测试的块上，写入指定数据            */
    /*    读出刚才写入的块数据，打印到串口助手观察      */
    /* 4、擦除测试：擦除指定块上的数据                  */
    /*    读出刚才擦除块的数据，打印到串口助手观察      */
    /* 5、写回原数据到指定位置                          */
    /*    读出刚才写入的块数据，打印到串口助手观察      */

#define SD_TEST_SIZE    1024                                       // 测试数据的字节数，刚好是2个块大小：2x512
    static uint8_t aOldData[SD_TEST_SIZE] = {0};                       // 用于存放旧数据，先读出来，测试完了，再把旧数据写回去
    static uint8_t aTestData[SD_TEST_SIZE] = {0};                      // 临时缓存，用来存放测试数据
    HAL_SD_CardInfoTypeDef pCardInfo = {0};                            // SD卡信息结构体

    uint8_t status = HAL_SD_GetCardState(&hsd);                        // SD卡状态标志值
    if (status == HAL_SD_CARD_TRANSFER)
    {
        /* 1、获取卡信息，打印到串口助手 */
        HAL_SD_GetCardInfo(&hsd, &pCardInfo);                          // 获取 SD 卡的信息
        printf("\r1、获取SD卡信息 ... \r\n");
        printf("卡类型：%d \r\n", pCardInfo.CardType);                 // 类型返回：0-SDSC、1-SDHC/SDXC、3-SECURED
        printf("卡版本：%d \r\n", pCardInfo.CardVersion);              // 版本返回：0-CARD_V1、1-CARD_V2
        printf("块数量：%d \r\n", pCardInfo.BlockNbr);                 // 可用的块数量
        printf("块大小：%d \r\n", pCardInfo.BlockSize);                // 每个块的大小; 单位：字节
        printf("卡容量：%lluG \r\n", ((unsigned long long)pCardInfo.BlockSize * pCardInfo.BlockNbr) / 1024 / 1024 / 1024);  // 计算卡的容量

        HAL_Delay(1000);                                               // 重要：稍作延时再开始读写测试; 避免有些仿真器烧录期间的多次复位，短暂运行了程序，导致下列读写数据不完整。

        /* 2、读测试：读出测试位置原数据，保存在 aOldData[] */
        printf("\r2、读取测试块的原数据 ... \r\n");
        memset(aOldData, 0, SD_TEST_SIZE);                             // 清0数组的数据
        if (HAL_SD_ReadBlocks(&hsd, aOldData, 7, 2, 3000) == HAL_OK)   // 读SD卡数据块; 参数：SD结构体、数据地址、块起始地址、读的块数量、超时时间;
        {
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER); // 等待卡的读写操作结束
            for (uint32_t i = 0; i < SD_TEST_SIZE; i++)                // 打印 原数据
                printf("%X ",  aOldData[i]);
            printf("\r\n");
        }
        else
        {
            printf("SD卡 读测试 失败!\n");
        }

        /* 3-1、写测试：在测试的块上写入数据 */
        printf("\r3、SD卡 写入测试 ...\r\n");
        memset(aTestData, 0x8, SD_TEST_SIZE);                          // 为数组准备要写入的测试数据：整个数组填充指定值
        if (HAL_SD_WriteBlocks(&hsd, aTestData, 7, 2, 3000) == HAL_OK) // 向SD卡写入数据块; 参数：SD结构体、数据地址、块起始地址、写入的块数量、超时时间;
        {
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER); // 等待卡的读写操作结束
            printf("对指定块写入结束! \r写入的数据是：\n");
            for (uint32_t i = 0; i < SD_TEST_SIZE; i++)                // 打印 写入的数据
                printf("%X ",  aTestData[i]);
            printf("\r\n");
        }
        else
        {
            printf("SD卡 写测试 失败!\n");
        }
        /* 3-2、读出刚才写测试的块内数据 */
        printf("\r现在块内的数据是：\r\n");
        memset(aTestData, 0, SD_TEST_SIZE);                            // 清0数组的数据
        if (HAL_SD_ReadBlocks(&hsd, aTestData, 7, 2, 3000) == HAL_OK)  // 读SD卡数据块; 参数：SD结构体、数据地址、块起始地址、读的块数量、超时时间;
        {
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER); // 等待卡的读写操作结束
            for (uint32_t i = 0; i < SD_TEST_SIZE; i++)                // 打印 写入后块内现在数据
                printf("%X ",  aTestData[i]);
            printf("\r\n");
        }
        else
        {
            printf("SD卡 读测试 失败!\n");
        }

        /* 4-1、擦除测试：擦除指定块上的数据  */
        printf("\r4、擦除块测试 ...\r\n");
        if (HAL_SD_Erase(&hsd, 7, 8) == HAL_OK)                        // 擦除SD卡上的数据; 参数：SD结构体、块的起始地址、块的结束地址
        {
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER); // 等待卡的读写操作结束
            printf("擦除 成功！ \r\n");
        }
        else
        {
            printf("擦除 失败！ \r\n");
        }
        /* 4-2、读取，擦除后指定块上的数据  */
        printf("擦除后，现在块内的数据是：\r\n");
        memset(aTestData, 0, SD_TEST_SIZE);                            // 清0数组的数据
        if (HAL_SD_ReadBlocks(&hsd, aTestData, 7, 2, 3000) == HAL_OK)  // 读SD卡数据块; 参数：SD结构体、数据地址、块起始地址、读的块数量、超时时间;
        {
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER); // 等待卡的读写操作结束
            for (uint32_t i = 0; i < SD_TEST_SIZE; i++)                // 打印 块内现在的数据
                printf("%X ",  aTestData[i]);
            printf("\r\n");
        }
        else
        {
            printf("SD卡 读测试 失败!\n");
        }

        /* 5-1、写回测试块上的原数据 */
        printf("\r5、写回原数据 ...\r\n");
        //memset(aOldData, 1, SD_TEST_SIZE);
        if (HAL_SD_WriteBlocks(&hsd, aOldData, 7, 2, 3000) == HAL_OK)  // 向SD卡写入数据块; 参数：SD结构体、数据地址、块起始地址、写入的块数量、超时时间;
        {
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER); // 等待卡的读写操作结束
            printf("写入结束! \n");
        }
        else
        {
            printf("SD卡 写回原数据 失败!\n");
        }
        /* 5-2、读取，写入后的数据 */
        printf("现在块内的数据是: \r\n");
        memset(aTestData, 0, SD_TEST_SIZE);                            // 清0数组的数据
        if (HAL_SD_ReadBlocks(&hsd, aTestData, 7, 2, 3000) == HAL_OK)  // 读SD卡数据块; 参数：SD结构体、数据地址、块起始地址、读的块数量、超时时间;
        {
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER); // 等待卡的读写操作结束
            for (uint32_t i = 0; i < SD_TEST_SIZE; i++)                // 打印 块内现在的数据
                printf("%X ",  aTestData[i]);
            printf("\r\n\r\n");
        }
        else
        {
            printf("SD卡 读测试 失败! \r\n");
        }

        printf("SD卡 读写测试结束！\r\n");
    }


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        /** 每500ms执行一次的任务 **/
        static uint32_t lastTime_500ms = 0;                    // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)             // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);             // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            lastTime_500ms =   HAL_GetTick();                  // 记录当前时间，作为新的基准点，为下一次判断做准备
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

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
