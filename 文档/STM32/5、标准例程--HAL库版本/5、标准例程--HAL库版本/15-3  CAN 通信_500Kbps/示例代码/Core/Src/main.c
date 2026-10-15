/* USER CODE BEGIN Header */
/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板       https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【实验名称】  CAN1通信_500Kbps
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【 CubeMX 】  1- 使用CubeMX开启CAN，
 **               2- 在CAN1的参数页面，配置波特率为500Kbps: Prescaler-6、BS1-11、BS2-2、SJW-1; 工作模式Operating Mode: Normal;
 **               3- 在CAN1的中断页面，打勾中断：CAN1 RX0 interrupts
 **               4- CAN1开启后，默认引脚为PA11+PA12, 修改为PB8_RX、PB9_TX
 **
 ** 【bsp_CAN】   CubeMX配置CAN1并重新生成后，会生成can.c和can.h，并在main.c中调用了MX_CAN1_Init(), 对其所用引脚、CAN配置，都进行了初始化。
 **               用户需要在CubeMX生成的代码基础上，自行增加编写发送、接收的处理代码。
 **               我们把上述CAN通信的自定义代码，写在了自行新建的bsp_CAN.h和bsp_CAN.c文件中; 这样处理，好处有二：
 **               1- 使代码更直观: 底层配置代码原封不动在can.c和h中, 而应用代码则在bsp_CAN.c和h里;
 **               2- 如果后续的工程，CubeMX没有使能CAN，重新生成后会删掉can.c和h文件，如果把自定义通信代码写在其中，也会随文件被删除，不利于代码日后再次使用;
 **
 ** 【移植说明】  1- 复制本工程的CAN文件夹，到目标工程文件夹中;
 **               2- 添加头文件路径： Keil > Option > C/C++ > Include Paths;
 **               3- 添加C文件到工程: Keil > 左侧工程管理器中双击目标文件夹 > 选择 bsp_CAN.c;
 **               4- 添加文件引用:    #include "bsp_CAN.h＂，即哪个文件中需要使用其函数功能，就在其代码文件顶部添加引用;
 **
 ** 【代码使用】  1- 初始化的代码: CubeMX已生成好了，我们无需修改、增删;
 **               2- 筛选器的配置: 在bsp_CAN.c中，已写了一个示例函数，调用即可：接收所有数据帧，包括标准帧和扩展帧
 **               3- 代码开启CAN1: 调用HAL函数 HAL_CAN_Start(&hcan1) ，本函数已在上述自定义的筛选器函数中调用;
 **               4- 发送，已写好一个自定义的发送函数，在函数中配置报文格式，需要发送时数据，调用函数传入数据即可, 可按需修改;
 **               5- 接收，已写好自定义的中断接收回调函数，在bsp_CAN.c中。在接收到新帧后，数据自动转存到xCAN1结构体，外部只需检查xCAN1.RXNum > 0，表示已接收到新数据，使用方法详见main.c中使用示范。
 **               6- 筛选器的配置，是CAN通信中比较难理解的关键点！列表模式、屏蔽模式，建议刨十来篇网上筛选器配置的文章，透彻地理解。
 **
 ** 【划 重 点】  CAN1有两组引脚，默认的PA11+PA12、PB8+PB9, 注意不要选错了;
 **               很多人在意波特率的计算原理，其实，新手不用太在意其公式和原理，直接上csdn搜文章、抄参数即可。来来去去都是那几个常用波特率了，没必要费时间。
 **               关于发送，很多教程会讲解发送邮箱的使用，新手们，写好帧格式，直接调用发送函数好了，先完成再完善吧，
 **               关于接收，同上，很多教程会花大量的时间讲解两组三级深度FIFO的原理, 新手们，只管在筛选器中决定用哪个FIFO, 收到数据后迅速处理即可。
 **               筛选器的灵活配置，才是CAN通信学习中的重中之重，比较难理解，建议多刨csdn文章，必须要理解透彻！
 **               关于错误帧、自动离线处理、离线唤醒等，这方面文章比较少。当你项目中需要深度使用CAN时，就需要刨寄存器部分描述了，才能有更深理解。
 **
 *******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp_UART.h"            // 串口通信底层驱动文件; 已重写好初始化、收发，调用函数即可使用串口
#include "bsp_CAN.h"             // CAN通信的自定义文件，包插：发送函数、筛选器配置、中断回调函数


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
    MX_CAN1_Init();
    /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    UART1_Init(115200);                                  // 初始化 串口1; 已写好底层，调用h中的函数即可使用; 引脚(PA9 +PA10)、波特率-None-8-1; 如果使用CubeMX配置，请使用前述引脚，但，不要在MX上进行中断及DMA配置，否则冲突
           
    CAN1_FilterInit();                                   // 配置CAN1的筛选器; 配置完后启动CAN1; 为方便调试，本函数配置为：接收所有数据帧，包括标准帧、扩展帧

    if (HAL_CAN_Start(&hcan1) == HAL_OK)                 // 启动CAN，并检查返回值：成功-0、错误-1、忙错误-2、超时-3
    {
        printf("CAN1 启动成功! \r");                     // 启动成功
    }
    else
    {
        printf("CAN1 启动失败! \r");                     // 启动失败
        printf("检查PB8、PB9是否已连接收发器！\r");
        //Error_Handler();                               // 错误处理
    }
    
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */

        /** 每500ms执行一次 **/
        static uint32_t lastTime_500ms = 0;                                             // 定义静态变量，用于记录上次执行这个任务的时间点(单位：毫秒)
        if (HAL_GetTick() - lastTime_500ms >= 500)                                      // 检查当前时间距离上次执行是否已经过了500ms; HAL_GetTick()是获取启动后经过的毫秒数;
        {
            // 1.绿色LED每500ms电平翻转（PC5引脚）
            // 作用：直观可视化系统状态——LED规律闪烁 = 程序正常运行，无卡死
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);                                      // 控制绿色LED引脚电平翻转(PC5); 规律地闪烁LED，方便外部观察系统是否正常运行
            // 2.每500ms发送一帧CAN测试报文，供总线上其他设备验证接收功能
            static uint8_t msgData[30] = { 0x00, 0x02, 0x04, 0x06, 0x08, 0x010 };       // 定义一个简单的数组，用于CAN的发送示范; 这里开辟了30字节的空间，暂时只赋值了前8个字节，后22个字节值为0;
            CAN1_SendData(CAN_ID_EXT, 0x123, msgData, 6);                               // CAN1发送数据(自定义函数)，每500ms一次，以方便外部设备测试接收; CAN每帧最大8字节，这里只发6字节是为了示范：CAN发送的字节数与数组定义的大小，没一毛钱关系。
            // 3. 更新时间戳
            lastTime_500ms = HAL_GetTick();                                             // 更新时间戳：将当前时间设为新基准，确保下一次周期准确从此时开始计算
        }

        /** 检查UART1是否收到数据 (串口助手) **/
        if (UART1_GetRxNum())                                                           // 判断USART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            // 获取数据
            uint8_t *rxData = UART1_GetRxData();                                        // 获取数据的地址
            uint16_t rxNum = UART1_GetRxNum();                                          // 获取数据的字节数
            // 把收到的数据，printf到串口助手观察
            printf("\r<<<<< USART1 收到一帧数据 \r");                                   // 提示
            printf("字节数：%d \r", rxNum);                                             // 显示字节数
            printf("ASCII : %s\r", (char *)rxData);                                     // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                                         // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < rxNum; i++)                                        // 逐个字节输出
                printf("0x%X ", rxData[i]);                                             // 以16进制显示
            printf("\r\r");                                                             // 显示换行
            // 通过CAN1转发，方便测试
            if (rxNum > 8)                                                              // 判断字节数是否>8
                rxNum = 8;                                                              // 如果大于8字节，就只发前8字节
            CAN1_SendData(CAN_ID_EXT, 0x123, rxData, rxNum);                            // 把串口助手发过来的数据，通过CAN1，发送到CAN总线
            UART1_ClearRx();                                                            // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0
        }

        /** 检查CAN1是否收到数据 **/
        if (CAN1_GetRxNum())                                                            // 检查接收字节数是否>0，以判断是否接收到新一帧数据
        {
            static uint16_t  canRxCNT = 1;                                              // 用于计算已接收了多少帧数据; 非必要;
            // 1.获取数据
            uint8_t  rxNum = CAN1_GetRxNum();                                           // 获取最后一帧的字节数
            uint8_t *rxData = CAN1_GetRxData();                                         // 获取最后一帧的数据(地址)
            CAN_RxHeaderTypeDef rxHeader = CAN1_GetRxHeader();                          // 获取最后一帧的头部信息
            // 2.处理数据：把收到的数据，printf到串口助手观察
            printf("\r****** CAN 接收到第 %d 帧报文 ******", canRxCNT++);               // 准备把CAN帧报文，详细地输出到串口软件，方便观察调试
            printf("\r 帧类型：%s",   rxHeader.RTR ? "遥控帧" : "数据帧");              // 帧类型
            printf("\r 帧格式：%s",   rxHeader.IDE ? "扩展帧" : "标准帧");              // 帧格式
            printf("\r 帧 ID ：0x%X", rxHeader.IDE ? rxHeader.ExtId : rxHeader.StdId);  // 帧ID    
            printf("\r 字节数：%d",   rxHeader.DLC);                                    // 字节数
            printf("\r 筛选器匹配序号：%d", rxHeader.FilterMatchIndex);                 // 筛选器匹配序号; 和筛选器编号，是不一样的。大概：从筛选器0开始，每个16位宽筛选器+2, 32位宽+1, 没有被使用的筛选器，默认是16位宽，+2;
            printf("\r 显示数据(16进制)：");                                            // 16进制方式显示数据，方便观察真实数据
            for (uint8_t i = 0; i < rxNum; i++)                                         // 逐个字节显示CAN的数据值
                printf(" 0x%02X ", rxData[i]);                                          // 格式：16进制显示
            printf("\r");                                                               // 换行，以使每帧的输出更清晰
            // 3.判断数据是否包含某些指令
            if (strstr((char *)rxData, "CAN_Test"))                                     // 判断是否测试工具发过来的指定数据
            {
                static char strTemp[8] = "CAN_OK";                                      // 测试工具约定好的返回数据
                CAN1_SendData(CAN_ID_EXT, 0x123, (uint8_t *)strTemp, 8);                // 发送CAN报文
            }
            // 4.清零本帧接收
            CAN1_ClearRx();                                                             // 字节数清0; 每次处理完一帧数据，都需要清0接收标志(其实清0的是接收字节数)，以方便下一轮的判断，避免重复处理同一帧数据;
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
