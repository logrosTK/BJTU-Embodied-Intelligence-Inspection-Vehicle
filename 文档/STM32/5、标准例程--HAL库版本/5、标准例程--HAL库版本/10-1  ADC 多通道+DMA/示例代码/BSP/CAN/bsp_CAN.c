#include "bsp_CAN.h"
#include "stdio.h"
#include "string.h"






// 定义三个变量，用于转存最后一帧接收的内容
static uint8_t canRxNum = 0;                    // 最后一帧的接收字节数
static uint8_t canRxData[9] = {0};              // 最后一帧的接收的数据; CAN一帧数据最大8字节，数组开辟9个字节，是为了适配以字符串输出调试信息，最后的1字节是0，即'\0'，是字符串结束符;
static CAN_RxHeaderTypeDef canRxHeader = {0};   // 最后一帧的头部信息
/** CAN_RxHeaderTypeDef 是HAL库的CAN接收头部信息结构体，成员如下：
    uint32_t  StdId              标准帧的ID, 11位, 值范围：0x00~0x7FF
    uint32_t  ExtId              扩展帧的ID, 29位, 值范围：0x00~0x1FFFFFF
    uint32_t  DLC                接收到的字节数, 单位：byte, 值范围：0~8
    uint32_t  FilterMatchIndex   筛选器编号, 值范围：0x00~0xFF
    uint32_t  IDE                帧格式; 0_标准帧、4_扩展帧
    uint32_t  RTR                帧类型; 0_数据帧、2_遥控帧
    uint32_t  Timestamp          使用时间触发模式时，时间戳，值范围：0x00~0xFFFF
**/



/******************************************************************************
 * 函  数： CAN1_FilterInit
 * 功  能： CAN1筛选器初始化
 * 备  注： 为方便调试，本函数配置为：接收所有数据帧，包括标准帧、扩展帧
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void CAN1_FilterInit(void)
{
    // 筛选器配置示例1: 接收所有帧数据 (标准帧、扩展帧)
#if 1
    CAN_FilterTypeDef sFilterConfig = {0};                                                                   // 过滤器配置结构体
    sFilterConfig.FilterBank = 0;                                                                            // 筛选器组编号
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                                                        // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;                                                       // 32位模式
    sFilterConfig.FilterMaskIdHigh     = 0x0000;                                                             // 掩码高16位; 位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterMaskIdLow      = 0x0000;                                                             // 掩码低16位；位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                                                       // 分配到 FIFO 0
    sFilterConfig.FilterActivation = ENABLE;                                                                 // 使能筛选器
    sFilterConfig.SlaveStartFilterBank = 14;                                                                 // CAN1、CAN2共用28个筛选器(0~27)，此值设置CAN2的筛选器从哪个编号开始; 两个特别的值：当值=0时，不会为CAN1分配任何筛选器，当值=28时，CAN可以使用CAN1的所有筛选器。
#endif

    // 筛选器配置示例2: 只接收标准数据帧(指定范围)
#if 0                                                                                                        // 标准帧ID验证码：0x000~0x7FF; ID验证码在寄存器的21~31位
    CAN_FilterTypeDef sFilterConfig = {0};                                                                   // 过滤器配置结构体
    sFilterConfig.FilterBank = 0;                                                                            // 筛选器组编号
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                                                        // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;                                                       // 32位模式
    sFilterConfig.FilterIdHigh         = (((uint32_t)0x123 << 21) & 0xFFFF0000) >> 16;                       // ID(验证码)的高16位
    sFilterConfig.FilterIdLow          = (CAN_ID_STD | CAN_RTR_DATA) & 0xFFFF;                               // ID(验证码)的低16位
    sFilterConfig.FilterMaskIdHigh     = (((uint32_t)0xFFE0 << 21) & 0xFFFF0000) >> 16;;                     // 掩码高16位; 位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterMaskIdLow      = (((uint32_t)0x04 | 0x02) & 0xFFFF);                                 // 掩码低16位; 位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                                                       // 分配到 FIFO 0
    sFilterConfig.FilterActivation = ENABLE;                                                                 // 使能筛选器
    sFilterConfig.SlaveStartFilterBank = 14;                                                                 // CAN1、CAN2共用28个筛选器(0~27)，此值设置CAN2的筛选器从哪个编号开始; 两个特别的值：当值=0时，不会为CAN1分配任何筛选器，当值=28时，CAN可以使用CAN1的所有筛选器。
#endif

    // 筛选器配置示例3: 只接收扩展数据帧(指定范围)
#if 0                                                                                                        // 扩展帧ID验证码：0x00000000~0x1FFFFFFF; 
    CAN_FilterTypeDef sFilterConfig = {0};                                                                   // 过滤器配置结构体
    sFilterConfig.FilterBank = 0;                                                                            // 筛选器组0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                                                        // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;                                                       // 32位模式
    sFilterConfig.FilterIdHigh         = (((uint32_t)0x02003FF0 << 3) & 0xFFFF0000) >> 16;                   // ID(验证码)的高16位
    sFilterConfig.FilterIdLow          = (((uint32_t)0x02003FF0 << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF; // ID(验证码)的低16位
    sFilterConfig.FilterMaskIdHigh     = (((uint32_t)0xFF00FFFF << 3) & 0xFFFF0000) >> 16;;                  // 掩码高16位; 位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterMaskIdLow      = (((uint32_t)0xFF00FFFF << 3 | 0x04 | 0x02) & 0xFFFF);               // 掩码低16位; 位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                                                       // 分配到 FIFO 0
    sFilterConfig.FilterActivation = ENABLE;                                                                 // 使能筛选器
    sFilterConfig.SlaveStartFilterBank = 14;                                                                 // CAN1、CAN2共用28个筛选器(0~27)，此值设置CAN2的筛选器从哪个编号开始; 两个特别的值：当值=0时，不会为CAN1分配任何筛选器，当值=28时，CAN可以使用CAN1的所有筛选器。
#endif

    // 初始化筛选器
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)                                              // 初始化筛选器，并检查返回值：成功-0、错误-1、忙错误-2、超时-3
    {
        printf("CAN 筛选器初始化: 失败！ \r");
        Error_Handler();                                                                                     // 错误处理
    }
    // 开启CAN接收
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)                         // 开启CAN接收，并检查返回值：成功-0、错误-1、忙错误-2、超时-3
    {
        printf("CAN 打开接收: 失败！ \r");
        Error_Handler();                                                                                     // 错误处理
    }
    // 开启CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK)                                                                     // 开启CAN，并检查返回值：成功-0、错误-1、忙错误-2、超时-3
    {
        printf("CAN 使能...              失败! \r");
        printf("检查PB8、PB9是否已连接收发器, 使总线维持隐性电平！\r");
        Error_Handler();                                                                                     // 错误处理
    }
}



/******************************************************************************
 * 函  数： CAN1_SendData
 * 功  能： CAN发送数据函数
 * 参  数： uint32_t type      帧格式; 参数：CAN_ID_EXT、CAN_ID_STD
 *          uint32_t ID        帧ID
 *          uint8_t* msgData   需发送数据的地址
 *          uint8_t  len       发送的字节数; 最大值：8
 * 返回值： 发送状态; 成功-0、错误-1、忙错误-2、超时-3
 * 备  注： CAN_TxHeaderTypeDef 是HAL库的CAN发送帧头部信息结构体，成员列表如下：
            uint32_t  StdId                标准帧的ID, 11位, 值范围：0x000~0x7FF
            uint32_t  ExtId                扩展帧的ID, 29位, 值范围：0x00000000~0x1FFFFFFF
            uint32_t  DLC                  接收到的字节数, 单位：byte, 值范围：0~8
            uint32_t  IDE                  帧格式; 0_标准帧、4_扩展帧
            uint32_t  RTR                  帧类型; 0_数据帧、2_遥控帧
            uint32_t  TransmitGlobalTime   使能时间戳，添加到Data[6]和Data[7]
 ******************************************************************************/
uint8_t CAN1_SendData(uint32_t type, uint32_t ID,  uint8_t *msgData, uint8_t num)
{
    // 定义两个变量
    static CAN_TxHeaderTypeDef canTxHeader = {0};                               // CAN发送消息的结构体，定义了发送报文的关键参数
    static uint32_t txMailbox = 0;                                              // 用于记录发送成功时所用的邮箱编号：0~2; 被发送函数HAL_CAN_AddTxMessage()赋值
    // 限制数据的字节数
    if (num > 8)                                                                // 判断字节是否超过8字节
        num = 8;                                                                // 如果超过8字节，只发送前8个字节
    // 配置帧信息
    if (type == CAN_ID_EXT)                                                     // 扩展帧
    {
        canTxHeader.ExtId = ID ;                                                // 扩展帧ID; 值有效范围：0x00000000~0x1FFFFFFF
        canTxHeader.IDE   = CAN_ID_EXT;                                         // 帧格式：扩展帧(CAN_ID_EXT); 此值是HAL库文件stm32f4xx_hal.h里的宏定义
    }
    else                                                                        // 标准帧
    {
        canTxHeader.StdId = ID;                                                 // 帧ID; 值有效范围：0x000~0x7FF
        canTxHeader.IDE   = CAN_ID_STD;                                         // 帧格式: 标准帧(CAN_ID_STD); 此值是HAL库文件stm32f4xx_hal.h里的宏定义
    }
    canTxHeader.RTR   = CAN_RTR_DATA;                                           // 数据帧
    canTxHeader.DLC   = num;                                                    // 数据字节数
    canTxHeader.TransmitGlobalTime = DISABLE;                                   // 使能时间戳添加到最后两个字节：Data[6]、Data[7]
    // 等待发送邮箱空闲
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)                        // 每次发送前，要等待至有发送邮箱空闲。共有3个发送邮箱。如果返回值为0，即没有发送邮箱空闲，继续等
    {
        uint8_t times = 0;                                                      // 注意：如果发送超时失败，总线一直在忙的可能性极低，更大的可能是，CAN没有连接到收发器
        HAL_Delay(1);
        if (times++ >= 5)
        {
            times = 0;
            printf("\r发送失败：等待超时！检查CAN1的线路是否已连接到收发器\r");
            return 3;
        }
    }
    // 发送，并返回发送状态：成功-0、错误-1、忙错误-2、超时-3
    return HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, msgData, &txMailbox);     // 返回发送状态; 成功-0、错误-1、忙错误-2、超时-3
}



/******************************************************************************
 * 函  数： HAL_CAN_RxFifo0MsgPendingCallback
 * 功  能： CAN接收中断的回调函数
 *          注意：只有在筛选器配置中接收规则使用的是FiFO0，接收到新一帧数据时才会触发此回调函数
 *          如果配置时使用的是FIFO1, 触发的就是HAL_CAN_RxFifo1MsgPendingCallback()，编写它的回调函数即可
 * 参  数： CAN_HandleTypeDef  *hcan
 * 返回值： 无
 ******************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    memset(canRxData, 0, 9);                                            // 数组清0; 存放接收有效数据的数组; CAN一帧数据最大有效负载8字节，数组中开辟9个字节，是为了适配以字符串输出调试信息，最后的1字节0='\0'，是字符串结束符;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, canRxData);  // 把收到的数据存放到结构体备用; 参数：帧信息：RxHeader，数据：RxData
    canRxNum = canRxHeader.DLC;                                         // 接收字节数、标志位; 外部判断此值接收字节数是否大于0，以判断是否接收到新一帧数据
}



/******************************************************************************
 * 函  数： CAN1_GetRxNum
 * 功  能： 获取最后一帧接收的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint8_t CAN1_GetRxNum(void)
{
    return canRxNum ;    // 返回最后一帧的接收字节数; 在接收到数据时，硬件触发HAL_CAN_RxFifo0MsgPendingCallback()函数，此值在函数内被重新赋值;
}



/******************************************************************************
 * 函  数： CAN1_GetRxData
 * 功  能： 获取最后一帧接收的数据(地址）
 * 参  数： 无
 * 返回值： 数据地址
 ******************************************************************************/
uint8_t *CAN1_GetRxData(void)
{
    return canRxData ;    // 返回最后一帧的接收数据(地址); 在接收到数据时，硬件触发HAL_CAN_RxFifo0MsgPendingCallback()函数，此数值在函数内被重新赋值;
}



/******************************************************************************
 * 函  数： CAN1_GetRxHeader
 * 功  能： 获取最后一帧的头部信息
 * 参  数： 无
 * 返回值： CAN_RxHeaderTypeDef 头部信息
 * 备  注： CAN_RxHeaderTypeDef 是HAL库的CAN接收头部信息结构体，成员：
 *          uint32_t  StdId              标准帧的ID, 11位, 值范围：0x00~0x7FF
 *          uint32_t  ExtId              扩展帧的ID, 29位, 值范围：0x00~0x1FFFFFF
 *          uint32_t  DLC                接收到的字节数, 单位：byte, 值范围：0~8
 *          uint32_t  FilterMatchIndex   筛选器编号, 值范围：0x00~0xFF
 *          uint32_t  IDE                帧格式; 0_标准帧、4_扩展帧
 *          uint32_t  RTR                帧类型; 0_数据帧、2_遥控帧
 *          uint32_t  Timestamp          使用时间触发模式时，时间戳，值范围：0x00~0xFFFF
 ******************************************************************************/
CAN_RxHeaderTypeDef  CAN1_GetRxHeader(void)
{
    return canRxHeader;    // 返回最后一帧的头部信息; 在接收到数据时，硬件触发HAL_CAN_RxFifo0MsgPendingCallback()函数，此结构体在函数内被重新赋值;
}



/******************************************************************************
 * 函  数： CAN1_ClearRx
 * 功  能： 清0最后一帧的接收
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void CAN1_ClearRx(void)
{
    canRxNum = 0 ;    // 清0最后一帧的接收字节数; 它在外部文件中用于判断是否接收到新一帧数据; 在接收到数据时，硬件触发HAL_CAN_RxFifo0MsgPendingCallback()函数，此值在函数内被重新赋值;
}



// 文件结尾，需要保留至少1空行
