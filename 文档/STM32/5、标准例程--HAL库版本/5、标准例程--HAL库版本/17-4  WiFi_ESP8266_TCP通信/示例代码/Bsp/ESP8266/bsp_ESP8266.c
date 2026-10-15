/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【最后版本】  2024-07-08-02
 ** 【淘    宝】  https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【文件名称】  bsp_ESP8266.c
 **
 ** 【文件功能】  对ESP8266常用AT指令组合 、功能，进行函数封装
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库/标准库
 **
************************************************************************************************************************************/
#include "bsp_ESP8266.h"
#include "stdlib.h"           // C语言的标准库函数，包含atoi等函数 


#define   sendData       UART3_SendData
#define   sendString     UART3_SendString
#define   sendAT         UART3_SendAT
#define   getRxNum       UART3_GetRxNum
#define   getRxData      UART3_GetRxData
#define   clearRX        UART3_ClearRx   




/******************************************************************************
 * 函  数： delay_ms
 * 功  能： ms 延时函数
 * 备  注： 1、系统时钟168MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
            3、编译优化级别：Level 3(-O3)
 * 参  数： uint32_t  ms  毫秒值
 * 返  回： 无
 ******************************************************************************/
static volatile uint32_t ulTimesMS;    // 使用volatile声明，防止变量被编译器优化
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;                   // 操作外部变量，防止空循环被编译器优化掉
}




//////////////////////////////////////////////////////////////  常用AT指令封装   ///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * 函    数： ESP8266_AT
 * 功    能： 发送AT命令：AT
 *            可用于测试线路连接是否正常，和测试代码收发机制的正确性
 * 参    数： 无
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT(void)
{

    if ( sendAT("AT\r\n",   "OK", 1500) )
    {
        printf("模块连接测试:  成功\r\n");                         // AT指令测试成功
        return 1;
    }
    else
    {
        printf("模块连接测试:  失败！    ");                       // AT指令测试失败
        printf("检查：ESP8266与USART的电路连接、跳线帽！！\r\n");  // 输出错误建议
    }

    return 0;                                                      // 返回, 初始化状态，0_失败，1_正常
}



/******************************************************************************
 * 函    数： ESP8266_AT_RESTORE
 * 功    能： 发送AT指令：AT+RESTORE
 *            恢复模块的出厂设置
 *            可用于清除芯片中已保存的wifi名称、密码
 * 参    数： 无
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT_RESTORE(void)
{
    uint8_t status = 0 ;
    status = sendAT("AT+RESTORE\r\n", "ready", 3000);                         // 恢复模块的出厂设置
    status ? printf("恢复出厂设置:  成功\r\n") : printf("恢复出厂设置:  失败\r\n");   // 输出提示信息
    return status;                                                                    // 返回：0-失败、1成功
}



/******************************************************************************
 * 函    数： ESP8266_AT_RST
 * 功    能： 发送AT指令: AT+RST
 *            复位、重启
 * 参    数： 无
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT_RST(void)
{
    uint8_t status = 0 ;
    status = sendAT("AT+RST\r\n", "ready", 3000);                            // 恢复模块的出厂设置
    status ? printf("重启 ESP8266:  成功\r\n") : printf("重启 ESP8266:  失败\r\n");  // 输出提示信息
    return status;                                                                   // 返回：0-失败、1成功
}



/******************************************************************************
 * 函    数： ESP8266_AT_CWMODE
 * 功    能： 设置ESP8266的工作模式
 * 参    数： 1-STA模式
 *            2-AP模式
 *            3-STA+AP
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT_CWMODE(uint8_t mode)
{
    uint8_t status = 0 ;
    char str[100] = {0};

    if (mode > 3)
    {
        printf("配置工作模式:  失败！参数范围：1、2、3; 参数解释：1-STA, 2-AP, 3-STA+AP\r\n");
    }

    sprintf(str, "AT+CWMODE=%d\r\n", mode);                                                // 工作模式：1-STA, 2-AP, 3-STA+AP
    status = sendAT(str,    "OK", 3000)  ;                                         // 发送给8266，并判断指令是否执行成功
    if (mode == 1)
        status ?  printf("配置 STA模式:  成功\r\n") : printf("配置 STA模式:  失败\r\n");   // 输出提示信息
    if (mode == 2)
        status ?  printf("配置 AP 模式:  成功\r\n") : printf("配置 AP 模式:  失败\r\n");   // 输出提示信息
    if (mode == 3)
        status ?  printf("配置 STA+AP :  成功\r\n") : printf("配置 STA+AP :  失败\r\n");   // 输出提示信息
    return status;                                                                         // 返回：0-失败、1成功
}



/******************************************************************************
 * 函    数： ESP8266_AT_CIPMUX
 * 功    能： 设置 多连接
 * 备    注： 透传必须AT+CIPMUX=0;
 * 参    数： uint8_t value   0_单连接，1_多连接
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t  ESP8266_AT_CIPMUX(uint8_t value)
{
    uint8_t flag = 0;
    char strTemp[50];
    sprintf(strTemp, "AT+CIPMUX=%d\r\n", value);
    sendAT(strTemp, "OK", 3000)  ? (flag = 1) : (flag = 0) ;
    flag ? printf("设置连接方式:  成功\r\n") : printf("设置连接方式: 失败\r\n");   // 多 连 接: 0_单连接，1_多连接
    return flag;
}



/******************************************************************************
 * 函    数： ESP8266_AT_CIPMODE
 * 功    能： 设置 传输方式
 * 参    数： uint8_t value   0_普通传输，1_透明传输
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t   ESP8266_AT_CIPMODE(uint8_t value)
{
    uint8_t flag = 0;
    char strTemp[50];
    sprintf(strTemp, "AT+CIPMODE=%d\r\n", value);
    sendAT(strTemp, "OK", 3000)  ? (flag = 1) : (flag = 0) ;
    flag ? printf("设置传输方式:  成功\r\n") : printf("设置传输方式: 失败\r\n");   // 传输方式，0_普通传输，1_透明传输
    return flag;
}



/******************************************************************************
 * 函    数： ESP8266_AT_SetPassThrough
 * 功    能： 设置 透传
 * 参    数： 0-关闭、1-开启
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT_SetPassThrough(uint8_t value)
{
    if (value == 1)
    {
        uint8_t status = 1;
        status &= sendAT("AT+CIPMODE=1\r\n", "OK", 3000);                         // 传输方式，0_普通传输，1_透明传输
        status &= sendAT("AT+CIPSEND\r\n",    ">", 3000);                         // 数据传输，当已设置透传时，无需参数
        status ? printf("打开透明传输:  成功\r\n") : printf("打开透明传输:  失败\r\n");
        return status;
    }
    sendString("+++");
    printf("已关闭透明传输！\r\n");
    return 1;
}








/******************************************************************************
 * 函  数： ESP8266_JoinAP
 * 功  能： 连接AP
 * 参  数： char* SSID       WiFi名称
 *          char* passWord   WiFi密码
 * 返  回： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_JoinAP(char *SSID, char *passWord)
{
    char strTemp[100];
    // 把ESP8266重新配置成SAT模式
    sendAT("AT+RESTORE\r\n", "ready", 3000)  ? printf("恢复出厂设置:  成功\r\n") : printf("恢复出厂设置:  失败\r\n");   // 恢复模块的出厂设置
    sendAT("AT+CWMODE=1\r\n",   "OK", 3000)  ? printf("配置 STA模式:  成功\r\n") : printf("配置 STA模式:  失败\r\n");   // 工作模式：1_STA, 2_AP, 3_STA+AP
    sendAT("AT+RST\r\n",     "ready", 3000)  ? printf("重启 ESP8266:  成功\r\n") : printf("重启 ESP8266:  失败\r\n");   // 重启模块: 设置工作模式后，需重启才生效
    //sendAT("AT+CIPMUX=0\r\n", "OK", 3000)  ? printf("单连接模式  :  成功\r\n") : printf("单连接模式  :  失败\r\n");   // 多 连 接: 0_单连接，1_多连接
    // 连接至指定WiFi热点
    printf("准备连接WIFI:  %s, %s\r\n", SSID, passWord);
    sprintf(strTemp, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, passWord);
    printf("开始连接   ... ");
    sendAT(strTemp,        "OK\r\n", 6000)  ? printf("成功\r\n") : printf("失败\r\n");
    // 检测连接状态
    printf("查询连接状态: ");
    if (sendAT("AT+CIPSTATUS\r\n", "OK", 5000))
    {
        if (strstr((char *)getRxData(), "STATUS:2"))
        {
            printf(" 成功 \r\n");
            return 1;
        }
    }
    printf(" 失败！\r\n");  // 未能获得连接状态
    return 0;
}



/******************************************************************************
 * 函  数： ESP8266_CreationAP
 * 功  能： 把模块设置成AP模式, 并建立热点网络
 * 参  数： char* SSID       WiFi名称
 *          char* passWord   WiFi密码
 * 返  回： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_CreationAP(char *SSID, char *passWord)
{
    char strTemp[100];

    printf("准备建立SSID：%s, %s\r\n", SSID, passWord);
    // 把ESP8266重新配置成AP模式
    sendAT("AT+RESTORE\r\n", "ready", 1000)  ? printf("恢复出厂设置: 成功\r\n") : printf("恢复出厂设置: 失败\r\n");   // 恢复模块的出厂设置
    sendAT("AT+CWMODE=2\r\n",   "OK", 3000)  ? printf("配置为AP模式: 成功\r\n") : printf("配置 STA模式: 失败\r\n");   // 工作模式：1_STA, 2_AP, 3_STA+AP
    sendAT("AT+RST\r\n",     "ready", 3000)  ? printf("重启 ESP8266: 成功\r\n") : printf("重启 ESP8266: 失败\r\n");   // 重启模块: 设置工作模式后，需重启才生效
    // 配置WiFi热点
    sprintf(strTemp, "AT+CWSAP=\"%s\",\"%s\",11,0\r\n", SSID, passWord);
    printf("开始建立AP... ");
    if (sendAT(strTemp, "OK\r\n", 10000))
    {
        printf("成功\r\n");
        return 1;
    }
    else
    {
        printf("失败\r\n");
        return 0;
    }
}



/******************************************************************************
 * 函  数： ESP8266_ConnectTCP
 * 功  能： 建立TCP连接
 * 参  数： char *IP        IP地址
 *          uint16_t port   端口
 * 返  回： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_ConnectTCP(char *IP, uint16_t port)
{
    char strTemp[100];
    uint8_t status = 0;

    printf("建立TCP通信... ");
    sprintf(strTemp, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", IP, port);
    status = sendAT(strTemp, "OK\r\n", 5000);
    if (status)
        printf("成功\r\n");
    else
        printf("失败, 请检查APP：IP地址、端口号、是否已断开旧的连接状态\r\n");
    return status;
}



/******************************************************************************
 * 函  数： ESP8266_GetIP
 * 功  能： 获取公网、局域网IP地址
 * 备  注： 获得的数据，共8个字节
 *          公网IP    [0]、[1]、[2]、[3]
 *          局域网IP  [4]、[5]、[6]、[7]
 *          注意：使用本函数时，服务器返回数据量较大，需要把UART3_RX_BUF_SIZE的值设置为>=8192
 * 参  数： 无
 * 返  回:  uint8_t*： 0_获取失败、非0_数据地址
 ******************************************************************************/
uint8_t *ESP8266_GetIP(void)
{
    static uint8_t aData[8];                               // 用于保存获得的数据
    static char strTem[2048]  = {0};                       // 存放IP字符串
    char *strData = NULL;                                  // 数据地址
    char *addr1 = NULL;                                    // 目标数据前的字符串样本地址
    char *addr2 = NULL;                                    // 目标数据后的字符串样本地址
                                                          
    /** 获取模块在局域网中的IP地址数据 **/                
    if (0 == sendAT("AT+CIFSR\r\n",  "OK", 1000))         
    {                                                     
        printf("获取局域网IP:  失败!\r\n");                // 查询本地IP地址、MAC地址
        return 0;
    }
    /** 提取局域网IP的字符串 **/
    strData = (char *)getRxData();                         // 取得最后一条AT指令发送后返回的数据
    addr1 = strstr(strData, "+CIFSR:STAIP") + 14;          // 从原始数据中，获取IP地址字符串的开始地址; +14是因为要减去这段字符串的长度
    addr2 = strstr(strData, "+CIFSR:STAMAC");              // 从原始数据中，获取IP地址字符串的结束地址
    memcpy(strTem, addr1, addr2 - addr1 - 3);              // 提取IP地址段的字符串; -3是因为IP后面有一个双引号和\r\n
    aData[4] = atoi(strtok(strTem, "."));                  // 转换第1个字节，即IP地址的第1个字段：xx.--.--.--
    aData[5] = atoi(strtok(NULL, "."));                    // 转换第2个字节，即IP地址的第2个字段: --.xx.--.--
    aData[6] = atoi(strtok(NULL, "."));                    // 转换第3个字节，即IP地址的第3个字段: --.--.xx.--
    aData[7] = atoi(strtok(NULL, "."));                    // 转换第4个字节，即IP地址的第4个字段: --.--.--.xx

    /** 连接服务器 **/
    if (0 == sendAT("AT+CIPMUX=0\r\n",   "OK", 1000))      // 多连接：0_关闭、1_多连接; 有些模式必须在单连接模式下运行，如透明传输等
    {
        printf("设置单路连接： 失败!\r\n");
        return 0;
    }
    if (0 == sendAT("AT+CIPSTART=\"TCP\",\"quan.suning.com\",80\r\n", "OK", 5000))  // 连接服务器,指TCP或UDP的连接
    {
        printf("连接到服务器:  失败!\r\n");
        return 0;
    }
    /** 获取公网IP数据 **/
    if (0 == sendAT("AT+CIPMODE=1\r\n",  "OK", 1000))      // 透明传输：0_关闭、1_打开
    {
        printf("打开透传模式:  失败!\r\n");
        return 0;
    }
    if (0 == sendAT("AT+CIPSEND\r\n",    "OK", 1000))      // 传输数据
    {
        printf("准备获取数据:  失败!\r\n");
        return 0;
    }
    if (0 == sendAT("GET http://quan.suning.com/getSysTime.do HTTP/1.1\r\nHost: quan.suning.com\r\n\r\n",   "x-request-ip", 3000))  // 获取数据
    {
        printf("获取公网地址:  失败!\r\n");
        return 0;
    }
    /** 提取公网IP的字符串 **/
    strData = (char *)getRxData();                         // 取得最后一条AT指令发送后返回的数据
    addr1 = strstr(strData, "x-request-ip");               // 从原始数据中，获取IP地址前的字样
    addr2 = strstr(strData, "x-tt-trace-tag");             // 从原始数据中，获取IP地址后的字样
    memset(strTem, 0, sizeof(strTem));                     // 清0数组的旧数据
    memcpy(strTem, addr1 + 14, addr2 - addr1 - 14);        // 提取IP地址段的字符串
    /** 处理IP字符串 **/
    aData[0] = atoi(strtok(strTem, "."));                  // 转换第1个字节，即IP地址的第1个字段：xx.--.--.--
    aData[1] = atoi(strtok(NULL, "."));                    // 转换第2个字节，即IP地址的第2个字段: --.xx.--.--
    aData[2] = atoi(strtok(NULL, "."));                    // 转换第3个字节，即IP地址的第3个字段: --.--.xx.--
    aData[3] = atoi(strtok(NULL, "."));                    // 转换第4个字节，即IP地址的第4个字段: --.--.--.xx
    /** 断开服务连接 **/
    sendString("+++");                                     // 退出透传
    delay_ms(50);                                          // 稍作延时，因为要等"+++"执行完毕
    if (0 == sendAT("AT+CIPCLOSE\r\n",   "OK", 3000))      // 断开服务器连接, 即断开TCP、UDP
    {
        printf("断开 TCP连接:  失败!\r\n");
        return 0;
    }

    return aData;                                          // 获取成功，返回: uint8_t*：数据地址
}



/******************************************************************************
 * 函  数： ESP8266_GetTime
 * 功  能： 获取当前网络时间
 * 备  注： 获得的数据，共6个字节
 *          [0]年(即：2000+[0]年)、[1]月、[2]日、[3]时、[4]分、[5]秒、[6]星期(1~周一、7~周日)
 * 参  数： 无
 * 返  回:  uint8_t*： 0_获取失败、非0_数据地址
 ******************************************************************************/
uint8_t *ESP8266_GetTime(void)
{
    static uint8_t pDate[7] = {0};                // 为了能返回局部变量，用static指定了。
    uint8_t status = 0;                           // 状态值
    uint8_t timeOut = 0;                          // 查询尝试次数
    char *timeStr;                                // 缓存
    char monthStr[4] = {0};                       // 月 字符串
    char weekStr[4] = {0};                        // 星期 字符串
    int year, month, day, hour, min, sec, week;   // 年月日时分秒星期
    int parsed;                                   // sscanf返回值
    
    /* 1.配置SNTP服务器*/
    status = sendAT("AT+CIPSNTPCFG=1,8,\"202.120.2.101\",\"time.nist.gov\",\"ntp.aliyun.com\"\r\n", "OK", 3000); // 可同时指定多个SNTP服务器，模块会依次尝试
    if (status == 0)
    {
        printf("SNTP配置失败，退出本函数! \r\n");
        return NULL;
    }
    
    /* 2.同步时间 */
    printf("时间同步中 ... \r\n");
    timeOut = 10;                                               // 设定10秒超时
    while ( timeOut-- )
    {
        HAL_Delay(1000);                                        // 每次查询间隔1秒
        if (sendAT("AT+CIPSNTPTIME?\r\n", "OK", 3000))          // 发送查询指令 AT+CIPSNTPTIME?（带问号，仅查询缓存）
        {     
            // 获取返回数据            
            timeStr = (char*)getRxData();                       // 获取返回的数据指针
            // 返回数据错误
            if(strstr(timeStr, "+CIPSNTPTIME:") == NULL)
            {
                //printf("同步时间返回异常，未找到时间前缀\r\n");
                continue;
            }
            // 含有1970字样，即为未同步
            if (strstr(timeStr, "1970") != NULL )               // 检查是否仍为1970的默认值, 即为未能更新时间
            {
                continue;
            }
            // 解析时间字符串
            timeStr += strlen("+CIPSNTPTIME:");                 // 跳过前缀"+CIPSNTPTIME:"，指向实际时间字符串（如 "Wed Mar 11 11:10:00 2026"）           
            parsed = sscanf(timeStr, "%3s %3s %d %d:%d:%d %d", weekStr, monthStr, &day, &hour, &min, &sec, &year);   // 提示数据; 格式：星期 月份 日 时:分:秒 年
            if (parsed != 7)
            {
                printf("时间格式解析失败，期望7项，实际解析%d项\r\n", parsed);
                continue;
            }           
            // 月份字符串转数字
            if (strcmp(monthStr, "Jan") == 0) month = 1;
            else if (strcmp(monthStr, "Feb") == 0) month = 2;
            else if (strcmp(monthStr, "Mar") == 0) month = 3;
            else if (strcmp(monthStr, "Apr") == 0) month = 4;
            else if (strcmp(monthStr, "May") == 0) month = 5;
            else if (strcmp(monthStr, "Jun") == 0) month = 6;
            else if (strcmp(monthStr, "Jul") == 0) month = 7;
            else if (strcmp(monthStr, "Aug") == 0) month = 8;
            else if (strcmp(monthStr, "Sep") == 0) month = 9;
            else if (strcmp(monthStr, "Oct") == 0) month = 10;
            else if (strcmp(monthStr, "Nov") == 0) month = 11;
            else if (strcmp(monthStr, "Dec") == 0) month = 12;
            else 
            {
                printf("未知月份：%s\r\n", monthStr);
                continue;
            }
            // 星期字符串转数字（周一=1 ... 周日=7）
            if (strcmp(weekStr, "Mon") == 0) week = 1;
            else if (strcmp(weekStr, "Tue") == 0) week = 2;
            else if (strcmp(weekStr, "Wed") == 0) week = 3;
            else if (strcmp(weekStr, "Thu") == 0) week = 4;
            else if (strcmp(weekStr, "Fri") == 0) week = 5;
            else if (strcmp(weekStr, "Sat") == 0) week = 6;
            else if (strcmp(weekStr, "Sun") == 0) week = 7;
            else
            {
                printf("未知星期：%s\r\n", weekStr);
                continue;
            }
            // 填充pDate
            pDate[0] = year - 2000;          // 年（基于2000）
            pDate[1] = month;                // 月
            pDate[2] = day;                  // 日 
            pDate[3] = hour;                 // 时
            pDate[4] = min;                  // 分
            pDate[5] = sec;                  // 秒
            pDate[6] = week;                 // 星期; 1-周一、2-周二、7-周日
            printf("时间同步成功!  \r\n");
            //printf("时间同步成功：%d年%02d月%02d日 %02d:%02d:%02d \r\n", year, month, day, hour, min, sec);
            return pDate;                    // 成功立即返回                  
        }            
    }    
    /* 3.获取失败 */
    printf("获取时间失败: 超时!\r\n");
    return NULL;
}



