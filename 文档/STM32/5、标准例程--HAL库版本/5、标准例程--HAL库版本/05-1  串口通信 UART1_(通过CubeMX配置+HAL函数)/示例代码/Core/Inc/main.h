/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* 所有用户代码，必须写在配对的BEGIO与END注释行之间，否则重新生成时会被删除 */

typedef struct                         // 声明一个结构体，方便管理变量
{
    uint16_t  RxNum;                   // 接收字节数; 在中断回调函数里被自动赋值; 只要字节数>0，即为接受到新一帧的数据
    uint8_t   RxData[512];             // 接收到的数据
    uint8_t   RxTemp[512];             // 接收缓存; 注意：这个数组，只是一个缓存，用于DMA逐个字节接收，当接收完一帧后，数据在回调函数中，会自动转存到 ReceivedData[ ] 存放。即：双缓冲，有效减少单缓冲的接收过程新数据覆盖旧数据
} xUATR_TypeDef;

// 声明串口的结构体; 为了方便使用，一次过全声明了
extern xUATR_TypeDef xUART1 ;          // 定义串口1的数据接收结构体，方便管理变量。也可以不用结构体，用单独的变量
extern xUATR_TypeDef xUART2 ;          // 定义串口2的数据接收结构体，方便管理变量。也可以不用结构体，用单独的变量
extern xUATR_TypeDef xUART3 ;          // 定义串口3的数据接收结构体，方便管理变量。也可以不用结构体，用单独的变量
extern xUATR_TypeDef xUART4 ;          // 定义串口4的数据接收结构体，方便管理变量。也可以不用结构体，用单独的变量
extern xUATR_TypeDef xUART5 ;          // 定义串口5的数据接收结构体，方便管理变量。也可以不用结构体，用单独的变量
extern xUATR_TypeDef xUART6 ;          // 定义串口6的数据接收结构体，方便管理变量。也可以不用结构体，用单独的变量


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_1_Pin GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOA
#define KEY_1_EXTI_IRQn EXTI0_IRQn
#define KEY_2_Pin GPIO_PIN_1
#define KEY_2_GPIO_Port GPIOA
#define KEY_2_EXTI_IRQn EXTI1_IRQn
#define KEY_3_Pin GPIO_PIN_4
#define KEY_3_GPIO_Port GPIOA
#define KEY_3_EXTI_IRQn EXTI4_IRQn
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_2
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
