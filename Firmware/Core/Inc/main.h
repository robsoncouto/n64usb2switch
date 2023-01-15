/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "n64.h"
//#include "stm32412g_discovery.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define LEDx_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()

#define LED2_PIN GPIO_PIN_13
#define LED3_PIN GPIO_PIN_13
#define LEDx_GPIO_PORT GPIOC


#define KEY_BUTTON0_PIN                       GPIO_PIN_0
//#define KEY_BUTTON1_PIN                       GPIO_PIN_3
#define KEY_BUTTON_GPIO_PORT                 GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn                 EXTI4_IRQn
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint32_t PCLK1_clock(void);
void Error_Handler(void);
#endif /* __MAIN_H */
