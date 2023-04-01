/**
  ******************************************************************************
  * @file    USB_Device/HID_Standalone/Src/stm32f4xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------ */
#include "main.h"
#include "stm32f4xx_it.h"

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
#define CURSOR_STEP     5
/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
extern PCD_HandleTypeDef hpcd;
uint8_t HID_Buffer[8];
extern USBD_HandleTypeDef USBD_Device;
extern TIM_HandleTypeDef htim9;
/* Private function prototypes ----------------------------------------------- */
/* Private functions --------------------------------------------------------- */
static void GetReportData(uint8_t * pbuf);
/******************************************************************************/
/* Cortex-M4 Processor Exceptions Handlers */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  static __IO uint32_t hid_report_counter = 0;
  HAL_IncTick();

  /* check Joystick state every polling interval (10ms) */
  if (hid_report_counter++ == USBD_HID_GetPollingInterval(&USBD_Device))
  {
    GetReportData(HID_Buffer);

    USBD_HID_SendReport(&USBD_Device, HID_Buffer, 8);
    hid_report_counter = 0;
  }



}

/******************************************************************************/
/* STM32F4xx Peripherals Interrupt Handlers */
/* Add here the Interrupt Handler for the used peripheral(s) (PPP), for the */
/* available peripheral interrupt handler's name please refer to the startup */
/* file (startup_stm32f4xx.s).  */
/******************************************************************************/

/**
  * @brief  This function handles USB-On-The-Go FS or HS global interrupt request.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_FS
void OTG_FS_IRQHandler(void)
#else
void OTG_HS_IRQHandler(void)
#endif
{
  HAL_PCD_IRQHandler(&hpcd);
}

/**
  * @brief  This function handles USB OTG FS or HS Wakeup IRQ Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_FS
void OTG_FS_WKUP_IRQHandler(void)
#else
void OTG_HS_WKUP_IRQHandler(void)
#endif
{
  if ((&hpcd)->Init.low_power_enable)
  {
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &=
      (uint32_t) ~
      ((uint32_t) (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

    /* Configures system clock after wake-up from STOP: enable HSE, PLL and
     * select PLL as system clock source (HSE and PLL are disabled in STOP
     * mode) */

    __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);

    /* Wait till HSE is ready */
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
    {
    }

    /* Enable the main PLL. */
    __HAL_RCC_PLL_ENABLE();

    /* Wait till PLL is ready */
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as SYSCLK */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_PLLCLK);

    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
    {
    }

    /* ungate PHY clock */
    __HAL_PCD_UNGATE_PHYCLOCK((&hpcd));
  }
#ifdef USE_USB_FS
  /* Clear EXTI pending Bit */
  __HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG();
#else
  /* Clear EXTI pending Bit */
  __HAL_USB_OTG_HS_WAKEUP_EXTI_CLEAR_FLAG();
#endif

}


/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
}


/**
  * @brief  Gets Pointer Data.
  * @param  pbuf: Pointer to report
  * @retval None
  */
static void GetReportData(uint8_t * pbuf)
{
  //clears buffer
  pbuf[0] = 0;
  pbuf[1] = 0;
  pbuf[2] = 0;
  pbuf[3] = 0;
  pbuf[4] = 0;
  pbuf[5] = 0;
  pbuf[6] = 0;

  //fills buffer with n64 hid report
  n64_prepare_hid_report(pbuf);

}




/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_2);
  LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_2);
  n64schedule_update();
  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}
