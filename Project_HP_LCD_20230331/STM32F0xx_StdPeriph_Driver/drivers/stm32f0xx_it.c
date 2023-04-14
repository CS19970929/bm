/**
 ******************************************************************************
 * @file    IO_Toggle/stm32f0xx_it.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    23-March-2012
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "main.h"
#include "bsp.h"
/** @addtogroup STM32F0_Discovery_Peripheral_Examples
 * @{
 */

/** @addtogroup IO_Toggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
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
  //	 if (CoreDebug->DHCSR & 1) {  //check C_DEBUGEN == 1 -> Debugger Connected
  //     __breakpoint(0);  // halt program execution here
  //  }
  /* Go to infinite loop when Hard Fault exception occurs */
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
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */

/**
 * @}
 */
void EXTI4_15_IRQHandler(void)
{
  // if (EXTI_GetITStatus(EXTI_Line12) != RESET)
  // {
  //   Delay_Base10us(OtherElement.u16CBC_DelayT / 100);
  //   if (MCUI_CBC_DSG == 0)
  //   {
  //     MCUO_MOS_DSG = CLOSE;
  //     MCUO_RELAY_DSG = CLOSE;
  //     MCUO_RELAY_PRE = CLOSE;
  //     MCUO_RELAY_MAIN = CLOSE;
  //     CBC_Element.u8CBC_DSG_ErrFlag = 1;
  //     // App_MOS_Relay_Ctrl();
  //     System_ERROR_UserCallback(ERROR_CBC_DSG);
  //   }
  //   EXTI_ClearITPendingBit(EXTI_Line12);
  // }

  // if (EXTI_GetITStatus(EXTI_Line13) != RESET)
  // {
  //   EXTI_ClearITPendingBit(EXTI_Line13);
  // }

  // if (EXTI_GetITStatus(EXTI_Line10) != RESET)
  // {
  //   EXTI_ClearITPendingBit(EXTI_Line10);
  // }

  // if (EXTI_GetITStatus(EXTI_Line14) != RESET)
  // {
  //   EXTI_ClearITPendingBit(EXTI_Line14);
  //   gu8_SOC_DI = 1;
  // }

  if (EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
    // MCUO_DEBUG_LED1 = ~MCUO_DEBUG_LED1;
    // SystemInit();
    // EXTI_ClearITPendingBit(EXTI_Line6);

    EXTI_ClearITPendingBit(EXTI_Line6);
    SystemInit();        // 重新设置时钟
    USART1->ISR = 0x00C0; // 复位状态寄存器
    USART1->TDR = 0x00;   // 复位数据寄存器

    USART2->ISR = 0x00C0; // 复位状态寄存器
    USART2->TDR = 0x00;   // 复位数据寄存器

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_AHBPeriph_GPIOA | RCC_APB1Periph_USART2, ENABLE); // 使能USART1，GPIOA时钟
  }
}

//   void EXTI15_10_IRQHandler(void)
//   {
//     if (EXTI_GetITStatus(EXTI_Line10) != RESET)
//     {
//       EXTI_ClearITPendingBit(EXTI_Line10);
//       SystemInit();                                                                 // 重新设置时钟
//       USART1->SR = 0x00C0;                                                          // 复位状态寄存器
//       USART1->DR = 0x00;                                                            // 复位数据寄存器
//       RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); // 使能USART1，GPIOA时钟
//     }
//   }
// }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
