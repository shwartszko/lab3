/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#define HIGH_THRESH_MIN 3500 // 5000<high<7500 (not really,just for the sports)
#define LOW_THRESH_MAX  1500 // 0<low<2000 (not really,just for the sports)
#define HIGH 'H'
#define LOW 'L'
#define IDLE 'I'
#define RX_SYNC_STATE 100
#define RX_DATA_STATE 101
#define SAMPLE_MAX 5
extern uint32_t clock;
extern uint32_t prev_tx_clock;
extern char samples[5];
extern uint8_t sample_counter;
static uint32_t temp = 0;
extern ADC_HandleTypeDef hadc1;
extern uint8_t received_bit;
extern uint8_t rx_preamble_counter; //counts the number of sync 1s that were received 
extern uint32_t rx_state; 
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	HAL_GPIO_TogglePin(interface_clock_GPIO_Port, interface_clock_Pin);
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	clock = 1 - clock;
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	int i = 0;
	static uint32_t throws_counter = 0; //counts the number of samples we threw because an unfamiliar pattern 
	/* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	if(sample_counter != 5)
	{
		while(HAL_ADC_PollForConversion(&hadc1,5) != HAL_OK){}
		temp = HAL_ADC_GetValue(&hadc1);
		if(temp > HIGH_THRESH_MIN)
			samples[sample_counter] = HIGH;
		else if(temp < LOW_THRESH_MAX)
			samples[sample_counter] = LOW;
		else
			samples[sample_counter] = IDLE;
		sample_counter++;
	}
	else
	{
		//rx_state = (rx_preamble_counter == 3) ? (RX_DATA_STATE) : (RX_SYNC_STATE);//if three sync 1s had been recived, Data state
		if(((samples[0] == LOW && samples[1] == LOW && samples[2] == LOW) && (samples[3] == HIGH && samples[4] == HIGH)) 
				|| ((samples[0] == LOW && samples[1] == LOW) && (samples[2] == HIGH &&samples[3] == HIGH && samples[4] == HIGH)))
		{ //low pulse
			/*
			if(rx_state == RX_SYNC_STATE)
			{
					return; //exit program
			}
			*/
			received_bit = 1;
			sample_counter = 0;
		}
		else if(((samples[0] == HIGH && samples[1] == HIGH && samples[2] == HIGH) && (samples[3] == LOW && samples[4] == LOW)) 
				|| ((samples[0] == HIGH && samples[1] == HIGH) && (samples[2] == LOW &&samples[3] == LOW && samples[4] == LOW))) 
		{ //high pulse 
			//rx_preamble_counter = (rx_preamble_counter < 3) ? (rx_preamble_counter+1) : (rx_preamble_counter);
				/*
				TO DO:
				- Add a timer to messure the time taken to receive preamble bits
				- Calculate the sampling frequency
				*/
			sample_counter = 0;
			//if(rx_state == RX_DATA_STATE)
			//{
				received_bit = 2;
			//}
		}
		else if(samples[0] == IDLE && samples[1] == IDLE && samples[2] == IDLE && samples[3] == IDLE && samples[4] == IDLE)
		{
			//if(rx_state == RX_DATA_STATE || (rx_preamble_counter > 0 && rx_preamble_counter < 3))
			//{
			//					return; //received idle in a middle of a byte, someone made a mistake...
			//}
			sample_counter = 0;
			received_bit = 0;
		}
		else//isn't a recognized pattern 
		{
			received_bit = 0;
			//if(throws_counter > 5)
			//	return; //exit program
			for(i=0; i < SAMPLE_MAX - 1; i++)
			{
				samples[i] = samples[i+1];
			}
			sample_counter--;
			//throws_counter++;
		}
	}
  /* USER CODE END TIM4_IRQn 1 */


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
}
