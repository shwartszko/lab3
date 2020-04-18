
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#define MASK_MAX 256
#define BYTE_COUNT 8
#define GPIOA_IDR 0x40010808
#define GPIOB_ODR 0x40010C0C
#define HIGH 'H'
#define LOW 'L'
#define IDLE 'I'
#define HIGH_THRESH_MIN 3500 // 5000<high<7500 (not really,just for the sports)
#define LOW_THRESH_MAX  1500 // 0<low<2000 (not really,just for the sports)

#define RX_SYNC_STATE 100
#define RX_DATA_STATE 101
#define SAMPLE_MAX 5

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint32_t clock = 0;
uint8_t sample_counter;
static uint32_t phy_to_dll_rx_bus;
static uint32_t dll_to_phy_tx_bus;
static uint32_t dll_to_phy_tx_bus_valid = 0;
static uint32_t phy_to_dll_rx_bus_valid = 0;
uint8_t dll_new_data = 0;
uint8_t phy_rx_new_data = 0;
char samples[5];

static uint32_t phy_tx_data_value = 0;
static uint32_t phy_rx_data_value = 0;
static uint32_t phy_rx_clock = 0;
static uint32_t interface_clock = 0;
uint32_t prev_tx_clock = 0;
uint32_t prev_rx_clock = 0;
uint32_t prev_interface_clock = 0;
uint8_t output_data = 0;

static uint32_t alive = 0;
static uint32_t phy_busy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*
The funcion receives data from dll_tx to send, than sends it over the communication line 
every time tx_clock is in rising edge.
In order to send the data the function starts a clock, than stops it in the end of the transfer.
*/

void send_state(char state, uint8_t *cur_state)
{
	switch(state){
		case HIGH:
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET); //C=1
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); //B=1
			break;
		case LOW:
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET); //C=1
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); //B=D
			break;
		case IDLE:
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET); //C=0
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); //B=DC
			break;
		}
	*cur_state = state;
}	
void phy_TX()
{
	uint8_t current_state;
	uint8_t next_state;
	uint8_t masked_bit = 0;
	static uint8_t tx_first_state = 1;
	static uint8_t tx_preamble_counter = 0;
	static uint8_t data_to_send = 0;
	static uint16_t mask = 1;
	
	if(tx_first_state) //function first iteration, send IDLE
	{
		send_state(IDLE,&current_state);
		tx_first_state = 0;
	}
	if(dll_new_data)//dll tranfered new data to send
	{
		HAL_GPIO_WritePin(phy_tx_busy_GPIO_Port, phy_tx_busy_Pin, GPIO_PIN_SET);//set phy_tx_busy to 1
		phy_busy = 1;
		HAL_TIM_Base_Start(&htim3); //start commi clock 
		HAL_TIM_Base_Start_IT(&htim3);
		dll_new_data = 0;
	}
	if(clock && !prev_tx_clock)
	{
		if(tx_preamble_counter < 3)
		{
			send_state(HIGH,&current_state);
			tx_preamble_counter++;
		}
		else
		{
			masked_bit = dll_to_phy_tx_bus & mask;
			switch(masked_bit)
			{
				case 0:
					send_state(LOW,&current_state);
					break;
				default: //any other value that 0, means current bit is 1.
					send_state(HIGH,&current_state);
					break;
			}
		}
	}
	else 	if(!clock && prev_tx_clock)
	{
		current_state	= (current_state == HIGH) ? (LOW) : (HIGH); //send HIGH if in the last raising edge LOW was sent vice versa
		send_state(current_state,&current_state);
	}
	else if(mask > MASK_MAX) //all 8-bits were sent 
	{
		HAL_GPIO_WritePin(phy_tx_busy_GPIO_Port, phy_tx_busy_Pin, GPIO_PIN_SET);//set phy_tx_busy to 0
		phy_busy = 0;
		HAL_TIM_Base_Stop(&htim3); //stop commi clock 
		HAL_TIM_Base_Stop_IT(&htim3);
		send_state(IDLE,&current_state); //send idle
	}
}

/*
The function receives a bit from the communication line every time the rx_clock is in falling edge.
After reeiving 8 bits the function transfers the data to the dll_rx.
*/
void phy_RX() //this is the function who is known in the streets as "big boy chili". 
{
	static uint8_t rx_preamble_counter = 0; //counts the number of sync 1s that were received 
	static uint32_t rx_state = RX_SYNC_STATE; 
	static uint32_t throws_counter = 0; //counts the number of samples we threw because an unfamiliar pattern 
	static uint32_t mask = 1; //isolate each bit from incoming data
	static uint32_t data_input = 0; 
	static int first_time_so_I_need_to_turn_on_the_timer_ok = 1;
	uint32_t i = 0;
	if(first_time_so_I_need_to_turn_on_the_timer_ok)
	{
		HAL_TIM_Base_Start(&htim4);
		HAL_TIM_Base_Start_IT(&htim4);
		first_time_so_I_need_to_turn_on_the_timer_ok = 0;
	}
	rx_state = (rx_preamble_counter == 3) ? (RX_DATA_STATE) : (RX_SYNC_STATE); //if three sync 1s had been recived, Data state
	if(sample_counter == 5) 
	{
		if(((samples[0] == LOW && samples[1] == LOW && samples[2] == LOW) && (samples[3] == HIGH && samples[4] == HIGH)) 
				|| ((samples[0] == LOW && samples[1] == LOW) && (samples[2] == HIGH &&samples[3] == HIGH && samples[4] == HIGH)))
		{ //low pulse
			if(rx_state == RX_SYNC_STATE)
			{				
				return; //exit program
			}
			mask*=2;
			sample_counter = 0;
		}
		else if(((samples[0] == HIGH && samples[1] == HIGH && samples[2] == HIGH) && (samples[3] == LOW && samples[4] == LOW)) 
				|| ((samples[0] == HIGH && samples[1] == HIGH) && (samples[2] == LOW &&samples[3] == LOW && samples[4] == LOW))) 
		{ //high pulse 
			rx_preamble_counter = (rx_preamble_counter < 3) ? (rx_preamble_counter+1) : (rx_preamble_counter);
			/*
			TO DO:
			- Add a timer to messure the time taken to receive preamble bits
			- Calculate the sampling frequency
			*/
			sample_counter = 0;
			if(rx_state == RX_DATA_STATE)
			{
				data_input+=mask;
				mask*=2;	
			}
		}
		else if(samples[0] == IDLE && samples[1] == IDLE && samples[2] == IDLE && samples[3] == IDLE && samples[4] == IDLE)
		{
			if(rx_state == RX_DATA_STATE || (rx_preamble_counter > 0 && rx_preamble_counter < 3))
			{
								return; //received idle in a middle of a byte, someone made a mistake...
			}
			sample_counter = 0;
		}
		else//isn't a recognized pattern 
		{
			if(throws_counter > 5)
				return; //exit program
			for(i=0; i < SAMPLE_MAX - 1; i++)
			{
				samples[i] = samples[i+1];
			}
			sample_counter--;
			throws_counter++;
		}
	}
	
	
	if(mask > MASK_MAX)//received a byte
	{
		phy_to_dll_rx_bus = data_input;
		phy_rx_new_data = 1;
		mask = 1;
		data_input = 0;
		rx_state = RX_SYNC_STATE;
		rx_preamble_counter = 0;
	}
}

/*
The function communicates with the dll layer
*/
void interface()
{
	static int first_rising_edge = 1;
	static int first_iteration = 1;
	static uint32_t *GPIOA_IDR_PTR = (uint32_t*)GPIOA_IDR;
	static uint32_t *GPIOB_ODR_PTR = (uint32_t*)GPIOB_ODR;
	static uint32_t da_bits = 0;
	if(first_iteration)//first iteration of the function means the interface clock isnt on.
	{
		HAL_TIM_Base_Start(&htim2);
		HAL_TIM_Base_Start_IT(&htim2);
		first_iteration = 0;
	}
	else if(prev_interface_clock && !interface_clock && HAL_GPIO_ReadPin(dll_to_phy_tx_bus_valid_GPIO_Port,dll_to_phy_tx_bus_valid_Pin))
	{//interface clock on falling edge and dll sent new data(valid == 1)
		dll_to_phy_tx_bus = (*GPIOA_IDR_PTR & 255);
		dll_new_data = 1;
	}
	else if(!prev_interface_clock && interface_clock)
	{//interface clock on rising edge
		if(first_rising_edge)
		{
			HAL_GPIO_WritePin(phy_alive_GPIO_Port,phy_alive_Pin,GPIO_PIN_SET);
			first_rising_edge = 0;
			alive = 1;
		}
		if(HAL_GPIO_ReadPin(phy_to_dll_rx_bus_valid_GPIO_Port,phy_to_dll_rx_bus_valid_Pin)) //reset valid one clock cycle after data had been already happend in the past progressive sent to the dll(grammer bitch)
		{//if the valid is 1, which means data was sent to the dll one clock cycke ago, we need to reset valid to 0 
			HAL_GPIO_WritePin(phy_to_dll_rx_bus_valid_GPIO_Port,phy_to_dll_rx_bus_valid_Pin,GPIO_PIN_RESET);
			phy_to_dll_rx_bus_valid = 0;
		}
		if(phy_rx_new_data)
		{//phy_tx received new data, send it to the dll than
			da_bits = (*GPIOB_ODR_PTR & 255) + (phy_to_dll_rx_bus << 8);
			*GPIOB_ODR_PTR = da_bits;
			HAL_GPIO_WritePin(phy_to_dll_rx_bus_valid_GPIO_Port,phy_to_dll_rx_bus_valid_Pin,GPIO_PIN_SET);
			phy_to_dll_rx_bus_valid = 1;
			phy_rx_new_data = 0;
		}
	}
}

void sampleClocks()
{
	prev_tx_clock = clock;
	prev_rx_clock = phy_rx_clock;
	prev_interface_clock = interface_clock;
	//tx_clock = HAL_GPIO_ReadPin(phy_tx_clock_GPIO_Port,phy_tx_clock_Pin);
	//phy_rx_clock = HAL_GPIO_ReadPin(phy_rx_clock_GPIO_Port,phy_rx_clock_Pin);
	interface_clock = HAL_GPIO_ReadPin(interface_clock_GPIO_Port,interface_clock_Pin);
	dll_to_phy_tx_bus_valid = HAL_GPIO_ReadPin(dll_to_phy_tx_bus_valid_GPIO_Port, dll_to_phy_tx_bus_valid_Pin);
	phy_to_dll_rx_bus_valid = HAL_GPIO_ReadPin(phy_to_dll_rx_bus_valid_GPIO_Port, phy_to_dll_rx_bus_valid_Pin);
}

void phy_layer()
{
	phy_TX();
	phy_RX();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc1);
	//HAL_GPIO_WritePin(phy_tx_clock_GPIO_Port,phy_tx_clock_Pin, GPIO_PIN_RESET);//set clock to 0
	HAL_GPIO_WritePin(interface_clock_GPIO_Port,interface_clock_Pin, GPIO_PIN_RESET);//set clock to 0
	HAL_GPIO_WritePin(phy_to_dll_rx_bus_valid_GPIO_Port,phy_to_dll_rx_bus_valid_Pin,GPIO_PIN_RESET);//set valid to 0
	clock = 0;
	phy_rx_clock = 0;
	interface_clock = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		phy_layer();
		interface();
		sampleClocks();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, B_Pin|C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, dll_rx_2_Pin|dll_rx_3_Pin|dll_rx_4_Pin|dll_rx_5_Pin 
                          |dll_rx_6_Pin|dll_rx_7_Pin|interface_clock_Pin|phy_alive_Pin 
                          |phy_tx_busy_Pin|phy_to_dll_rx_bus_valid_Pin|dll_rx_0_Pin|dll_rx_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B_Pin C_Pin */
  GPIO_InitStruct.Pin = B_Pin|C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : dll_tx_0_Pin dll_tx_1_Pin dll_tx_2_Pin dll_tx_3_Pin 
                           dll_tx_4_Pin dll_tx_5_Pin dll_tx_6_Pin dll_tx_7_Pin */
  GPIO_InitStruct.Pin = dll_tx_0_Pin|dll_tx_1_Pin|dll_tx_2_Pin|dll_tx_3_Pin 
                          |dll_tx_4_Pin|dll_tx_5_Pin|dll_tx_6_Pin|dll_tx_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : dll_rx_2_Pin dll_rx_3_Pin dll_rx_4_Pin dll_rx_5_Pin 
                           dll_rx_6_Pin dll_rx_7_Pin interface_clock_Pin phy_alive_Pin 
                           phy_tx_busy_Pin phy_to_dll_rx_bus_valid_Pin dll_rx_0_Pin dll_rx_1_Pin */
  GPIO_InitStruct.Pin = dll_rx_2_Pin|dll_rx_3_Pin|dll_rx_4_Pin|dll_rx_5_Pin 
                          |dll_rx_6_Pin|dll_rx_7_Pin|interface_clock_Pin|phy_alive_Pin 
                          |phy_tx_busy_Pin|phy_to_dll_rx_bus_valid_Pin|dll_rx_0_Pin|dll_rx_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : dll_to_phy_tx_bus_valid_Pin */
  GPIO_InitStruct.Pin = dll_to_phy_tx_bus_valid_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(dll_to_phy_tx_bus_valid_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
