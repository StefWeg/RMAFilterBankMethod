/*
 * main.c
 *
 *  Created on: 15.07.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* Private user code ---------------------------------------------------------*/
static void NVIC_Init(void);

/* Exported variables --------------------------------------------------------*/
uint32_t meas = 0, meas_max = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* SysInit & Initialize all configured peripherals */
  USART_Init();
  stdio_Setup();
  DWT_Init();
  DAC_Init();
  ADC_Init();
  APP_Init();

  MX_GPIO_Init();
  NVIC_Init();

  DAC_Calibrate();

#ifdef CH1_DAC_SNIFFER
  ADC_DACOutput();
#endif
#ifdef CH2_DAC_SNIFFER
  ADC_DACOutput();
#endif
  ADC_EnableOverrunMonitor();
  ADC_Disable();

#ifndef PYTHON_RESULTS
  printf("ready\n\r");
#endif

  /* Infinite loop */
  while (1)
  {
    timeMeasStart();

	switch(getRequest())
	{

	case PROC_MEAS_RESULTS:
		APP_ResetMAFilters();
		break;

	case PREPARE_BUF_SWITCH:
		DAC_HandleBufSwitch();
		break;

	case SEND_MEAS_REULTS:
		APP_SendMeasResultsIteration();
		break;

	case RUN_IDENT_PROC:
		APP_RunIdentificationIteration();
		break;

	case HANDLE_ERROR:
		USART_NotifyAboutError();
		break;

	case START_MEAS:
		ADC_Enable();
		DAC_Start();
		APP_ResetMAFiltersInit();
		APP_ResetIdentParams();
#ifndef PYTHON_RESULTS
		printf("start\n\r");
#endif
		ADC_MeasStart();
		meas_max = 0;
		timeMeasStart();
		break;

	case STOP_MEAS:
		ADC_Disable();
		DAC_Stop();
#ifndef PYTHON_RESULTS
		printf("stop\n\r");
		printf("max loop time: %lu us\n\r", meas_max);
#endif
		break;

	default:
		break;
	}

	meas = timeMeasStop();
	if (meas > meas_max) meas_max = meas;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* Configure the main internal regulator output voltage */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

static void NVIC_Init(void)
{
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);		// 4 group priorities & 4 subpriorities
	// 2 bits for pre-emption priority, 2 bits for subpriority
	// page 229 of Programming manual (PMO214)
	// the PRIGROUP field of AIRCR register indicates the position of the binary point
	// that splits the PRI_n fields in the IPR into separate group priority and subpriority fields.
	// NVIC_PRIORITYGROUP_2 = 5  >>  binary point 0bxx.yy  - group priority bits [7:6], subpriority bits [5:4]

	// DMA (DAC)
	NVIC_SetPriority(DMA1_Channel3_IRQn, 0b0101);		// DAC DMA transfer complete (group 1, priority 1);
	// more about setting priority can be found on page 215 of Programming manual (PM0214)
		// __STATIC_INLINE void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
		/*
		 * NVIC->IP[IRQn] = (uint8_t)((priority << 4) & 0x000000FF);
		 */
		// Each priority field holds a priority value (0-255)
		// the lower the value the greater the priority of the corresponding interrupt
		// processor implements only bits [7:4] of each fiels, bits [3:0] read as zero and ignore write
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	// ADC
	NVIC_SetPriority(ADC1_2_IRQn, 0b0100);	 // ADC interrupt (EOS, OVR) (group 1, priority 0);
	NVIC_EnableIRQ(ADC1_2_IRQn);

	// UART
	NVIC_SetPriority(USART2_IRQn, 0b1010);			// USART2 read register not empty (group 3, priority 0)
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(DMA1_Channel7_IRQn, 0b1001);	// USART2_TX DMA transfer complete (group 3, priority 1)
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	// DAC
	NVIC_SetPriority(TIM6_DAC_IRQn, 0b1100); // TIM6 global and DAC1&2 underrun error interrupts (group 4, priority 0)
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
