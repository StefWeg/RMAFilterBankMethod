/*
 * main.h
 *
 *  Created on: 15.07.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <complex.h> // complex numbers
#include <tgmath.h>  // sin cos tg ctg
#include <string.h>  // memset

#include "stm32l4xx_hal.h"

#include "app.h"
#include "ADC.h"
#include "DAC.h"
#include "signals.h"
#include "utils.h"
#include "UART.h"

/* Toggles -------------------------------------------------------------------*/

//#define PYTHON_RESULTS
//#define IMPEDANCE_RESULTS
//#define FILTRATION_RESULTS

//#define STEP_CHANGE_SIM

//#define OSCILLOSCOPE_MEAS_MODE

/* Exported variables --------------------------------------------------------*/
extern uint32_t meas, meas_max;

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
