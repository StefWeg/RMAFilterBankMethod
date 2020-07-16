/*
 * utils.h
 *
 *  Created on: 26.09.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef UTILS_H_
#define UTILS_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32l4xx_hal.h"

/* Exported constants --------------------------------------------------------*/

#define DWT_SECOND 1000000ul
#define REQ_QUEUE_LEN 10

/* Exported types ------------------------------------------------------------*/

typedef enum eBackgroundCodeReq
{
	NONE = 0,
	PROC_MEAS_RESULTS,
	PREPARE_BUF_SWITCH,
	SEND_MEAS_REULTS,
	RUN_IDENT_PROC,
	HANDLE_ERROR,
	START_MEAS,
	STOP_MEAS,
} BackgroundCodeReq;

typedef enum eErrorTypes
{
	ERROR_NONE,
	DAC_DMA_UNDERRUN,
	ADC1_OVERRUN,
	ADC2_OVERRUN,
	PROCESSING_UNDERRUN,
	BUF_PREP_UNDERRUN,
	ERROR_NUM
} ErrorTypes;

/* Exported functions --------------------------------------------------------*/

void DWT_Init();
void delayUS_DWT(uint32_t us);

void timeMeasStart();
uint32_t timeMeasStop();

void putRequest(BackgroundCodeReq req);
BackgroundCodeReq getRequest(void);

#endif /* UTILS_H_ */
