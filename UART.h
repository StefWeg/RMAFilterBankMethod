/*
 * UART.h
 *
 *  Created on: 26.07.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef UART_H_
#define UART_H_

/* Exported variables --------------------------------------------------------*/

extern ErrorTypes errorToReport;

/* Exported types ------------------------------------------------------------*/

typedef enum eCommandTypes
{
	START_MEAS_CMD      = (uint8_t)'s',
	END_MEAS_CMD        = (uint8_t)'e',
	GET_RESULTS_CMD     = (uint8_t)'r',
	IDENTIFY_PARAMS_CMD = (uint8_t)'i'
} CommandTypes;

/* Exported functions --------------------------------------------------------*/

void USART_Init(void);
void stdio_Setup(void);
void USART_SendMessage(char* str, uint8_t len);
void USART_NotifyAboutError();

#endif /* UART_H_ */
