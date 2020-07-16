/*
 * ADC.h
 *
 *  Created on: 14.08.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

#ifndef ADC_H_
#define ADC_H_

/* Define to prevent recursive inclusion -------------------------------------*/

#undef CH1_DAC_SNIFFER
#undef CH2_DAC_SNIFFER

/* Exported functions --------------------------------------------------------*/

void ADC_Init(void);
void ADC_Enable(void);
void ADC_EnableOverrunMonitor(void);
void ADC_MeasStart(void);
void ADC1_2_IRQHandler(void);
void ADC_Disable(void);
void ADC_Deinit(void);
void ADC_MeasVREF(void);
void ADC_DACOutput(void);

#endif /* ADC_H_ */
