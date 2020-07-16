/*
 * DAC.h
 *
 *  Created on: 26.07.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef DAC_H_
#define DAC_H_

/* Exported functions --------------------------------------------------------*/

void DAC_Init(void);
void DAC_Calibrate(void);
void DAC_Start(void);
void DAC_HandleBufSwitch(void);
void DAC_Stop(void);

#endif /* DAC_H_ */
