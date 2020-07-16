/*
 * utils.c
 *
 *  Created on: 26.09.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

#include "main.h"

void DWT_Init()
{
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk << DWT_CTRL_CYCCNTENA_Pos;	// enable the counter
}

#pragma GCC push_options
#pragma GCC optimize("03")

void delayUS_DWT(uint32_t us)
{
	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while(DWT->CYCCNT - start < cycles);
}

volatile uint32_t measStart = 0;
volatile uint32_t measStop = 0;
void timeMeasStart()
{
	measStart = DWT->CYCCNT;
}
uint32_t timeMeasStop()
{
	volatile uint32_t cycles = (SystemCoreClock/1000000L);
	measStop = DWT->CYCCNT;
	return (measStop - measStart) / cycles;
}

#pragma GCC pop_options

// ------------------------------------------------------------

volatile uint8_t requestQueue[REQ_QUEUE_LEN] = {0};
volatile uint8_t inputIdx = 0;
volatile uint8_t outputIdx = 0;

void putRequest(BackgroundCodeReq req)
{
	requestQueue[inputIdx++] = (uint8_t)req;
	inputIdx = inputIdx % REQ_QUEUE_LEN;
}

BackgroundCodeReq getRequest(void)
{
	BackgroundCodeReq request = NONE;
	if (inputIdx != outputIdx) {
		request = requestQueue[outputIdx++];
		outputIdx = outputIdx % REQ_QUEUE_LEN;
	}
	return request;
}
