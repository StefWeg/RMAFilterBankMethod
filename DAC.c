/*
 * DAC.c
 *
 *  Created on: 26.07.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

#include "main.h"

#define OUTPUT_BUFFER_SIZE 10
uint16_t outBufA[OUTPUT_BUFFER_SIZE];
uint16_t outBufB[OUTPUT_BUFFER_SIZE];
uint16_t* nextBuf = outBufA;

uint32_t sampleIdx = 0;	// sample index (enough for 49 days of signal generation with sampling period 0.1 ms)

volatile bool bufPrepInProgress = false;

void DAC_Init(void)
{
	// DAC ----------------------------------------

	// DAC1 clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

	// disable DAC1 channel 1
	DAC1->CR &= ~(uint32_t)(DAC_CR_EN1);

	// DAC1 channel1 DMA mode enable
	DAC1->CR |= DAC_CR_DMAEN1;

	// DAC1 channel1 triggered by TIM6 TRGO
	DAC1->CR &= ~(uint32_t)(DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0);

	// DAC channel1 trigger enable
	DAC1->CR |= DAC_CR_TEN1;

	// DAC Channel 1 in normal mode connected to external pin with buffer enabled
	DAC1->MCR &= ~(DAC_MCR_MODE1_2 | DAC_MCR_MODE1_1 | DAC_MCR_MODE1_0);

	// GPIO ---------------------------------------

	// For DAC configure the desired I/O in analog mode in the GPIOx_MODER
	// register and configure the required function in DAC registers.

	// PA4 (DAC1_OUT1) - [A2]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	// GPIOA clock enable

	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL4);				// port A, pin 4, no alternative function

	GPIOA->PUPDR &= GPIO_NOPULL << GPIO_PUPDR_PUPD4_Pos;		// 0b00 << 8 // port A, pin 4, no pull-up no pull-down

	GPIOA->MODER |= GPIO_MODE_ANALOG << GPIO_MODER_MODE4_Pos;	// 0b11 << 8 // port A, pin 4, analog mode

	// DMA ----------------------------------------

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;		// DMA1 clock enable

	// disable DMA1 channel 3
	DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_EN);				// DMA1 channel3 disabled

	// mapping DMA1 CH3
	DMA1_CSELR->CSELR &= ~(uint32_t)(DMA_CSELR_C3S);
	DMA1_CSELR->CSELR |= DMA_REQUEST_6 << DMA_CSELR_C3S_Pos;	// requests from DAC_CH1 - CxS[3:0] = 0b0110

	// set peripheral register address
	DMA1_Channel3->CPAR = (uint32_t)&(DAC1->DHR12R1);			// DAC1 channel1 12-bit right-aligned data holding register

	// set memory address
	DMA1_Channel3->CMAR = (uint32_t)(outBufA);					// first of output buffers

	// set number of data to transfer
	DMA1_Channel3->CNDTR = OUTPUT_BUFFER_SIZE;					// number of elements in output buffer

	// set channel priority
	DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_PL);
	DMA1_Channel3->CCR |= DMA_PRIORITY_HIGH << DMA_CCR_PL_Pos;	// 0b10 << 12; // high priority level (< very high)

	// set data transfer direction
	DMA1_Channel3->CCR |= DMA_CCR_DIR;							// read from memory

	// set circular mode
	DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_CIRC);			// circular mode disabled

	// set peripheral and memory increment mode
	DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_PINC);			// peripheral increment disabled
	DMA1_Channel3->CCR |= DMA_CCR_MINC;							// memory increment enabled

	// set peripheral and memory data size
	DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_PSIZE);
	DMA1_Channel3->CCR |= DMA_PDATAALIGN_HALFWORD; 				// 0b01 << 8;  // 16 bits (halfword)
	DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_MSIZE);
	DMA1_Channel3->CCR |= DMA_MDATAALIGN_HALFWORD; 				// 0b01 << 10; // 16 bits (halfword)

	// set interrupt (switches buffers and triggers filling of inactive buffer)
	DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_TCIE);			// transfer complete interrupt disabled

	// TIM ----------------------------------------

	// General purpose TIM6 16-bit (basic timer)
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;			// TIM6 clock enable

	TIM6->CR1 &= ~(uint32_t)(TIM_CR1_CEN);			// disable TIM6 timer

	// Setting prescaler value
	// tick freq: f_CK_CNT = f_CK_PSC / (PSC+1)
	TIM6->PSC = 99;									// 80 MHz / 100 = 800 kHz

	// Setting auto-reload value
	// timer counts from 0 to ARR
	TIM6->ARR = 79;							// update evet freq: f_CK_CNT / (79 + 1) = 10 kHz

	TIM6->CR2 &= ~(uint32_t)(TIM_CR2_MMS);
	TIM6->CR2 |= TIM_CR2_MMS_1;				//  0b010 << 4 - the update event is selected as a trigger output (TRGO)

	TIM6->CR1 |= TIM_CR1_ARPE;				// auto-reload preload enable (TIM6_ARR register changes on update event)
	TIM6->EGR |= TIM_EGR_UG;				// reinitialize the counter - prescaler counter too (auto cleared by hardware)

}

void DAC_Calibrate(void)
{
	// disable DAC1 channel 1
	DAC1->CR &= ~(uint32_t)(DAC_CR_EN1);

	// DAC Channel 1 in normal mode connected to external pin with buffer enabled
	DAC1->MCR &= ~(DAC_MCR_MODE1_2 | DAC_MCR_MODE1_1 | DAC_MCR_MODE1_0);	// 0b000

	// start DAC1 channel 1 calibration
	DAC1->CR |= (uint32_t)(DAC_CR_CEN1);

	// apply trimming algorithm
	for (uint16_t val = 0; val <= 0b11111; val++) {

		// put subsequent values into OTRIM1[4:0] bits
		DAC1->CCR &= DAC_CCR_OTRIM2_Msk;		// leave OTRIM2[4:0] bits untouched
		DAC1->CCR |= (val & DAC_CCR_OTRIM1_Msk);

		// wait for t_TRIM delay (min 50us according to datasheet)
		delayUS_DWT(100);

		// check if CAL_FLAG1 bit is set to 1
		if (DAC1->SR & DAC_SR_CAL_FLAG1) {
			break;	// proper trimming code OTRIM1 is found
		}
	}

	// finish DAC1 channel 1 calibration
	DAC1->CR &= ~(uint32_t)(DAC_CR_CEN1);
}

void DAC_Start(void)
{
	sampleIdx = 0;

	for (uint32_t n = 0; n < OUTPUT_BUFFER_SIZE; n++) {
	  outBufA[n] = (uint16_t)((sin1[sampleIdx%SIN1_LEN] + sin2[sampleIdx%SIN2_LEN]
					+ sin3[sampleIdx%SIN3_LEN] + sin4[sampleIdx%SIN4_LEN] + 4) * 450);
	  sampleIdx++;
	}
	for (uint32_t n = 0; n < OUTPUT_BUFFER_SIZE; n++) {
	  outBufB[n] = (uint16_t)((sin1[sampleIdx%SIN1_LEN] + sin2[sampleIdx%SIN2_LEN]
		 			+ sin3[sampleIdx%SIN3_LEN] + sin4[sampleIdx%SIN4_LEN] + 4) * 450);
	  sampleIdx++;
	}
	nextBuf = outBufB;

	DAC1->CR |= DAC_CR_EN1;					// enable DAC1 CH1
	DAC1->CR |= DAC_CR_DMAUDRIE1;			// DAC CH1 DMA underrun interrupt enabled
	DMA1_Channel3->CCR |= DMA_CCR_TCIE;		// transfer complete interrupt enabled
	DMA1_Channel3->CCR |= DMA_CCR_EN;		// DMA1_CH3 enabled
	TIM6->CR1 |= TIM_CR1_CEN;				// enable TIM6 timer
}

void DAC_HandleBufSwitch(void)
{
	static uint32_t n = 0;

	if (!bufPrepInProgress)
	{
		bufPrepInProgress = true;

		// determine next output buffer to be filled in
		if (nextBuf == outBufA) nextBuf = outBufB;
		else nextBuf = outBufA;

		n = 0;
		putRequest(PREPARE_BUF_SWITCH);
	}
	else
	{
#ifdef OSCILLOSCOPE_MEAS_MODE
		nextBuf[n] = (uint16_t)( (sin2[sampleIdx%SIN2_LEN] + 1.1 ) * 1800 );
		sampleIdx++;
		n++;
		nextBuf[n] = (uint16_t)( (sin2[sampleIdx%SIN2_LEN] + 1.1 ) * 1800 );
		sampleIdx++;
		n++;
#else
		// merge signals (2 samples at a time)
		nextBuf[n] = (uint16_t)((sin1[sampleIdx%SIN1_LEN] + sin2[sampleIdx%SIN2_LEN]
					+ sin3[sampleIdx%SIN3_LEN] + sin4[sampleIdx%SIN4_LEN] + 4.1) * 450);
		sampleIdx++;
		n++;
		nextBuf[n] = (uint16_t)((sin1[sampleIdx%SIN1_LEN] + sin2[sampleIdx%SIN2_LEN]
					+ sin3[sampleIdx%SIN3_LEN] + sin4[sampleIdx%SIN4_LEN] + 4.1) * 450);
		sampleIdx++;
		n++;
#endif

		if (n < OUTPUT_BUFFER_SIZE) putRequest(PREPARE_BUF_SWITCH);
		else bufPrepInProgress = false;
	}
}

void DMA1_Channel3_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF3)			// if DAC DMA TC (transfer complete) flag caused interrupt
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF3;					// transfer complete flag clear for channel 3

		DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_EN);		// disable DMA1 CH3 (DAC_CH1)
		DMA1_Channel3->CMAR = (uint32_t)(nextBuf);			// set memory address: next output buffer
		DMA1_Channel3->CNDTR = OUTPUT_BUFFER_SIZE;			// number of elements in output buffer
		DMA1_Channel3->CCR |= DMA_CCR_EN;					// activate DMA1 CH3 (DAC_CH1)

		if (bufPrepInProgress) {
			errorToReport = BUF_PREP_UNDERRUN;
			putRequest(HANDLE_ERROR);
		}
		putRequest(PREPARE_BUF_SWITCH);		// signal output buffer switch
	}
}

void TIM6_DAC_IRQHandler(void)
{
	if (DAC1->SR & DAC_SR_DMAUDR1)	// if DAC CH1 DMA underrun flag caused interrupt
	{
		DAC1->SR |= DAC_SR_DMAUDR1; 	// flag cleared by software (by writing it to 1)

		errorToReport = DAC_DMA_UNDERRUN;
		putRequest(HANDLE_ERROR);
	}
}

void DAC_Stop(void)
{
	DAC1->DHR12R1 = 0;
	DMA1_Channel3->CCR &= ~(uint32_t)(DMA_CCR_EN);		// DMA1_CH3 disabled
	// (no pending requests from the peripheral will be served by DMA)
	DAC1->CR &= ~(uint32_t)(DAC_CR_EN1);				// disable DAC1 CH1
	DAC1->CR &= ~(uint32_t)(DAC_CR_DMAUDRIE1);			// DAC CH1 DMA underrun interrupt disabled
	TIM6->CR1 &= ~(uint32_t)(TIM_CR1_CEN);				// disable TIM6 timer
}
