/*
 * ADC.c
 *
 *  Created on: 14.08.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

#include "main.h"

void ADC_Init(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;	// ADC Clock enable (ADC123 are AHB2 peripherals)

	ADC1->CR &= ~(uint32_t)(ADC_CR_ADEN);	// clear the bit - make sure ADC1 is disabled
	ADC2->CR &= ~(uint32_t)(ADC_CR_ADEN);	// clear the bit - make sure ADC2 is disabled

	// Define the ADC clock scheme - common to both master and slave ADCs
	// ADC Synchronous clock mode - no jitter in the delay from a timer trigger to the start of conversion
	// (ADC clock derived from the AHB clock of the ADC bus interface)
	ADC123_COMMON->CCR &= ~(uint32_t)(ADC_CCR_CKMODE);
	ADC123_COMMON->CCR |= ADC_CCR_CKMODE_0 & ADC_CCR_CKMODE_Msk; // set CKMODE[1:0] to 0b01 - HCLK/1 (ADC clk = 80 Mhz)

	// Disable ADC1 & ADC2 deep-power-down mode
	ADC1->CR &= ~(uint32_t)(ADC_CR_DEEPPWD);	// clear the bit to exit deep-power-down mode
	ADC2->CR &= ~(uint32_t)(ADC_CR_DEEPPWD);	// clear the bit to exit deep-power-down mode

	// Enable the ADC1 & ADC2 internal voltage regulator
	// (required before launching a calibration or enabling the ADC)
	ADC1->CR |= ADC_CR_ADVREGEN;	// set the bit to enable voltage regulator
	ADC2->CR |= ADC_CR_ADVREGEN;	// set the bit to enable voltage regulator

	// Wait for the regulator start-up time
	delayUS_DWT(20);	// according to uC datasheet T_ADCVREG_STUP = max 20 us

	// Calibration (ADC calculates a 7-bit calibration factor, removing any offset error)
	ADC1->CR &= ~(uint32_t)(ADC_CR_ADCALDIF);	// clear the bit - calibration applies for single-ended inputs mode
	ADC2->CR &= ~(uint32_t)(ADC_CR_ADCALDIF);	// clear the bit - calibration applies for single-ended inputs mode
	ADC1->CR |= ADC_CR_ADCAL;	// set the bit - initialize the calibration
	ADC2->CR |= ADC_CR_ADCAL;	// set the bit - initialize the calibration
	// bit ADCAL is cleared by hardware as soon as the calibration completes
	while((ADC1->CR & ADC_CR_ADCAL) || (ADC2->CR & ADC_CR_ADCAL));
	// (associated calibration factors are stored in the bits ADC_CALFACT_CALFACT_S[6:0] of ADC->CALFACT registers)

	// Wait at least 4 ADC clock cycle after ADCAL bit is cleared by hardware (ADEN bit cannot be set earlier)
	delayUS_DWT(1);	// 1 us is enough as ADC clk is 80 MHz

	// ADC1 and ADC2 are coupled and can operate in dual mode (ADC1 is master)
	// Dual mode configuration
	ADC123_COMMON->CCR &= ~(uint32_t)(ADC_CCR_DUAL);
	ADC123_COMMON->CCR |= 0b00110 << ADC_CCR_DUAL_Pos; // regular simultaneous mode

	// Configure ADC1 channel 1 to be single-ended (channels 1-5 are fast channels)
	ADC1->DIFSEL &= ~(uint32_t)(ADC_DIFSEL_DIFSEL_1);	// clear the bit to configure channel 1 in single-ended mode
	// Configure ADC2 channel 2 to be single-ended (channels 1-5 are fast channels)
	ADC1->DIFSEL &= ~(uint32_t)(ADC_DIFSEL_DIFSEL_2);	// clear the bit to configure channel 2 in single-ended mode
	// (converted analog voltage is the difference between the external voltage V_INP1 and V_REF- = GND)
	// (do not convert the same channel on the two ADCs in dual mode)
	// (ADC1 channel 0 used for VREF measurement is permanently in single-ended mode, DIFSEL_0 is read-only)

	// Enable ADC1 & ADC2
	ADC_Enable();

	// -----------------------------------------------------------------------------------------
	// Configure GPIO

	// ADC1 Channel 1 input coming from GPIO pad
	// PC0 (ADC123_IN1) - [A6] - additional function: ADC123 CH1 input
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;	// GPIOC clock enable
	GPIOC->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL0);	// port C, pin 0, no alternative function
	GPIOC->PUPDR &= GPIO_NOPULL << GPIO_PUPDR_PUPD0_Pos;	// 0b00 << 0 // port C, pin 0, no pull-up no pull-down
	GPIOC->MODER |= GPIO_MODE_ANALOG << GPIO_MODER_MODE0_Pos;	// 0b11 << 0 // port C, pin 0, analog mode
	GPIOC->ASCR |= GPIO_ASCR_ASC0;	// port C, pin 0, connect analog switch to the ADC1 input

	// ADC2 Channel 2 input coming from GPIO pad
	// PC1 (ADC123_IN2) - [A4] - additional function: ADC123 CH2 input
	// RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;	// GPIOC clock enable
	GPIOC->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL1);	// port C, pin 1, no alternative function
	GPIOC->PUPDR &= GPIO_NOPULL << GPIO_PUPDR_PUPD1_Pos;	// 0b00 << 0 // port C, pin 1, no pull-up no pull-down
	GPIOC->MODER |= GPIO_MODE_ANALOG << GPIO_MODER_MODE1_Pos;	// 0b11 << 0 // port C, pin 1, analog mode
	GPIOC->ASCR |= GPIO_ASCR_ASC1;	// port C, pin 1, connect analog switch to the ADC2 input

	// -----------------------------------------------------------------------------------------

	// Configure regular conversion group for ADC1
	// set CH1 as the 1st conversion in regular sequence
	ADC1->SQR1 &= ~(uint32_t)(ADC_SQR1_SQ1);
	ADC1->SQR1 |= 1 << ADC_SQR1_SQ1_Pos;
	// set total number of conversions in the regular channel conversion sequence to 1
	ADC1->SQR1 &= ~(uint32_t)(ADC_SQR1_L);	// 0b0000 - convert single channel

	// Configure regular conversion group for ADC2
	// set CH2 as the 1st conversion in regular sequence
	ADC2->SQR1 &= ~(uint32_t)(ADC_SQR1_SQ1);
	ADC2->SQR1 |= 2 << ADC_SQR1_SQ1_Pos;
	// set total number of conversions in the regular channel conversion sequence to 1
	ADC2->SQR1 &= ~(uint32_t)(ADC_SQR1_L);	// 0b0000 - convert single channel
	// (in regular simultaneous mode ADCs must convert sequences with the same length)

	// Configure ADC1 resolution
	ADC1->CFGR &= ~(uint32_t)(ADC_CFGR_RES);	// 0b00 - set resolution of the conversion to 12-bit
	// Configure ADC2 resolution
	ADC2->CFGR &= ~(uint32_t)(ADC_CFGR_RES);	// 0b00 - set resolution of the conversion to 12-bit

	// Configure ADC1 CH1 sampling time
	ADC1->SMPR1 &= ~(uint32_t)(ADC_SMPR1_SMP1);	// set SMP[2:0] to 0b000 (2.5 ADC clock cycles)
	// Configure ADC2 CH2 sampling time
	ADC2->SMPR1 &= ~(uint32_t)(ADC_SMPR1_SMP2);	// set SMP[2:0] to 0b000 (2.5 ADC clock cycles)
	// Total conversion time: sampling time + successive approximation time
	// T_CONV = 2.5 + 12.5 (for 12-bit resolution) = 15 ADC cycles (187.5ns for 80 MHz)
	// (2.5 ADC clock cycles available only for fast channels 1-5)

	// Configure single conversion mode for regular conversions
	// (ADC performs once all the conversions from the regular sequence)
	// (Then ADC stops until new software/>external< trigger occurs)
	ADC1->CFGR &= ~(uint32_t)(ADC_CFGR_CONT);	// clear the bit to set single conversion mode
	// [[ COMMON ]]

	// -----------------------------------------------------------------------------------------
	// Configure timer 2

	// General purpose TIM2 32-bit
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;	// TIM2 clock enable

	TIM2->CR1 &= ~(uint32_t)(TIM_CR1_CEN);	// disable TIM2 timer

	// Setting prescaler value
	// tick freq: f_CK_CNT = f_CK_PSC / (PSC+1)
	TIM2->PSC = 99;	// 80 MHz / 100 = 800 kHz

	// Setting auto-reload value
	// timer counts from 0 to ARR
	TIM2->ARR = 79;	// update event freq: f_CK_CNT / (79 + 1) = 10 kHz

	TIM2->CR2 &= ~(uint32_t)(TIM_CR2_MMS);
	TIM2->CR2 |= TIM_CR2_MMS_1;	//  0b010 << 4 - the update event is selected as a trigger output (TRGO)

	TIM2->CR1 |= TIM_CR1_ARPE;	// auto-reload preload enable (TIM2_ARR register changes on update event)
	TIM2->EGR |= TIM_EGR_UG;	// reinitialize the counter - prescaler counter too (auto cleared by hardware)

	// -----------------------------------------------------------------------------------------

	// Enable external trigger and set its polarity for regular group
	ADC1->CFGR &= ~(uint32_t)(ADC_CFGR_EXTEN);
	ADC1->CFGR |= 0b01 << ADC_CFGR_EXTEN_Pos;	// 0b01 - hardware trigger detection on the rising edge
	// (update event signal of timer has a form of an impulse)
	// [[ MASTER ONLY ]]

	// Set external hardware trigger event (for regular channel)
	ADC1->CFGR &= ~(uint32_t)(ADC_CFGR_EXTSEL);
	// Choose timer 2 (32-bit general purpose timer) trigger output as hardware trigger
	ADC1->CFGR |= 0b1011 << ADC_CFGR_EXTSEL_Pos; // choose event 11 = EXT11 = TIM2_TRGO
	// Each ADC master shares the same input triggers with its ADC slave
	// [[ MASTER ONLY ]]

	// Set right alignment of data stored in ADC_DR
	ADC1->CFGR &= ~(uint32_t)(ADC_CFGR_ALIGN);	// clear the bit to set right data alignment
	ADC2->CFGR &= ~(uint32_t)(ADC_CFGR_ALIGN);	// clear the bit to set right data alignment
	// (offset disabled, unsigned value)

	// Make ADC1 & ADC2 overwrite data register with the last conversion result when an overrun is detected
	ADC1->CFGR |= ADC_CFGR_OVRMOD;	// set the bit
	ADC2->CFGR |= ADC_CFGR_OVRMOD;	// set the bit
	// Enable ADC1 & ADC2 overrun monitoring
	ADC_EnableOverrunMonitor();


	// Enable end of regular sequence (EOS) interrupt
	ADC1->IER |= ADC_IER_EOSIE;	// set the bit to enable end of regular sequence interrupt
	// (EOC flag will be cleared by reading ADC_DR register in end of sequence interrupt)
	// If the duration of the master regular sequence is equal to the duration of the slave one,
	// it is possible for the software to enable only one of the two EOC interrupt (ex: master EOC)
	// and read both converted data from the Common Data register (ADCx_CDR).

#ifdef STEP_CHANGE_SIM
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->AFR[1] &= ~(uint32_t)(GPIO_AFRH_AFRH0);
	GPIOB->PUPDR &= GPIO_NOPULL << GPIO_PUPDR_PUPD8_Pos;
	GPIOB->MODER = 0;
	GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE8_Pos;
	GPIOB->ODR = 0x0000;
#endif
}

void ADC_Enable(void)
{
	// Clear the ADRDY bit in the ADC1 ISR register by writing '1'
	ADC1->ISR |= ADC_ISR_ADRDY;	// flag cleared by software
	// Clear the ADRDY bit in the ADC2 ISR register by writing '1'
	ADC2->ISR |= ADC_ISR_ADRDY;	// flag cleared by software

	// Start ADC1 & ADC2 enable procedure
	// (required t_STAB delay - power-up time for ADC stabilization = 1 conversion cycle - 15 clock cycles)
	ADC1->CR |= ADC_CR_ADEN;	// set the bit - enable ADC1
	ADC2->CR |= ADC_CR_ADEN;	// set the bit - enable ADC2
	// Wait until ADC1/ADC2 is ready for operation (ADRDY is set after the ADC start-up time)
	// ADRDY flag is set by hardware as soon as ADC is ready for operation
	while(!((ADC1->ISR & ADC_ISR_ADRDY) && (ADC2->ISR & ADC_ISR_ADRDY)));

	// Clear the ADRDY bit in the ADC1 ISR register by writing '1' (optional)
	ADC1->ISR |= ADC_ISR_ADRDY;	// flag cleared by software
	// Clear the ADRDY bit in the ADC2 ISR register by writing '1' (optional)
	ADC2->ISR |= ADC_ISR_ADRDY;	// flag cleared by software
}

void ADC_EnableOverrunMonitor(void)
{
	// OVR flag notifies of buffer overrun event when regular converted data was not read
	// by CPU before new converted data became available
	// (OVR flag is set if the OEC flag is still 1 at the time when a new conversion completes)

	// Enable the ADC1 & ADC2 overrun interrupt
	ADC1->IER |= ADC_IER_OVRIE;
	ADC2->IER |= ADC_IER_OVRIE;
}

void ADC_MeasStart(void)
{
	// Measure V_REFINT before start of the measurement
	ADC_MeasVREF();	// determine value of V_DDA

	// Enable TIM2 timer
	TIM2->CR1 |= TIM_CR1_CEN;

	// Start regular conversions
	ADC1->CR |= ADC_CR_ADSTART;	// set the bit
	// Because hardware trigger is selected:
	//  - conversion will start at the next active edge of the selected regular hardware trigger
	//  - ADSTART is not cleared by hardware - software doesn't need to reset ADSTART again
	//    for next hardware trigger event (no hardware triggers are missed)
	// [[ MASTER ONLY ]]
}

#define FULL_SCALE_12BIT 4095u
volatile uint32_t ADC_DATA_COMMON = 0;
volatile int16_t CH1_DATA = 0, CH2_DATA = 0;	// int16 with 15 bit resolution is enough to store 12-bit conversion result
volatile float V_DDA = 0.0f;
#ifdef STEP_CHANGE_SIM
uint32_t measCnt = 0;
uint32_t totalCnt = 0;
#endif

// required: NVIC_SetPriority(ADC1_2_IRQn, 0b0100);
void ADC1_2_IRQHandler(void)
{
	if (ADC1->ISR & ADC_ISR_EOS)	// if ADC1 EOS (end of regular sequence) flag caused interrupt
	{
		ADC1->ISR |= ADC_ISR_EOS;	// clear ADC1 EOS flag by writing '1' to it
		ADC2->ISR |= ADC_ISR_EOS;	// clear ADC2 EOS flag by writing '1' to it

		// Reading ADC1 & ADC2 common data register
		ADC_DATA_COMMON = ADC123_COMMON->CDR;
		// both EOC flags are cleared when reading the ADCx_CDR register.

		// Reading ADC1 CH1 conversion result
		CH1_DATA = (int16_t)(ADC_DATA_COMMON);
#ifdef CH1_DAC_SNIFFER
		DAC1->DHR12R2 = (uint16_t)CH1_DATA;
		// DAC1 channel2 12-bit right-aligned data holding register
#endif

		// Reading ADC2 CH2 conversion result
		CH2_DATA = (int16_t)(ADC_DATA_COMMON >> 16);
#ifdef CH2_DAC_SNIFFER
		DAC1->DHR12R2 = (uint16_t)CH2_DATA;
		// DAC1 channel2 12-bit right-aligned data holding register
#endif

		// V_CHANNEL = (V_DDA / FULL_SCALE) * ADC_DATA
		ui = ((float)V_DDA / (float)FULL_SCALE_12BIT) * (float)(CH1_DATA-CH2_DATA);	// value can be negative
		uu = ((float)V_DDA / (float)FULL_SCALE_12BIT) * (float)(CH2_DATA);

		if (measProcInProgress) {
			errorToReport = PROCESSING_UNDERRUN;
			putRequest(HANDLE_ERROR);
		}
		putRequest(PROC_MEAS_RESULTS);

#ifdef STEP_CHANGE_SIM
		if ( measCnt < 40 )
			measCnt++;
		else {
			measCnt = 0;
			putRequest(SEND_MEAS_REULTS);
		}

		if ( totalCnt == 319245 ) {
			GPIOB->ODR = 0x0100;
		}
		totalCnt++;
#endif
	}

	if (ADC1->ISR & ADC_ISR_OVR)	// if ADC1 OVR (overrun) flag caused interrupt
	{
		ADC1->ISR |= ADC_ISR_OVR;	// clear ADC1 OVR flag by writing '1' to it
		errorToReport = ADC1_OVERRUN;
		putRequest(HANDLE_ERROR);
	}
	if (ADC2->ISR & ADC_ISR_OVR)	// if ADC2 OVR (overrun) flag caused interrupt
	{
		ADC2->ISR |= ADC_ISR_OVR;	// clear ADC2 OVR flag by writing '1' to it
		errorToReport = ADC2_OVERRUN;
		putRequest(HANDLE_ERROR);
	}
	// when overrun condition occurs, ADC is still operating and continue to convert
	// unless the software decides to stop and reset the sequence by setting ADCSTP=1
}

void ADC_Disable(void)
{
	// Stop running conversions by setting ADSTP=1
	// (first check is conversions are running)
	if ((ADC1->CR & ADC_CR_ADSTART) || (ADC2->CR & ADC_CR_ADSTART)) {
		ADC1->CR |= ADC_CR_ADSTP;
		// [[ MASTER ONLY ]]
	}

	// Wait until ADSTART=0 and ADSTP=0	(both bits are cleared by hardware)
	while((ADC1->CR & ADC_CR_ADSTART) || (ADC1->CR & ADC_CR_ADSTP) ||
			(ADC2->CR & ADC_CR_ADSTART) || (ADC2->CR & ADC_CR_ADSTP));

	// Start ADC1 disable procedure
	ADC1->CR |= ADC_CR_ADDIS;	// set the bit - disable ADC1
	// Start ADC2 disable procedure
	ADC2->CR |= ADC_CR_ADDIS;	// set the bit - disable ADC2

	// Wait until disable procedure is finished
	// (ADEN and ADDIS are automatically cleared by hardware as soon as ADC is effectively disabled)
	while((ADC1->CR & ADC_CR_ADEN) || (ADC1->CR & ADC_CR_ADDIS) ||
			(ADC2->CR & ADC_CR_ADEN) || (ADC2->CR & ADC_CR_ADDIS));

#ifdef STEP_CHANGE_SIM
	measCnt = 0;
	totalCnt = 0;
	GPIOB->ODR = 0x0000;
#endif
}

void ADC_Deinit(void)
{
	ADC_Disable();

	// Disable the ADC1 & ADC2 internal voltage regulator
	// (internal analog calibration is kept - no need to re-apply the calibration)
	ADC1->CR &= ~(uint32_t)(ADC_CR_ADVREGEN);	// clear the bit to disable voltage regulator
	ADC2->CR &= ~(uint32_t)(ADC_CR_ADVREGEN);	// clear the bit to disable voltage regulator

	// Disable TIM2 timer
	TIM2->CR1 &= ~(uint32_t)(TIM_CR1_CEN);
}

#define VREFINT_CAL_ADDR ((uint16_t*)((uint32_t)0x1FFF75AA)) // according to STM32L476xx datasheet (val = 1654)
// Internal reference voltage V_REFINT is connected to ADC1 CH0
// VREFEN analog source must be enabled by programming bit VREFEN in ADC1->CCR
// Reference voltage monitoring: it is recommended to recalibrate when reference voltage V_REF+ changed more then 10%
void ADC_MeasVREF(void)
{
	// implement VREF (~1.2V) measurement as single injected conversion with software trigger

	// ADC1 must be enabled before

	// Configure injected conversion group for ADC1
	// set CH0 as the 1st conversion in injected sequence
	ADC1->JSQR &= ~(uint32_t)(ADC_JSQR_JSQ1);
	ADC1->JSQR |= 0 << ADC_JSQR_JSQ1_Pos;
	// set total number of conversions in the injected channel conversion sequence to 1
	ADC1->JSQR &= ~(uint32_t)(ADC_JSQR_JL);	// 0b00 - convert single channel

	// Disable external trigger for injected group
	ADC1->JSQR &= ~(uint32_t)(ADC_JSQR_JEXTEN);

	// Configure ADC1 CH0 sampling time
	// Minimal required sampling time for VREFINT measurement is equal to 4 us (320 80 MHz ADC clock cycles)
	ADC1->SMPR1 &= ~(uint32_t)(ADC_SMPR1_SMP0);	// set SMP[2:0] to 0b111 (640.5 ADC clock cycles)

	// Implement averaging through oversampling:
	// (The oversampling ratio N is defined using the OVSR[2:0] bits in the ADCx_CFGR2 register,
	// and can range from 2x to 256x. The division coefficient M consists of a right bit shift
	// up to 8 bits, and is defined using the OVSS[3:0] bits in the ADCx_CFGR2 register.)
	ADC1->CFGR2 &= ~(uint32_t)(ADC_CFGR2_OVSR);
	ADC1->CFGR2 |= 0b100 << ADC_CFGR2_OVSR_Pos;		// set oversampling ratio 32x
	ADC1->CFGR2 &= ~(uint32_t)(ADC_CFGR2_OVSS);
	ADC1->CFGR2 |= 0b0100 << ADC_CFGR2_OVSS_Pos;	// set 4-bit shift
	// (oversampling ratio 32x + 4-bit shift => output value range 13-bit)
	ADC1->CFGR2 |= ADC_CFGR2_JOVSE;	// enable injected oversampling

	// set VREFEN bit in CCR register to enable the conversion of V_REFINT
	ADC123_COMMON->CCR |= ADC_CCR_VREFEN;
	delayUS_DWT(15);	// wait minimum 12 us for VREFINT source to stabilize

	// Start injected conversions
	ADC1->CR |= ADC_CR_JADSTART;	// set the bit
	// End of conversions implemented in ADC ISR

	while(!(ADC1->ISR & ADC_ISR_JEOS));

	ADC1->ISR |= ADC_ISR_JEOS;	// clear ADC1 JEOS flag by writing '1' to it

	// V_DDA = 3.0V ⋅ VREFINT_CAL / VREFINT_DATA
	//uint16_t VREFINT_CAL = 1422;	// (*VREFINT_CAL_ADDR) (for 12-bit resolution)
	//uint16_t VREFINT_CAL = 2972;	// ~2*(*VREFINT_CAL_ADDR) (for resolution increased to 13-bit)

	uint16_t VREFINT_CAL = 2886;

	uint16_t VREFINT_DATA = ADC1->JDR1;	// clear ADC1 JEOC flag
	V_DDA = 3.0f * (float)(VREFINT_CAL) / (float)(VREFINT_DATA);

	// stop running injected conversions by setting JADSTP=1
	ADC1->CR |= ADC_CR_JADSTP;
	// clear VREFEN bit in CCR register to disable V_REFINT source
	ADC123_COMMON->CCR &= ~(uint32_t)(ADC_CCR_VREFEN);
}

void ADC_DACOutput(void)
{
	// DAC ---------------------------------------------------------------

	// DAC1 clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

	// disable DAC1 channel 2
	DAC1->CR &= ~(uint32_t)(DAC_CR_EN2);

	// DAC1 channel2 triggered by TIM2_TRGO
	//DAC1->CR &= ~(uint32_t)(DAC_CR_TSEL2_2);	// 0b100
	// DAC1 channel2 triggered by TIM6_TRGO
	DAC1->CR &= ~(uint32_t)(DAC_CR_TSEL2_2 | DAC_CR_TSEL2_1 | DAC_CR_TSEL2_0);	// 0b111

	// DAC channel2 trigger enable
	DAC1->CR |= DAC_CR_TEN2;

	// DAC Channel 2 in normal mode connected to external pin with buffer enabled
	DAC1->MCR &= ~(DAC_MCR_MODE2_2 | DAC_MCR_MODE2_1 | DAC_MCR_MODE2_0);

	// GPIO --------------------------------------------------------------

	// For DAC configure the desired I/O in analog mode in the GPIOx_MODER
	// register and configure the required function in DAC registers.

	// PA5 (DAC1_OUT2) - [D13]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	// GPIOA clock enable

	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL5);				// port A, pin 5, no alternative function

	GPIOA->PUPDR &= GPIO_NOPULL << GPIO_PUPDR_PUPD5_Pos;		// 0b00 << 8 // port A, pin 5, no pull-up no pull-down

	GPIOA->MODER |= GPIO_MODE_ANALOG << GPIO_MODER_MODE5_Pos;	// 0b11 << 8 // port A, pin 5, analog mode

	// START DAC OUTPUT

	DAC1->CR |= DAC_CR_EN2;					// enable DAC1 CH2
}
