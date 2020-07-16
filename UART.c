/*
 * UART.c
 *
 *  Created on: 26.07.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

#include "main.h"

volatile bool transferInProgress = false;

// USART INITIALIZATION ----------------------------------------------------------------------------------

void USART_Init(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // USART2 clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	// GPIOA clock enable

	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL2);
	GPIOA->AFR[0] &= ~(uint32_t)(GPIO_AFRL_AFRL3);
	GPIOA->AFR[0] |= GPIO_AF7_USART2 << GPIO_AFRL_AFSEL2_Pos; // 0b0111 << 8; // port A, pin 2, alternative function AF7 (USART2_TX)
	GPIOA->AFR[0] |= GPIO_AF7_USART2 << GPIO_AFRL_AFSEL3_Pos; // 0b0111 << 12; // port A, pin 3, alternative function AF7 (USART2_RX)

	GPIOA->PUPDR &= ~(uint32_t)(GPIO_PUPDR_PUPDR3);
	GPIOA->PUPDR |= GPIO_PULLDOWN << GPIO_PUPDR_PUPD3_Pos; // 0b10 << 6; // portA, pin 3, pull-down

	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE2);
	GPIOA->MODER &= ~(uint32_t)(GPIO_MODER_MODE3);
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE2_Pos; // 0b10 << 4; // port A, pin 2, alternate function mode (push-pull)
	GPIOA->MODER |= GPIO_MODE_AF_PP << GPIO_MODER_MODE3_Pos; // 0b10 << 6; // port A, pin 3, alternate function mode (push-pull)

	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED2);
	GPIOA->OSPEEDR &= ~(uint32_t)(GPIO_OSPEEDR_OSPEED3);
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << GPIO_OSPEEDR_OSPEED2_Pos; // 0b11 << 4; // port A, pin 2, very high output speed
	GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << GPIO_OSPEEDR_OSPEED3_Pos; // 0b11 << 6; // port A, pin 3, very high output speed

	// setting word length
	// M bits in USART2_CR1 left "00": 8-bit character length

	// OVER8 bit in USART2_CR1 left "0": oversampling by 16
	// ONEBIT bit in USART2_CR3 left "0": three sampling majority vote method

	// setting baud rate (oversampling by 16):
	// USARTDIV = 80 000 000 Hz / 115200 baud = 694
	USART2->BRR = 694;
	// USARTDIV = 80 000 000 Hz / 230400 baud = 347
	//USART2->BRR = 347;

	// setting stop bits
	// STOP bits in USART2_CR2 left "00": 1 stop bit

	// setting parity control
	// PCE bit in USART2_CR1 left "0": no parity bit
	// PC bit in USART2_CR1 left "0": even parity

	// enable USART
	// USART2->CR1 = 0;
	USART2->CR1 = USART_CR1_UE;

	// RX interrupt enable
	USART2->CR1 |= USART_CR1_RXNEIE;	// RXNE (read register not empty) interrupt enabled

	// enable transmission
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// DMA1 CONFIGURATION -----------------------------------------------------

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;		// DMA1 clock enable

	// MEAS DATA OUTPUT TRANSFER

	// disable DMA1 channel 7
	DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_EN);

	// mapping DMA1 CH7
	DMA1_CSELR->CSELR &= ~(uint32_t)(DMA_CSELR_C7S);
	DMA1_CSELR->CSELR |= DMA_REQUEST_2 << DMA_CSELR_C7S_Pos;	// requests from USART2_TX

	// set peripheral register address
	DMA1_Channel7->CPAR = (uint32_t)&(USART2->TDR);	// TransmitDataRegister of USART2

	// set channel priority
	DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_PL);
	DMA1_Channel7->CCR |= DMA_PRIORITY_LOW << DMA_CCR_PL_Pos;	// 0b00 << 12; // low priority level

	// set data transfer direction
	DMA1_Channel7->CCR |= DMA_CCR_DIR;	// read from memory

	// set circular mode
	DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_CIRC);	// circular mode disabled

	// set peripheral and memory increment mode
	DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_PINC);	// peripheral increment disabled
	DMA1_Channel7->CCR |= DMA_CCR_MINC;					// memory increment enabled

	// set peripheral and memory data size
	DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_PSIZE);
	DMA1_Channel7->CCR |= DMA_PDATAALIGN_BYTE << DMA_CCR_PSIZE_Pos; 	// 0b00 << 8; // 8 bits (byte)
	DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_MSIZE);
	DMA1_Channel7->CCR |= DMA_MDATAALIGN_BYTE << DMA_CCR_MSIZE_Pos; 	// 0b00 << 10; // 8 bits (byte)

	// set interrupt
	DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_TCIE);	// transfer complete interrupt disabled
}

// STDIO CONFIGURATION -----------------------------------------------------------------------------------

void stdio_Setup(void)
{
	// set all standard streams as unbuffered
	// (I/O occurs immediately without waiting for '\n')
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    // source: http://www.openstm32.org/forumthread1055
}

int __io_putchar(int ch) {
    // Code to write character 'ch' on the UART
	USART2->TDR = ch;	// write 'ch' to TransmitDataRegister of USART2
	delayUS_DWT(100);	// added 100 us delay (otherwise does not work!)
	return 0;
}

int __io_getchar(void) {
    // Code to read a character from the UART
	return USART2->RDR;	// read ReceiveDataRegister of USART2
}

// SEND MEASUREMENT RESULTS -------------------------------------------------------------------------------

void USART2_IRQHandler(void)
{
	if (USART2->ISR & USART_ISR_RXNE)	// if RXNE (read register not empty) flag caused interrupt
	{
		uint8_t cmd = USART2->RDR;			// read received data
		USART2->RQR |= USART_RQR_RXFRQ;		// clear RXNE (read register not empty) flag

		switch(cmd)
		{
		case START_MEAS_CMD:      // start measurement session
			putRequest(START_MEAS);
			break;
		case END_MEAS_CMD:        // end measurement session
			putRequest(STOP_MEAS);
			break;
		case GET_RESULTS_CMD:     // read measurement output
			USART2->CR1 &= (uint32_t)(~USART_CR1_RXNEIE);	// RXNE (read register not empty) interrupt disabled
			putRequest(SEND_MEAS_REULTS);
			break;
		case IDENTIFY_PARAMS_CMD: // perform parameter identification
			USART2->CR1 &= (uint32_t)(~USART_CR1_RXNEIE);	// RXNE (read register not empty) interrupt disabled
			if (currentIdentificationStage == READY_TO_START) putRequest(RUN_IDENT_PROC);
			break;
		default:
			break;
		}
	}
}

void USART_SendMessage(char* str, uint8_t len)
{
	if (!transferInProgress)
	{
		transferInProgress = true;

		// enable DMA1 channel 7 (USART2_TX)
		USART2->CR3 |= USART_CR3_DMAT;						// DMA mode enabled for transmission (sending requests)
		USART2->ICR |= USART_ICR_TCCF;						// clear TC (transmission complete) flag
		DMA1_Channel7->CMAR = (uint32_t)(str);				// set memory address
		DMA1_Channel7->CNDTR = len;							// number of chars to transfer
		DMA1_Channel7->CCR |= DMA_CCR_EN;					// activate DMA1 CH7 (USART2_TX)

		DMA1_Channel7->CCR |= DMA_CCR_TCIE;				// transfer complete interrupt enabled
	}
}

// SEND ERROR INFORMATION -------------------------------------------------------------------------------

ErrorTypes errorToReport = 0;

#define MAX_NOTIFICATION_LEN 32
char errorStrings[ERROR_NUM][MAX_NOTIFICATION_LEN] =
{
	"ERROR: DAC DMA underrun\n\r",
	"ERROR: ADC1 overrun\n\r",
	"ERROR: ADC2 overrun\n\r",
	"ERROR: Processing underrun\n\r",
	"ERROR: Buffer switch underrun\n\r"
};

void USART_NotifyAboutError()
{
	if (errorToReport < ERROR_NUM) {
		USART_SendMessage((char *)&errorStrings[errorToReport], MAX_NOTIFICATION_LEN);
	}
}

// COMMON DMA TRANSFER COMPLETE ISR ----------------------------------------------------------------------

void DMA1_Channel7_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF7)			// if RXNE TC (transfer complete) flag caused interrupt
	{
		transferInProgress = false;

		DMA1->IFCR |= DMA_IFCR_CTCIF7;					// transfer complete flag clear for channel 7
		DMA1_Channel7->CCR &= ~(uint32_t)(DMA_CCR_EN);  // disable DMA1 CH7 (USART2_TX)
		USART2->CR3 &= ~(uint32_t)(USART_CR3_DMAT);		// DMA mode disabled for transmission (stop sending DMA requests)
		USART2->CR1 |= USART_CR1_RXNEIE;				// RXNE (read register not empty) interrupt enabled
	}
}
