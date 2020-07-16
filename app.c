/*
 * app.c
 *
 *  Created on: 31.08.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

#include "main.h"

float doublePI = 0;	// initialized in appInit()

float optimalFrequencies[NUM_OF_CHANNELS]  = { OPT_FREQ_1, OPT_FREQ_2, OPT_FREQ_3, OPT_FREQ_4 };
float optimalOmegas[NUM_OF_CHANNELS]       = { 0 };

uint32_t MAFiltersLengths[NUM_OF_CHANNELS] = { MA_FILT_LEN_1, MA_FILT_LEN_2, MA_FILT_LEN_3, MA_FILT_LEN_4 };
complex float invMAFiltersLengths[NUM_OF_CHANNELS] = { 1.219512195121951E-04, 6.147037128104254E-05, 1.491268622216920E-05, 5.011827913876749E-06 };

volatile float uu = 0;
volatile float ui = 0;
volatile float avgUu = 0;
volatile float avgUi = 0;

complex float filteredSignals[NUM_OF_BANKS][NUM_OF_CHANNELS] = {0};
complex float averagedSignals[NUM_OF_BANKS] = {0};

void APP_Init()
{
	doublePI = 2*acos(-1);
	for (uint8_t channel = e1stChannel; channel < NUM_OF_CHANNELS; channel++) {
		optimalOmegas[channel] = doublePI * optimalFrequencies[channel];
		invMAFiltersLengths[channel] = 1/(complex float)MAFiltersLengths[channel];
	}
}

// MEAS DATA PROCESSING --------------------------------------------------------------------

volatile bool measProcInProgress = false;

complex float y_last[NUM_OF_BANKS][NUM_OF_CHANNELS]; // last output value of every channel of every bank
uint32_t filtrSmplIdx[NUM_OF_CHANNELS] = { 0, 0, 0, 0 };

void APP_ResetMAFiltersInit()
{
	for (uint8_t bank = 0; bank < NUM_OF_BANKS; bank++) {
		for(uint8_t channel = 0; channel < NUM_OF_CHANNELS; channel++) {
			filteredSignals[bank][channel] = 0;
		}
		averagedSignals[bank] = 0;
	}
	for (uint8_t channel = 0; channel < NUM_OF_CHANNELS; channel++) {
		y_last[e1stBank][channel] = 0;
		y_last[e2ndBank][channel] = 0;
		filtrSmplIdx[channel] = 0;
	}
}

void APP_ResetMAFilters()
{
	static uint32_t argument1 = 0, argument2 = 0;
	static complex float modulator = 0, bipolar_uu = 0, bipolar_ui = 0, input_uu = 0, input_ui = 0;

	measProcInProgress = true;

	bipolar_uu = (complex float)(uu-avgUu);
	bipolar_ui = (complex float)(ui-avgUi);

	// --- FILTER 1 --------------------------------------------------

	// e^(- I * x) = cos(x) - I * sin(x)
	argument1 = (filtrSmplIdx[e1stChannel]+COS1_OFFSET) % SIN1_LEN;
	argument2 = (filtrSmplIdx[e1stChannel])             % SIN1_LEN;
	modulator = sin1[argument1] - I * sin1[argument2];

	input_uu = bipolar_uu * modulator;
	input_ui = bipolar_ui * modulator;

	y_last[e1stBank][e1stChannel] += input_uu*invMAFiltersLengths[e1stChannel];
	y_last[e2ndBank][e1stChannel] += input_ui*invMAFiltersLengths[e1stChannel];

	filtrSmplIdx[e1stChannel]++;

	if ( filtrSmplIdx[e1stChannel] == MAFiltersLengths[e1stChannel] )
	{
		filteredSignals[e1stBank][e1stChannel] = y_last[e1stBank][e1stChannel];
		filteredSignals[e2ndBank][e1stChannel] = y_last[e2ndBank][e1stChannel];
		y_last[e1stBank][e1stChannel] = 0;
		y_last[e2ndBank][e1stChannel] = 0;
		filtrSmplIdx[e1stChannel] = 0;
	}

	// --- FREQ 2 --------------------------------------------------

	// e^(- I * x) = cos(x) - I * sin(x)
	argument1 = (filtrSmplIdx[e2ndChannel]+COS2_OFFSET) % SIN2_LEN;
	argument2 = (filtrSmplIdx[e2ndChannel])             % SIN2_LEN;
	modulator = sin2[argument1] - I * sin2[argument2];

	input_uu = bipolar_uu * modulator;
	input_ui = bipolar_ui * modulator;

	y_last[e1stBank][e2ndChannel] += input_uu*invMAFiltersLengths[e2ndChannel];
	y_last[e2ndBank][e2ndChannel] += input_ui*invMAFiltersLengths[e2ndChannel];

	filtrSmplIdx[e2ndChannel]++;

	if ( filtrSmplIdx[e2ndChannel] == MAFiltersLengths[e2ndChannel] )
	{
		filteredSignals[e1stBank][e2ndChannel] = y_last[e1stBank][e2ndChannel];
		filteredSignals[e2ndBank][e2ndChannel] = y_last[e2ndBank][e2ndChannel];
		y_last[e1stBank][e2ndChannel] = 0;
		y_last[e2ndBank][e2ndChannel] = 0;
		filtrSmplIdx[e2ndChannel] = 0;
	}

	// --- FREQ 3 --------------------------------------------------

	// e^(- I * x) = cos(x) - I * sin(x)
	argument1 = (filtrSmplIdx[e3rdChannel]+COS3_OFFSET) % SIN3_LEN;
	argument2 = (filtrSmplIdx[e3rdChannel])             % SIN3_LEN;
	modulator = sin3[argument1] - I * sin3[argument2];

	input_uu = bipolar_uu * modulator;
	input_ui = bipolar_ui * modulator;

	y_last[e1stBank][e3rdChannel] += input_uu*invMAFiltersLengths[e3rdChannel];
	y_last[e2ndBank][e3rdChannel] += input_ui*invMAFiltersLengths[e3rdChannel];

	filtrSmplIdx[e3rdChannel]++;

	if ( filtrSmplIdx[e3rdChannel] == MAFiltersLengths[e3rdChannel] )
	{
		filteredSignals[e1stBank][e3rdChannel] = y_last[e1stBank][e3rdChannel];
		filteredSignals[e2ndBank][e3rdChannel] = y_last[e2ndBank][e3rdChannel];
		y_last[e1stBank][e3rdChannel] = 0;
		y_last[e2ndBank][e3rdChannel] = 0;
		filtrSmplIdx[e3rdChannel] = 0;
	}

	// --- FREQ 4 & AVERAGER ---------------------------------------

	// e^(- I * x) = cos(x) - I * sin(x)
	argument1 = (filtrSmplIdx[e4thChannel]+COS4_OFFSET) % SIN4_LEN;
	argument2 = (filtrSmplIdx[e4thChannel])             % SIN4_LEN;
	modulator = sin4[argument1] - I * sin4[argument2];

	input_uu = bipolar_uu * modulator;
	input_ui = bipolar_ui * modulator;

	y_last[e1stBank][e4thChannel] += input_uu*invMAFiltersLengths[e4thChannel];
	y_last[e2ndBank][e4thChannel] += input_ui*invMAFiltersLengths[e4thChannel];
	averagedSignals[e1stBank] += uu*invMAFiltersLengths[e4thChannel];
	averagedSignals[e2ndBank] += ui*invMAFiltersLengths[e4thChannel];

	filtrSmplIdx[e4thChannel]++;

	if ( filtrSmplIdx[e4thChannel] == MAFiltersLengths[e4thChannel] )
	{
		filteredSignals[e1stBank][e4thChannel] = y_last[e1stBank][e4thChannel];
		filteredSignals[e2ndBank][e4thChannel] = y_last[e2ndBank][e4thChannel];
		y_last[e1stBank][e4thChannel] = 0;
		y_last[e2ndBank][e4thChannel] = 0;
		avgUu = crealf(averagedSignals[e1stBank]);
		avgUi = crealf(averagedSignals[e2ndBank]);
		averagedSignals[e1stBank] = 0;
		averagedSignals[e2ndBank] = 0;
		filtrSmplIdx[e4thChannel] = 0;
	}

	// ---------------------------------------------------------------

	measProcInProgress = false;
}

// IDENTIFICATION --------------------------------------------------------------------------

complex float measZ[NUM_OF_CHANNELS] = {0};
complex float resultsUu[NUM_OF_CHANNELS] = {0};
complex float resultsUi[NUM_OF_CHANNELS] = {0};

#define MEAS_RESISTANCE 9800	// [Ohm]
#define IDENT_STEPS 5

#define INIT_Cc 0.0000001f
#define INIT_Rp 100000.0f
#define INIT_Cdl 0.000001f
#define INIT_Rct 100000.0f

inline static void cGetABCD(uint32_t paramIdx, float* paramVec, complex float* coef)
{
    float Cc = paramVec[0];
    float Rp = paramVec[1];
    float Cdl = paramVec[2];
    float Rct = paramVec[3];

    float omega = optimalOmegas[paramIdx];

    switch (paramIdx) {
        case 0:       // param 1 = Cc
            // A1:
            coef[0] = 0;
            // B1:
            coef[1] = I*omega*Cdl*Rct*Rp+Rp+Rct;
            // C1:
            coef[2] = -omega*omega*Cdl*Rct*Rp+I*omega*(Rp+Rct);
            // D1:
            coef[3] = I*omega*Cdl*Rct+1;
            break;
        case 1:       // param 2 = Rp
            // A2:
            coef[0] = 1 + I*omega*Cdl*Rct;
            // B2:
            coef[1] = Rct;
            // C2:
            coef[2] = I*omega*Cc - omega*omega*Cc*Cdl*Rct;
            // D2:
            coef[3] = 1 + I*omega*Rct*(Cdl+Cc);
            break;
        case 2:        // param 3 = Cdl
            // A3:
            coef[0] = I*omega*Rct*Rp;
            // B3:
            coef[1] = Rp+Rct;
            // C3:
            coef[2] = -omega*omega*Cc*Rct*Rp+I*omega*Rct;
            // D3:
            coef[3] = I*omega*Cc*(Rp+Rct) + 1;
            break;
        case 3:        // param 4 = Rct
            // A4:
            coef[0] = I*omega*Cdl*Rp + 1;
            // B4:
            coef[1] = Rp;
            // C4:
            coef[2] = -omega*omega*Cc*Cdl*Rp + I*omega*(Cc+Cdl);
            // D4:
            coef[3] = I*omega*Cc*Rp + 1;
            break;
        default:
            break;
    }
}

volatile identificationStage currentIdentificationStage = READY_TO_START;
float identParams[NUM_OF_CHANNELS] =  {INIT_Cc, INIT_Rp, INIT_Cdl, INIT_Rct};
float identErrors[NUM_OF_CHANNELS] =  {0};

char numbers[10] = {'0','1','2','3','4','5','6','7','8','9'};

char identOutputString[120] = {'P','a','r','a','m','x',':',' ',' ',' ',' ',' ',' ','.',' ',' ',' ','e','r','r',':',' ',' ',' ','.',' ',' ','%','\n','\r',
		'P','a','r','a','m','x',':',' ',' ',' ',' ',' ',' ','.',' ',' ',' ','e','r','r',':',' ',' ',' ','.',' ',' ','%','\n','\r',
		'P','a','r','a','m','x',':',' ',' ',' ',' ',' ',' ','.',' ',' ',' ','e','r','r',':',' ',' ',' ','.',' ',' ','%','\n','\r',
		'P','a','r','a','m','x',':',' ',' ',' ',' ',' ',' ','.',' ',' ',' ','e','r','r',':',' ',' ',' ','.',' ',' ','%','\n','\r'};


void APP_RunIdentificationIteration()
{
	static float properParams[NUM_OF_CHANNELS] =  {0.000000206782f, 9465.29f, 0.00000183801f, 9823.92f};
	static complex float ABCD[NUM_OF_CHANNELS] =  {0};
	static uint8_t channelIdx = 0, identStep = 0, elemIdx = 0, reversedBilinearTransofmStep = 0;
	static complex float tmp = 0;

	switch (currentIdentificationStage)
	{

	case READY_TO_START:
		currentIdentificationStage = CAPTURE_RESULTS;
		putRequest(RUN_IDENT_PROC);
		break;

	case CAPTURE_RESULTS:
		for (uint8_t channel = e1stChannel; channel < NUM_OF_CHANNELS; channel++)
		{
			resultsUu[channel] = filteredSignals[e1stBank][channel];
			resultsUi[channel] = filteredSignals[e2ndBank][channel];
		}
		currentIdentificationStage = CALC_MEASURED_IMPEDANCES;
		putRequest(RUN_IDENT_PROC);
		break;

	case CALC_MEASURED_IMPEDANCES:
		if (channelIdx < NUM_OF_CHANNELS)	// iterate over filters
		{
			measZ[channelIdx] = resultsUu[channelIdx] / (resultsUi[channelIdx]/(complex float)MEAS_RESISTANCE);
			channelIdx++;
		}
		else
		{
			channelIdx = 0;
			currentIdentificationStage = IDENTIFY_PARAMS;
		}
		putRequest(RUN_IDENT_PROC);
		break;

	case IDENTIFY_PARAMS:
		if (identStep < IDENT_STEPS)		// perform certain number of identification steps
		{
			if (elemIdx < NUM_OF_CHANNELS)	// iterate over the elements of the model
			{
				switch (reversedBilinearTransofmStep)
				{
				case 0:
					cGetABCD(elemIdx, identParams, ABCD);
					reversedBilinearTransofmStep++;
					break;
				case 1:
					tmp = (ABCD[3]*measZ[elemIdx] - ABCD[1])/(ABCD[0] - ABCD[2]*measZ[elemIdx]);
					reversedBilinearTransofmStep++;
					break;
				default:
					identParams[elemIdx] = cabsf(tmp);
					reversedBilinearTransofmStep = 0;
					elemIdx++;
					break;
				}
			}
			else
			{
				elemIdx = 0;
				identStep++;
			}
		}
		else
		{
			identStep = 0;
			currentIdentificationStage = CONVERT_PARAM_TO_STRING;
		}
		putRequest(RUN_IDENT_PROC);
		break;

	case CONVERT_PARAM_TO_STRING:
		if (elemIdx < NUM_OF_CHANNELS)	// iterate over the elements of the model
		{
			float value = identParams[elemIdx];

			// express capacities in nF
			if (!(elemIdx % 2)) value *= 1000000000;

			uint8_t digit  = 0;
			char decimal[8] = {0};

			digit = (uint8_t)(value/10000);
			decimal[0] = numbers[digit];
			value -= digit * 10000;
			digit = (uint8_t)(value/1000);
			decimal[1] = numbers[digit];
			value -= digit * 1000;
			digit = (uint8_t)(value/100);
			decimal[2] = numbers[digit];
			value -= digit * 100;
			digit = (uint8_t)(value/10);
			decimal[3] = numbers[digit];
			value -= digit * 10;

			digit = (uint8_t)(value);
			decimal[4] = numbers[digit];
			value -= digit;
			decimal[5] = '.';
			value *= 10;	// extract 0.x0
			digit = (uint8_t)(value);
			decimal[6] = numbers[digit];
			value -= digit;
			value *= 10;	// extract 0.0x
			digit = (uint8_t)(value);
			value -= digit;	// round last digit
			value *= 10;
			if (value >= 5) digit++;
			decimal[7] = numbers[digit];
			for (uint8_t d = 0; d < 4; d++) { // remove 0's at the beginning
				if (decimal[d] == '0') decimal[d] = ' ';
				else break;
			}

			identOutputString[elemIdx*30+5] = numbers[elemIdx+1];
			// display 00000.00
			for (uint8_t k = 0; k < 8; k++) identOutputString[elemIdx*30+8+k] = decimal[k];

			elemIdx++;
		}
		else
		{
			elemIdx = 0;
			currentIdentificationStage = CALC_IDENT_ERRORS;
		}
		putRequest(RUN_IDENT_PROC);
		break;

	case CALC_IDENT_ERRORS:
		if (elemIdx < NUM_OF_CHANNELS)	// iterate over the elements of the model
		{
			identErrors[elemIdx] = 100.0f*fabs(identParams[elemIdx]-properParams[elemIdx])/properParams[elemIdx];
			elemIdx++;
		}
		else
		{
			elemIdx = 0;
			currentIdentificationStage = CONVERT_ERROR_TO_STRING;
		}
		putRequest(RUN_IDENT_PROC);
		break;

	case CONVERT_ERROR_TO_STRING:
		if (elemIdx < NUM_OF_CHANNELS)	// iterate over the elements of the model
		{
			float value = identErrors[elemIdx];
			uint8_t digit  = 0;
			char decimal[8] = {0};

			digit = (uint8_t)(value/10000);
			decimal[0] = numbers[digit];
			value -= digit * 10000;
			digit = (uint8_t)(value/1000);
			decimal[1] = numbers[digit];
			value -= digit * 1000;
			digit = (uint8_t)(value/100);
			decimal[2] = numbers[digit];
			value -= digit * 100;
			digit = (uint8_t)(value/10);
			decimal[3] = numbers[digit];
			value -= digit * 10;

			digit = (uint8_t)(value);
			decimal[4] = numbers[digit];
			value -= digit;
			decimal[5] = '.';
			value *= 10;	// extract 0.x0
			digit = (uint8_t)(value);
			decimal[6] = numbers[digit];
			value -= digit;
			value *= 10;	// extract 0.0x
			digit = (uint8_t)(value);
			value -= digit;	// round last digit
			value *= 10;
			if (value >= 5) digit++;
			decimal[7] = numbers[digit];
			for (uint8_t d = 0; d < 4; d++) { // remove 0's at the beginning
				if (decimal[d] == '0') decimal[d] = ' ';
				else break;
			}

			// display only 00.00
			for (uint8_t k = 0; k < 5; k++) identOutputString[elemIdx*30+22+k] = decimal[k+3];

			elemIdx++;
		}
		else
		{
			elemIdx = 0;
			currentIdentificationStage = SEND_IDENT_DATA;
		}
		putRequest(RUN_IDENT_PROC);
		break;

	case SEND_IDENT_DATA:
		USART_SendMessage(identOutputString, 120);
		currentIdentificationStage = READY_TO_START;
		break;

	default:
		channelIdx = 0;
		identStep = 0;
		elemIdx = 0;
		currentIdentificationStage = READY_TO_START;
		break;

	}
}

void APP_ResetIdentParams()
{
	identParams[0] = INIT_Cc;
	identParams[1] = INIT_Rp;
	identParams[2] = INIT_Cdl;
	identParams[3] = INIT_Rct;
}

// SEND MEAS RESULTS -----------------------------------------------------------------------

typedef enum eSendResultsStages
{
	CALCULATE_IMPEDANCE,
	CALCULATE_ABS_VAL,
	CONVERT_TO_STRING_1,
	CONVERT_TO_STRING_2,
	CREATE_OUTPUT_STRING,
	START_DMA_TRANSFER
} SendResultsStages;

SendResultsStages sendResultsStage = CALCULATE_IMPEDANCE;

#ifdef PYTHON_RESULTS
#ifdef IMPEDANCE_RESULTS
float results[NUM_OF_CHANNELS] = {0};
#endif
#ifdef FILTRATION_RESULTS
float results[2*NUM_OF_CHANNELS] = {0};
#endif
#else
char measOutputString[56] = {'Z','x',':',' ',' ',' ',' ',' ',' ','.',' ',' ','\n','\r',
						 	 'Z','x',':',' ',' ',' ',' ',' ',' ','.',' ',' ','\n','\r',
							 'Z','x',':',' ',' ',' ',' ',' ',' ','.',' ',' ','\n','\r',
							 'Z','x',':',' ',' ',' ',' ',' ',' ','.',' ',' ','\n','\r'};
#endif

void APP_SendMeasResultsIteration()
{
#ifdef PYTHON_RESULTS
#ifdef IMPEDANCE_RESULTS
	measZ[e1stChannel] = (complex float)MEAS_RESISTANCE * ( filteredSignals[e1stBank][e1stChannel] / filteredSignals[e2ndBank][e1stChannel] );
	results[e1stChannel] = cabsf(measZ[e1stChannel]);
	measZ[e2ndChannel] = (complex float)MEAS_RESISTANCE * ( filteredSignals[e1stBank][e2ndChannel] / filteredSignals[e2ndBank][e2ndChannel] );
	results[e2ndChannel] = cabsf(measZ[e2ndChannel]);
	measZ[e3rdChannel] = (complex float)MEAS_RESISTANCE * ( filteredSignals[e1stBank][e3rdChannel] / filteredSignals[e2ndBank][e3rdChannel] );
	results[e3rdChannel] = cabsf(measZ[e3rdChannel]);
	measZ[e4thChannel] = (complex float)MEAS_RESISTANCE * ( filteredSignals[e1stBank][e4thChannel] / filteredSignals[e2ndBank][e4thChannel] );
	results[e4thChannel] = cabsf(measZ[e4thChannel]);
	USART_SendMessage( (char *)results, sizeof(results) );
#endif
#ifdef FILTRATION_RESULTS
	results[0] = cabsf( filteredSignals[e1stBank][e1stChannel] );
	results[1] = cabsf( filteredSignals[e1stBank][e2ndChannel] );
	results[2] = cabsf( filteredSignals[e1stBank][e3rdChannel] );
	results[3] = cabsf( filteredSignals[e1stBank][e4thChannel] );

	results[4] = cabsf( filteredSignals[e2ndBank][e1stChannel] );
	results[5] = cabsf( filteredSignals[e2ndBank][e2ndChannel] );
	results[6] = cabsf( filteredSignals[e2ndBank][e3rdChannel] );
	results[7] = cabsf( filteredSignals[e2ndBank][e4thChannel] );

	USART_SendMessage( (char *)results, sizeof(results) );
#endif
#else
	static uint8_t channel = 0;
	static float result = 0.0f;
	static uint8_t digit = 0;
	static char decimal[8] = {0};

	switch (sendResultsStage)
	{
	case CALCULATE_IMPEDANCE:
		resultsUu[channel] = filteredSignals[e1stBank][channel];
		resultsUi[channel] = filteredSignals[e2ndBank][channel];
		measZ[channel] = (complex float)MEAS_RESISTANCE * ( resultsUu[channel] / resultsUi[channel] );
		sendResultsStage = CALCULATE_ABS_VAL;
		putRequest(SEND_MEAS_REULTS);
		break;
	case CALCULATE_ABS_VAL:
		result = cabsf(measZ[channel]);
		sendResultsStage = CONVERT_TO_STRING_1;
		putRequest(SEND_MEAS_REULTS);
		break;
	case CONVERT_TO_STRING_1:
		digit = (uint8_t)(result/10000);
		decimal[0] = numbers[digit];
		result -= digit * 10000;
		digit = (uint8_t)(result/1000);
		decimal[1] = numbers[digit];
		result -= digit * 1000;
		digit = (uint8_t)(result/100);
		decimal[2] = numbers[digit];
		result -= digit * 100;
		digit = (uint8_t)(result/10);
		decimal[3] = numbers[digit];
		result -= digit * 10;
		sendResultsStage = CONVERT_TO_STRING_2;
		putRequest(SEND_MEAS_REULTS);
		break;
	case CONVERT_TO_STRING_2:
		digit = (uint8_t)(result);
		decimal[4] = numbers[digit];
		result -= digit;
		decimal[5] = '.';
		result *= 10;	// extract 0.x0
		digit = (uint8_t)(result);
		decimal[6] = numbers[digit];
		result -= digit;
		result *= 10;	// extract 0.0x
		digit = (uint8_t)(result);
		result -= digit;	// round last digit
		result *= 10;
		if (result >= 5) digit++;
		decimal[7] = numbers[digit];
		for (uint8_t d = 0; d < 4; d++) { // remove 0's at the beginning
			if (decimal[d] == '0') decimal[d] = ' ';
			else break;
		}
		sendResultsStage = CREATE_OUTPUT_STRING;
		putRequest(SEND_MEAS_REULTS);
		break;
	case CREATE_OUTPUT_STRING:
		measOutputString[channel*14+1] = numbers[channel+1];
		for (uint8_t k = 0; k < 8; k++) measOutputString[channel*14+4+k] = decimal[k];
		channel++;
		if (channel < NUM_OF_CHANNELS) sendResultsStage = CALCULATE_IMPEDANCE;
		else sendResultsStage = START_DMA_TRANSFER;
		putRequest(SEND_MEAS_REULTS);
		break;
	case START_DMA_TRANSFER:
		USART_SendMessage(measOutputString, 56);
		sendResultsStage = CALCULATE_IMPEDANCE;
		channel = 0;
		break;
	default:
		sendResultsStage = CALCULATE_IMPEDANCE;
		channel = 0;
		break;
	}
#endif
}
