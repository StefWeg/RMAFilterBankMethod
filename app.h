/*
 * app.h
 *
 *  Created on: 31.08.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef APP_H_
#define APP_H_

/* Includes ------------------------------------------------------------------*/

#include "signals.h"

#include <complex.h>// complex numbers
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

#define CONV_SMP_FREQ            10000.0f // Hz
#define MAX_SIG_PERIOD_LENGTH    SIN4_LEN // samples

#define MA_FILTER_LEN            ( MAX_SIG_PERIOD_LENGTH )

#define OPT_FREQ_1 250.000000000000000f
#define OPT_FREQ_2  65.789473684210520f
#define OPT_FREQ_3   5.617977528089887f
#define OPT_FREQ_4   0.050118279138767f

#define MA_FILT_LEN_1 8200
#define MA_FILT_LEN_2 16268
#define MA_FILT_LEN_3 67057
#define MA_FILT_LEN_4 199528

/* Exported types -----------------------------------------------------------*/

enum eBank
{
	e1stBank = 0,
	e2ndBank,
	NUM_OF_BANKS
};

enum eChannel
{
	e1stChannel = 0,
	e2ndChannel,
	e3rdChannel,
	e4thChannel,
	NUM_OF_CHANNELS
};

typedef enum eIdentificationStage
{
	READY_TO_START,
	CAPTURE_RESULTS,
	CALC_MEASURED_IMPEDANCES,
	IDENTIFY_PARAMS,
	CONVERT_PARAM_TO_STRING,
	CALC_IDENT_ERRORS,
	CONVERT_ERROR_TO_STRING,
	SEND_IDENT_DATA
} identificationStage;

/* Exported variables --------------------------------------------------------*/

extern volatile float uu;
extern volatile float ui;
extern volatile float avgUu;
extern volatile float avgUi;

extern complex float filteredSignals[NUM_OF_BANKS][NUM_OF_CHANNELS];
extern complex float averagedSignals[NUM_OF_BANKS];

extern volatile bool measProcInProgress;

extern complex float resultsUu[NUM_OF_CHANNELS];
extern complex float resultsUi[NUM_OF_CHANNELS];

extern volatile identificationStage currentIdentificationStage;
extern float identParams[NUM_OF_CHANNELS];
extern float identErrors[NUM_OF_CHANNELS];

/* Exported functions --------------------------------------------------------*/

void APP_Init();

void APP_ResetMAFiltersInit();
void APP_ResetMAFilters();

void APP_RunIdentificationIteration();
void APP_ResetIdentParams();

void APP_SendMeasResultsIteration();

#endif /* APP_H_ */
