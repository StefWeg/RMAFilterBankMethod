/*
 * signals.h
 *
 *  Created on: 02.09.2019
 *      Author: Stefan Węgrzyn (162430)
 *     Project: Hardware and software implementation of filter-bank based
 *              implementation of multisine impedance measurement method
 */

#ifndef SIGNALS_H_
#define SIGNALS_H_

// Signals for 10 kHz sampling
#define SIN1_LEN 40ul
#define COS1_OFFSET SIN1_LEN/4
extern const float sin1[];

#define SIN2_LEN 152ul
#define COS2_OFFSET SIN2_LEN/4
extern const float sin2[];

#define SIN3_LEN 1780ul
#define COS3_OFFSET SIN3_LEN/4
extern const float sin3[];

#define SIN4_LEN 199528ul
#define COS4_OFFSET SIN4_LEN/4
extern const float sin4[];

// Remember about 1 MB FLASH limit
// Current usage is equal to around 0.80 MB

#endif /* SIGNALS_H_ */
