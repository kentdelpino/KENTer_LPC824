/*****************************************************************************
 * TODO:
 *
 *  Remove HIGH_CURRENT_DIRECT from MRT_PWM.
 *
 * Go through the IO operations, a new lpc812_gpio.h is included.
 * NXP, Recommended practices (others say the opposite):
 * • To change the state of one pin, write a Byte Pin or Word Pin register.
 *
 */



/*****************************************************************************
 *                                       @Link www.linkedin.com/in/kentdelpino
 *                                       @Author Kent DEL PINO
 * @file: 	 MRT_PWM.c
 * @version: 0.6
 * @date: 	 1. Aug. 2014
 *
 * @brief Pulse-Width Modulation(PWM) based on Multi-Rate-Timer(MRT)
 *
 * Term used: PULSE and SPACE time-period of the Pulse-Width signal is the
 * two conditions that together form the duty-cycle of the PWM.
 * Note: __SYSTEM_CLOCK is the core clock (~30MHz)
 *
 * @par
 * This software is supplied "AS IS" without any warranties of any kind,
 * if You use this code, the responsibility is Yours only.
 * Kent del Pino reserves the right to make changes in the software
 * without notification.
 *
 * @par
 * Permission to use, modify, and distribute this software is hereby granted,
 * without fee, provided that it is used in conjunction with hardware and/or
 * software sold, delivered and/or made by Kent del Pino.
 * This copyright, permission, and disclaimer notice must appear in all copies
 * of this code.
 */

#ifndef __MRT_FOR_PWM_H_
#define __MRT_FOR_PWM_H_

#include "core_cm0plus.h"
#include "lpc_types.h"
#include "LPC8xx.h"


/* If HIGH_CURRENT is defined then 4 output-PINs in parallel is used,
 * to source the load. Only on the first channel, as in one channel:
 * -> GIOp 14, 15, 16, 17
 *
 * NOTE: The hex-no. 0x03C000 represent the 4 port bits.
 */
//#define HIGH_CURRENT_DIRECT  0x03C000

// Number of channels used. NOTE, at present cannot be changed
#define MRT_PWM_CHANNELS     4

/* GPIO-port(pin) for the soft-PWM signal, Update as needed and these
 * GPIO-pins should also be allocated as part main-configurePins(). */
#define MRT_PWM_PIN_0        14     /* , 15, 16, 17 if defined */
#define MRT_PWM_PIN_1        2
#define MRT_PWM_PIN_2        3
#define MRT_PWM_PIN_3        13


// Set the frequency, keep it under 1600Hz, for LED use higher then 300Hz
#define PULSE_WIDTH_FREQUENCY   (__SYSTEM_CLOCK / 660)
// Set the PWM resolution, under 100, BIG code size, BEST from 127 to 255
#define PULSE_WIDTH_RESOLUTION  256
// The bit-time (one unit)
#define PULSE_WIDTH_BIT_TIME    (PULSE_WIDTH_FREQUENCY / PULSE_WIDTH_RESOLUTION)


/* Enable the Multi-Rate Timer with interrupt, to be called separately
 */
void MRT_Chip_Enable(void);

/* Start Pulse-Width Modulation(PWM) on Multi-Rate Timer channels[0-3]
 * parameters:   Channel no. and percentPW[1- 100%](Duty cycle)
 * NOTE: The Multi-Rate Timer has to be enabled first, via the
 * MRT_Chip_Enable */
void MRT_PWM_Start(uint8_t channel, uint8_t percentPW);

/* Set a output-channel either full ON or OFF. NOTE: Be very careful on using
 * the full ON. Inductive load and other will draw large current.
 *
 * This function will mainly be used to switch OFF, as in setOnOff(ch, 0).
 */
void PWM_setOnOff(uint8_t channel, uint8_t ON);

/* Up-date and convert duty-cycle-percent (1- 99%) into two times, a
 * pulse-time and a pause-time(space-time) */
void PWM_percentToCounter(uint8_t channel, uint8_t percentPW);

/* Up-date an insert the two times for a full duration, pulse time and
 * pause time, based on pulseWidth.
 */
void PWM_pulseValToCounter(uint8_t channel, uint8_t pulseWidth);



#endif /* __MRT_FOR_PWM_H_*/
