
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



#include "MRT_PWM.h"

#include "lpc812_gpio.h"
#include "lpc812_iocon.h"
#include "lpc812_mrt.h"


/*****************************************************************************
 * Private types/enumerations/variables
 */

// Control bits for ongoing PWM-operations
typedef enum M_CH_STATUS {
  M_PWM_STARTED   = 0x01,	// The channel has been started
  M_PWM_FULL_ON   = 0x02,   // Constant HIGH/ ON, no PWM
  M_PWM_OFF       = 0x04,   // Constant LOW/ OFF, no PWM
  M_PWM_IN_PULSE  = 0x08,   // HIGH: Ongoing PWM with duty cycle [1..99%]
} M_CH_STATUS_TYPE;


typedef struct {
 __IO uint32_t pulseCount;
 __IO uint32_t spaceCount;
} MRT_PWM_DURATION_Type;

typedef struct {
  MRT_PWM_DURATION_Type duration;
 __IO M_CH_STATUS_TYPE runStat;
 __IO uint8_t pwLimitation;
} MRT_PWM_ModulaCrtl_Type;

typedef struct {
 MRT_PWM_ModulaCrtl_Type chCTRL[MRT_PWM_CHANNELS];
} MRT_PWM_Control_Type;

MRT_PWM_Control_Type pwFullCTRL;

static const uint8_t MRT_PWM_GPIO[] = { MRT_PWM_PIN_0, MRT_PWM_PIN_1, MRT_PWM_PIN_2, MRT_PWM_PIN_3 };


/* MRT_IRQHandler
 * Multi-Rate Timer interrupt handler, for all 4 channels
*/
void MRT_IRQHandler(void) {

 // Read the Global interrupt flag register
  uint32_t ch = 0;
  uint32_t chIntR = Chip_MRT_GetIntPending();

LPC_GPIO_PORT->CLR0 = (1 << 2);
LPC_GPIO_PORT->CLR0 = (1 << 3);


  while(ch < MRT_PWM_CHANNELS) {
   // Any INT. from this channel[ch]
    if (chIntR & MRTn_INTFLAG(ch)) {
     // The clear the flag and handle it
      Chip_MRT_ClearIntPending(MRTn_INTFLAG(ch));
     // If full-ON or OFF, ignore the rest
      if (!(pwFullCTRL.chCTRL[ch].runStat & (M_PWM_FULL_ON | M_PWM_OFF))) {
       // From PWM-pulse to PWM-space
        if (pwFullCTRL.chCTRL[ch].runStat & M_PWM_IN_PULSE) {
         // Pause/space time-part
          pwFullCTRL.chCTRL[ch].runStat &= ~(M_PWM_IN_PULSE);
         // Turn OFF load(LED or motor on PIN)
        #ifndef HIGH_CURRENT_DIRECT
          LPC_GPIO_PORT->CLR0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
        #else
          if (ch != 0)
            LPC_GPIO_PORT->CLR0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
          else
            LPC_GPIO_PORT->CLR0 = HIGH_CURRENT_DIRECT;
        #endif
         // Set Up counter for the space-time
          Chip_MRT_SetInterval(LPC_MRT_CH(ch), pwFullCTRL.chCTRL[ch].duration.spaceCount);
         }
        else {
         // New Pulse-time
          pwFullCTRL.chCTRL[ch].runStat |= M_PWM_IN_PULSE;
        #ifndef HIGH_CURRENT_DIRECT
          LPC_GPIO_PORT->SET0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
        #else
          if (ch != 0)
            LPC_GPIO_PORT->SET0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
          else
            LPC_GPIO_PORT->SET0 = HIGH_CURRENT_DIRECT;
        #endif
   	      Chip_MRT_SetInterval(LPC_MRT_CH(ch), pwFullCTRL.chCTRL[ch].duration.pulseCount);
         }
       }
     }
    // Check the next,
   ch++;
  }

  return;
} // MRT_IRQHandler()


/*  */
void PWM_setDuration(uint8_t ch, MRT_PWM_DURATION_Type* duret) {

 //
  pwFullCTRL.chCTRL[ch].duration.pulseCount = (uint32_t) PULSE_WIDTH_BIT_TIME * duret->pulseCount;
  pwFullCTRL.chCTRL[ch].duration.spaceCount = (uint32_t) PULSE_WIDTH_BIT_TIME * duret->spaceCount;
 // If NOT full ON or OFF, re-start the timer
  if (pwFullCTRL.chCTRL[ch].runStat & (M_PWM_FULL_ON | M_PWM_OFF)) {
    pwFullCTRL.chCTRL[ch].runStat &= ~(M_PWM_FULL_ON | M_PWM_OFF);
    pwFullCTRL.chCTRL[ch].runStat |= M_PWM_IN_PULSE;
#ifndef HIGH_CURRENT_DIRECT
    LPC_GPIO_PORT->SET0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
#else
    if (ch != 0)
      LPC_GPIO_PORT->SET0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
    else
      LPC_GPIO_PORT->SET0 = HIGH_CURRENT_DIRECT;
#endif
   // START with NEW pulse-time
    Chip_MRT_SetInterval(LPC_MRT_CH(ch), pwFullCTRL.chCTRL[ch].duration.pulseCount);
  }
} // PWM_setDuration


/*  */
void PWM_setFullON(uint8_t ch) {

 // Do not re-start the counter
  pwFullCTRL.chCTRL[ch].runStat &= ~(M_PWM_OFF | M_PWM_IN_PULSE);
  pwFullCTRL.chCTRL[ch].runStat |= M_PWM_FULL_ON;
  pwFullCTRL.chCTRL[ch].duration.pulseCount = 0;
  pwFullCTRL.chCTRL[ch].duration.spaceCount = 0;
 // SET the PIN ON
#ifndef HIGH_CURRENT_DIRECT
  LPC_GPIO_PORT->SET0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
#else
  if (ch != 0)
    LPC_GPIO_PORT->SET0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
  else
    LPC_GPIO_PORT->SET0 = HIGH_CURRENT_DIRECT;
#endif
} //  PWM_setFullON


/*  */
void PWM_setOFF(uint8_t ch) {

 // Do not re-start the counter
  pwFullCTRL.chCTRL[ch].runStat &= ~(M_PWM_FULL_ON | M_PWM_IN_PULSE);
  pwFullCTRL.chCTRL[ch].runStat |= M_PWM_OFF;
  pwFullCTRL.chCTRL[ch].duration.pulseCount = 0;
  pwFullCTRL.chCTRL[ch].duration.spaceCount = 0;
 // Clear the PIN
#ifndef HIGH_CURRENT_DIRECT
  LPC_GPIO_PORT->CLR0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
#else
  if (ch != 0)
    LPC_GPIO_PORT->CLR0 = (1 << (uint32_t) MRT_PWM_GPIO[ch]);
  else
    LPC_GPIO_PORT->CLR0 = HIGH_CURRENT_DIRECT;
#endif
} // PWM_setOFF



/*****************************************************************************
 * Public types/enumerations/variables
 */


/* Set a output-channel either full ON or OFF. NOTE: Be very careful on using
 * the full ON. Inductive load and other will draw large current.
 *
 * This function will mainly be used to switch OFF.
 */
void PWM_setOnOff(uint8_t channel, uint8_t ON) {

  if (ON)
    PWM_setFullON(channel);
  else
    PWM_setOFF(channel);
} // PWM_setOnOff


/* Convert duty-cycle-percent (1- 99%) into two times, a pulse-time
 * and a pause-time(space-time)
 */
void PWM_percentToCounter(uint8_t channel, uint8_t percentPW) {

  MRT_PWM_DURATION_Type duration;

 // Is the any modulation (> 0)
  if (percentPW >= 1) {
   // Full ON: If 100%
    if (percentPW >= 100)
      PWM_setFullON(channel);
   // 1 to 99 percent
    else {
     // Calculate the duty-cycle
#if (PULSE_WIDTH_RESOLUTION <= 100)
#include <stdio.h>
     /* Heavy and slow code */
      duration.pulseCount = (uint32_t) (PULSE_WIDTH_RESOLUTION * (percentPW / (float) 100));
#else
     /* Small code size, and POWER-full code */
      duration.pulseCount = (uint32_t) (PULSE_WIDTH_RESOLUTION * percentPW) / 100;
#endif
      duration.spaceCount = (uint32_t) PULSE_WIDTH_RESOLUTION - duration.pulseCount;
     // And update
      PWM_setDuration(channel, &duration);
     }
   }
 // OFF: Modulation is 0(zero)
  else
    PWM_setOFF(channel);
} // PWM_percentToCounter


/* Up-date an insert the two times for a full duration, pulse-time and
 * pause-time, based on pulseWidth.
 */
void PWM_pulseValToCounter(uint8_t channel, uint8_t pulseWidth) {

  MRT_PWM_DURATION_Type duration;

  if (pulseWidth >= 1) {
   // Full ON
    if (pulseWidth >= PULSE_WIDTH_RESOLUTION)
      PWM_setFullON(channel);
    else {
     // Calculate the two times based on pulse-width
      duration.pulseCount = (uint32_t) pulseWidth;
      duration.spaceCount = (uint32_t) PULSE_WIDTH_RESOLUTION - duration.pulseCount;
     // and update
      PWM_setDuration(channel, &duration);
    }
   }
 // If OFF, do not re-start counter after timer interrupt
  else
    PWM_setOFF(channel);
} // PWM_pulseValToCounter


/* Start Pulse-Width Modulation(PWM) on Multi-Rate Timer channels[0-3]
 *
 * Set-Up pulse width(Duty cycle) with the Multi-Rate-Timer in
 * ONE-SHOT interrupt mode.
 * parameters:      percentPW(Duty cycle / modulation); 1 - 100%)
 * Returned value:	None
 *
 * NOTE: The Multi-Rate Timer has to be enabled first,
 */
void MRT_PWM_Start(uint8_t channel, uint8_t percentPW) {

  if (channel <= MRT_PWM_CHANNELS) {
   // Set the MRT in ONE-SHOT interrupt mode
    Chip_MRT_SetMode(LPC_MRT_CH(channel), MRT_MODE_ONESHOT);
    Chip_MRT_SetEnabled(LPC_MRT_CH(channel));
   // SetUp port PIN
#ifndef HIGH_CURRENT_DIRECT
    LPC_GPIO_PORT->DIR0 |= (1 << (uint32_t) MRT_PWM_GPIO[channel]);
#else
    if (channel != 0)
      LPC_GPIO_PORT->DIR0 |= (1 << (uint32_t) MRT_PWM_GPIO[channel]);
    else
     // Hard-coding: The first ch(0). 4 PINs in parallel.
      LPC_GPIO_PORT->DIR0 |= HIGH_CURRENT_DIRECT;   // (0x0E << 14);
#endif
    // Set running flags (STARTED) and OFF
    pwFullCTRL.chCTRL[channel].runStat |= (M_PWM_STARTED | M_PWM_OFF);
    // Cal. pulse width duration
    PWM_percentToCounter(channel, percentPW);
   }
} // MRT_PWM_Start()


/* Enable the Multi-Rate Timer with interrupt, to be called separately
 */
void MRT_Chip_Enable(void) {
 // Enable the clock to the register interface
  LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<10);
 // Reset MRT
  LPC_SYSCON->PRESETCTRL &= ~(0x1<<7);
  LPC_SYSCON->PRESETCTRL |= (0x1<<7);
 // Enable the MRT Interrupt
  NVIC->ISER[0] = (1 << ((uint32_t)(MRT_IRQn) & 0x1F));
} // MRT_Chip_Enable

