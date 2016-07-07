/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: task_our_app.c
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.1
 * Date:     1. Apr. 2016
 *
 *  -- This source-code is released into the public domain, by the author --
 *
 * This file contains, is Unlicensed material:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The author of this material /source-code kindly ask all users and distribu-
 * tors of it, to leave this notice / text in full, in all copies of it, for 
 * all time to come.
 */


/* Include standard c library as needed. NOTE: we use newlib-nano */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>

/* Application tools/ protocols to be used */
#include "minimum_board.h"  /* Everything in Hardware */
#include "mick.h"           /*   -"-   in OS and threads*/



/***************************************************************************
 * Declare Variables here for storage directly in the bss-section(data seg.)
 */







/***************************************************************************
 * 
 * IC-pin 26 
 */

#define ACMP_POS_INPUT    ACMP_POSIN_ACMP_I4 /* On positive comparator input */
#define ACMP_NEG_INPUT    ACMP_POSIN_VLO     /* On the negative, LADDER out */
#define ACMP_HYSTERESIS   ACMP_HYS_20MV      /* 20mVolt Comparator hysteresis */ 
#define ACMP_LADDER_REF   3.3                /* Here equal to VDD (in Volt) */ 
#define ACMP_POS_TRIG     1.2                /* Positive trigger level in Volt */
#define ACMP_LADDER_STEP  (uint32_t)((float)ACMP_POS_TRIG / ((float)ACMP_LADDER_REF /31))

#if defined (MICK_OS)
// The time of the event in CLOCKS_PER_SEC (small platform 1mSec)
volatile clock_t comparatorEventTime;
#endif

// DIAG_LED_ is only used for test
   //#define DIAG_LED_GPIO_PIN   9


void CMP_IRQHandler(void) {

 // Clear the interrupt
  Chip_ACMP_EdgeClear(LPC_CMP);

/* TEMP here */   MinimumLED_ON();


#if defined (MICK_OS)
 // What is the time(in CLOCKS_PER_SEC)

 // Set Signal-flag
  
#endif  
} /* CMP_IRQHandler */


void ACMP_ActiveCtrl(uint8_t _active) {
  
#if defined (MICK_OS)
  comparatorEventTime = 0;
#endif

  if (_active == 0) {
    Chip_ACMP_DisableVoltLadder(LPC_CMP);
    Chip_ACMP_Deinit(LPC_CMP);
   }
  else {  // PowerUp and Initialize the ACMP
    Chip_ACMP_Init(LPC_CMP);
   }
} /* ACMP_ActiveCtrl */



/* ACMP Pin SetUp - note for diagnostics and configuration wee use a LED
 * for indicator on the comparator output state. */
void ACMP_PinSetUp(void) {

 // Enable the clock to the Switch Matrix */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);

 // ACMP_I4 on pin26 (PIO_23)
  Chip_SWM_DisableFixedPin(SWM_FIXED_ADC3);
  Chip_SWM_EnableFixedPin(SWM_FIXED_ACMP_I4);

 // Comparator output 
#ifndef DIAG_LED_GPIO_PIN
  // Out goes to interrupt  
#else
  Chip_SWM_DisableFixedPin(SWM_FIXED_XTALOUT); 
  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, DIAG_LED_GPIO_PIN); 
  Chip_SWM_MovablePinAssign(SWM_ACMP_O_O, DIAG_LED_GPIO_PIN); 
#endif

 // Disable the clock to the Switch Matrix to save power */
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON); 
} /* ACMP_PinSetUp */


void ACMP_servoProperties(void) {
  
 // How to use comparator output 
#ifndef DIAG_LED_GPIO_PIN
 // Used with interrupt (sync)
//  Chip_ACMP_EnableSyncCompOut(LPC_CMP); 
  Chip_ACMP_DisableSyncCompOut(LPC_CMP);
#else
 // Use directly (no sync) 
  Chip_ACMP_DisableSyncCompOut(LPC_CMP);
#endif  
 // 
  Chip_ACMP_SetupAMCPRefs(LPC_CMP, ACMP_EDGESEL_BOTH, ACMP_POS_INPUT, ACMP_NEG_INPUT, ACMP_HYSTERESIS);
 // We use VDD as voltage on the ladder, therefore ->  false(0).
  Chip_ACMP_SetupVoltLadder(LPC_CMP, ACMP_LADDER_STEP, false);
  Chip_ACMP_EnableVoltLadder(LPC_CMP);

#ifndef DIAG_LED_GPIO_PIN 
 // Enable compare output Interrupt
  NVIC_EnableIRQ(CMP_IRQn);
#endif  
} /* ACMP_servoProperties */



/***************************************************************************
 * 1 or 2 channels Servo control (PWM outputs).
 * 
 * There are two types of RC-servos on the market, the analog and the digital 
 * servos. The main difference is how the internal motor is driven by the 
 * internal controller. The analog RC-servos need position refresh signal at 
 * about 40 to 60 times a second(as 50Hz:-). The digital servos can work with 
 * position refresh rates at up to 300 times per second.
 * 
 * 
 * Hardware resources used, on a KENTer_LPC824:
 * 
 *    DPIO_16 / chip-pin 10 (DO_4): First RC-Servo PWM
 *    DPIO_19 / chip-pin 30 (DO_5): Power Control (ON/OFF). 
 *    DPIO_20 / chip-pin 29       : *)Second RC-Servo PWM, if needed  
 * 
 *    The SCTimer use:
 *      16bit High-part counter / DisableIRQ
 *       SCT_OUTPUT_4 -> *)SCT_OUTPUT_5
 *       SCT_MATCH_0  -> SCT_MATCH_1 (High-part) and *)SCT_MATCH_2
 *       SCT_EVT_5    -> SCT_EVT_6 and *)SCT_EVT_7
 *
 * NOTE: /Speed on a medium strong Servo is around 0.24sec/60degrees and
 *        specific for the Micro Servo HK5320, it's 0.05sec/60degrees @4.2v.
 *       /Power Control is for a Voltage Regulator Enable-pin, for Voltage 
 *        Regulator that is used separately on the Servos.
 *        
 */

// From SERVO Technical Specifications
#define SERVO_SPEED_SEC_60         ((float) 0.05) // Servo Speed in sec/degrees 

// If two(2) Servos. 
#define SCT_IF_TWO_SERVOS           
// If Power ON/OFF is used.
#define SCT_IF_POWER_CTRL           

#define SCT_CLOCK_HIGH_PART   (2000000) // Pulse resolution 0.5uS
#define SCT_SERVO_REFRESH_RATE     (50) // Pulse refresh in Hz
#define SCT_SERVO_MIDDLE_POS       ((uint16_t) (0.0015 / ((float)1 / SCT_CLOCK_HIGH_PART)) )  // Center pos. 1.5 mS 
#define SCT_SERVO_PLUS_90          ((uint16_t) (0.0025 / ((float)1 / SCT_CLOCK_HIGH_PART)) )  -500     // +90dg, 2.5 mS   - 5000
#define SCT_SERVO_MINUS_90         ((uint16_t) (0.0005 / ((float)1 / SCT_CLOCK_HIGH_PART)) )  +500     // -90dg, 0.5 mS   - 1000
// Servo rotation speed
#ifndef SERVO_SPEED_SEC_60
#define SERVO_SPEED_SEC_60         ((float) 0.24) // IN_NOT_DEFINED, then 0.24
#endif
#define SERVO_SPEED_SEC_FULL_SCALE (SERVO_SPEED_SEC_60 * 3)
//#define SERVO_SPEED_SEC_RESOLUTION   ((float)SERVO_SPEED_SEC_FULL_SCALE / (SCT_SERVO_PLUS_90 - SCT_SERVO_MINUS_90))
#if defined (MICK_OS)
//SERVO_SPEED_DEG_RESOLUTION
//TICKS  SYS_TICK_COUNTER

clock_t servoTimeOfStart = 0;
clock_t servoTimeOfExpArr = 0;
#endif


void SCT_servoPulsePins(void) {

 // If NO Clock is enabled, then do enable the SCT clock, just remember it includes 
 // a SCT-Reset. 
  register uint32_t clkEnabled = (1 << SYSCTL_CLOCK_SCT) & LPC_SYSCTL->SYSAHBCLKCTRL;
  if (!clkEnabled) 
    Chip_SCT_Init(LPC_SCT); 
  
  // Enable clock for IOCON logic
   Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);  
   Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

  // PWM-Output 4 to PIO0_16(pin10) 
   Chip_SWM_MovablePinAssign(SWM_SCT_OUT4_O, 16);   // SCT_OUT4 at PIO0_16  
  // IO-pin connected directly to RC-servo pulse-pin. 
   Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO16, PIN_MODE_PULLUP); 
   
#ifdef SCT_IF_TWO_SERVOS
   Chip_SWM_MovablePinAssign(SWM_SCT_OUT5_O, 20);   // SCT_OUT5 at PIO0_20 
   Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO20, PIN_MODE_PULLUP);  
#endif 
#ifdef SCT_IF_POWER_CTRL   
  // The Servo Power-Control-Pin.
   Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 19);
   Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO19, PIN_MODE_PULLUP); 
#endif   
  // Consume less power
   Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);  
   Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

} /* SCT_servoPulsePins */

 
void SCT_servoPulseProperties(void) /* should we have a initPos */    {

 // Upper 16bit is used as automatic re-load counter, running @ 2MHz.
  Chip_SCT_Config(LPC_SCT, SCT_CONFIG_AUTOLIMIT_H);
  Chip_SCT_SetControl(LPC_SCT, SCT_CTRL_PRE_H(SystemCoreClock/SCT_CLOCK_HIGH_PART -1)); 
 // Set the refresh-rate counter and reload reg (MATCH_0 is MATCH0_High). 
  Chip_SCT_SetMatchCountH(LPC_SCT, SCT_MATCH_0, (uint16_t)(SCT_CLOCK_HIGH_PART / SCT_SERVO_REFRESH_RATE));  
  Chip_SCT_SetMatchReloadH(LPC_SCT, SCT_MATCH_0, (uint16_t)(SCT_CLOCK_HIGH_PART / SCT_SERVO_REFRESH_RATE));

 // DO_4/PIO_16/pin10, not real pulse quantities, that comes later 
  Chip_SCT_SetMatchCountH(LPC_SCT, SCT_MATCH_1, 1);   
  Chip_SCT_SetMatchReloadH(LPC_SCT, SCT_MATCH_1, 0);
 // Output Servo PW-register (OUTPUT_4)
  Chip_SCT_SetOutput(LPC_SCT, SCT_OUTPUT_4, SCT_EVT_5);           
  Chip_SCT_ClearOutput(LPC_SCT, SCT_OUTPUT_4, SCT_EVT_6);
 // Rising(SCT_EVT_5) pulse-edge in all state.
  Chip_SCT_EventState(LPC_SCT, SCT_EVENT_5, ENABLE_ALL_STATES); 
  Chip_SCT_EventControl(LPC_SCT, SCT_EVENT_5, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH0 | SCT_HEVENT_H | SCT_COMBMODE_MATCH ));
 // Falling(SCT_EVT_6) pulse-edge
  Chip_SCT_EventState(LPC_SCT, SCT_EVENT_6, ENABLE_ALL_STATES); 
  Chip_SCT_EventControl(LPC_SCT, SCT_EVENT_6, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH1 | SCT_HEVENT_H | SCT_COMBMODE_MATCH ));

#ifdef SCT_IF_TWO_SERVOS
  Chip_SCT_SetMatchCountH(LPC_SCT, SCT_MATCH_2, 1);   
  Chip_SCT_SetMatchReloadH(LPC_SCT, SCT_MATCH_2, 0);
 // Output (OUTPUT_5)
  Chip_SCT_SetOutput(LPC_SCT, SCT_OUTPUT_5, SCT_EVT_5);           
  Chip_SCT_ClearOutput(LPC_SCT, SCT_OUTPUT_5, SCT_EVT_7);
 // Rising(SCT_EVT_5) 
       // already done
 // Falling(SCT_EVT_7)
  Chip_SCT_EventState(LPC_SCT, SCT_EVENT_7, ENABLE_ALL_STATES); 
  Chip_SCT_EventControl(LPC_SCT, SCT_EVENT_7, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH2 | SCT_HEVENT_H | SCT_COMBMODE_MATCH ));
#endif     
 
  // No interrupt need
   NVIC_DisableIRQ(SCT_IRQn);
} /* SCT_servoPulseProperties */


#ifdef SCT_IF_POWER_CTRL

// =1; Default, after Reset, pin is pulled Up and High
uint8_t servoPowerState = 1; 

// When turning power on, there is a delay(3mS) until stable voltage-supply.
void setServoPower(uint8_t _on) {

  if (_on == 0) { // OFF
    servoPowerState = 0;
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 19, 0);
   }
  else {
    servoPowerState = 1;
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 19, 1);
  }
} /* setServoPower */

#endif 


// Starts(RUN) or Stops(HALT) the pulse repetition
void SCT_servoRunCtrl(uint8_t _run) {

  if (_run == 0) // HALT
    Chip_SCT_SetControl(LPC_SCT , SCT_CTRL_HALT_H);
  else           // Unhalt SCTimer(Run)
    Chip_SCT_ClearControl(LPC_SCT , SCT_CTRL_HALT_H);
} /* SCT_servoRunCtrl */ 


// Init/Start or Stop/Power-Down the SCT-Servo function 
void SCT_servoActiveCtrl(uint8_t _active) {
  
  if (_active == 0) { // End Servo activities
#ifdef SCT_IF_POWER_CTRL
    setServoPower(0);
#endif 
    SCT_servoRunCtrl(0);

   // SCTimer full-stop(power-down)
    //Chip_SCT_DeInit(LPC_SCT);    
   } 
  else {              // Init and Run
    SCT_servoPulsePins();
    SCT_servoPulseProperties();
#ifdef SCT_IF_POWER_CTRL
    setServoPower(1);
#endif 
   }
} /* SCT_servoActiveCtrl */


int SCT_servoPosAbsolute(int ch, int moveTo) {
 // If HALT_H is set, then clear to RUN SCT 
  register uint32_t halt = SCT_CTRL_HALT_H & LPC_SCT->CTRL_U;
   
  if ((moveTo >= SCT_SERVO_MINUS_90) && (moveTo <= SCT_SERVO_PLUS_90)) {  

#if defined (MICK_OS)
    //servoTimeOfStart = clock();
    //timeOfServoArrival = xxx;
#endif
    
    switch(ch) {
      case 0 :                                               // -1 !!
          Chip_SCT_SetMatchReloadH(LPC_SCT, SCT_MATCH_1, (uint16_t) moveTo);  
        break; 

#ifdef SCT_IF_TWO_SERVOS
      case 1 :
          Chip_SCT_SetMatchReloadH(LPC_SCT, SCT_MATCH_2, (uint16_t) moveTo);  
        break; 
#endif

     // All motors
      default : 
        Chip_SCT_SetMatchReloadH(LPC_SCT, SCT_MATCH_1, (uint16_t) moveTo);  
#ifdef SCT_IF_TWO_SERVOS
        Chip_SCT_SetMatchReloadH(LPC_SCT, SCT_MATCH_2, (uint16_t) moveTo);  
#endif
        break; 
     }
    
   // Unhalt SCTimer(Run)
    if (halt)
      Chip_SCT_ClearControl(LPC_SCT, SCT_CTRL_HALT_H);
    
    return moveTo;
   }

  return -1;  
} /* SCT_servoPosAbsolute */


int SCT_servoPosDegrees(int ch, int deg) {
  
} /* SCT_servoPosDegrees */



/***************************************************************************
 * 4 channels PWM outputs. 
 * Making use of the State Configurable Timer or just SCTimer. On a LPC824 
 * based KENTer the DO_0 to DO_3 is is routed to inverted drivers. 
 * The Alive-pin on PIO_15 is use to Output-Enable this drivers. This PWM 
 * is meant for LED dimming.  
 * 
 * Hardware resources used, on a KENTer_LPC824:
 *
 *    PIO_15 / chip-pin 15 (Output Enable Driver, if needed)
 *    PIO_24 / chip-pin 14 (DO_0)
 *    PIO_25 / chip-pin 13 (DO_1)
 *    PIO_26 / chip-pin 12 (DO_2)
 *    PIO_27 / chip-pin 11 (DO_3)
 *    The SCTimer use:
 *      16bit Low-part counter / DisableIRQ
 *      SCT_OUTPUT_0 -> SCT_OUTPUT_3
 *      SCT_MATCH_0  -> SCT_MATCH_4 (Low-part)
 *      SCT_EVT_0    -> SCT_EVT_4
 */

#define SCT_CLOCK_LOW_PART         (620000) // 620 KHz
#define SCT_PWM_LOW_CYCLE ((uint16_t) 1000) // Gives a Pulse Width Frequency @ 620Hz 
#define SCT_PWM_MUM_OF_ALL       4          // and a PWM dynamic of 0-1000. 


void SCT_ledPulsePins(void) {

  // If no Clock is Enabled, then enable the SCT clock 
  register uint32_t clkEnabled = (1 << SYSCTL_CLOCK_SCT) & LPC_SYSCTL->SYSAHBCLKCTRL;
  if (!clkEnabled) 
    Chip_SCT_Init(LPC_SCT); 
  
 // Enable clock for IOCON logic
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);  
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

 // 4 * PWM output 
  Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 24);   // SCT_OUT0 at P0_24  
  Chip_SWM_MovablePinAssign(SWM_SCT_OUT1_O, 25);   // SCT_OUT1 at P0_25 
  Chip_SWM_MovablePinAssign(SWM_SCT_OUT2_O, 26);   // SCT_OUT2 at P0_26 
  Chip_SWM_MovablePinAssign(SWM_SCT_OUT3_O, 27);   // SCT_OUT3 at P0_27 

 // While using inverting buffer/line driver(3-state) the pins is external pulled-up. 
  Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO24, PIN_MODE_INACTIVE); 
  Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO25, PIN_MODE_INACTIVE); 
  Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO26, PIN_MODE_INACTIVE);  
  Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO27, PIN_MODE_INACTIVE); 
  
 // The Alive-pin(3-state) for inverting buffer is alse external pulled-up.
    //Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO15, PIN_MODE_INACTIVE); 
  
 // Consume less power
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);  
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);  
} /* SCT_ledPulsePins */


void SCT_ledPulseProperties(void) {

 // AUTOLIMIT_Low part (16bit counter) 
  Chip_SCT_Config(LPC_SCT, SCT_CONFIG_AUTOLIMIT_L);
 // Set prescaler, SCT clock = 620 KHz
  Chip_SCT_SetControl(LPC_SCT, SCT_CTRL_PRE_L(SystemCoreClock/SCT_CLOCK_LOW_PART -1));
  // Set Count match/Counter-cycle (MATCH_0 is MATCH0_L)
  Chip_SCT_SetMatchCountL(LPC_SCT, SCT_MATCH_0, SCT_PWM_LOW_CYCLE);       
  Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_0, SCT_PWM_LOW_CYCLE);

  // DO_0/Channel 0, Set init value
  Chip_SCT_SetMatchCountL(LPC_SCT, SCT_MATCH_1, SCT_PWM_LOW_CYCLE -1);   
  Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_1, SCT_PWM_LOW_CYCLE -1);
 // DO_1/Channel 1
  Chip_SCT_SetMatchCountL(LPC_SCT, SCT_MATCH_2, SCT_PWM_LOW_CYCLE -1);
  Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_2, SCT_PWM_LOW_CYCLE -1);
 // DO_2/Channel 2
  Chip_SCT_SetMatchCountL(LPC_SCT, SCT_MATCH_3, 1); 
  Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_3, 1);
 // DO_3/Channel 3
  Chip_SCT_SetMatchCountL(LPC_SCT, SCT_MATCH_4, 1); 
  Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_4, 1);

 // Set Output registers
  Chip_SCT_SetOutput(LPC_SCT, SCT_OUTPUT_0, SCT_EVT_0);           
  Chip_SCT_ClearOutput(LPC_SCT, SCT_OUTPUT_0, SCT_EVT_1);
  Chip_SCT_SetOutput(LPC_SCT, SCT_OUTPUT_1, SCT_EVT_0);           
  Chip_SCT_ClearOutput(LPC_SCT, SCT_OUTPUT_1, SCT_EVT_2);
 // Inverted output
  Chip_SCT_SetOutput(LPC_SCT, SCT_OUTPUT_2, SCT_EVT_3);         
  Chip_SCT_ClearOutput(LPC_SCT, SCT_OUTPUT_2, SCT_EVT_0);
  Chip_SCT_SetOutput(LPC_SCT, SCT_OUTPUT_3, SCT_EVT_4);            
  Chip_SCT_ClearOutput(LPC_SCT, SCT_OUTPUT_3, SCT_EVT_0);

 // Set EVENT registers
  Chip_SCT_EventState(LPC_SCT, SCT_EVENT_0, ENABLE_ALL_STATES); 
  Chip_SCT_EventControl(LPC_SCT, SCT_EVENT_0, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH0 | SCT_HEVENT_L | SCT_COMBMODE_MATCH ));
  Chip_SCT_EventState(LPC_SCT, SCT_EVENT_1, ENABLE_ALL_STATES); 
  Chip_SCT_EventControl(LPC_SCT, SCT_EVENT_1, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH1 | SCT_HEVENT_L | SCT_COMBMODE_MATCH ));
  Chip_SCT_EventState(LPC_SCT, SCT_EVENT_2, ENABLE_ALL_STATES);
  Chip_SCT_EventControl(LPC_SCT, SCT_EVENT_2, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH2 | SCT_HEVENT_L | SCT_COMBMODE_MATCH ));
  Chip_SCT_EventState(LPC_SCT, SCT_EVENT_3, ENABLE_ALL_STATES); 
  Chip_SCT_EventControl(LPC_SCT, SCT_EVENT_3, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH3 | SCT_HEVENT_L | SCT_COMBMODE_MATCH ));
  Chip_SCT_EventState(LPC_SCT, SCT_EVENT_4, ENABLE_ALL_STATES);
  Chip_SCT_EventControl(LPC_SCT, SCT_EVENT_4, (CHIP_SCT_EVENTCTRL_T) ( SCT_EVECTRL_MATCH4 | SCT_HEVENT_L | SCT_COMBMODE_MATCH ));

 // No interrupt need
  NVIC_DisableIRQ(SCT_IRQn);
 // Unhalt SCTimer(Run)
  Chip_SCT_ClearControl(LPC_SCT , SCT_CTRL_HALT_L);
} /* SCT_ledPulseProperties */


int SCT_ledPulseUpDate(int ch, int val) {
  
  if ((val >= 0) && (val <= SCT_PWM_LOW_CYCLE)) {  
    switch(ch) {
      case 0 :
        Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_1, SCT_PWM_LOW_CYCLE - val);
        break; 
      case 1 :
        Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_2, SCT_PWM_LOW_CYCLE - val);
        break; 
      case 2 : // PWM inverted 
        Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_3, val);
        break; 
      case 3 : // PWM inverted 
        Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_4, val);
        break; 
    
     // All 4 channals
      default : 
        Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_1, SCT_PWM_LOW_CYCLE - val);
        Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_2, SCT_PWM_LOW_CYCLE - val);
       // 2 and 3 is inverted 
        Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_3, val);
        Chip_SCT_SetMatchReloadL(LPC_SCT, SCT_MATCH_4, val);
        break; 
     }
    return val;
   }
  
  return -1;
} // SCT_ledPulseUpDate





/** FUN,FUN, just a little different LED-Blinking *************************/
typedef struct {
  uint8_t pwm0;
  uint8_t pwm1;
  uint8_t pwm2;
  uint8_t pwm3;
} WORD_OF_BYTES_T __attribute__ ((aligned (4)));   

const WORD_OF_BYTES_T ledData[] = { 
   [0].pwm0 = 0,    [0].pwm1 = 255,  [0].pwm2 = 0,    [0].pwm3 = 255, 
   [1].pwm0 = 5,    [1].pwm1 = 225,  [1].pwm2 = 5,    [1].pwm3 = 225, 
   [2].pwm0 = 11,   [2].pwm1 = 201,  [2].pwm2 = 11,   [2].pwm3 = 201, 
   [3].pwm0 = 22,   [3].pwm1 = 188,  [3].pwm2 = 22,   [3].pwm3 = 188,
   [4].pwm0 = 32,   [4].pwm1 = 174,  [4].pwm2 = 32,   [4].pwm3 = 174, 
   [5].pwm0 = 42,   [5].pwm1 = 160,  [5].pwm2 = 42,   [5].pwm3 = 160, 
   [6].pwm0 = 52,   [6].pwm1 = 149,  [6].pwm2 = 52,   [6].pwm3 = 149, 
   [7].pwm0 = 64,   [7].pwm1 = 138,  [7].pwm2 = 64,   [7].pwm3 = 138,   
   [8].pwm0 = 78,   [8].pwm1 = 124,  [8].pwm2 = 78,   [8].pwm3 = 124,   
   [9].pwm0 = 98,   [9].pwm1 = 103,  [9].pwm2 = 98,   [9].pwm3 = 103, 
  [10].pwm0 = 113, [10].pwm1 = 92,  [10].pwm2 = 113, [10].pwm3 = 92, 
  [11].pwm0 = 125, [11].pwm1 = 81,  [11].pwm2 = 125, [11].pwm3 = 81, 
  [12].pwm0 = 139, [12].pwm1 = 70,  [12].pwm2 = 139, [12].pwm3 = 70,
  [13].pwm0 = 151, [13].pwm1 = 59,  [13].pwm2 = 151, [13].pwm3 = 59, 
  [14].pwm0 = 170, [14].pwm1 = 48,  [14].pwm2 = 170, [14].pwm3 = 48, 
  [15].pwm0 = 198, [15].pwm1 = 37,  [15].pwm2 = 198, [15].pwm3 = 37, 
  [16].pwm0 = 226, [16].pwm1 = 26,  [16].pwm2 = 226, [16].pwm3 = 26,   
  [17].pwm0 = 247, [17].pwm1 = 15,  [17].pwm2 = 247, [17].pwm3 = 15,   
  [18].pwm0 = 255, [18].pwm1 = 9,   [18].pwm2 = 255, [18].pwm3 = 9,
  [19].pwm0 = 225, [19].pwm1 = 4,   [19].pwm2 = 225, [19].pwm3 = 4, 
  [20].pwm0 = 201, [20].pwm1 = 0,   [20].pwm2 = 201, [20].pwm3 = 0, 
  [21].pwm0 = 188, [21].pwm1 = 5,   [21].pwm2 = 188, [21].pwm3 = 5, 
  [22].pwm0 = 174, [22].pwm1 = 11,  [22].pwm2 = 174, [22].pwm3 = 11,
  [23].pwm0 = 160, [23].pwm1 = 22,  [23].pwm2 = 160, [23].pwm3 = 22, 
  [24].pwm0 = 149, [24].pwm1 = 32,  [24].pwm2 = 149, [24].pwm3 = 32, 
  [25].pwm0 = 138, [25].pwm1 = 42,  [25].pwm2 = 138, [25].pwm3 = 42, 
  [26].pwm0 = 124, [26].pwm1 = 64,  [26].pwm2 = 124, [26].pwm3 = 64,   
  [27].pwm0 = 103, [27].pwm1 = 78,  [27].pwm2 = 103, [27].pwm3 = 78,   
  [28].pwm0 = 92,  [28].pwm1 = 98,  [28].pwm2 = 92,  [28].pwm3 = 98, 
  [29].pwm0 = 81,  [29].pwm1 = 113, [29].pwm2 = 81,  [29].pwm3 = 113, 
  [30].pwm0 = 70,  [30].pwm1 = 125, [30].pwm2 = 70,  [30].pwm3 = 125, 
  [31].pwm0 = 59,  [31].pwm1 = 139, [31].pwm2 = 59,  [31].pwm3 = 139,
  [32].pwm0 = 48,  [32].pwm1 = 151, [32].pwm2 = 48,  [32].pwm3 = 151, 
  [33].pwm0 = 36,  [33].pwm1 = 170, [33].pwm2 = 36,  [33].pwm3 = 170, 
  [34].pwm0 = 24,  [34].pwm1 = 198, [34].pwm2 = 24,  [34].pwm3 = 198, 
  [35].pwm0 = 10,  [35].pwm1 = 226, [35].pwm2 = 10,  [35].pwm3 = 226,   
  [36].pwm0 = 2,   [36].pwm1 = 247, [36].pwm2 = 2,   [36].pwm3 = 247,  
}; 






void task_application(void) { /* The TASK entry - its main() */


  /* LOCAL STACK/MEM: Variables here are stored on local Task Stack. Remember 
   * to adjust, according to Your needs, the Stack-size in _DEF_YOUR_TASKS. 
   * 
   * Doing Thread initialization this Mem-area is filled with NULLs(0).
   */


  int c, p = 0;


 // Setting up 4 channel PWM for LEDs(Red, blue, warm white and cold white)
  SCT_ledPulsePins(); 
  SCT_ledPulseProperties();  
  SCT_ledPulseUpDate(SCT_PWM_MUM_OF_ALL, 0);
  

  ACMP_ActiveCtrl(1);
  ACMP_PinSetUp();
  ACMP_servoProperties();  
  ACMP_ActiveCtrl(0);


  MinimumLED_OFF();  
  
  
  while (1) {  /* Task infinite loop */

    
    for (c = 0; c < 37; c++) {
      SCT_ledPulseUpDate(0, (int)(ledData[c].pwm0 *1.0));
      SCT_ledPulseUpDate(1, (int)(ledData[c].pwm1 *1.0));
      SCT_ledPulseUpDate(2, (int)(ledData[c].pwm2 *1.0));
      SCT_ledPulseUpDate(3, (int)(ledData[c].pwm3 *1.0));

      osDelay(TICKS_MILLISEC(75));

      
      p++;
      if (p >= 10) {
        
        switch(p) {

        case 44 :
           // Power Up the Servos
            SCT_servoActiveCtrl(1); 
  MinimumLED_ON();
          break;

          
        case 48 : /* #0 */
            SCT_servoPosAbsolute(0, SCT_SERVO_MIDDLE_POS);
          break;

              case 56 : 
                SCT_servoRunCtrl(0);
                MinimumLED_OFF();
              break;
          
          case 60 : /* #1 */
              SCT_servoPosAbsolute(1, SCT_SERVO_MIDDLE_POS);
            break;

              case 92 : 
                SCT_servoRunCtrl(0);
                MinimumLED_OFF();
              break;

              
              
        case 96 : /* #0 */
            SCT_servoPosAbsolute(0, SCT_SERVO_PLUS_90);
          break; 

              case 104 : 
                SCT_servoRunCtrl(0);
                MinimumLED_OFF();
              break;
          
          case 108 : /* #1 */
              SCT_servoPosAbsolute(1, SCT_SERVO_PLUS_90);
            break; 
            
              case 116 : 
                SCT_servoRunCtrl(0);
                MinimumLED_OFF();
              break;

              
              
        case 120 : /* #0 */
            SCT_servoPosAbsolute(0, SCT_SERVO_MIDDLE_POS);
          break; 

              case 128 : 
                SCT_servoRunCtrl(0);
                MinimumLED_OFF();
              break;          
          
          case 132 : /* #1 */
              SCT_servoPosAbsolute(1, SCT_SERVO_MIDDLE_POS);
            break; 

              case 140 : 
                SCT_servoRunCtrl(0);
                MinimumLED_OFF();
              break;          

              
              
        case 144 : /* #0 */
            SCT_servoPosAbsolute(0, SCT_SERVO_MINUS_90);
          break; 

              case 152 : 
                SCT_servoRunCtrl(0);
                MinimumLED_OFF();
              break;          
          
          case 156 : /* #1 */
              SCT_servoPosAbsolute(1, SCT_SERVO_MINUS_90);
            break; 
            
              case 164 : 
                SCT_servoRunCtrl(0);
                MinimumLED_OFF();
              break;          
         
         }
       }

      
      if (p >= 180) { 
        p = 0;
       // Power OFF the Servos
        SCT_servoActiveCtrl(0);
       }

     }


   } /* Task infinite loop */

} /* Our application Task*/


