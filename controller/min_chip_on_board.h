/***************************************************************************
 *                                   Link: www.linkedin.com/in/kentdelpino
 *                                   Author: Kent DEL PINO
 * File: min_chip_on_board.h
 * Used with: GCC ARM Embedded,      https://launchpad.net/gcc-arm-embedded
 * Version:  0.2
 * Date:     31 Dec. 2015
 *
 * HAL for on-board chip, PCF85063A(Real-Time Clock) and the LM75B(tempera-
 * ture sensor) are always connected via I2C_#1, on NOT real open-drain PINs.
 * 
 * NXP's ROM-API is used in polling-based mode. There is no include of 
 * time.h here, this is HAL software and time.h is higher-Up.
 *
 *
 * -- This source-code is released into the public domain, by the author --
 *
 * This file contains, is Unlicensed material:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABI-
 * LITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT 
 * SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT 
 * OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE.
 *
 * The author of this material /source-code kindly ask all users and distri-
 * butors of it, to leave this notice / text in full, in all copies of it, 
 * for all time to come.
 */

#ifndef __MIN_CHIP_ON_BOARD_H
#define __MIN_CHIP_ON_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>



/****************************************************************************
 * Coworking chips (temperature sensor and Real-Time Clock/Calendar) 
 * definitions follows here.
 */

/* The I2C channel used on the MCU, if changes is needed, see i2c_onMPU_Init */
#if defined (CHIP_LPC82X)
#define MPU_I2C              LPC_I2C1  
#define MPU_I2C_BASE         LPC_I2C1_BASE
#define MPU_I2C_BITRATE      (400000)
#define MPU_I2C_SDA_PIN      17
#define MPU_I2C_SCL_PIN      18
#endif


/* PCF85063A Real-Time Clock/calendar */
#define PCF85063A_ON_BOARD   ((0x51) << 1)  /* Chip I2C address (A2h) */
  /* - PCF85063A Control Register addresses -> */
#define PCF85063A_CTRL1      0x00 /* Status bits */
#define PCF85063A_CTRL2      0x01
#define PCF85063A_OFFSET     0x02 /* For calibration/ Temperature compensation */
#define PCF85063A_BYTE       0x03 /* 1(one) byte storing */
  /* - PCF85063A Time and Date registers - */
#define PCF85063A_SC         0x04 /* Time/date holder */
#define PCF85063A_MN         0x05
#define PCF85063A_HR         0x06
#define PCF85063A_DM         0x07
#define PCF85063A_DW         0x08
#define PCF85063A_MO         0x09
#define PCF85063A_YR         0x0A
#define PCF85063A_SC_READY   PCF85063A_SC /* Then We point to first(sec) reg. */
/* Block integrity-Bit, stored in seconds register(_SC), SET by Default */
#define PCF85063A_INTEGRITY       0x80
/* AM/PM indicator, stored in hours register(_SC) bit-5, IF ENABLED in CTRL reg.*/
#define PCF85063A_PM_INDICATOR    0x20 

/* - Control-1 reg bits -> */
#define PCF85063A_CTRL1_STOP      0x20 /* STOP and set all RTC dividers to 0 */ 
                                       /* NOTE: We do NOT use the Software Reset */
#define PCF85063A_CTRL1_CIE       0x04 /* Correction interrupt enable */
#define PCF85063A_CTRL1_12_24     0x02 /* If SET the 12-hour mode AM/PM */
#define PCF85063A_CTRL1_CAP_SEL   0x01 /* SET for Xtel capacitance at 12.5 pF */ 

/* - Control-2 reg bits -> */
#define PCF85063A_CTRL2_AIE       0x80 /* Enable Time-alarm interrupts */
#define PCF85063A_CTRL2_AF        0x40 /* Alarm flag, Write-To-Clear */
#define PCF85063A_CTRL2_MI        0x20 /* Enable Minute Interrupt */
#define PCF85063A_CTRL2_HMI       0x10 /* Enable Half Minute Interrupt */
#define PCF85063A_CTRL2_TF        0x08 /* Highest Priority in ISR. Timer-Flag SET when 
                                        * timer interrupt is generated. */
/* COF[2:0] CLKOUT(a PIN IO) frequency in Hz, these frequencies are NOT effect of 
 * the STOP-bit */ 
#define PCF85063A_CLKOUT_32768    0x00 /* (Default value) */
#define PCF85063A_CLKOUT_16384    0x01
#define PCF85063A_CLKOUT_8192     0x02

/* - PCF85063A Alarm registers - */
#define PCF85063A_AEN_SC      0x0B /* SECOND_ALARM (0 to 59) */
#define PCF85063A_AEN_MN      0x0C /* MINUTE_ALARM (0 to 59) */
#define PCF85063A_AEN_HR      0x0D /* HOUR_ALARM (0 to 23) in 24-hour mode */
#define PCF85063A_AEN_DM      0x0E /* DAY_ALARM (1 to 31) */
#define PCF85063A_AEN_DW      0x0F /* WEEKDAY_ALARM (0 to 6) */

/* - PCF85063A Timer Register - */
#define PCF85063A_TIMER      0x10 /* Timer value */
#define PCF85063A_TIMER_M    0x11 /* Timer mode */
/* TCF[1:0] is bit [4] & [3] in Timer mode reg. */ 
#define PCF85063A_TIMER_M60SEC    0x18 /* TIMER_M Bit[4] & [3]. Timer Duration, 1 min. */
#define PCF85063A_TIMER_M_TE      0x04 /* Enable Timer */
#define PCF85063A_TIMER_M_TIE     0x02 /* Enable Timer Interupt (Just Reg. Flag) */
#define PCF85063A_TIMER_M_TI_TP   0x01 /* Enable Timer interrupt pulse on PIN IO */


/* LM75B temperature sensors */
#define LM75B_ON_BOARD       ((0x48 + 0x07) << 1)  
#define LM75B_OFF_BOARD      ((0x48) << 1)
#define LM75B_MIN_TEMP       (-55)
#define LM75B_MAX_TEMP       125
#define LM75B_TEMP_REG       0x00    /* Used as two bytes depth */
#define LM75B_STAT_REG       0x01    /* One byte */
#define LM75B_WAKE_UP        0x00    /* Power-down Bit */
#define LM75B_SLEEP_BIT      0x01    /* Power-down Bit */

/* Constant-strings for controlling I2C temperature sensors power-down stat */
extern const uint8_t strTempOnBoardPwUp[];
extern const uint8_t strTempOnBoardPwDown[];
extern const uint8_t strTempOffBoardPwUp[];
extern const uint8_t strTempOffBoardPwDown[];




/****************************************************************************
 * Co-chips API -->
 */


/* Init the on-MPU/board I2C channel used for Real-Time Clock and temperature */
void i2c_onMPU_Init(void);
/* Short pause, without RTOS involvement (units is: 1-3 mSec.) */
void i2c_minDelay(uint32_t count);

// BCD, normal NIPPEL(4-bit) based in, out comes unsigned decimal.
STATIC INLINE uint8_t BCD_ToD(uint8_t bcdFormat)
{
  return ((bcdFormat & 0xf0) >> 4) * 10 + (bcdFormat & 0x0f);
} /* BCD_ToD */

STATIC INLINE uint8_t D_ToBCD(uint8_t intByte)
{
  return ((intByte / 10) << 4) + (intByte % 10);
} /* D_ToBCD */

 
// TEMP, here
uint32_t i2c_onMPU_write(uint8_t* send8, uint32_t numOfByte, bool stopFlag); 
uint32_t i2c_onMPU_read_direct(uint8_t* recv8, uint32_t numToRecv);  /* numToRecv + 1 (the addr) */







/* BitsToCelsius(): 
 * Convert two bytes from LM75B(I2C) to a signed Int in Degrees Celsius. 
 * 
 * Warning: 
 * In this code-piece Ternary Operator are used and it isn't compliant 
 * with MISRA-C rules, but here it looks right. */
//STATIC INLINE int8_t BitsToCelsius(uint8_t bitsInt, uint8_t bitsFrac) 
STATIC INLINE int8_t BitsToCelsius(uint8_t bitsInt, uint8_t bitsFrac) 
{
 /* FIRST: Cast to signed number */
 /* THEN: Round-up and add-on the decimal part if greater than 0.5 degree C */

  return (int8_t) bitsInt + (bitsFrac > 128 ? 1 : 0);
} /* BitsToCelsius */


/* Functions/INLINEs for handling the temperature sensors */

/* Tell the temp. sensor to wake Up. (Note: first sampling is available 
 * after 100 mS.) */
STATIC INLINE uint32_t TempOnBoardPwUp(void) 
{
  return i2c_onMPU_write((uint8_t*) &strTempOnBoardPwUp, 3, TRUE);
} /* TempOnBoardPwUp */

/* Tell the temp. sensor to go to sleep */
STATIC INLINE uint32_t TempOnBoardPwDown(void) 
{
  return i2c_onMPU_write((uint8_t*) &strTempOnBoardPwDown, 3, TRUE);  
} /* TempOnBoardPwDown */

/* For the Off-board temperature sensor */
STATIC INLINE uint32_t TempOffBoardPwUp(void) 
{
  return i2c_onMPU_write((uint8_t*) &strTempOffBoardPwUp, 3, TRUE);  
} /* TempOffBoardPwDown */

STATIC INLINE uint32_t TempOffBoardPwDown(void) 
{
  return i2c_onMPU_write((uint8_t*) &strTempOffBoardPwDown, 3, TRUE);  
} /* TempOffBoardPwDown */





#ifdef __cplusplus
}
#endif

#endif /* __MIN_CHIP_ON_BOARD_H */

