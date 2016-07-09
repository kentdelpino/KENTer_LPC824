/***************************************************************************
 *                                   Link: www.linkedin.com/in/kentdelpino
 *                                   Author: Kent DEL PINO
 * File: min_chip_on_board.c
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "minimum_board.h"
#include "min_chip_on_board.h"






/***************************************************************************
 * Shared I2C low-level variables, no interrupts here, one thing at the 
 * time. */

/* I2C handler (master) and memory for ROM API */
static I2C_HANDLE_T *i2c_1_Handler;
static I2C_RESULT_T i2c_1_Result;
/* Handler work-space, use other return-butter for time/date and temperature */ 
static uint32_t i2c_1_HandlerMEM[24]; 




/***************************************************************************
 * I2C low-level, temperature sensors */

/* Constant-string (in code/flash), for controlling I2C temperature sensors */
//static 
const uint8_t strTempOnBoardPwUp[] = { LM75B_ON_BOARD, LM75B_STAT_REG, LM75B_WAKE_UP };
const uint8_t strTempOnBoardPwDown[] = { LM75B_ON_BOARD, LM75B_STAT_REG, LM75B_SLEEP_BIT };
const uint8_t strTempOffBoardPwUp[] = { LM75B_OFF_BOARD, LM75B_STAT_REG, LM75B_WAKE_UP };
const uint8_t strTempOffBoardPwDown[] = { LM75B_OFF_BOARD, LM75B_STAT_REG, LM75B_SLEEP_BIT };



/* NOTE: Power-save, leaving this function with no clock on, just 
 * initialized the unit. */
void i2c_onMPU_Init(void) {

 // Set up PINs on Micro controller and init handler 
#if defined (CHIP_LPC82X)

  /* page 34  Table 23. Peripheral reset control register (PRESETCTRL, address 0x4004 8004) bit 
  description */  
  
 // Reset the unit 
  
  
  
 // Power-Up, by Enable the clock to I2C-unit
  Chip_I2C_Init(MPU_I2C);

 // Assign i2c_01 to the PINs
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
  Chip_SWM_MovablePinAssign(SWM_I2C1_SCL_IO, MPU_I2C_SCL_PIN); 
  Chip_SWM_MovablePinAssign(SWM_I2C1_SDA_IO, MPU_I2C_SDA_PIN); 
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
 // Set PINs for Open-drain
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
  Chip_IOCON_PinSetMode(LPC_IOCON, MPU_I2C_SCL_PIN, PIN_MODE_INACTIVE);
  Chip_IOCON_PinSetMode(LPC_IOCON, MPU_I2C_SDA_PIN, PIN_MODE_INACTIVE);
  Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON, MPU_I2C_SCL_PIN, TRUE);
  Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON, MPU_I2C_SDA_PIN, TRUE);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);

 // Setup the I2C_1 ROM-handler
  i2c_1_Handler = LPC_I2CD_API->i2c_setup(MPU_I2C_BASE, i2c_1_HandlerMEM);
 // Set I2C bitrate, just slow 
  LPC_I2CD_API->i2c_set_bitrate(i2c_1_Handler, Chip_Clock_GetSystemClockRate(), MPU_I2C_BITRATE);
 // Set timeout, 1 = 16* i2c-function-clock.
  LPC_I2CD_API->i2c_set_timeout(i2c_1_Handler, 100000); 
  
 // Power-down: by Stoping the Clock to MPU-internal I2C-unit.
  Chip_I2C_DeInit(MPU_I2C); 
#else
#error "Only support for CHIP_LPC82X"
#endif 
} /* i2c_onMPU_Init */



/* Short pause, without RTOS involvement */
void i2c_minDelay(uint32_t count) {

 // La-la,  bla is 1-3 mSec.
  uint32_t bla = 0x01fff;
  uint32_t blaBla = count;

  while (blaBla >= 1) {
    while (bla-- >= 1) {}
    blaBla--;
   }  
} /* i2c_minDelay */


/* Master write in polling mode, always 7bit addr. at pos. send8[0] and target 
 * reg. in send8[1] */
//static
uint32_t i2c_onMPU_write(uint8_t* send8, uint32_t numOfByte, bool stopFlag) {

  I2C_PARAM_T param;

  param.num_bytes_send = numOfByte;
  param.buffer_ptr_send = send8;
  param.num_bytes_rec = 0;
 // Acknowledge on each byte, maybe ends with stop-signal.
  if (stopFlag)
    param.stop_flag = 1;
  else
    param.stop_flag = 0;    

 // Set timeout, 1 = 16* i2c-function-clock.
  LPC_I2CD_API->i2c_set_timeout(i2c_1_Handler, 100000); 

 // Do master write transfer and return zero(0) if OK.
  return LPC_I2CD_API->i2c_master_transmit_poll(i2c_1_Handler, &param, &i2c_1_Result);
} /* i2c_onMPU_Write */



/* Master Read in polling mode, always 7bit addr. at pos. recv8[0] */
//static /* numToRecv + 1 (the addr) */
uint32_t i2c_onMPU_read_direct(uint8_t* recv8, uint32_t numToRecv) {

  I2C_PARAM_T param;
  
 /* Setup I2C paameters for number of bytes with stop - appears as:
       Start - address7(value 0) - ack
       value 1 (read) - ack
       value x (read) - ack
       value x (read) - ack
       value x (read) - ack - stop */
  
  param.num_bytes_send    = 1;  // Yes write the Addr, but nothing else
  param.num_bytes_rec     = numToRecv;
  param.buffer_ptr_rec    = recv8;
  param.stop_flag = 1;

 /* Do the Master-read  */
  return LPC_I2CD_API->i2c_master_receive_poll(i2c_1_Handler, &param, &i2c_1_Result);
} /* i2c_onMPU_read_direct */











