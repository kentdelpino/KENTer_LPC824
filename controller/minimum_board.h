/****************************************************************************
 * TODO:
 * 
 * In C-source I faulty use !(“logical NOT”) where I should use Bitwise NOT "~", like ~(value)
 *
 * TODO: Error when more "\r\n", like printf("\r\n - p = %d\r\n", ipcArrayPtr)        
 * 
 *
 */




/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: minimum_board.h
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.5.5
 * Date:     21 Oct. 2015
 *
 * This minimum_board file contains a limited codebase for initialization of 
 * a minimum functionality. UART0 is set up and can be used directly or via 
 * stdio C tools such as stdin, and stdout.  
 *
 *  NOTE: 
 *  
 *    a/ This file is distributed as a part of the KENTer_LPC package. 
 * 
 *    b/ Core hardware setup such as core(CPU)-speed and memory wait states, 
 *       is done in startup_lpc code.
 *
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


#ifndef __MINIMUM_BOARD_H
#define __MINIMUM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
/* The micro-controller */
#include "chip.h"
/* The on-board chip for Real-Time Clock/calendar and temperature */
#include "min_chip_on_board.h"

#if defined (MICK_OS)
#include "mick.h"
#endif



/****************************************************************************
 * Board and coworking chip (temperature sensor and Real-Time Calendar) 
 * definitions are defined in min_chip_on_board.h.
 */







/****************************************************************************
 * The Micro-controller (MPU) clock and pin definitions  -->
 */


/* The on-board LED-indicator on a KENTer, would normally be cathode-
 * connected to IO-PIN. If NOT defined, non initialization of a output-
 * pin is done. */
#if defined (CHIP_LPC82X)
#define BOARD_LED_PIN        9   /* On LPC824, it's always the XTALOUT */
#else
 //#define BOARD_LED_PIN       17 
#endif



/* -- Limitations of inrush current on directly connected N-channels MOSFETs.
 * NOTE: NOT a BIG effect, but helps a little AND WE DON'T WANT pull-up on 
 * something that is pulled-down anyway, the MOSFET gate  --
 * 
 * Due to Inrush current on connected N-channels MOSFETs, we can, here
 * disable the IO-pins pull-up, it makes us remember the problem and helps 
 * a little while starting up. Main issues is the delay after RESET time 
 * until code executes, is more than 3 ms. */

//#ifdef DISABLE_PIN_PULL_UP

#define MOSFET_PIN0        IOCON_PIO15
#define MOSFET_PIN1        IOCON_PIO6
#define MOSFET_PIN2        IOCON_PIO16
#define MOSFET_PIN3        IOCON_PIO2



/* Baud-rate for default serial IO, stdin, stdout and stderr. If NOT defined
 * non up-starts initialization of the UART is done. */
#define DEFAULT_BAUD_RATE   115200
#define MIN_STDIN_BUFFER_SIZE   96


/* CPUcore speed and MAIN-clock:
 * While running from internal RC(12MHz) oscillator, tell how fast should 
 * the CPUcore-clock and memory run in MHz. 
 * 
 * NOTE: If __WANTED_PLL_MAIN is defined then the oscillator initialization-
 * code will be stored in flash memory, not the ROM. For different possibi-
 * lities with PLL_MAIN, see LPC_8XX_PLL_T config_tab (in irc_8xx.c). 
 */ 
// MAIN-clock, the PLL output in MHz. 
//#define  __WANTED_PLL_MAIN  48000000 
// The CPUcore speed in MHz.
#define __WANTED_CORE_FREQ  24000000 

/* Forced low FLASH access time
 * For test purpose or special applications, force FLASH Access time to only 
 * 1 CPUcore clock cycle, even at CPU speeds greater than 20MHz. 
 * 
 * NOTE: Usually disabled(undefined). Should be used with caution, but can 
 * ALWAYS be used with CPU speed lower than 20MHz. 
 */
//#define _FORCED_FLASH_SPEED

/* XTAL and CLKIN For use in an external clock-source
 */
// System Xtal oscillator rate
#define __EXT_XTAL          25000000
// CLKIN pin frequency
#define __EXT_CLKIN         25000000



/****************************************************************************
 * Board initialization
 */

/*****************************************************************************
 * MinimumConsumption(): Turn OFF perifer clocks to limit power consumption.
 *
 * If the open-drain pins PIO0_10 and PIO0_11 are not available or used on 
 * the package, prevent the pins from internally floating as follows: 
 * Set bits 10 and 11 in the GPIO DIR0 register to 1 to enable the output 
 * driver and write 1 to bits 10 and 11 in the GPIO CLR0 register to 
 * drive the outputs LOW internally. (TEXT from: UM10601.pdf page 86 ).
 */
void MinimumConsumption(void);


/*****************************************************************************
 * MinimumPinConf: Chip-pin assignment for functions and PCB-layout.
 *
 * For visualisation of the PIN configuration We use: 
 *   http://nxpblmcusw.com:8080/lpcconfig/lpcconfig
 *
 * The notation is: LPCOpen Peripheral Registers, not CMSIS Peripheral. 
 *
 * NOTE: We are enable clocks to the IOCON register and SwitchMatrixin the 
 * SYSAHBCLKCTRL register (Table 30, bit 18). Once the pins are configured, 
 * We disable these clocks again to conserve power.
 * 
 * And for I2C -on -standard digital I/O pin, disable the internal pull-up
 * in the IOCON register.
 */
void MinimumPinConf(void);



/* */
void MinimumInitLED(void);

/* */
void MinimumInitUART(const uint32_t baudrate);


STATIC INLINE void MinimumSetUp(uint32_t baudrate) 
{
 /* First the minimum PIN config */
  MinimumPinConf();
 /* Enable peripheral I2C #1 (Real-Time Clock/calendar and temperatur) */
  i2c_onMPU_Init();  
  
 /* Then other functions */
#ifdef BOARD_LED_PIN
  MinimumInitLED();
#endif
#ifdef DEFAULT_BAUD_RATE
  MinimumInitUART((uint32_t) baudrate);
#endif  
}




/****************************************************************************
 *  Blinking LED on board, only if needed 
 */

#ifdef BOARD_LED_PIN

#define BOARD_LED_OFF      0
#define BOARD_LED_ON       0x55

extern volatile uint8_t onBoardIndicator_CTRL;

STATIC INLINE void MinimumToggleLED(void) 
{
  if (++onBoardIndicator_CTRL == 1)
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, BOARD_LED_PIN, 0);
  else {
    onBoardIndicator_CTRL = BOARD_LED_OFF;
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, BOARD_LED_PIN, 1);
   } 
}

STATIC INLINE void MinimumLED_ON(void) 
{
  onBoardIndicator_CTRL = BOARD_LED_ON;
  Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, BOARD_LED_PIN, 0);
}

STATIC INLINE void MinimumLED_OFF(void) 
{
  onBoardIndicator_CTRL = BOARD_LED_OFF;
  Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, BOARD_LED_PIN, 1);
}

#endif /* BOARD_LED_PIN */




/****************************************************************************
 * Serial(UART0) communication (UART0) for stdin and stdout. The tools can 
 * be used directly for asynchronous serial communications purpose. The 
 * functionality here is also part of Kent DEL PINO's newlib retargeting.
 * 
 * These services/functions are built upon the microcontrollers ROM-based 
 * code (included from factory) and alled via API, runs in interrupt-mode. 
 */


/* Flags used to indicate comm. status, signaling END of a TX or RX process 
 * etc. */
#define TX_FLAG_MASK_OUT        (0xFF000000)
#define TX_COMPLETED            (0x10000000)
#define TX_STARTED_FLAG         (0x20000000)
#define TX_BUSY_FLAG            (0x01000000)

#define RX_FLAG_MASK_OUT        (0x00FF0000)
#define RX_COUNT_MASK_OUT       (0x0000FFFF)
#define RX_ALL_MASK_OUT         (RX_FLAG_MASK_OUT | RX_COUNT_MASK_OUT)
#define RX_AUTO_RESTART         (0x40000000)
#define AUTO_RESTART             TRUE
#define RX_COMPLETED            (0x00100000)
#define RX_REQUESTED            (0x00200000)
#define RX_BUSY_FLAG            (0x00400000)
#define RX_ROM_BUSY_FLAG        (0x00080000)

// Holder of the flags, for TX or RX process.
extern volatile uint32_t minCommSendStat;
extern volatile uint32_t minCommReceiveStat;

// Is there data-ready
#define MIN_RECEIVED_DATA         (minCommReceiveStat & RX_COMPLETED)
// Length of data-package, NOT INCLUDING THE ZERO-termination.
#define MIN_RECEIVED_LEN_OF_DATA  ((int) ((uint32_t) minCommReceiveStat & RX_COUNT_MASK_OUT))
// Process halt until Put-Completed, previous MinimumPut().
STATIC INLINE void MinWaitPutCompleted(void)
{
  while (!(minCommSendStat & TX_COMPLETED)) {}
}


/* MinimumPut(): 
 * Put Number-Of-Bytes(len) from buffer(buff) to the UART0 (stdout). 
 * This function is best suited on strings/byte-arrays, not good at single-
 * byte level. 
 * 
 * Return, On success; Number-Of-Bytes or on failure; -1.
 *
 * NOTE: The buffer(buff) should not be changed until TX_COMPLETED. if You 
 * need to insert a pause, use MinWaitPutCompleted(void).
 */
int MinimumPut(uint8_t *buff, int len);


/* MinimumGetStartUp(): 
 * Prepare the receipt of incoming bytes or a array-of-chars from UART0
 * (stdin). The bytes a filled in a buffer and a flag (RX_COMPLETED) is 
 * set upon the reception of CR+LF or full-buffer (MIN_STDIN_BUFFER_SIZE). 
 * The reception takes place in the background (interrupt based).
 * 
 * Takes autoRestart(bool), if TRUE the receiver process is to be continued 
 * automatic after emptying the buffer with MinimumGet().
 * 
 * Returns, On success; MIN_STDIN_BUFFER_SIZE or on failure; -1.
 * 
 * NOTE: 
 * a/ Starting this function will Null the receiver-buffer, erase previous 
 * received data.
 */
int MinimumGetStartUp(bool autoRestart);


/* MinimumGet(): 
 * Transfer the UART receive-buffer to a higher application-layer or to 
 * newlib stubs. This function is best suited on strings/byte-arrays and 
 * minimum length of target-buffer is 2.
 * 
 * Takes a pointer to the target-buffer (buff) and the length/ size of the 
 * target-buffer (maxLen).
 * 
 * Returns, On success; Actual-Number-Of-Bytes transferred or on failure; -1.
 * 
 * NOTE: The returned byte-array is terminated by a ZERO(a add on), so if a 
 * byte-array is later interpret as a string, there is a terminator.
 */
int MinimumGet(uint8_t *buff, int maxLen);





#ifdef __cplusplus
}
#endif

#endif /* __MINIMUM_BOARD_H */
