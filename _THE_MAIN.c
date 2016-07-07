/***************************************************************************
 *                                     @Link www.linkedin.com/in/kentdelpino
 *                                       @Author Kent DEL PINO
 * @file: 	 _THE_MAIN.c
 * @version: 0.2.0
 * @date: 	 10 Mar. 2016
 *
 * @par
 * A small test-setup to run on a NXP LPC824 (Cortex-M0+ based) using 
 * makefile and linker script on a GNU compiler. newlib-nano is used and
 * among other things, this program-example shows printf() use via 
 * retarget-stubs. A simple Kernel Scheduler is involved, so this main() 
 * is about init and starting Threads/Tasks up.
 * 
 * One goal is to make use of the On-chip ROM APIs for integer division, 
 * ADC, SPI, I2C, USART, power configuration (power profiles).
 * 
 *  - This software DO NOT include DMA controller of the LPC824 -
 * 
 * Complied with ARM Embedded GCC 5.2.x-2015q4 (or newer), see:
 *    https://launchpad.net/gcc-arm-embedded
 *
 * The HAL layer is LPCopen_2_19, for info see: 
 *    http://www.lpcware.com/content/nxpfile/lpcopen-platform 
 *    
 * but, but we are using the 2.19 HAL layer files from NXPs SCT-
 * Timer/PWM cookbook distribution, for LPC842.
 * 
 * 
 * Running on the internal RC-oscillator (12MHz +-1.5%) with the PLL
 * set to core speed at 24 or 30MHz.
 *
 * One-liner comments in this files can be enclosed as:
 *   // TEXT-COMMENT,,, <\n>    ;as accepted in ANSI C, not MISRA C.
 *   
 * And I use spaces, not TABs in c and h files, Yeah it's a issue:
 *   http://jarrodoverson.com/blog/spaces-vs-tabs/  
 *   
 *
 *   
 * --This source-code is released into the public domain, by the author--
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
 **************************************************************************/


/* Minimum hardware access on this level */
#include "minimum_board.h"
/* Minimum Inter-process Communication Kernel (MICK), it also included: _DEF_YOUR_TASKS.h and time.h*/
#include "mick.h"



/**************************************************************************/
int main(void) {

 /* MPU: Reset all peripheral units, exception the flash controller */
//	PRESETCTRL
 /*	MPU: Enables clock for GPIO port-registers and GPIO-pin interrupts 
  * and let it run. - Yes it is also default -. */
  Chip_GPIO_Init(LPC_GPIO_PORT);
 /* MPU: Update the system-Var with correct CPU(core) speed-information. */
  SystemCoreClockUpdate();
 /* Setup some IO-PINs, functions on PINs, a board-LED, I2C monitor 
  * chip and UART0 related buffers (FIFO). Use UART0 for stderr stdin 
  * and stdout via printf() and fgets(). */
  MinimumSetUp(DEFAULT_BAUD_RATE);
 /* Turn-OFF unused internal clocks and peripheral units. Remember here
  * We are using the ROM. Unused IO-pins is set to output in LOW-stat, 
  * with the pull-up disabled, ALSO ON IO-PINs that are only internal.
  * 
  * If perifer-clocks are needed, use Chip_Clock_EnablePeriphClock().
  */
  MinimumConsumption();

 /* MPU: Read Part Identification number */

 /* MPU: Read ROM/ Boot-code version */

 /* MPU: Why are we here, Is it due to the normal power-up!. This info. can 
  * be found in lastResetDueTo. */

 /* Initialization of communication to extern peripheral units. */ 
//  MinimumInitRTclock();
//  MinimumInitTemp();
//  MinimumInitEEPROM();

 /* Start scan the stdin port, with AUTO_RESTART after <CR> <NL> */
  MinimumGetStartUp(AUTO_RESTART);
 /* Start MPU-watchdog before starting the scheduler, the OS */


 /* First the Kernel and the the Tasks */
  osKernelInit(); 
 /* JSON objects incoming on STDIN(USB/WiFi or ttyS0/com1) or on RS-485(ttyS2/COM3) */
  osThreadCreate(IN_JSON_ID, task_stdin_json, IN_JSON_STACK, IN_JSON_PRIO);
 /* On queries, We respond on STDOUT or on RS-485 */
  osThreadCreate(OUT_JSON_ID, task_stdout_json, OUT_JSON_STACK, OUT_JSON_PRIO);
 /* A safety thread/task looking into system health and is nulling the watchdog timer */
  osThreadCreate(SYS_HEALTH_ID, task_sys_health, SYS_HEALTH_STACK, SYS_HEALTH_PRIO);
 /* The actual application */
  osThreadCreate(OUR_APP_ID, task_application, OUR_APP_STACK, OUR_APP_PRIO);

 /* Master NODEs can have Use-interface/Display */
  osThreadCreate(DISPLAY_ID, task_display, DISPLAY_STACK, DISPLAY_PRIO);

 /* Is there more to say! */
  osKernelRun();

 /* Just as a reminder */  
  NVIC_SystemReset();
} // main()

