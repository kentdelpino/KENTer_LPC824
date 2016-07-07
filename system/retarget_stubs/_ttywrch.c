/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * Linker:   _ttywrch.c
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.2
 * Date:     28 May. 2015
 *
 * NOTE: 
 *    Distributed as a part of the KENTer_LPC8xx_sys package for LPC8xx MCUs.
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


/****************************************************************************
 * TODO:
 *  
 */


#include "_ttywrch.h"


/* _ttywrch() - This function writes a character to the stderr-port/UART0 
 * and can used as a last resort error handling routine. The default (n.1) 
 * implementation of this function uses the semihosting SWI. Here each char
 * is outputted directly to the UART0 transmitter register. UART0 is used 
 * for stderr, UART0 have to be initialized before use. 
 *
 * n.1) Default in the compiler distribution: 
 *                     https://launchpad.net/gcc-arm-embedded
 */


__attribute__ ((used)) void _ttywrch(uint8_t ch) {

 // Wait until we're ready to send 
  while (!(Chip_UART_GetStatus(LPC_USART0) & UART_STAT_TXRDY));
 // Put ch, to the port
  Chip_UART_SendByte(LPC_USART0, ch);  
} // _ttywrch

