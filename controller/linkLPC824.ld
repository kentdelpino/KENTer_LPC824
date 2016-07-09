/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * Linker:   linkLPC824.ld
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  1.0.0
 * Date:     9 Nov. 2015
 *
 * NOTE: This is only the memory definition for the specific MCU and size of 
 * stack. The code and data is linked according to linkCOMMON.ld.
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
 * 
 * 
 * A little on Linker Script:
 *         http://www.hertaville.com/2012/06/29/a-sample-linker-script/
 */


/* Main and Interrupt stack-size, in Bytes. OS tasks uses stack-space in the 
 * .bss area, NOT here. 
 */
SIZE_OF_MAIN_STACK = 640;


MEMORY
{
	/* NXP LPC824: 32Kb FLASH and 8Kb RAM. In some cases one want to protect 
         * the upper-part of RAM, which can be used by the bootloader (only if 
         * active): -0xC0; -0x60; -0x30.
	 */
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00008000
	RAM (rwx) :  ORIGIN = 0x10000000+0x00, LENGTH = 0x00002000-0x00
}


/****************************************************************************
 * The code and data is linked according to linkCOMMON.ld.
 */

INCLUDE ./controller/linkCOMMON.ld

