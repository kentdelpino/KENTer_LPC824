/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * Linker:   linkLPC812.ld
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
 */

/* Stack size in bytes. MICK-OS tasks uses Stacks in the .bss area, NOT this on */
SIZE_OF_MAIN_STACK = 256;


MEMORY
{
	/* NXP LPC812: 16Kb FLASH and 4Kb RAM(- 32byte at top for IAP flash
	 * programming).
	 */
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00004000
	RAM (rwx) :  ORIGIN = 0x10000000, LENGTH = 0x00001000 - 0x020
}


/****************************************************************************
 * The code and data is linked according to linkCOMMON.ld.
 */

INCLUDE ./controller/linkCOMMON.ld

