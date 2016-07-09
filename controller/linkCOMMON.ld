/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * Linker:   linkLPC812.ld
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  1.3
 * Date:     27 May. 2015
 *
 * NOTE: This linker script included SECTIONS for Code/Software protection
 * on the NXP LPC8xx series.
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
 * For the future development of makerfiles and link-script files, the
 * templates from the compiler distribution (GCC ARM Embedded in Launchpad)
 * should be used as inspiration. The templates/samples can be found:
 *    _installation_folder_\share\gcc-arm-none-eabi\samples
 *
 * Inserted by Kent DEL PINO; 17 Apr. 2015
 */



/****************************************************************************
 * MAIN_STACK:    <- In this version we run another MEM-model ->
 *
 * Define the size of Stack here, in byte. Only the Main Stack Pointer
 * (MSP) of the Cortex-M0 is used in this setup.
 *
 * NOTE: Here the Stack (MSP) is placed as the first in RAM memory, the
 * stack grows downward, Stack overflow can then generate Hard-Fault
 * exception and the Stack cannot overwrite other data-segments.
 *
 * The Heap is placed on top of all the others data-segments, and is growing
 * up when used. The heap-size is equal to, what is left over.
 */



ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        FILL(0xff)
        _text = .;
       /* First(at 0x0) The reset/interrupt vector */
        KEEP(*(.isr_vector))
       /* The Startup part with initialization of variables, statics and ISR */
        KEEP(*(.after_vectors))

       /* @BEGIN: Code Read Protection */
       /* If Code/Software protection is NOT needed at all, and ENABLE_CODE
        * _PROTECTION is NOT_DEFINED, then alle text from "@BEGIN: Code,," to
        * "@END: Code,," can be deleted (or made inactive). */
       . = 0x000002FC;
        KEEP(*(.code_protect))   
       /* @END: Code Read Protection */

        *(.text*)
	   /* The applications code */
		*(.rodata*)
	   /* WE DON'T WANT IT, but for now: For exception unwinding among others. */
		KEEP(*(.eh_frame*))
    } > FLASH

	.ARM.extab : 
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > FLASH

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > FLASH
	__exidx_end = .;


   /* End Of Text FLASH-ROM */
	__etext = .;


   /* RAM memory: Now organization it */
	. = ORIGIN(RAM);

   /* We placed the Stack as the first element in RAM, it grows downwards
    * NOTE: Normally a down-growing Stack, will be placed in TOP-OF-RAM.
    */
    .stack :
	{
		__StackLimit = .;
		. = . + SIZE_OF_MAIN_STACK;
		. = ALIGN(4);
		__StackTop = .;
	} > RAM
    PROVIDE(__stack = __StackTop);

	.data : AT (__etext)
	{
		__data_start__ = .;
		*(vtable)
		*(.data*)
		__data_end__ = .;
	} > RAM

	.bss :
	{
		. = ALIGN(4);
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
	} > RAM

   /* The Heap gets the rest of RAM, Remenber ALIGN(4), but __bss_end__ was just ALIGN(4) */ 
	.heap :
	{
		__end__ = .;
		PROVIDE(end = .);
	} > RAM
	PROVIDE(__HeapLimit = ORIGIN(RAM) + LENGTH(RAM));

}
