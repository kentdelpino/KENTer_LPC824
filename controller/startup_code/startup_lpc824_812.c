/**************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: startup_lpc824_812.c
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  1.5
 * Date:     12 Nov. 2015
 *
 *
 * This file contain source-code under different licenses:
 * - Code additions done by Kent DEL PINO, License: Public Domain.
 *   - Simple crt0 (LPC810_CodeBase) made by Kamal Mostafa, License: 
 *     Public Domain. The cr_startup_lpc8xx.c is the original inspiration
 *     The cr_startup_lpc8xx.c is from NXP Semiconductor and NXP sorce-
 *     code is for NXP manufactured hardware.
 *
 * Common for all license-holder:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OFMERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


/* There is none C++ support */
#if defined (__cplusplus)
#error KENTer has no C++ support, mañana
#else /* Go from here */


/**************************************************************************
 * Code Read Protection (CRP): 4 bytes allocate, no protection.(DISABLED) =
 * 0xffffffff = CRP_NO_CRP. To enable customers to update firmware via the
 * ROM based ISP tool FlashMagic (flashmagictool.com) or similar, and be
 * slightly protected use: CRP_CRP1
 * If this feature is to be used, then define:(ENABLE_CODE_PROTECTION), in
 * the makefile and enable the .code_protect section at address 0x000002FC 
 * in the linker script-file.
 */
#if defined (ENABLE_CODE_PROTECTION)
#include "protect.h"
__attribute__ ((used, section(".code_protect")))
const unsigned int PROT_WORD = CRP_NO_CRP;
#endif


#include <stdint.h>
// For coreCPU clock-settings and chip.h diff..
#include "minimum_board.h"


/**************************************************************************
 *
 */

// Patch the AEABI integer divide functions to use MCU's romdivide library
#if defined (__USE_ROMDIVIDE)
// Location in memory that holds the address of the ROM Driver table
#define PTR_ROM_DRIVER_TABLE ((unsigned int *)(0x1FFF1FF8))
// Variables to store addresses of idiv and udiv functions within MCU ROM
unsigned int *pDivRom_idiv;
unsigned int *pDivRom_uidiv;
#endif



/**************************************************************************
 * WEAK forward declarations of Cortex-M0 core exception and interrupt
 * handlers. If the application programmer define a new handler with
 * the same name, this new handler will automatically take precedence
 * over these WEAK define. Note that the Reset_Handler here, is final.
 *
 * The group of core exception following here confirm to:
 *     Cortex-M0 Devices Generic User Guide, page 2-20.  (so I remember)
 */

/* Make it a little easy */
#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

     void Reset_Handler(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);



/**************************************************************************
 * Forward declaration of the specific MPU IRQ handlers. These are aliased
 * to the IntDefaultHandler, which is a 'forever' loop. When in the applica-
 * tion a new handler is defines with the same name, this will automatically
 * take precedence over these weak definitions
 */
WEAK void IntDefaultHandler(void);

#if defined (CHIP_LPC82X)
void SPI0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C1_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SCT_IRQHandler(void) ALIAS(IntDefaultHandler);
void MRT_IRQHandler(void) ALIAS(IntDefaultHandler);
void CMP_IRQHandler(void) ALIAS(IntDefaultHandler);
void WDT_IRQHandler(void) ALIAS(IntDefaultHandler);
void BOD_IRQHandler(void) ALIAS(IntDefaultHandler);
void FLASH_IRQHandler(void) ALIAS(IntDefaultHandler);
void WKT_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC_SEQA_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC_SEQB_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC_THCMP_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC_OVR_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C2_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C3_IRQHandler(void) ALIAS(IntDefaultHandler);
#else 
void SPI0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C_IRQHandler(void) ALIAS(IntDefaultHandler);
void SCT_IRQHandler(void) ALIAS(IntDefaultHandler);
void MRT_IRQHandler(void) ALIAS(IntDefaultHandler);
void CMP_IRQHandler(void) ALIAS(IntDefaultHandler);
void WDT_IRQHandler(void) ALIAS(IntDefaultHandler);
void BOD_IRQHandler(void) ALIAS(IntDefaultHandler);
void WKT_IRQHandler(void) ALIAS(IntDefaultHandler);
#endif

void PININT0_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT2_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT3_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT4_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT5_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT6_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT7_IRQHandler(void) ALIAS(IntDefaultHandler);



/**************************************************************************
 * The vector table, placed first in address-space(0x0000).
 */

extern uint32_t __StackTop[];
extern void (* const g_pfnVectors[])(void);

__attribute__ ((used, section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
#if defined (CHIP_LPC82X)
  // M0+ r0p1 Core functions //
  (const void *)__StackTop, // The initial stack pointer
  Reset_Handler,            // The reset handler
  NMI_Handler,              // The NMI handler
  HardFault_Handler,        // The hard fault handler
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  SVC_Handler,              // SVCall handler
  0,                        // Reserved
  0,                        // Reserved
  PendSV_Handler,           // The PendSV handler
  SysTick_Handler,          // The SysTick handler
 // LPC824 Microcontroller peripherals //
  SPI0_IRQHandler,          // SPI0 controller
  SPI1_IRQHandler,          // SPI1 controller
  0,                        // Reserved
  UART0_IRQHandler,         // UART0
  UART1_IRQHandler,         // UART1
  UART2_IRQHandler,         // UART2
  0,                        // Reserved
  I2C1_IRQHandler,          // I2C1 controller
  I2C0_IRQHandler,          // I2C0 controller
  SCT_IRQHandler,           // Smart Counter Timer
  MRT_IRQHandler,           // Multi-Rate Timer
  CMP_IRQHandler,           // Comparator
  WDT_IRQHandler,           // Watchdog
  BOD_IRQHandler,           // Brown Out Detect
  FLASH_IRQHandler,         // Flash Interrupt
  WKT_IRQHandler,           // Wakeup timer
  ADC_SEQA_IRQHandler,      // ADC sequence A completion
  ADC_SEQB_IRQHandler,      // ADC sequence B completion
  ADC_THCMP_IRQHandler,     // ADC threshold compare
  ADC_OVR_IRQHandler,       // ADC overrun
  DMA_IRQHandler,           // DMA
  I2C2_IRQHandler,          // I2C2 controller
  I2C3_IRQHandler,          // I2C3 controller
  0,                        // Reserved
  PIN_INT0_IRQHandler,      // PIO INT0
  PIN_INT1_IRQHandler,      // PIO INT1
  PIN_INT2_IRQHandler,      // PIO INT2
  PIN_INT3_IRQHandler,      // PIO INT3
  PIN_INT4_IRQHandler,      // PIO INT4
  PIN_INT5_IRQHandler,      // PIO INT5
  PIN_INT6_IRQHandler,      // PIO INT6
  PIN_INT7_IRQHandler,      // PIO INT7
#else
 // M0+ Core functions //
  (const void *)__StackTop, // The initial stack pointer
  Reset_Handler,            // The reset handler
  NMI_Handler,              // The NMI handler
  HardFault_Handler,        // The hard fault handler
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  SVC_Handler,              // SVCall handler
  0,                        // Reserved
  0,                        // Reserved
  PendSV_Handler,			// The PendSV handler
  SysTick_Handler,          // The SysTick handler
 // LPC812 Microcontroller peripherals //
  SPI0_IRQHandler,          // SPI0 controller
  SPI1_IRQHandler,          // SPI1 controller
  0,                        // Reserved
  UART0_IRQHandler,         // UART0
  UART1_IRQHandler,         // UART1
  UART2_IRQHandler,         // UART2
  0,                        // Reserved
  0,                        // Reserved
  I2C_IRQHandler,           // I2C controller
  SCT_IRQHandler,           // Smart Counter Timer
  MRT_IRQHandler,           // Multi-Rate Timer
  CMP_IRQHandler,           // Comparator
  WDT_IRQHandler,           // PIO1 (0:11)
  BOD_IRQHandler,           // Brown Out Detect
  0,                        // Reserved
  WKT_IRQHandler,           // Wakeup timer
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  PININT0_IRQHandler,       // PIO INT0
  PININT1_IRQHandler,       // PIO INT1
  PININT2_IRQHandler,       // PIO INT2
  PININT3_IRQHandler,       // PIO INT3
  PININT4_IRQHandler,       // PIO INT4
  PININT5_IRQHandler,       // PIO INT5
  PININT6_IRQHandler,       // PIO INT6
  PININT7_IRQHandler,       // PIO INT7
#endif
}; /* g_pfnVectors */



/**************************************************************************
 * Reset-entry point Reset_Handler() for our firmware / application.
 * Sets up a simple C runtime environment with initialization of variables.
 * NOTE: No heap initialization done here.
 */

extern unsigned int __etext;
extern unsigned int __data_start__;
extern unsigned int __data_end__;
extern unsigned int __bss_start__;
extern unsigned int __bss_end__;

extern int main(void);


/* inline defined GCC-compatible C runtime initialization, will be placed
 * within the calling function and will therefore, here, be placed within
 * the Reset_Handler() function right.
 */
static inline void crt0(void) {

  unsigned int *src, *dest, *dend;

 // Copy initialization value to data section from flash
  src  = (unsigned int *)(&__etext); 
  dest = (unsigned int *)(&__data_start__);
  dend = (unsigned int *)(&__data_end__);
  while (dest < dend)
    *(dest++) = *(src++);

 // Zero-fill/Blank the bss section
  dest = (unsigned int *)(&__bss_start__);
  dend = (unsigned int *)(&__bss_end__);
  while (dest < dend)
    *(dest++) = 0;

} /* crt0() */



__attribute__ ((used)) void Reset_Handler(void) {

/* StartUp/ Inrush current limitation, There is connected N-channels
 * MOSFET (or alike) on this PINs  */
#ifdef DISABLE_PIN_PULL_UP

 // Enable clock to the IOCON registers
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 18);
// Set PINs to internal pull-DOWNs, to turn-OFF the N-MOSFETs
 // First   
#ifdef MOSFET_PIN0
//  Chip_IOCON_PinSetMode(LPC_IOCON, MOSFET_PIN0, PIN_MODE_PULLDN);
#endif
 // Second
#ifdef MOSFET_PIN1
//  Chip_IOCON_PinSetMode(LPC_IOCON, MOSFET_PIN1, PIN_MODE_PULLDN);
#endif
 // Third
#ifdef MOSFET_PIN2
//  Chip_IOCON_PinSetMode(LPC_IOCON, MOSFET_PIN2, PIN_MODE_PULLDN);
#endif
 // Fourth(last) 
#ifdef MOSFET_PIN3
//  Chip_IOCON_PinSetMode(LPC_IOCON, MOSFET_PIN3, PIN_MODE_PULLDN);
#endif
 // Disable the clock again
  LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 18);
#endif

 // Why are we here!, find answers and continue. 
/* TODO: */

 // Initialization of data value
  crt0();

 // Patch the AEABI integer divide functions to use MCU's romdivide library
#if defined (__USE_ROMDIVIDE)
 // Get address of Integer division routines function table in ROM
  unsigned int *div_ptr = (unsigned int *)((unsigned int *)*(PTR_ROM_DRIVER_TABLE))[4];
 // Get addresses of integer divide routines in ROM
 // These address are then used by the code in aeabi_romdiv_patch.s
  pDivRom_idiv = (unsigned int *)div_ptr[0];
  pDivRom_uidiv = (unsigned int *)div_ptr[1];
#endif

 // Init PLL, CPUcore and BUS -clock.
#if defined (__WANTED_PLL_MAIN)
  Chip_IRC_SetFreq(__WANTED_PLL_MAIN, __WANTED_CORE_FREQ);
#else
  Chip_IRC_SetFreq_ROM(__WANTED_CORE_FREQ);
#endif

 // Jump to program.
  main();
  
 // Why are we here! try to find out, then RESET.
/* TODO: */  while (1);

} /* Reset_Handler */



/**************************************************************************
 * Processor ends up here if an unexpected interrupt occurs or a specific
 * handler is not present in the application code.
 */
__attribute__ ((used, section(".after_vectors")))
void IntDefaultHandler(void) {
 /* Stay here forever */
  while(1) { }
}


/**************************************************************************
 * Default exception handlers. Override the ones here by defining your own
 * handler routines in your application code.
 */

__attribute__ ((used, section(".after_vectors")))
void NMI_Handler(void) {
    while(1) { }
}

__attribute__ ((used, section(".after_vectors")))
void HardFault_Handler(void) {
    while(1) { }
}


__attribute__ ((section(".text")))
void SVC_Handler(void) {
    while(1) { }
}

__attribute__ ((section(".text")))
void PendSV_Handler(void) {
    while(1) { }
}

__attribute__ ((section(".text")))
void SysTick_Handler(void) {
    while(1) { }
}


#endif /* NOT __cplusplus */

