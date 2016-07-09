/***************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: minimum_board.h
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.5.5
 * Date:     21 Oct. 2015
 *
 * The minimum_board files contains a limited codebase for initialization of 
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

#include <string.h>
#include "minimum_board.h"

#if defined (MICK_OS)
#include "mick.h"
#endif


/***************************************************************************
 * Board clock variable, used in LPCopen, the HAL
 */
const uint32_t ExtRateIn = __EXT_CLKIN; 
const uint32_t OscRateIn = __EXT_XTAL; 




/***************************************************************************
 * MinimumConsumption(): Turn OFF perifer clocks to limit power consumption.
 */
__attribute__ ((used)) void MinimumConsumption(void){
#if defined (CHIP_LPC82X)
	
	//  Put the I2C_OPEN_DRAIN_NOT_USED in MinimumConsumption()
#ifdef I2C_OPEN_DRAIN_NOT_USED
/* Set bits 10 and 11 in the GPIO DIR0 register to 1 to enable the
 * output driver and CLR0 bits 10 and 11 (LOW internally)
 */
  LPC_GPIO_PORT->DIR[0] |= (1 << 10);
  LPC_GPIO_PORT->DIR[0] |= (1 << 11);
  LPC_GPIO_PORT->CLR[0] = (1 << 10);
  LPC_GPIO_PORT->CLR[0] = (1 << 11);
#endif
  
#endif	
} /* MinimumConsumption */


/***************************************************************************
 * MinimumPinConf: Chip-pin assignment for functions and PCB-layout.
 */

__attribute__ ((used)) void MinimumPinConf(void) {


#if defined (CHIP_LPC82X)

 /* Enable the clock to the Switch Matrix */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
   
 /* Pin Assign: U0_TXD  U0_RXD */
  LPC_SWM->PINASSIGN[0] = 0xffff0004UL;
 /* I2C1: ONLY Real time clock and temperature sensor: 
  *       I2C1_SDA = PIO0_17; I2C1_SCL = PIO0_17 */
  LPC_SWM->PINASSIGN[9] = 0xff1211ffUL;

 /* WKTCLKIN on the wake-up timer, connected to INT on the Real time clock.
  *
  * While using the WKTCLKIN, disable the hysteresis for that pin in the 
  * DPDCTRL register. 
  */

 /* Enable as default: RESET, SWCLK and SWDIO */
  LPC_SWM->PINENABLE0 = 0xfffffecfUL;

  
 /* IO-Pin Configuration */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);

 /* Remember I2C1_SDA  I2C1_SCL, desable pull-Up */

 /* LPC82x: Reset SCT input multipler and DMA triggers input multipler */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SCT);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_DMA);
  LPC_INMUX->SCT0_INMUX[0] = 15;   
  LPC_INMUX->SCT0_INMUX[1] = 15;   
  LPC_INMUX->SCT0_INMUX[2] = 15;   
  LPC_INMUX->SCT0_INMUX[3] = 15;   
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[0] = 15;  
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[1] = 15;   
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[2] = 15;   
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[3] = 15;  
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[4] = 15;  
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[5] = 15;   
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[6] = 15;    
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[7] = 15;  
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[8] = 15;   
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[9] = 15;  
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[10] = 15;  
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[11] = 15;  
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[12] = 15;    
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[13] = 15;    
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[14] = 15;    
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[15] = 15;   
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[16] = 15;   
  LPC_DMATRIGMUX->DMA_ITRIG_INMUX[17] = 15;   
  LPC_INMUX->DMA_INMUX_INMUX[0] = 31;    
  LPC_INMUX->DMA_INMUX_INMUX[1] = 31;    
  
 /* To conserve power disable clocks again */
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SCT);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_DMA);
#else /* LPC812 */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
 /* Enable RESET-pin function */
  LPC_SWM->PINENABLE0 = 0xffffffbfUL;
 /* Pin Assign of U0_TXD and U0_RXD */
  LPC_SWM->PINASSIGN[0] = 0xffff0004UL;
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);
#endif /* LPC812 */
 

} /* MinimumPinConf */




/***************************************************************************
 * Tools for the on-board LED. On a KENTer it would normally be cathode-
 * connected to a IO-PIN. 
 */
#ifdef BOARD_LED_PIN

  volatile uint8_t onBoardIndicator_CTRL;

/* Config output-pin for board-LED and turn it OFF */
__attribute__ ((used)) void MinimumInitLED(void) {

#if defined (CHIP_LPC82X)
// TODO: Only on first ptototype
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);	
  Chip_SWM_DisableFixedPin(SWM_FIXED_XTALOUT);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

#endif

  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, BOARD_LED_PIN);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);

  MinimumLED_OFF();
} /* MinimumInitLED */

#endif /* BOARD_LED_PIN */




/***************************************************************************
 * Serial(UART0) communication (UART0) for stdin and stdout. The tools can 
 * be used directly for asynchronous serial communications purpose. The 
 * functionality here is also part of Kent DEL PINO's newlib retargeting.
 * 
 * These services/functions are built upon the microcontrollers ROM-based 
 * code (included from factory) and alled via API, runs in interrupt-mode. 
 */

// UART handle as part of ROM API
static UART_HANDLE_T *uart0_Handle;
// The ROM-based UART-handler also need some work-space 
static uint32_t uart0_HandleMEM[10];      //[0x20];
// Stat-Flags used to indicate TX/RX process, assigned by ROM callback-functions
volatile uint32_t minCommSendStat;
volatile uint32_t minCommReceiveStat;
// Buffer for incoming(Rx) data. The +2 is hidden space.
static uint8_t uartReceiverBuffer[MIN_STDIN_BUFFER_SIZE +2];



/* ROM-API (UART0_TX) callback, is called by the ROM-code to indicate 
 * End-Of-Transmission(TX), as a result of a former started MinimumPut()
 * process.
 * 
 * NOTE: Keep it short and simple, is called from a Interrupt handler.
 */
static void UART0_TX_callback(uint32_t err_code, uint32_t n) {

 // Non-Errors: On End-Of-Transmission(TX), err_code is always zero.

 // End-Of-Transmission(TX) !!
  if ((n != 0) && (minCommSendStat & TX_STARTED_FLAG)) {
    minCommSendStat &= !TX_FLAG_MASK_OUT;
    minCommSendStat |= TX_COMPLETED; 
   }
} // UART0_TX_callback


/* ROM-API (UART0_RX) callback function, used to signal conditions 
 * about UART-receiver, forexsampel that there is a data-packet in the
 * receiver-buffer (RX_COMPLETED). 
 * 
 * NOTE: Keep it short and simple, IS CALLED from a Interrupt handler.
 */
static void UART0_RX_callback(uint32_t err_code, uint32_t n) {

  uint8_t *pToBuff = (uint8_t *) &uartReceiverBuffer;
  uint32_t autoRestatBit = minCommReceiveStat;

 // Were we expecting incoming data, then register it.
  if ((err_code == 0) && (minCommReceiveStat & RX_REQUESTED)) {
   // Copy this bit 
    autoRestatBit &= RX_AUTO_RESTART; 
   // Reception ended(terminate) on CR+LF 
    if (n < MIN_STDIN_BUFFER_SIZE) {
     // Add CR+LF, the ROM-code took it from us.
      pToBuff += n;
      *pToBuff++ = '\r';
      *pToBuff++ = '\n';
      *pToBuff = '\0';
     // Store No-Of-Bytes
      minCommReceiveStat = (uint32_t) (n + 2);
     }
    else  // numOf == MIN_STDIN_BUFFER_SIZE
      minCommReceiveStat = (uint32_t) n;

    // We have a package, set flag
     minCommReceiveStat |= autoRestatBit | RX_COMPLETED; 

#if defined (MICK_OS)
    // Send Signal to kernel
     if (IRS_SIGNAL_STDIN == osISR_SetSignal(IRS_SIGNAL_STDIN)) {

      }
#endif
   }

 // Is the ROM-code returning Errors
  if (err_code != 0) {

   }
} // UART0_RX_callback



/* In later versions this ISR will be used to timestamp events such as 
 * break-conditions, synchronization signal with Real Time Clock etc.
 * 
 * Can also be used to monitor the background-process running in the ROM-
 * code space. 
 */
void UART0_IRQHandler(void) {

 // Read UART status
  //register uint32_t statReg = Chip_UART_GetStatus(LPC_USART0);
  
 // Check for break-conditions(RXBRK(FRAME))


 // UNconditional jump to the handler in ROM.
  LPC_UARTD_API->uart_isr(uart0_Handle);

} // UART0_IRQHandler



/* MinimumGetStartUp(): Prepare the receipt of incoming bytes or a array-
 * of-chars from UART0(stdin). The bytes a filled in a buffer and a flag 
 * (RX_COMPLETED) is set upon the reception of CR+LF or full-buffer 
 * (MIN_STDIN_BUFFER_SIZE). The reception takes place in the background 
 * (interrupt based).
 * 
 * Takes autoRestart(bool), if TRUE the receiver process is to be continued 
 * automatic after emptying the buffer with MinimumGet().
 * 
 * Returns, On success; MIN_STDIN_BUFFER_SIZE or on failure, -1.
 * 
 * NOTE: 
 * a/ Starting this function will Null the receiver-buffer, erase previous 
 * received data.
 */

static const UART_PARAM_T RX_param = {
  uartReceiverBuffer,
  MIN_STDIN_BUFFER_SIZE,
  RX_MODE_CRLF_RECVD,      /* Fill-up buffer until (CR+)LF, or buffer-full */
  DRIVER_MODE_INTERRUPT,
  (UART_CALLBK_T) UART0_RX_callback,
  0
};


__attribute__ ((used)) int MinimumGetStartUp(bool autoRestart) {

  // Clear the buffer and start receiving
  uartReceiverBuffer[0] = 0;
  if (LPC_UARTD_API->uart_get_line(uart0_Handle, (UART_PARAM_T *) &RX_param) == ERR_UART_RXD_BUSY) {
    minCommReceiveStat = RX_ROM_BUSY_FLAG; 
    return -1; 
   }

  if (autoRestart)
    minCommReceiveStat = (RX_AUTO_RESTART | RX_REQUESTED);
  else
    minCommReceiveStat = RX_REQUESTED;
  
 // Return Number-Of-Bytes, REQUESTED.
  return (int) MIN_STDIN_BUFFER_SIZE;
} // MinimumGetStartUp



/* MinimumGet(): Transfer the UART receive-buffer to a higher application-
 * layer or to newlib stubs. This function is best suited on strings/byte- 
 * arrays and minimum length of target-buffer is 2.
 * 
 * Takes a pointer to the target-buffer (buff) and the length/ size of the 
 * target-buffer (maxLen).
 * 
 * Returns, On success; Actual-Number-Of-Bytes transferred or on failure; -1.
 * 
 * NOTE: The returned byte-array is terminated by a ZERO(a add on), so if a 
 * byte-array is later interpret as a string, there is a terminator.
 */
__attribute__ ((used)) int MinimumGet(uint8_t *buff, int maxLen) {

  uint8_t *ptr;
  uint32_t lenOfReceiverBuffer;
  uint32_t autoRestatBit = minCommReceiveStat;
  int actualLen = -1;
 
  if (MIN_RECEIVED_DATA) {
    autoRestatBit &= RX_AUTO_RESTART; 
    lenOfReceiverBuffer = MIN_RECEIVED_LEN_OF_DATA;

   // Perform sanity check of buff != NULL 
  
   // Are we transferring the whole buffer ! 
    if (lenOfReceiverBuffer <= (uint32_t) maxLen) {
     // Take the whole thing INCLUDING the '\0'.
      memcpy(buff, uartReceiverBuffer, lenOfReceiverBuffer);
      actualLen = lenOfReceiverBuffer;
      if (autoRestatBit)
        MinimumGetStartUp(autoRestatBit);
      else {
       // Clean our local buffer and RX-flags
        uartReceiverBuffer[0] = 0;
        minCommReceiveStat = 0;
       }
     }
    // Perform check on maxLen, can't be 0 or -
    else if (maxLen > 1) {
     // Give them what they ask, -1, room needed for ZERO-termination.
      actualLen = maxLen -1;
     // Move block from 
      memcpy(buff, uartReceiverBuffer, actualLen);
     // Terminate target buffer
      ptr = buff + maxLen;
      *ptr = '\0';
     // Move down what is left over, in local buffer.
      ptr = uartReceiverBuffer + actualLen;
      memcpy(uartReceiverBuffer, ptr, lenOfReceiverBuffer - actualLen);
     // Update RX-flags, with NEW/remaining Numbers-Of-Bytes
      minCommReceiveStat = (uint32_t) lenOfReceiverBuffer - actualLen;
     // And and we still have a package, the remaining.
      minCommReceiveStat |= (autoRestatBit | RX_COMPLETED);
     }
    else 
      return -1;
   }

  return actualLen;  
} // MinimumGet 



/* MinimumPut(): Put Number-Of-Bytes(len) from buffer(buff) to the UART0
 * (stdout). This function is best suited on strings/byte-arrays, not good 
 * at single-byte level. 
 * 
 * Return, On success; Number-Of-Bytes or on failure, -1.
 *
 * NOTE: The buffer(buff) should not be changed until TX_COMPLETED. if You 
 * need to insert a pause, use MinWaitPutCompleted(void).
 */
__attribute__ ((used)) int MinimumPut(uint8_t *buff, int len) {

  UART_PARAM_T param;

  param.buffer = buff;
  param.size = (uint32_t) len;
 // Interrupt based, binary transmission
  param.driver_mode = DRIVER_MODE_INTERRUPT;
  param.transfer_mode = TX_MODE_SZERO;
 // Callback, The function to call on completion
  param.callback_func_pt = (UART_CALLBK_T) UART0_TX_callback;
 // Transmit the data
  minCommSendStat &= !TX_FLAG_MASK_OUT;
  if (LPC_UARTD_API->uart_put_line(uart0_Handle, &param) == ERR_UART_TXD_BUSY) {
    minCommSendStat |= TX_BUSY_FLAG; 
   // Tx error
    return -1; 
   }

  minCommSendStat |= TX_STARTED_FLAG;
 // Return Number-Of-Bytes, accepted.
  return (int) param.size;
} // MinimumPut



/* Setup UART0 handle and comm parameters 
 */
__attribute__ ((used)) void MinimumInitUART(const uint32_t baudrate) {

  uint32_t frg_mult;

  // baudrate in KBPS, 8N1, ASYNC mode, errors, clock filled in later
  UART_CONFIG_T cfg = {
    0,          /* !!!!!!!!!!!!!   U_PCLK frequency in Hz */
    115200,
    1,
    0,          /* Asynchronous Mode */
    NO_ERR_EN   /* Errors to detect */
   };

 // Perform a sanity check on the storage allocation 
  if (LPC_UARTD_API->uart_get_mem_size() > sizeof(uart0_HandleMEM)) {
     while(1) {};
   }


 // Setup the UART handle
  uart0_Handle = LPC_UARTD_API->uart_setup((uint32_t) LPC_USART0, (uint8_t *) &uart0_HandleMEM);
  if (uart0_Handle == NULL) {
      while(1) {};
   }


 // Need to tell UART ROM API function the current UART peripheral clock speed */
  cfg.sys_clk_in_hz = Chip_Clock_GetSystemClockRate();

    /* Initialize the UART with the configuration parameters */
    frg_mult = LPC_UARTD_API->uart_init(uart0_Handle, &cfg);
    if (frg_mult) {
        Chip_SYSCTL_SetUSARTFRGDivider(0xFF);   /* value 0xFF should be always used */
        Chip_SYSCTL_SetUSARTFRGMultiplier(frg_mult);
    }


/* OLD Setup UART, be course I had problems, I don't understand the ROM INIT */
    Chip_UART_Init(LPC_USART0);
    Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
    Chip_Clock_SetUSARTNBaseClockRate((baudrate * 16), true);
    Chip_UART_SetBaud(LPC_USART0, baudrate);
    Chip_UART_Enable(LPC_USART0);
    Chip_UART_TXEnable(LPC_USART0);
/* OLD Setup UART */

 // 
  minCommSendStat = (uint32_t) TX_COMPLETED;
  minCommReceiveStat = 0;
  
 // Alle rx og fejl settes 
  Chip_UART_IntEnable(LPC_USART0, UART_INTEN_RXRDY);

 // We give this the highest priority
  //NVIC_SetPriority(UART0_IRQn, 0); 

 // Enable the IRQ for the UART
  NVIC_EnableIRQ(UART0_IRQn);
  
} // MinimumInitUART






