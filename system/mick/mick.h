/**************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: minimum_kernel.h
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.3.9
 * Date:     1 Apr. 2016
 *
 *
 *                      API to the mimimum kernel.
 *       -- Minimum Inter-process Communication Kernel (MICK) --  
 *
 * The algorithm of MICK is priority driven time-slicing on a preemptive 
 * scheduler, a more "Round-Robin" behavior can be achieved by giving the 
 * tasks equal priority. The MICK is made to support the ARM Cortex-M0+ 
 * core, a smaller MCU-core, so data-unit(int) is 32 bits wide. This 
 * architecture is equally as good, in terms of CPU clock cycles, of loading 
 * and storing 8, 16 and 32 bits-words and size matter, so therefore the 
 * approach is: What is needed, not more. 
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
 * OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
 * SOFTWARE.
 *
 * The author of this material /source-code kindly ask all users and distri
 * butors of it, to leave this notice / text in full, in all copies of it, 
 * for all time to come.
 */

#ifndef MICK_H
#define MICK_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <time.h>
#include "_DEF_YOUR_TASKS.h"


/* The SYS_TICK_FREQ determines how often the scheduler interrupts the current 
 * running thread, to make a task-shift. Good balance in the system is needed, 
 * not too many task-shifts (waste of resources) and at the same time a required 
 * shot System reaction-time.
 * 
 * Set SYS_TICK_FREQ to 200, 400, 600 or 800(even 100s). SYS_TICK_FREQ at 400Hz, 
 * give as minimum a task-shift(context shift) every 2.5mS. The physical SYS_TICK 
 * _COUNTER runs at a speed that is 4 times higher to lower the response-time on 
 * signals(HW-events) and to enhance the resolution on CLOCK/TIME. 
 *
 * NOTE: The newLib macro CLOCKS_PER_SEC is NOT the same as this SYS_TICK_FREQ.
 */
#define SYS_TICK_FREQ         200
#define SYS_TICK_COUNTER      (SYS_TICK_FREQ * 4)  /* Do NOT alter this ratio*/


/* The ratio between POSIX CLOCKS_PER_SEC and this firmwares SYS_TICK_FREQ */
/* POSIX standard defines CLOCKS_PER_SEC value is 1000000. -However-, the 
 * current value in newlib for ARM and THUMB, is 100. 
 */
#define RATIO_IN_CLOCK        ( (uint32_t)((SYS_TICK_COUNTER) / CLOCKS_PER_SEC) )


/* Using milliseconds in a system that use Ticks, for example as:
 *       osDelay(TICKS_MILLISEC(500)); // half a second  
 *
 * NOTE: Good for constant, as in Known at compile-time.
 */
#define TICKS_MILLISEC(M_SEC)  ( (uint32_t) ((((float)M_SEC / 1000)) * SYS_TICK_COUNTER) )


/* This task is meeting its deadlines, if it often 'ends' itself by osThreadYield() */
typedef uint8_t DEADLINE_T;
/* Thread/Task priority, FIXED from creation */
typedef uint8_t PRIORITY_T;
/* Type for Thread states */
typedef uint8_t STATE_T;
/* Type for Thread ID-numbers. */
typedef uint8_t THREADID_T;  

/* Thread info in bytes(*4) stored as one word(uint32_t) aligned */
typedef struct {
  DEADLINE_T success;
  PRIORITY_T priority;
  STATE_T state;
  THREADID_T tid;
} THREAD_WORD_INFO_T __attribute__ ((aligned (4))); 




// TODO: Move this to _DEF_YOUR_TASKS.h
/**************************************************************************
 * SIGNALs: Flags set by Interrupt Service Routines(ISR) to make the kernel 
 * aware of Hardware event, such as sysTickTime Interrupt, Interrupts due 
 * to data reception on stdin and others. Flag-priority is: the higher bit-
 * number the lower process-priority. 
 * 
 * MASSAGEs: Inter-Process(Thread) Communication (IPC)
 */

             /* HW-Interrupt: Combined flag-holder, for signals      */
             /* osEvent in CMSIS terms, Do not use bit[31](sign-bit) */
// TODO: HERE we need a IfNotDefined (MICK_IRS_SIGNAL) 
typedef volatile int SIGNAL_LEN_T;


/* HW-Interrupt: Signals from Interrupt Service Routines (IRS) */
typedef enum {
  MASSAGE_NULL            = 0x00000000,
 /* Error = -1 */
  MASSAGE_ERROR           = (0x80000000 + 0x7fffffff),
 /* Maximum length of massage 4095 bytes */
  MASSAGE_LEN_MASK        = 0x00000fff,

 /* RESERVED */
  __RESERVED              = 0x40000000,

  /* READY, all Signals/Events are accepted. */
  IRS_SIGNAL_READY        = 0x20000000,
 /* Initialization: Doing start-Up only SYSTICK-event are accepted */
  IRS_SIGNAL_INIT         = 0x10000000,

  /* Mask-out 16 Signals, from bit[12] to bit[27] = 0x0ffff000 */
  IRS_SIGNAL_MASK         = (0xffff << 12), 
 /* Mask-out Signals */
  IRS_SIGNAL_KERNEL_MASK  = (IRS_SIGNAL_MASK | IRS_SIGNAL_INIT | IRS_SIGNAL_READY),
 /* Mask-out Counter: Start-up delay/countdown, until kernel is READY for signals */
  IRS_SIGNAL_INIT_C_MASK  = MASSAGE_LEN_MASK,  /* = 0x00000fff */

 /* Bit[27]: SysTickTimer Signal, normal scheduler time. */
  IRS_SIGNAL_SYSTICK      = (1 << 27), 

  
 /* -> Bit[12]..[26]: High bit-number, lower the process-priority -> */
  
 /* RX_COMPLETED on UARTs */
  IRS_SIGNAL_TTYS0_RXD    = (1 << 12),
  IRS_SIGNAL_STDIN        = IRS_SIGNAL_TTYS0_RXD,

    IRS_SIGNAL_TTYS1_RXD    = (1 << 13),

  IRS_SIGNAL_TTYS2_RXD    = (1 << 14),
  IRS_SIGNAL_RS485_RXD    = IRS_SIGNAL_TTYS2_RXD,

 /* Maybe it's time for time-sync. Part-Of-Seconds is now in sync */ 
  IRS_SIGNAL_RTC_30_SEC   = (1 << 15), 
  

  
  
 /* TX_COMPLETED on UARTs */
  IRS_SIGNAL_TTYS2_TX     = (1 << 24),
  IRS_SIGNAL_RS485_TX     = IRS_SIGNAL_TTYS2_TX,
  
    IRS_SIGNAL_TTYS1_TX     = (1 << 25),

  IRS_SIGNAL_TTYS0_TX     = (1 << 26),
  IRS_SIGNAL_STDOUT       = IRS_SIGNAL_TTYS0_TX,
  IRS_SIGNAL_STDERR       = IRS_SIGNAL_TTYS0_TX

} SIGNAL_LENGTH_CONST_T;


/**************************************************************************
 * API. Some functions a based on Supervisor Calls (software-interupts) and 
 * some are made as calls directly into the kernel-world, in thread-mode and 
 * on thread-stack, these can be temporarily disable interrupts, sorry.
 */

      /* Kernel administration calls. */

/* First init the Kernel, then add Threads/Tasks. */
void osKernelInit(void);

/* You start the kernel, after initialisation by osKernelInit() and set-Up's 
 * of all the Threads/tasks, There is no return to main(). */
void osKernelRun(void);

/* Prepare a tread for running, the threadID and threadPriority are constants 
 * defined by user. Define each threadID as a unique number (from 1 and 
 * consecutively up. threadPriority greater than 1 (the higher the number the 
 * LOWER thread priority. The thread Stack needed is in words(4*bytes). 
 */
void osThreadCreate(THREADID_T threadID, void (*thread)(void), uint32_t stackSize, PRIORITY_T priority);

/* Terminate execution of a thread, but can NOT free for allocated memory. */
 /* void osThreadTerminate(threadID_t id); */

/* Yield thread execution and pass control to another READY thread */
void osThreadYield(void);

/* Wait for timeout, time is in ticks(one fourth of sysCounter), if milli-
 * seconds is wanted, used the marcro TICKS_MILLISEC(M_SEC) */
void osDelay(uint32_t ticks);

/* osWait() We Do not have a Wait-function, insted setup a osThreadRecv() */



 /*  --  Inter-Process(Thread) Communication (IPC)  --  */
      /* IPC caution: There is no control, on the parameters handed-over 
       * (-: at all :-). The Send-function are blocking until the receiver 
       * request data-input via the Recv-function. On send notification, the
       * Signal or Massage is NOT automatically handed over, the receiver have 
       * to grab-it doing its process-time */


/* Send notification about a ready Massage or Signal to other task/thread with 
 * ID different then 0(the kernel). While sending a Massage, the signal_len 
 * parameter is equal to length of string or numbers of elements(len >= 1) that 
 * the pointer(massage), is pointing to.
 * While sending a Signal, the pointer(massage) is 0 and signal_len parameter 
 * points to the Signal to transmit.
 * 
 * On Error: signal_len will be -1.
 */ 
void osThreadSend(THREADID_T recvID, SIGNAL_LEN_T *signal_len, uint8_t **massage);


/* On receiving a notification, signalMAXLEN is set equal to the length of the 
 * string ready to be taken, if any. On receiver request the signalMAXLEN is 
 * use to indicate  
 * 
 * If only a Signals is wanted, then set mas-
 * sage_ptr to null(0) and SIGNALmaxlen will on received notification, hold a 
 * Signal.
 * 
 * Using it for Signals only, do as:
 * 
 *           osThreadRecv(&iProcessID, &iSignal, 0);
 *           
 * or for receiving a Massages, do as:
 * 
 *           iProcessID = 0xFF;
 *           iLength = TTY_RX_MAX;
 *           iReadyPtr = ttyS0_RX; 
 * 
 *           osThreadRecv(&iProcessID, &iLength, &iReadyPtr);
 */
void osThreadRecv(THREADID_T *sendID, SIGNAL_LEN_T *signal_maxlen, uint8_t **massage_ptr);




/****************************************************************************
 * clock() and time() (for the Machine/ Controller an small MCU (32bit))
 * 
 * The functions clock() and time() is defined here. This project focus on 
 * POSIX/Linux time, 1 January 1970. For now the second-counter is of type 
 * long,  * which result in the 2038 problem. In the embedded ARM GCC, _CLOCK_T_ 
 * is defined as unsigned long (32 bit) and _TIME_T_ long (32 bit).
 * 
 * See the functions before using them and for user inspiration: 
 *                                http://www.catb.org/esr/time-programming/
 */


/* Return Ticks since start/reset, based on newlib for ARM and THUMB, meaning 
 * that Seconds = clock() / CLOCKS_PER_SEC. */
clock_t clock(void);  


/* /* //Remenber for time() "time(NULL)"
 *  */
//time_t time(time_t *timer);   */
       // OLD void mick_kernel_uptime(MICK_TIME_T *timeptr); /* In seconds and ticks */




/**************************************************************************
 * Following API-calls is only to be used by/within Interrupt Service Routines
 */


/* Set Signal-flags for Interrupt Service Routine (ISR) call to handler. 
 * On success the signal-flag is returned, on failure zero(0) is returned.
 */ 
SIGNAL_LEN_T osISR_SetSignal(SIGNAL_LEN_T signal);



#ifdef __cplusplus
}
#endif

#endif /* MICK_H */

