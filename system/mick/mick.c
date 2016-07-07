
 // TODO: Fill with 0. Fill with 0. Fill with 0. Fill with 0.
 // Yes, fill some pattern in


 // TODO: !! MAybe it is OK 
 //   SVC and Pending Interrupt Request have low priority, but the same
  // priority due to limitations in Cortex-m0 
  // NVIC_SetPriority(SVCall_IRQn, 3);
  // NVIC_SetPriority(PendSV_IRQn, 3);

// TODO: 
// This thing about Clearing the PendSV_IRQn, 







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


/**************************************************************************
 *  - Some very importen documents, while programing a Kernel. - 
 *
 * ARM GCC Inline Assembler Cookbook: 
 *   http://www.ethernut.de/en/documents/arm-inline-asm.html
 * 
 * A Beginner’s Guide on Interrupt Latency - and Interrupt Latency:  
 *   https://community.arm.com/docs/DOC-2607
 */


/**************************************************************************
 * A note to myself :-)
 *
 * ? Little-KENTer, if you really wanted some System-based IO, do it right, 
 * as in your code from the LPC2106.
 * 
 *           // Device table entry used to add this device.
 *           const struct device_table_entry com1 = {
 *              "com1",             // Device name
 *               com1Open,           // Open method
 *               com1Close,          // Close method
 *              com1Read,           // Read method
 *              com1Write,          // Write method
 *              com1Control };      // Ioctl method
 */       


#include "mick.h"
#include "tcb.h"
#include "tcblink.h"

/* Our Hardware */
#include "chip.h"



/* Kernel clock/time */
typedef struct {
  time_t seconds;
  uint32_t ticks;
} MICK_TIME_T; 

/* Memory space for all the Thread-Stacks(including the Kernel) */
static volatile unsigned int all_stack_mem[OS_STACK_NEEDED];
/* Pointer to last Thread-Stack-Top allocated */
static unsigned int *lastStackInit = (unsigned int *) &all_stack_mem[0];

/* Kernel structure */
typedef volatile struct {
  TCB *tcb_current;       /* OFFSET #0 -> to current process */  
  TCBLINK tcblink_ready;
  TCBLINK tcblink_sleep;
  TCBLINK tcblink_block;
  MICK_TIME_T time;
  SIGNAL_LEN_T signals;
} kernel_t __attribute__ ((aligned (8))); 

/* Offset into kernel_t struct, use in ASM(inline) */
#define OFS_TCB_CURRENT       (0)  // Offset in -words-
/* Kernel main structure */
static kernel_t kernel; 
/* A Task Control Block for each of the Threads/Tasks */
static TCB tcb_list[NUM_OF_THREAD];


/* Task operation-states */
typedef enum {
 /* Not Created */
  STATE_NON,
 /* A thread that is ready to execute */
  STATE_READY,
 /* A task on pause, for requested time */
  STATE_SLEEP,
 /* Sending task is blocked, while waiting for a receiver */
  STATE_BLOCKED_SENDER,
 /* Receiver task that is blocked, until until there is a sender */
  STATE_BLOCKED_RECEIVER,
} STATE_TO_BE_IN;



/**************************************************************************
 * Signal handling.
 */

//NOTE: Signal result in use of EVENT_SEND, a event from kernel(not another Task) 


 /* Clear Signal-flag from ISR. While manipulation with the Interrupt 
 * Signals, we disable the Interrupt capability.
 *  
 * NOTE: Only to be used by the Kernel. */ 
STATIC INLINE void ISR_ClearSignal(SIGNAL_LEN_T signal)
{
  __disable_irq();
  kernel.signals &= ~signal;
  __enable_irq();
} /* ISR_ClearSignal */ 


/* Set Signal-flags for Interrupt Service Routine (ISR) call to handler. 
 * While manipulation with the Interrupt Signals, we disable the Interrupt 
 * capability. 
 * 
 * On success the signal-flag is returned, on failure zero(0) is returned.
 */ 
SIGNAL_LEN_T osISR_SetSignal(SIGNAL_LEN_T signal) {

  SIGNAL_LEN_T theRes = 0;

  __disable_irq();
 /* Only when the kernel is READY to receive */
  if (IRS_SIGNAL_READY & kernel.signals) {
    kernel.signals |= (IRS_SIGNAL_MASK & signal);
    theRes = (IRS_SIGNAL_MASK & signal);
   }
  __enable_irq();

  return theRes;
} /* osISR_SetSignal */ 


/* Return next Signal in priority order */
SIGNAL_LEN_T ISR_GetNextSignal(SIGNAL_LEN_T signal) {

  SIGNAL_LEN_T signal_s;

  // There is only IRS_SIGNAL_STDIN, for now
  signal_s = IRS_SIGNAL_STDIN & signal;

  return signal_s;
} /* ISR_GetSignal */



/**************************************************************************
 * API based on directly read-outs on Kernel variables.
 */ 


/* Return machine run-time in (adjusted)Ticks */
clock_t clock(void) {

  MICK_TIME_T sysTicksFromStart;
  uint32_t workNum;

 // Get the sys-time in seconds + fraction-part 
  __disable_irq();
  sysTicksFromStart.seconds = kernel.time.seconds;
  sysTicksFromStart.ticks = kernel.time.ticks;
  __enable_irq(); 
  
 // Calculated all ticks 
  workNum = (uint32_t) (sysTicksFromStart.seconds * SYS_TICK_FREQ) + sysTicksFromStart.ticks;

 // Adjust to fit POSIX for newlib on ARM
  return (clock_t) ((uint32_t) workNum / RATIO_IN_CLOCK);
} /* clock */


/* time_t time(time_t *timer); */
void mick_kernel_uptime(MICK_TIME_T *timeptr) {

  __disable_irq();
  timeptr->seconds = kernel.time.seconds;
  timeptr->ticks = kernel.time.ticks;
  __enable_irq(); 
} /* mick_kernel_uptime */




/**************************************************************************
 * API based on Supervisor Calls (software-interupts)
 * 
 * Good, when naming API-functions:-)
 * https://www.keil.com/pack/doc/CMSIS/RTOS/html/group___c_m_s_i_s___r_t_o_s___thread_mgmt.html
 *
 * Good doc on SV Calls:-)
 * https://falstaff.agner.ch/2013/02/18/cortex-m3-supervisor-call-svc-using-gcc/
 */

/* System Calls that result in Pending Request */
#define SVCFUNC_START         (0x20) /* Do NOT change */
#define SVCFUNC_SLEEP         (0x00) 
#define SVCFUNC_YIELD         (0x01) /* From 0x01 to 0x1F */  
#define SVCFUNC_SEND          (0x02)  
#define SVCFUNC_RECV          (0x03)  


/* Macro to start-Up a Pending Interrupt Request */
#define REQUEST_PENDSV()            do { SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk; } while (0)


/* SVC with the given immediate [%0] and a NOP to insure a safe/aligned return address */
#define SVC(svcFunc) __asm volatile("svc %0  \n": : "I" (svcFunc)); \
                     __asm volatile("nop   \n")



__attribute__ ((used)) void osThreadSend(THREADID_T recvID, SIGNAL_LEN_T *signal_len, uint8_t **massage) {

 /* The gcc-kompiler for ARM automatic place parameters 1..4 in registers r0 to r3, but
  * we need to make sure, that the types are correct, so We work with/ use them here. 
  * 
  * NOTE: Problem seems to be octal numbers in word registers.
  */ 
  kernel.tcb_current->sys.sender.destID = (THREADID_T) recvID;  
  kernel.tcb_current->sys.sender.signall_p = (SIGNAL_LEN_T *) signal_len;  
  if (massage == 0)
    kernel.tcb_current->sys.sender.massage_pp = 0; 
  else // Or the address of the hand-over pointer
    kernel.tcb_current->sys.sender.massage_pp = (uint8_t **) massage; 
  
/* TODO: Test with NULL input */
  
 // Do NOT send to Kernel
  if ((recvID == 0) || (recvID >= (THREADID_T) NUM_OF_THREAD)) {
    if (signal_len != (SIGNAL_LEN_T *) NULL)
      *signal_len = 0;  // -1; 
    return;
   }

  SVC(SVCFUNC_SEND);
} 


__attribute__ ((used)) void osThreadRecv(THREADID_T *sendID, SIGNAL_LEN_T *signal_maxlen, uint8_t **massage_ptr) {

  kernel.tcb_current->sys.receiver.sourID_p = (THREADID_T *) sendID; 
  kernel.tcb_current->sys.receiver.signall_p = (SIGNAL_LEN_T *) signal_maxlen; 
  if (massage_ptr == 0)
    kernel.tcb_current->sys.receiver.massageReady_pp = 0;
  else  
    kernel.tcb_current->sys.receiver.massageReady_pp = (uint8_t **) massage_ptr;   

/* TODO: Test with NULL input */
  
 // Just a little check, IF !, just return 
  if ((sendID == (THREADID_T *) NULL) || (signal_maxlen == (SIGNAL_LEN_T *) NULL)) {
    return;
   }

  SVC(SVCFUNC_RECV);
} 


__attribute__ ((used)) void osDelay(uint32_t ticks) {

 // Doing the delay this value is steped-down by 4. 
  if (ticks < 4) 
    kernel.tcb_current->sys.sleep.count = 4;
  else 
    kernel.tcb_current->sys.sleep.count = ticks; 

  SVC(SVCFUNC_SLEEP);
} 


__attribute__ ((used)) void osThreadYield(void) {

  SVC(SVCFUNC_YIELD);
} 



/* A code-placed pointer, pointing to, Ha-ha, a data-placed data-structure. 
 * This code is for an ARM Cortex-M with thump address-methodes and on top 
 * of that We use GNU Link time optimization(LTO). 
 * 
 * WARNING: 
 * In Thump-mode bit[0] in an instruction/function address is 1, but this 
 * pointer should point to word-aligned data, with bit[0] = 0. 
 */
__attribute__ ((used, aligned (8))) void (* const kernel_ptr)(void) asm (".kernel_ptr") = (const void *) &kernel;


/* SVC handler(_in_c) 
 * The main purpose is to insure Task State-shift required by the Task itself, at 
 * a higher run-priority level then the one that is a result of SysTick(schedule).
 * 
 * svcArgs(in r0) is pointing to a Stack frame contains: r0, r1, r2, r3, r12, 
 * r14(LR), PC and xPSR and is addressed as:
 *          Stacked R0 = svc_args[0] or Stacked PC = svc_args[6].
 */
__attribute__ ((used, aligned (4))) void SVC_Handler_in_c(uint32_t *svcArgs) asm (".handler_in_c");

void SVC_Handler_in_c(uint32_t *svcArgs) {
 
 // Find SVC-function number, in caller SVC-instruction 
  register uint32_t svcNumber = ((uint8_t *)svcArgs[6])[-2];

  switch(svcNumber) { 
      
    case SVCFUNC_SLEEP:
      kernel.tcb_current->thread.info.state = STATE_SLEEP;
      REQUEST_PENDSV();
      break;
      
    case SVCFUNC_YIELD:
     // For now bla-bla
      kernel.tcb_current->thread.info.success = 0x08;  
      REQUEST_PENDSV();
      break;
      
    case SVCFUNC_SEND: 
      kernel.tcb_current->thread.info.state = STATE_BLOCKED_SENDER;
      REQUEST_PENDSV();
      break;
      
    case SVCFUNC_RECV: 
      kernel.tcb_current->thread.info.state = STATE_BLOCKED_RECEIVER;
      REQUEST_PENDSV();
      break;
      
    case SVCFUNC_START:
      __asm volatile(
        "  LDR   r3,=.kernel_ptr       \n\t"
        "  LDR   r1,[r3,#0]            \n\t"
      );
      
      __asm volatile(
        "  LDR    r0,[r1,%0]           \n\t"
        "  LDR    r2,[r0,%1]           \n\t"
        "  MSR    PSP,r2               \n\t"
        :
        : "I" (OFS_TCB_CURRENT), "I" (OFS_TCB_STACK)
       ); /* NOTE: OFS_ are used for offsets of word aligned data, 
           * resulting in [rN + ZeroExt(OFS_ << 2)]  */

      __asm volatile(
        "  MOV    r1,#4                \n\t"
        "  MOV    r0,LR                \n\t"
        "  ORR    r0,r0,r1             \n\t"
        "  MOV    LR,r0                \n\t"
       );
      REQUEST_PENDSV();
      break;

    default: // Unknown SVC request 
      break;
   }
  
} /* SVC_Handler_in_c */


/**************************************************************************
 * SVC_Handler(): System entering. Here extract SVCaller's stack address 
 */
__attribute__ ((used, naked)) void SVC_Handler(void) {

 // Which Stack was used when this interrupt happened 
  __asm volatile(
    "  MOVS   r0, #4               \n\t"
    "  MOV    r1, LR               \n\t"
    "  TST    r0, r1               \n\t"
   );
  __asm volatile(
    "  BEQ    .it_was_MSP          \n\t"
    "  MRS    R0, PSP              \n\t"  // The Stack used was PSP (Thread)
    "  B      .branch_to_c         \n\t"  // 16-bit Thumb, then B limit ±2KB
    ".it_was_MSP:                  \n\t"  
    "  MRS    R0, MSP              \n\t"  // The Stack used, was the MSP 

  ); 
  __asm volatile(
    ".branch_to_c:                 \n\t"
    "  LDR    R3,=.handler_in_c    \n\t"
    "  MOV    r2, #0x01            \n\t"  // bit[0] to 1 for Thumb code 
    "  ORR    r3, r2               \n\t" 
    "  BX     R3                   \n\t"  // JUMP, with no-return 
    :
    :
    : "r3", "cc"  
   );
} // SVC_Handler 




    /* -- Schedule, schedule-tool and SysTick -> */


/**************************************************************************
 * SysTick_Handler: Count Up seconds and start scheduler 
 */
void SysTick_Handler(void) {

 /* Update kernel-Up time, is also the system-clock */
  if (++kernel.time.ticks >= SYS_TICK_COUNTER) {
   /* Ticks is reset every second */ 
    kernel.time.ticks = 0;
    kernel.time.seconds++;
   }

 /* Is it scheduler-timer, due to sys-tick, only every 4 times */
  if ((0x03 & kernel.time.ticks) == 0x03) {
    __disable_irq();
    kernel.signals |= IRS_SIGNAL_SYSTICK;
    __enable_irq();  
   }
  
 /* Scheduler-timer on ALL signals*/
  if (IRS_SIGNAL_MASK & kernel.signals)  
    REQUEST_PENDSV();
} /* SysTick_Handler */




/* Inter-Process Communication(IPC): Search for pair(two blocked process's 
 * waiting for one another) */
__attribute__ ((used)) static void procBlockedTasks(void) {

 //  TODO:  do { } while !!  If we are here, find them, try harder !!!
  
  
  
 // ! Is there a Massage/Signal sender 
  TCB *send_tcb = tcblink_pull_state(&(kernel.tcblink_block), STATE_BLOCKED_SENDER);
  if (send_tcb != 0) {
   // ! Is there a Massage/Signal receiver  
    TCB *recv_tcb = tcblink_pull_state(&(kernel.tcblink_block), STATE_BLOCKED_RECEIVER);
    if (recv_tcb != 0) {
     // ! are they actually waiting for its-other 
      if ((THREADID_T) send_tcb->sys.sender.destID == recv_tcb->thread.info.tid) {
       // 
       // Is the SENDER sending an Massage or just a Signal
       //
        int sentMassage = 0;
        if (send_tcb->sys.sender.massage_pp != 0) 
          sentMassage = (int) *send_tcb->sys.sender.signall_p;
       //
       // Is the receiver ready for an Massage or just a signal
       //
        if (recv_tcb->sys.receiver.massageReady_pp != 0) { 
          // Is there a Massage from SENDER to receiver 
          if ((sentMassage > 0) && (*recv_tcb->sys.receiver.signall_p >= sentMassage)) {
           // We have a MASSAGE
            *recv_tcb->sys.receiver.massageReady_pp = *send_tcb->sys.sender.massage_pp; 
            *recv_tcb->sys.receiver.signall_p = sentMassage; 
           }
          else {
           // The Array is too BIG
            if (sentMassage > 0) {
             // Sender Error. 
              *send_tcb->sys.sender.signall_p = -1;
             }
           // No massage for receiver, MAKE POINTER NULL
            *recv_tcb->sys.receiver.massageReady_pp = 0;
            // We have a SIGNAL, but was waiting a massage
            *recv_tcb->sys.receiver.signall_p = *send_tcb->sys.sender.signall_p; 
           }
         }
       //
       // SIGNAL ONLY, receiver wants a signal
        else {
         // Did sender plan to send an array, It can NOT.
          if (sentMassage > 0) {   
            // Sender Error. 
            *send_tcb->sys.sender.signall_p = -1;
           // Empty notification: The receiver receives a zero(0) 
            *recv_tcb->sys.receiver.signall_p = 0;
           } 
         // We have a SIGNAL
          else { 
            *recv_tcb->sys.receiver.signall_p = *send_tcb->sys.sender.signall_p;
           }
         } 
       //
       // Hand-Over sender Thread-ID.
        *recv_tcb->sys.receiver.sourID_p = (THREADID_T) send_tcb->thread.info.tid;
       // Shift both sender and receiver back to STATE_READY.
        recv_tcb->thread.info.state = STATE_READY;
        tcblink_push(&(kernel.tcblink_ready), recv_tcb);
        send_tcb->thread.info.state = STATE_READY;
        tcblink_push(&(kernel.tcblink_ready), send_tcb);
       } 
      else {
       // No, not waiting for eachother
        tcblink_push(&(kernel.tcblink_block), send_tcb);
        tcblink_push(&(kernel.tcblink_block), recv_tcb);
       }
     } 
    else
     // There is no receiver ready yet
      tcblink_push(&(kernel.tcblink_block), send_tcb);
    
   }
} /* procBlockedTasks */


/* Countdown the Delay-counter, when Delay is over, send Thread back in
 * STATE_READY */
__attribute__ ((used)) static int procThreadDelayCounter(TCB *tcb) {

  register uint32_t countDown = tcb->sys.sleep.count;

 /* SYS_TICK_COUNTER is 4 times as fast as SYS_TICK_FREQ */
  if (countDown >= 4) 
    countDown -= 4;

  if (countDown < 4) {
    TCB *p = tcblink_pull_with_tcb(&(kernel.tcblink_sleep), tcb);
    if (p != TCB_NULL) {
      p->thread.info.state = STATE_READY;
      tcblink_push(&(kernel.tcblink_ready), p);
     }
   }
  else 
   /* Storage for future countdown */
    tcb->sys.sleep.count = countDown;
        
  return 0;
} /* procThreadDelayCounter */


/* Go through all Delayed(sleeping) threads, to see if it is time to 
 * wake-up */
__attribute__ ((used)) static void procDelayedTasks(void) {

  tcblink_list(&(kernel.tcblink_sleep), procThreadDelayCounter);
} /* procDelayedTasks */


/* Schedule */
__attribute__ ((used, aligned (4))) void schedule(void) asm (".schedule");

void schedule(void) {

   /* We are ALWAYS down here for a reason, something about the State on the current task. 
    * Maybe it's just been preempted by the SYSTICK or it want to be blocked(change State).
    */
    if (kernel.tcb_current != TCB_NULL) {
      switch (kernel.tcb_current->thread.info.state) {

        case STATE_READY:
          tcblink_push(&(kernel.tcblink_ready), kernel.tcb_current);
          break;
        
        case STATE_SLEEP:
          tcblink_push(&(kernel.tcblink_sleep), kernel.tcb_current);
          break;
        
        case STATE_BLOCKED_SENDER:
          tcblink_push(&(kernel.tcblink_block), kernel.tcb_current);
          break;
        
        case STATE_BLOCKED_RECEIVER:
          tcblink_push(&(kernel.tcblink_block), kernel.tcb_current);
          break;

        default:
          // osErr = xxState;
          break;
       }
     }

    
    register uint32_t signalCopy = kernel.signals;
    int count = 0;

    /* INIT to READY: delay signal-settings from ISR */
     if (IRS_SIGNAL_INIT & signalCopy) {
   
       uint32_t work = signalCopy & IRS_SIGNAL_INIT_C_MASK;
       work--;
       if (work == 0) {
        /* Change state to READY */
         __disable_irq();
         kernel.signals &= ~(IRS_SIGNAL_INIT | IRS_SIGNAL_INIT_C_MASK);
         kernel.signals |= IRS_SIGNAL_READY;     
         __enable_irq();      
        }
       else {
        /* Store new counter value */
         __disable_irq();
         kernel.signals &= ~IRS_SIGNAL_INIT_C_MASK;
         kernel.signals |= (IRS_SIGNAL_INIT_C_MASK & work);     
         __enable_irq();      
        }
      }
    
    
 /* Result of SYSTICK-signal: Normally scheduling */
  if (IRS_SIGNAL_SYSTICK & signalCopy) {
   /* Before We process, We clear signals */
    ISR_ClearSignal(IRS_SIGNAL_SYSTICK);
    signalCopy &= ~IRS_SIGNAL_SYSTICK;
    procDelayedTasks();
   }

  
 /* ISR-signaling task-handlers */
  if (IRS_SIGNAL_MASK & signalCopy) {
   /* Extract the One, with the highest priority */ 
    signalCopy = ISR_GetNextSignal(signalCopy);

    
   // !! which thread
    THREADID_T sigThread = IN_JSON_ID; 

    count = NUM_OF_THREAD;
    TCB *recv_tcb = TCB_NULL;
    
    do {
     
     // Find the receiver-blocked(waiting) Thread
      recv_tcb = tcblink_pull_state(&(kernel.tcblink_block), STATE_BLOCKED_RECEIVER);
      if (recv_tcb != TCB_NULL) {

       // Is the ONE !            
        if (recv_tcb->thread.info.tid == 1) { 
         // Hand over the parameter values.
          *recv_tcb->sys.receiver.sourID_p = 0;  // Kernel-ID
          *recv_tcb->sys.receiver.signall_p = IRS_SIGNAL_STDIN;
         // Is receiver waiting for Massager
          if (recv_tcb->sys.receiver.massageReady_pp != 0) {
           // Then tell it THIS is only a SIGNAL 
            *recv_tcb->sys.receiver.massageReady_pp = 0; 
           }
         // Put the receiver into STATE_READY.
          recv_tcb->thread.info.state = STATE_READY;
          tcblink_push(&(kernel.tcblink_ready), recv_tcb);
         // 
          count = 0;
          recv_tcb = TCB_NULL;
         }   
        else
         // NOT the ONE, put it back 
          tcblink_push(&(kernel.tcblink_block), recv_tcb);
       }
     } while ((recv_tcb != 0) && (count-- > 0));

    
   //  
    ISR_ClearSignal(signalCopy);

    // Other-use-of signalCopy, skip the next proc_block_event() 
    count = 1;
   }

  
 // Task Inter-process Communication, IF NON SIGNALs 
  if (count == 0) 
    procBlockedTasks();

 // Next-in-queue(in priority order) are now current.
  TCB *next = tcblink_pull_with_priority(&(kernel.tcblink_ready));
  if (next != TCB_NULL) 
    kernel.tcb_current = next;
} /* schedule */



/* Handle Pending Request at lower interrupt priority, for schedule (context switching) */
__attribute__ ((used, naked)) void PendSV_Handler(void) {

  __asm volatile(
    "  MRS   r2,PSP                \n\t"
    "  SUB   r2,#32                \n\t"
    "  STMIA r2!,{r4-r7}           \n\t"
    "  MOV   r4,r8                 \n\t"
   );

  __asm volatile(
    "  MOV   r5,r9                 \n\t"
    "  MOV   r6,r10                \n\t"
    "  MOV   r7,r11                \n\t"
    "  STMIA r2!,{r4-r7}           \n\t"
   );

  __asm volatile(
    "  SUB   r2,#32                \n\t"
    "  LDR   r3,=.kernel_ptr       \n\t"
    "  LDR   r1,[r3,#0]            \n\t"
   ); 
  
  __asm volatile(
    "  LDR   r0,[r1,%0]            \n\t"
    "  STR   r2,[r0,%1]            \n\t"
    "  PUSH  {LR}                  \n\t"
    :
    : "I" (OFS_TCB_CURRENT), "I" (OFS_TCB_STACK)    
   );  /* NOTE: the "I" for constant 0..225 */


  __asm volatile(
    "  LDR    r3,=.schedule        \n\t"
    "  MOV    r2, #0x01            \n\t"  /* bit[0] to 1 for Thumb code */
    "  ORR    r3, r2               \n\t" 
    "  BLX    r3                   \n\t"
    :
    :
    : "r3", "cc"
   ); 

  
  __asm volatile(
    "  POP   {r0}                  \n\t"
    "  MOV   LR,r0                 \n\t"
    "  LDR   r3,=.kernel_ptr       \n\t"
    "  LDR   r1,[r3,#0]            \n\t"
   );

  __asm volatile(
    "  LDR   r0,[r1,%0]            \n\t"
    "  LDR   r2,[r0,%1]            \n\t"
    "  ADD   r2,#16                \n\t"
    "  LDMIA r2!,{r4-r7}           \n\t"
    :
    : "I" (OFS_TCB_CURRENT), "I" (OFS_TCB_STACK)    
   );

  __asm volatile(
    "  MOV   r8,r4                 \n\t"
    "  MOV   r9,r5                 \n\t"
    "  MOV   r10,r6                \n\t"
    "  MOV   r11,r7                \n\t"
   );

  __asm volatile(
    "  SUB   r2,#32                \n\t"
    "  LDMIA r2!,{r4-r7}           \n\t"
    "  ADD   r2,#16                \n\t"
    "  MSR   PSP,r2                \n\t"
   );


  
  
  // TODO:
  
  /* clear int !! */
  /*
  * If you use a preemptive kernel on ARM Cortex-M0/M3, perhaps you could check how your 
  * kernel handles PendSV. If you don’t see an explicit write to the PENDSVCLR bit, I would 
  * recommend that you think through the consequences of re-entering PendSV. I’d be very 
  * interested to collect a survey of how the existing kernels for ARM Cortex-M handle this 
  * situation.
  * 
  * http://embeddedgurus.com/state-space/2011/09/whats-the-state-of-your-cortex/
  */
  
  
  __asm volatile(
    "  BX    LR                    \n\t" 
   ); /* Branch to address contained in LR */ 
  
} /* PendSV_Handler */





   /* -- Thread Creation and Start-Up -> */


/*  */
void osThreadCreate(THREADID_T threadID, void (*thread)(void), uint32_t stackSize, PRIORITY_T priority) {

  TCB *tcb = (TCB *) &tcb_list[threadID];
  CPUREGS *regs;
  int i;

 // This is the button of THIS-ONE Stack 
  tcb->stack = lastStackInit;
 // Remember for the next init of Stack
  lastStackInit += stackSize;

  
  
 // TODO: Fill with 0.
 // Yes, fill some pattern in
  for (i = 0; i < stackSize; i++) 
    tcb->stack[i] = (('0' + threadID) << 24) | (('0' + threadID) << 16) | (('0' + threadID) <<  8) | (('0' + threadID) <<  0);

 // The top of THIS-ONE Stack(it grows downwards)
  tcb->stack += (stackSize - (TCB_HWSTACK_WORDSIZE + TCB_SWSTACK_WORDSIZE));
 //
  tcb->prev = TCB_NULL;
  tcb->next = TCB_NULL;


  
  // TODO:    
  //if (threadID == 0) {  /* Fixed priority = 1 -  KERNEL_ID = 0 */
  //  
  tcb->thread.info.success = 0x10;         // Init with some success-level
  tcb->thread.info.priority = priority;

  
  
  tcb->thread.info.state = STATE_READY;
  tcb->thread.info.tid = threadID;
 //
  regs = (CPUREGS *)(tcb->stack);
  regs->hw.r0 = 0x00;
  regs->hw.r1 = 0x00;
  regs->hw.r14_lr = 0x00;    // !!! TODO: Is this safe, ?make it reset?   
  regs->hw.r15_pc = (uint32_t)(thread);
  regs->hw.xpsr = 0x01000000;

 // Is it the kernel itself
  if (threadID == 0) {  /* KERNEL_ID = 0 */
    tcb->stack += TCB_SWSTACK_WORDSIZE;
    kernel.tcb_current = tcb;
   } 
  else
    tcblink_push(&(kernel.tcblink_ready), tcb);
    
} /* osThreadCreate */


/* When nothing better to do, like handing over resources to the next thread, 
 * the Kernel itself fall to sleep until next HW-interrupt. 
 * 
 * For testing the effect on power consumption use osThreadYield() instead 
 * of Chip_PMU_SleepState().
 */
__attribute__ ((used)) static void kernelPowerDown(void) {

  while (1) {
 
    // -No, keep this clean, don't get tempted, here We go to sleep-
    
   // Enter MCU Sleep mode
    Chip_PMU_SleepState(LPC_PMU);
   }
} /* kernelPowerDown */


/*  */
void osKernelInit(void) {
  
  // if (osErr = xxState) {bleBla}


  kernel.tcb_current = TCB_NULL;  

 //  
  tcblink_init(&(kernel.tcblink_ready));
  tcblink_init(&(kernel.tcblink_sleep));
  tcblink_init(&(kernel.tcblink_block));

 // NO Deep-sleep or Power-down , only Sleep mode.
   LPC_PMU->PCON |= PMU_PCON_NODPD;
   
 // The kernel is our first "Thread", Thread #0, priority 1. 
  osThreadCreate(0, kernelPowerDown, KERNEL_STACK, 1);
} /* osKernelInit */


/*  */
void osKernelRun(void) {

 /* INIT ISR-signals: Delay start before Ready. */
  kernel.signals = (NUM_OF_THREAD & IRS_SIGNAL_INIT_C_MASK);
  kernel.signals |= IRS_SIGNAL_INIT;

 /* We give the SysTick a high priority, but not The highest */ 
  NVIC_SetPriority(SysTick_IRQn, 1); 
 /* SVC and Pending Interrupt Request have low priority, but the same
  * priority due to limitations in Cortex-m0 */
  NVIC_SetPriority(SVCall_IRQn, 3);
  NVIC_SetPriority(PendSV_IRQn, 3);

 /* Set kernel clock, get the first time from board-RTC */
  kernel.time.ticks = 0;   
  kernel.time.seconds = 0; 
  
 /* System Tick-Timer Initialisation and run it. */
  SysTick_Config((SystemCoreClock / SYS_TICK_COUNTER) -1);
 /* Run schedule for the first time */
  SVC(SVCFUNC_START);

 /* -> Shouldn't be here, the watchdog timer will handle this. */
  // osErr = xxState;
} /* osKernelRun */





