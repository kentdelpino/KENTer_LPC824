
 /* last change, Date:     1 Apr. 2016  */




#ifndef TASK_CONTROL_BLOCK_H
#define TASK_CONTROL_BLOCK_H


#ifdef __cplusplus
extern "C" {
#endif


#include "mick.h"



/* IPC parameters, for a send(er).
 * 3 parameters give meaning on ARM Cortex-M series (reg. r0-r2). 
 * Do think about the alignment, a (char *) need to be aligned = 4. 
 * 
 * NOTE: We use Double pointer.
 */ 
typedef struct {
  uint8_t **massage_pp;      /* (r2) Words need to be aligned = 4. */
  SIGNAL_LEN_T *signall_p;   /* (r1) Pointer to Signal or Length */
  THREADID_T destID;         /* (r0), Sender ID is only 8-bit */
} CALL_BLOCKED_EVENT_SEND_T __attribute__ ((aligned (4)));

/* IPC parameters, for receiver */ 
typedef struct {
  uint8_t **massageReady_pp;
  SIGNAL_LEN_T *signall_p;
  THREADID_T *sourID_p;  /* (r0), Here the 'ID' is a pointer to a ID type, 32-bit*/
} CALL_BLOCKED_EVENT_RECV_T __attribute__ ((aligned (4)));

/* A Delay-counter for when a Thread/Task is asking for System Delay */
typedef struct {
  uint32_t count;
} CALL_BLOCKED_SLEEP_T __attribute__ ((aligned (4)));

/* Thread/Task local Stack type */
typedef volatile unsigned int TCBSTACK;

/* Thread/Task Control Block */
typedef volatile struct TCB TCB;
/* Organization of the Control Block */ 
struct TCB {
  TCBSTACK *stack;  /* OFFSET #0 -> to current Stack, need alignment */
  TCB *prev;
  TCB *next;
  union {
    CALL_BLOCKED_SLEEP_T sleep;
    CALL_BLOCKED_EVENT_SEND_T sender;
    CALL_BLOCKED_EVENT_RECV_T receiver;
   } sys;
  union {
    uint32_t info_as32;
    THREAD_WORD_INFO_T info;
   } thread;
};

/* Offset, in words, to Stack-pointer in TCB structure */
#define OFS_TCB_STACK        (0)
/* The size of one Task Control Block, in words */
#define TCB_TCB_WORDSIZE     ((sizeof(TCB)) / sizeof(uint32_t))
/* Empty or non-used Control Block */
#define TCB_NULL             ((TCB *)0)



// -> TODO: Use CMSIS, can I !!
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
/**************************************************************************
 */
#define TCB_HWSTACK_WORDSIZE    ((sizeof(HWSTKREGS)) / sizeof(unsigned int))
#define TCB_SWSTACK_WORDSIZE    ((sizeof(SWSTKREGS)) / sizeof(unsigned int))


typedef struct {
    /**
     * @brief Low Register R0
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r0;

    /**
     * @brief Low Register R1
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r1;

    /**
     * @brief Low Register R2
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r2;

    /**
     * @brief Low Register R3
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r3;

    /**
     * @brief High Register R12
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r12;

    /**
     * @brief Link Register
     * @details
     * The Link Register (LR) is register R14.
     * It stores the return information for subroutines, function calls, and exceptions.
     * On reset, the LR value is Unknown.
     */
    unsigned int r14_lr;

    /**
     * @brief Program Counter
     * @details
     * The Program Counter (PC) is register R15.
     * It contains the current program address.
     * On reset, the processor loads the PC with the value of the reset vector, that is at address 0x00000004.
     * Bit[0] of the value is loaded into the EPSR T-bit at reset and must be 1.
     */
    unsigned int r15_pc;

    /**
     * @brief Program Status Register
     * @details
     * The Program Status Register (PSR) combines:
     * - Application Program Status Register (APSR).
     * - Interrupt Program Status Register (IPSR).
     * - Execution Program Status Register (EPSR).
     * .
     * These registers are allocated as mutually exclusive bitfields within the 32-bit PSR.
     * The PSR bit assignments are:
     * @image html program_status_register.png
     * Access these registers individually or as a combination of any two or all three registers,
     * using the register name as an argument to the MSR or MRS instructions.
     * For example:
     * - Read all of the registers using PSR with the MRS instruction.
     * - Write to the APSR using APSR with the MSR instruction.
     * .
     */
    unsigned int xpsr;
} HWSTKREGS;

typedef struct {
    /**
     * @brief Low Register R4
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r4;

    /**
     * @brief Low Register R5
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r5;

    /**
     * @brief Low Register R6
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r6;

    /**
     * @brief Low Register R7
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r7;

    /**
     * @brief High Register R8
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r8;

    /**
     * @brief High Register R9
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r9;

    /**
     * @brief High Register R10
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r10;

    /**
     * @brief High Register R11
     * @details R0-R12 are 32-bit general-purpose registers for data operations.
     */
    unsigned int r11;
} SWSTKREGS;

typedef struct {
    SWSTKREGS sw;
    HWSTKREGS hw;
} CPUREGS;



#ifdef __cplusplus
}
#endif

#endif /* TASK_CONTROL_BLOCK_H */

