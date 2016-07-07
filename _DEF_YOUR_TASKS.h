/**************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: DEF_YOUR_TASKS.h
 * Used with: GCC ARM Embedded,      https://launchpad.net/gcc-arm-embedded
 * Version:  0.2.0
 * Date:     1 JAN. 2016
 *
 * Declarations of:
 *    - Threads to run on the MICK-scheduler, their priority and Stack size.
 *    - IRS_SIGNAL routing 
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
#ifndef DEF_YOUR_TASKS_H
#define DEF_YOUR_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif


/* Task included header-files. Remember to add them to makefile, also */
#include "task_stdin_json.h"
#include "task_stdout_json.h"
#include "task_sys_health.h"
#include "task_our_app.h"
#include "task_display.h"


/* Define each thread's/task's ID here. Each thread with a unique number 
 * from 1 and consecutively up, and with a priority greater than 1 (higher 
 * number, LOWER-PRIORITY). The Kernel itself has ID 0 and priority 1, this 
 * is NOT to be changed.
 * 
 * The third number We need to define is the stack size of each thread/task.
 * Remember the stack size is in words of 4-bytes. 
 * 
 *  * NOTE: Stack used by IAP commands can be up-to 148 bytes/37 words. 
 * 
 * And then at last, update SYS_STACK_NEEDED, the sum of all Stacks.
 * 
 * Communications receiver Threads should have high priority.
 */

/* Thread-ID */
enum TID {
     KERNEL_ID,           // First, is #0, always named KERNEL_ID
  IN_JSON_ID,
  OUT_JSON_ID,
  SYS_HEALTH_ID,     
  OUR_APP_ID,
  DISPLAY_ID,
     NUM_OF_THREAD        // Last, always named NUM_OF_THREAD - MUY IMPORTANTE -
};

/* Thread priority */
enum TPRIORITY {
     KERNEL_PRIO = 1,     // Kernel priority, always 1
  IN_JSON_PRIO = 2,
  OUT_JSON_PRIO = 2,
  SYS_HEALTH_PRIO = 2,
  OUR_APP_PRIO = 3,
  DISPLAY_PRIO = 3, 
};

/* Stack size of individuel threads in words (4* bytes) */
enum TSTACK {
     KERNEL_STACK = 48,   // The Kernel do not use so much.
  IN_JSON_STACK = 128,    // If Big local buffer is allocated or if Newlib is used 
  OUT_JSON_STACK = 128,   // IO-formatting of text, good Stack is needed.
  SYS_HEALTH_STACK = 192,  // 48
  OUR_APP_STACK = 192,
  DISPLAY_STACK = 128,
};

/* Warning: It is of course important to get this right. */                                /* + DISPLAY_STACK */
#define OS_STACK_NEEDED (KERNEL_STACK + IN_JSON_STACK + OUT_JSON_STACK + SYS_HEALTH_STACK + OUR_APP_STACK + DISPLAY_STACK)



/**************************************************************************
 *
 *   TODO: HERE we need a IfNotDefined (MICK_IRS_SIGNAL) 
 *   
 */


/*  */
#define STDIN_HANDLER          IN_JSON_ID          
#define STDOUT_HANDLER         OUT_JSON_ID          




#ifdef __cplusplus
}
#endif

#endif /* DEF_YOUR_TASKS_H */

