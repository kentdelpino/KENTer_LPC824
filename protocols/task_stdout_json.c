/**************************************************************************
 *                                   Link: www.linkedin.com/in/kentdelpino
 *                                   Author: Kent DEL PINO
 * File: task_stdout_json.c
 * Used with: GCC ARM Embedded,      https://launchpad.net/gcc-arm-embedded
 * Version:  0.1.1
 * Date:     1 Apr. 2016
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

/* Include standard c library as needed. NOTE: we use newlib-nano */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Application tools/ protocols to be used */
#include "minimum_board.h"
#include "mick.h"


/**************************************************************************
 * Declare Variables here for storage directly in the bss-section(data seg.)
 */



/* Task main() */
void task_stdout_json(void) {
  
  
 /* LOCAL STACK: Variables here are stored on local Task Stack. Remember to 
  * adjust, according to Your needs, the Stack-size in _DEF_YOUR_TASKS. 
  */

 // Two holders for receiving Inter-Process Communication(IPC)
  THREADID_T iProcID;
  SIGNAL_LEN_T iProcVal;
  uint8_t *iProcPtr;
 // Stay awake a little, mayby there is more to do
  int stayAwake;

 // Communications Buffer
  #define TTY_TxD_MAX     64
  int ttyResult;
  uint8_t ttyS0_TxD[TTY_TxD_MAX + 2]; /* USB/Debug */
  uint8_t ttyS2_TxD[TTY_TxD_MAX + 2]; /* RS-485 */


  
  
  
  
int tempCounter = 0;
stayAwake = 0;


 /* Infinity loop */ 
  while (1) {


   // Go sleep until Event/Signal 
    iProcID = 0xFF;
    iProcVal = TTY_TxD_MAX;
    iProcPtr = ttyS0_TxD;
    if (--stayAwake <= 0) {
       osThreadRecv(&iProcID, &iProcVal, &iProcPtr);
      // 
       stayAwake = 1;
      }


    // IPC Signals only
     if (iProcPtr == 0) {
       switch (iProcID) {

         case 0:  // (Kernel-ID = 0)

           
          
           
           
           break;

         default:

         // - Inter-Process Communication(IPC), Run-time test -
         #if defined (BOARD_LED_PIN)
            if (iProcID == SYS_HEALTH_ID) {
              switch (iProcVal) {
                case 0xAA:
                    MinimumLED_ON();
                  break;
                case 0x55:
                    MinimumLED_OFF();
                  break;
                }
              }  
         #endif

           break;
        } 
      }
    // Or Massage 
     else {
       
       if (iProcID == IN_JSON_ID) {
         tempCounter++;
         printf(" c.l (%d.%d): \n\r", tempCounter, iProcVal);
         ttyResult = MinimumPut(iProcPtr, iProcVal); 
        }

      }
       

  }  /* Infinity loop */ 
} /* Task main() */





