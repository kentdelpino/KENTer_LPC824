/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: display_task.c
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.1
 * Date:     Nov. 2015
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

/* Give access to hardware */
#include "minimum_board.h"
/* Minimum Inter-process Communication Kernel (MICK), it also included: _DEF_YOUR_TASKS.h and time.h*/
#include "mick.h"
/* Display driver */
#include "ST7036i.h"


/***************************************************************************
 * Declare Variables here for storage directly in the bss-section(data seg.)
 */



/*  */
static void init_display(void) { 


  dispInit_ST7036i();

} /* init_display */




/** main(), for this task *************************************************/
void task_display(void) {

  
 /* LOCAL STACK/MEM: Variables here are stored on local Task Stack. Remember 
  * to adjust, according to Your needs, the Stack-size in _DEF_YOUR_TASKS. 
  * 
  * Doing Thread initialization this Mem-area is filled with NULLs(0).
  */

 // Task Inter-Comm
  THREADID_T ipcID;
  SIGNAL_LEN_T ipcSigLen;
  uint8_t *ipcDataPtr;
 // Display Buffer and State
  #define DISPLAY_MAX    40
  int stateOfUserInt;
  uint8_t displayMEM[DISPLAY_MAX + 2]; 


 //   
  ipcID = 0xff;
  ipcDataPtr = 0;
  stateOfUserInt = 3;



 /* Task loop(endless) */ 
  while (1) {


    // Thread notifications
     if (ipcID <= NUM_OF_THREAD) {
       
      // Text-display Massages 
       if (ipcDataPtr != 0) {



        }
      // Control-Signals
       else {
        // - Inter-Process Communication(IPC), Run-time test -
         #if defined (BOARD_LED_PIN)
           if (ipcID == SYS_HEALTH_ID) {  
             switch (ipcSigLen) {
                case 0xAA:
                  MinimumLED_ON();
                  break;
                case 0x55:
                  MinimumLED_OFF();
                  break;
                default:
                  break;
               } 
            }
         #else
         #endif


           
           
        }
      }  // Thread notifications  
    
    
   // More to put on Display
    if (stateOfUserInt >= 1) {

      if (stateOfUserInt == 2)
        init_display();
      
      if (stateOfUserInt == 1)
        dispWrite_ST7036i();

      stateOfUserInt--;
      }


    // Wait for new input 
     ipcID = 0xFF;
     ipcSigLen = DISPLAY_MAX;
    // To indicated We are ready with Array/String -buffer
     ipcDataPtr = displayMEM; 
     osThreadRecv(&ipcID, &ipcSigLen, &ipcDataPtr); 

     
     
     
if (stateOfUserInt == 0)
  stateOfUserInt = 1;   
     




  } /* Task loop(endless) */
} /* main(), task_display */




