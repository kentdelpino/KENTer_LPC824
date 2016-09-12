/***************************************************************************
 *                                   Link: www.linkedin.com/in/kentdelpino
 *                                   Author: Kent DEL PINO
 * File: task_stdin_json.c
 * Used with: GCC ARM Embedded,      https://launchpad.net/gcc-arm-embedded
 * Version:  0.1.1
 * Date:     1 Apr. 2016
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
 * OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE.
 *
 * The author of this material /source-code kindly ask all users and distri-
 * butors of it, to leave this notice / text in full, in all copies of it, 
 * for all time to come.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Application tools/ protocols to be used */
#include "minimum_board.h"
#include "mick.h"
#include "jsmn.h"


/***************************************************************************
 * Declare Variables here for storage directly in the bss-section(data seg.)
 */





/* Task main() */
void task_stdin_json(void) {

 /* LOCAL STACK/MEM: Variables here are stored on local Task Stack. Remember 
  * to adjust, according to Your needs, the Stack-size in _DEF_YOUR_TASKS. 
  * 
  * Doing Thread initialization this Mem-area is filled with NULLs(0).
  */


 // Two holders for receiving Inter-Process Communication(IPC)
  THREADID_T ipcID;
  SIGNAL_LEN_T ipcSignalLen;
  uint8_t *ipcArrayPtr;
 // Communications Buffer
  #define TTY_RX_MAX 96
  int ttyResult = 0;
  uint8_t ttyS0_RX[TTY_RX_MAX + 1]; /* USB/Debug */
  uint8_t ttyS2_RX[TTY_RX_MAX + 1]; /* RS-485 */


 /* Task loop(endless) */
  while (1) {


   // Wait for new input 
    ipcID = 0xFF;
    ipcSignalLen = TTY_RX_MAX;
   // To indicated We are ready with Array/String -buffer
    ipcArrayPtr = ttyS0_RX; 
    osThreadRecv(&ipcID, &ipcSignalLen, &ipcArrayPtr);

  
    switch (ipcID) {
   
     // Kernel SIGNALs (Kernel-ID = 0)
      case 0:  
     
        switch (ipcSignalLen) {
         // USB/WiFi/ttyS0 (stdin)
          case IRS_SIGNAL_STDIN:   
  
              ttyResult = MinimumGet(ttyS0_RX, TTY_RX_MAX);
              if (ttyResult != -1) {

               
                if ((ttyResult == 7) && (ttyS0_RX[0] != '{')) {  // "reset<CR><NL>"

                  if ((ttyS0_RX[0] == 'r') && (ttyS0_RX[2] == 's') && (ttyS0_RX[4] == 't')) 
                    /* reset rest reset reset */          NVIC_SystemReset();
    
                } // TEMP here 
               

                if (ttyS0_RX[0] == '{') {
                  ipcSignalLen = ttyResult;
                  ipcArrayPtr = ttyS0_RX; 
                  osThreadSend(OUT_JSON_ID, &ipcSignalLen, &ipcArrayPtr);
                
                 }
                else 
                  ttyResult = MinimumPut(ttyS0_RX, ttyResult);
              
             }
            break;

         // RS-485/DMX/ttyS2
          case IRS_SIGNAL_RS485_RXD: 
            
            break;
            
          default:
            break;
        } 

     // Other ID (Massagers/Signals)   
      default:  
          
       // Signals 
        if (ipcArrayPtr == 0) {  
          

          
          // - Inter-Process Communication(IPC), Run-time test -
          #if defined (BOARD_LED_PIN)
             if (ipcID == SYS_HEALTH_ID) {
               switch (ipcSignalLen) {
                 case 0xAA:
                     MinimumLED_ON();
                   break;
                 case 0x55:
                     MinimumLED_OFF();
                   break;
                 }
               }  
          #endif          
          
          
 
         }   
       // Massagers 
        else { 



         }

        break;
 
     } // switch (ipcID)


  } /* Task loop() */
} /* Task main() */



