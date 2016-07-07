/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: task_alive.c
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.1
 * Date:     Nov. 2015
 *
 *
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

/* Include standard c library as needed. NOTE: we use newlib-nano */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Application tools/ protocols to be used */
#include "minimum_board.h"
#include "mick.h"


/***************************************************************************
 * Declare Variables here for storage directly in the bss-section(data seg.)
 */




void task_sys_health(void) {  /* The TASK entry - its main() */


 /* LOCAL STACK/MEM: Variables here are stored on local Task Stack. Remember 
  * to adjust, according to Your needs, the Stack-size in _DEF_YOUR_TASKS. 
  * 
  * Doing Thread initialization this Mem-area is filled with NULLs(0).
  */

  
 // Inter-Process Communication(IPC), run-time test
  THREADID_T ipcID;
  SIGNAL_LEN_T ipcSigLen;
 // Nice to have, blinking LED  
#if defined (BOARD_LED_PIN)
  int boardTestLED;
#endif    

  
  uint8_t dData[10];
  int8_t conv;




 // Initialization of Watchdog Timer 



  while (1) { /* Task infinite loop */

   // For later, enable clock to I2C-unit (RTC chip and temp. sensors)
    Chip_I2C_Init(MPU_I2C);    


  /****** Inter-Process Communication(IPC), run-time test ******/

#if defined (BOARD_LED_PIN)
/*
   // A simple way to testing it. Remember it is blocking if no receiver
    boardTestLED = !boardTestLED;
    if (boardTestLED)
      ipcSigLen = 0xAA; // ON
    else      
      ipcSigLen = 0x55; // OFF     
   // A Output task is not as high priorities as an input task
    osThreadSend(OUT_JSON_ID, &ipcSigLen, 0);
*/    
#else

#endif    


  /******              Kernel health check                ******/

    
            // !!! somehow make it posible to ask system something //tcb->state = STATE_READY
  



  /****** Time-synchronization and temperature monitoring ******/


   // Read RTC chip, for time-synchronization     
    dData[0] = PCF85063A_ON_BOARD | 0; // Write
    dData[1] = PCF85063A_SC_READY;
    if (I2CM_STATUS_OK != i2c_onMPU_write((uint8_t*) &dData, 2, TRUE)) {
     // printf("Error RTC\r\n");
     }

   // Read time HH:MM:SS
    dData[0] = PCF85063A_ON_BOARD | 1;   // Read  
    if (I2CM_STATUS_OK != i2c_onMPU_read_direct((uint8_t*) &dData, 4)) {
     // printf("Error RTC\r\n");
     }
    else {
      dData[6] = BCD_ToD(dData[3]);
      dData[5] = BCD_ToD(dData[2]);
      dData[4] = BCD_ToD(dData[1] & ~PCF85063A_INTEGRITY);

      printf("\r\n %d : %d : %d ", dData[6], dData[5], dData[4]);  
     }

   
     
   /* Reading PCB-Board temperature. Also IF DEFINED, read OFF_BOARD_TEMP_SENSOR */

   // Set Register-Pointer to Temperature
    dData[0] = LM75B_ON_BOARD | 0; // Write
   // Register-Pointer is 00.
    dData[1] = 0x00;
    if (I2CM_STATUS_OK != i2c_onMPU_write((uint8_t*) &dData, 2, TRUE)) {
     // printf("Error TEMP\r\n");
     }
   // Read Temperature from Register-Pointer
    dData[0] = LM75B_ON_BOARD | 1;   // Read  // 3 is including the Addr.
    if (I2CM_STATUS_OK != i2c_onMPU_read_direct((uint8_t*) &dData, 3)) {
     // printf("Error TEMP\r\n");
     }
    else {
     // Cal the temp
      conv = BitsToCelsius(dData[1], dData[2]);
      printf(" temp. = %d\r\n", conv);  
     }


   // Temperatur sensor: Go to SLEEP
  //  TempOnBoardPwDown();
    //if (sensorOffBoard) ,,,,,
    
   // Disable Clock to I2C1
  //  Chip_I2C_DeInit(LPC_I2C1); 

   // Task to sleep 
    osDelay(TICKS_MILLISEC(1000));     
    

   // Re-start clocking the I2C1
  //  Chip_I2C_Init(LPC_I2C1);    

  //  TempOnBoardPwUp();
    //if (sensorOffBoard) ,,,,,
    
  //  Chip_I2C_DeInit(LPC_I2C1); 

   // Give things time to wake-up (>100 ms)
  //  osDelay(TICKS_MILLISEC(100));

   
  /******            NULLing watchdog timer               ******/     

 



  } /* Task loop */ 
} /* task_sys_health */










