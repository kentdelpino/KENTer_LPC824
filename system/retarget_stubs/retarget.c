/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * Linker:   retarget.c
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.4
 * Date:     26 Jun. 2015
 *
 * NOTE: 
 *    Distributed as a part of the KENTer_LPC8xx_sys packagefor LPC8xx MCUs.
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

#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

#include "minimum_board.h"


/* Write "len" of char from "ptr3" to file id "fd". Return number of char
 * written. This function is used by fprintf() and fwrite() among others.
 */ 
__attribute__ ((used)) int _write (int fd, char *ptr, int len) {

  int result = -1;

 // "stdout" and "srderr", these files are opened as a minimum
  if ((fd == STDOUT_FILENO) || (fd == STDERR_FILENO)) {
   // Wait for previous "Writes" to end, limit buffer need.
    MinWaitPutCompleted();
   // Put data out on UART
    result = MinimumPut((uint8_t *) ptr, len);
    if (result == -1) {
     // "Permission denied", but find "Device or resource busy"(EBUSY) better.
      errno = EACCES;
     }
    else {
     // OK: Now, wait until MinimumPut() is completed 
      MinWaitPutCompleted();
     } 
   }
  else {
   // "Bad file number" the fd. 
    errno = EBADF;
   }

  return result;
}  // _write



/* Read "len" of char to "ptr" from file id "fd". Return number of 
 * char that has been read. 
 */
__attribute__ ((used)) int _read (int fd, char *ptr, int len) {
 
  int result = -1;
 
  // "stdin" is opened as a minimum
   if (fd == STDIN_FILENO)  {
    // Is there data available
     if (MIN_RECEIVED_DATA)
       return MinimumGet((uint8_t *) ptr, len);
     else
       return 0; // EOF;
    }
   else {
    // "Bad file number" the fd. 
     errno = EBADF;
    }

   return result;
}  // _read




/****************************************************************************
 * All other retarget functions are in separate files
 */


