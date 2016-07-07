/****************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * Linker:   retarget.c
 * Used with: GCC ARM Embedded,       https://launchpad.net/gcc-arm-embedded
 * Version:  0.2
 * Date:     28 May. 2015
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


/****************************************************************************
 * TODO:
 *  
 * In the end, of course, if We don't need this service it should not be 
 * included at link time, see: __attribute__ ((used)).
 *
 */


#include <errno.h>


/*
void * _sbrk (int nbytes) {

  extern char lds_hp_end[];     // Defined by the linker. 
  extern char lds_hp_start[];       // Defined by the linker.
 
  static char *heap_end;
  char *prev_heap_end;
 
  if (heap_end == NULL)
    heap_end = &lds_hp_start;
 
  prev_heap_end = heap_end;
 
  if (heap_end + nbytes > &lds_hp_end) {
    errno = ENOMEM;
    return (void *) -1;
   }
 
  heap_end += nbytes;
 
  return prev_heap_end;
} // _sbrk 
*/





/* Absolut ikke OK, end og heap_ptr skal in i funktionen */


extern char end[];              /*  end is set in the linker command    */
                /* file and is the end of statically    */
                /* allocated data (thus start of heap). */


static char *heap_ptr;      /* Points to current end of the heap.   */


/**************************  *************************************/
/*  Support function.  Adjusts end of heap to provide more memory to    */
/* memory allocator. Simple and dumb with no sanity checks.     */


__attribute__ ((used)) void * _sbrk (int nbytes) {

  char  *base;       //  errno should be set to ENOMEM on error

 if (!heap_ptr){    //  Initialize if first time through
    heap_ptr = end;
    }
 
 // Vi mangler nogeet kontrol, er der mere mem!
 // errno = ENOMEM;
 
 base = heap_ptr;   //  Point to end of heap.
 heap_ptr += nbytes;    //  Increase heap.

 return base;       //  Return pointer to start of new heap area. 
} /* _sbrk */



