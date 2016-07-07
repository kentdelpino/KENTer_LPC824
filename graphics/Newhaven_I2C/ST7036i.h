/**************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: ST7036i.h
 * Used with: GCC ARM Embedded,      https://launchpad.net/gcc-arm-embedded
 * Used on: LPC824                   Display types:
 *                                             NHD-C0220BiZ-FSW-FBW-3VM
 *                                             NHD-C0220BiZ-FS(RGB)-FBW-3VM
 * Version:  0.1.0
 * Date:     Nov. 2015
 *
 *  
 * This driver always use I2C_#2 on pin 6(SWCLK) and 7(SWDIO) on a KENTer
 * LPC824 based board. Also it runs pulling-mode via NXP ROM-based code 
 * and only output is supported (ST7036i lim.).
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

#ifndef __ST7036i_DRIVER_H
#define __ST7036i_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif




uint32_t dispInit_ST7036i(void);
uint32_t dispWrite_ST7036i(void);




#ifdef __cplusplus
}
#endif

#endif /* __ST7036i_DRIVER_H */
