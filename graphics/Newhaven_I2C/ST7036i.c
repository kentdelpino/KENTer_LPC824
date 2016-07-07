/**************************************************************************
 *                                    Link: www.linkedin.com/in/kentdelpino
 *                                    Author: Kent DEL PINO
 * File: ST7036i.c
 * Used with: GCC ARM Embedded,      https://launchpad.net/gcc-arm-embedded
 * Used on: LPC824                   Display types:
 *                                             NHD-C0220BiZ-FSW-FBW-3VM
 *                                             NHD-C0220BiZ-FS(RGB)-FBW-3VM
 * Version:  0.1.0
 * Date:     Nov. 2015
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

/* Include standard c library as needed. NOTE: we use newlib-nano */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "ST7036i.h"

/* LPC824 or other chip from NXP */
#include "chip.h"


/**************************************************************************
 * Variables defined here for storage directly in the bss-section(data seg.)
 */


/*  */
typedef enum {
 /*  */
  STATE_NON,
 /* */
  STATE_READY,
 /* */
  STATE_SLEEP,
} DISPLAT_STATES;


typedef volatile struct {
  uint32_t state;
  uint32_t backLight;
} display_status_t;


static display_status_t displayState = {
  .state = STATE_NON,
  .backLight = 0,
};

// Driver DDRAM addressing
const uint8_t dram_dispAddr [][3] = {
  { 0x00, 0x00, 0x00 },  // One line display address
  { 0x00, 0x40, 0x00 },  // Two line display address
  { 0x00, 0x10, 0x20 }   // Three line display address
};


// LCD Command set
#define       ADDR_ST7036         0x78   // I2C-display address
#define       DISP_CMD             0x0   // Command for the display
#define       RAM_WRITE_CMD       0x40   // Write to display RAM
#define       CLEAR_DISP_CMD      0x01   // Clear display command
#define       HOME_CMD            0x02   // Set cursos at home (0,0)
#define       DISP_ON_CMD         0x0C   // Display on command
#define       DISP_OFF_CMD        0x08   // Display off Command
#define       SET_DDRAM_CMD       0x80   // Set DDRAM address command
#define       CONTRAST_CMD        0x70   // Set contrast LCD command
#define       FUNC_SET_TBL0       0x38   // Function set - 8 bit, 2 line display 5x8, inst table 0
#define       FUNC_SET_TBL1       0x39   // Function set - 8 bit, 2 line display 5x8, inst table 1

// LCD bitmap definition
#define       CURSOR_ON_BIT ( 1 << 1 )   // Cursor selection bit in Display on cmd.
#define       BLINK_ON_BIT  ( 1 << 0 )   // Blink selection bit on Display on cmd. 
#define       PIXEL_ROWS_PER_CHAR    8   // Number of pixel rows in the LCD character
#define       MAX_USER_CHARS        16   // Maximun number of user defined characters




/**************************************************************************
 * This driver always use I2C_#2 on pin 6(SWCLK/PIO_3) and 7(SWDIO/PIO_2) 
 * on a KENTer-LPC824 based board. */


/* I2C handler (master) and memory for ROM API */
static I2C_HANDLE_T *i2c_2_Handler;
/* Handler work-space, no expected return */ 
static uint32_t i2c_2_HandlerMEM[0x20]; 
/* 100kbps I2C bit-rate, standard-low */
#define I2C_2_BITRATE         (100000)
#define I2C_OK          I2CM_STATUS_OK



/* NOTE: Power-save, leaving this function with no clock on, just 
 * initialized the unit. */
static void i2c_02_Init(void) {

 // Set up PINs on Micro controller and init handler 
#if defined (CHIP_LPC82X)

  // Debugger p.439
  
 // Reset Unit
  
  /* I2C page 227  and 233 */ 
  /* page 34  Table 23. Peripheral reset control register (PRESETCTRL, address 0x4004 8004) bit 
  description */  

 // Power-Up, by Enable the clock to I2C-unit
  Chip_I2C_Init(LPC_I2C2);

 // TODO: Slow down  
  //void Chip_I2C_SetClockDiv(LPC_I2C_T *pI2C, uint32_t clkdiv)
  
 
   
 // Assign i2c_01 to the PINs
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

  Chip_SWM_DisableFixedPin(SWM_FIXED_RST); //PIO0_5
  Chip_SWM_DisableFixedPin(SWM_FIXED_SWCLK); //PIO0_3
  Chip_SWM_DisableFixedPin(SWM_FIXED_SWDIO); //PIO0_2

  Chip_SWM_MovablePinAssign(SWM_I2C2_SCL_IO, 3);  //PIO0_3
  Chip_SWM_MovablePinAssign(SWM_I2C2_SDA_IO, 2);  //PIO0_2
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

 // Set PINs for Open-drain
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
         // Chip_IOCON_PinSetMode(LPC_IOCON, 3, PIN_MODE_INACTIVE);
         // Chip_IOCON_PinSetMode(LPC_IOCON, 2, PIN_MODE_INACTIVE);
  Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON, 3, TRUE);
  Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON, 2, TRUE);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);


  // Setup the I2C_2 ROM-handler
  i2c_2_Handler = LPC_I2CD_API->i2c_setup(LPC_I2C2_BASE, i2c_2_HandlerMEM);
 // Set I2C bitrate, just slow 
  LPC_I2CD_API->i2c_set_bitrate(i2c_2_Handler, Chip_Clock_GetSystemClockRate(), I2C_2_BITRATE);
 // Set timeout, 1 = 16* i2c-function-clock.
  LPC_I2CD_API->i2c_set_timeout(i2c_2_Handler, I2C_2_BITRATE); 

 // Power-down: by Stoping the Clock to MPU-internal I2C-unit.


  
 // 
//  Chip_I2C_DeInit(LPC_I2C2); 
  
  
#else
#error "Only support for CHIP_LPC82X"
#endif 
} /* i2c_02_Init */



/* Master transmit in polling mode, always 7bit addr. */
static uint32_t i2c_02_writeStart(uint8_t* send8, uint32_t numOfByte, bool stopFlag) {

  I2C_PARAM_T param;
  I2C_RESULT_T result;
  ErrorCode_t error_code = I2CM_STATUS_ERROR;
  
#if defined (CHIP_LPC82X)

  
  
  param.num_bytes_send = numOfByte;
  param.buffer_ptr_send = send8;
  param.num_bytes_rec = 0;
 // Acknowledge on each byte, maybe ends with stop-signal.
  if (stopFlag)
    param.stop_flag = 1;
  else
    param.stop_flag = 0;    

 // Set timeout, 1 = 16* i2c-function-clock.
  LPC_I2CD_API->i2c_set_timeout(i2c_2_Handler, 32); 
 // Do master write transfer
  error_code = LPC_I2CD_API->i2c_master_transmit_poll(i2c_2_Handler, &param, &result);
  
  
#else
#error "Only support for CHIP_LPC82X"
#endif 

  return error_code;
} /* i2c_02_Write */


/* For now, but, needed delay with-out the RTOS */
static void simpleDelay(uint32_t count) {

 // La-la,  bla is 1-3 mSec.
  uint32_t bla = 0x01fff;
  uint32_t blaBla = count;

  while (blaBla >= 1) {
    while (bla-- >= 1) {}
    blaBla--;
   }  

} /* simpleDelay */


/*  */
uint32_t dispInit_ST7036i(void) {

  uint8_t bytes[12];
  
 // Init port and I2C 
  i2c_02_Init();

 // Handler RST-PIN on display  

  
 //  
  bytes[0] = ADDR_ST7036;
  bytes[1] = DISP_CMD;
  bytes[2] = FUNC_SET_TBL0;
  i2c_02_writeStart((uint8_t *) &bytes, 3, FALSE);
  
  simpleDelay(52);
  bytes[0] = FUNC_SET_TBL1;
  i2c_02_writeStart((uint8_t *) &bytes, 1, FALSE);
  
  simpleDelay(51);
  bytes[0] = 0x14;  // Set BIAS - 1/5
  bytes[1] = 0x73;  // 73 -> 78!, Set contrast low byte
  bytes[2] = 0x5E;  // ICON disp on, Booster on, Contrast high byte 
  bytes[3] = 0x6D;  // Follower circuit (internal), amp ratio (6)
  bytes[4] = 0x0C;  // Display on
  bytes[5] = 0x01;  // Clear display
  bytes[6] = 0x06;  // Entry mode set - increment

  if (I2C_OK == i2c_02_writeStart((uint8_t *) &bytes, 7, TRUE))
    displayState.state = STATE_READY;
  else
    displayState.state = STATE_NON;

  
 //
  displayState.backLight = 1;   

 // <> 0(zero) if OK.
  return displayState.state;
} /* dispInit_ST7036i */


/*
void Show(unsigned char *text)
{
    int n,d;
    d=0x00;
    I2C_Start();
    I2C_out(Slave); RAM_WRITE_CMD
    I2C_out(Datasend);
    for(n=0;n<20;n++){
        I2C_out(*text);
        ++text;
        }
    I2C_Stop();
}
*/


/*  */
uint32_t dispWrite_ST7036i(void) {
  
  uint8_t bytes[48];
   
  if (displayState.state == STATE_SLEEP) {

   }

  if (displayState.state == STATE_READY) {
 
    bytes[0] = ADDR_ST7036;  
    bytes[1] = RAM_WRITE_CMD;  
    bytes[2] = 'K';  
    bytes[3] = 'E';  
    bytes[4] = 'N';  
    bytes[5] = 'T';  
  
    I2C_OK == i2c_02_writeStart((uint8_t *) &bytes, 6, TRUE);  
   }
  
  
_ttywrch((uint8_t) '"');  
  
  return displayState.state;
} /* dispWritee_ST7036i */






/* ---->  // Class private constants and definition 
// ----------------------------------------------------------------------------
const int     CMD_DELAY           = 1;  // Command delay in miliseconds
const int     CHAR_DELAY          = 0;  // Delay between characters in miliseconds
const int     PIXEL_ROWS_PER_CHAR = 8;  // Number of pixel rows in the LCD character
const int     MAX_USER_CHARS      = 16; // Maximun number of user defined characters

// LCD Command set
const uint8_t DISP_CMD       = 0x0;  // Command for the display
const uint8_t RAM_WRITE_CMD  = 0x40; // Write to display RAM
const uint8_t CLEAR_DISP_CMD = 0x01; // Clear display command
const uint8_t HOME_CMD       = 0x02; // Set cursos at home (0,0)
const uint8_t DISP_ON_CMD    = 0x0C; // Display on command
const uint8_t DISP_OFF_CMD   = 0x08; // Display off Command
const uint8_t SET_DDRAM_CMD  = 0x80; // Set DDRAM address command
const uint8_t CONTRAST_CMD   = 0x70; // Set contrast LCD command
const uint8_t FUNC_SET_TBL0  = 0x38; // Function set - 8 bit, 2 line display 5x8, inst table 0
const uint8_t FUNC_SET_TBL1  = 0x39; // Function set - 8 bit, 2 line display 5x8, inst table 1

// LCD bitmap definition
const uint8_t CURSOR_ON_BIT  = ( 1 << 1 );// Cursor selection bit in Display on cmd.
const uint8_t BLINK_ON_BIT   = ( 1 << 0 );// Blink selection bit on Display on cmd. 

// Driver DDRAM addressing
const uint8_t dram_dispAddr [][3] =
{
   { 0x00, 0x00, 0x00 },  // One line display address
   { 0x00, 0x40, 0x00 },  // Two line display address
   { 0x00, 0x10, 0x20 }   // Three line display address
};

// Static  member variable definitions
// ----------------------------------------------------------------------------

// Static file scope variable definitions
// ----------------------------------------------------------------------------

// Private support functions
// ----------------------------------------------------------------------------

// CLASS METHODS
// ----------------------------------------------------------------------------

// Constructors:
// ---------------------------------------------------------------------------
ST7036::ST7036(uint8_t num_lines, uint8_t num_col, 
               uint8_t i2cAddr )
{
   _num_lines    = num_lines;
    _num_col      = num_col;
   _i2cAddress   = i2cAddr; // removed I2C address shift (>> 1)
   _cmdDelay     = CMD_DELAY;
   _charDelay    = CHAR_DELAY;
   _initialised  = false;
   _backlightPin = -1;
   
}

ST7036::ST7036(uint8_t num_lines, uint8_t num_col, 
               uint8_t i2cAddr, int8_t backlightPin )
{
   _num_lines    = num_lines;
    _num_col      = num_col;
   _i2cAddress   = i2cAddr; // removed I2C address shift (>> 1)
   _cmdDelay     = CMD_DELAY;
   _charDelay    = CHAR_DELAY;
   _initialised  = false;
   _backlightPin = backlightPin;
   
   // If there is a pin assigned to the BL, set it as an output
   // ---------------------------------------------------------
   if ( _backlightPin != 0 )
   {
      pinMode ( _backlightPin, OUTPUT );
   }
   
}

// Functions: modifiers (set), selectors (get) and class methods
// ---------------------------------------------------------------------------
void ST7036::init () 
{
   size_t retVal; 
   // Initialise the Wire library.
   Wire.begin();
   
   Wire.beginTransmission ( _i2cAddress );
   Wire.write ( (byte)0x0 );   // Send command to the display
   Wire.write ( FUNC_SET_TBL0 );
   delay (10);
   Wire.write ( FUNC_SET_TBL1 );
   delay (10);
   Wire.write ( 0x14 );  // Set BIAS - 1/5
   Wire.write ( 0x73 );  // Set contrast low byte
   Wire.write ( 0x5E );  // ICON disp on, Booster on, Contrast high byte 
   Wire.write ( 0x6D );  // Follower circuit (internal), amp ratio (6)
   Wire.write ( 0x0C );  // Display on
   Wire.write ( 0x01 );  // Clear display
   Wire.write ( 0x06 );  // Entry mode set - increment
   _status = Wire.endTransmission ();
   
   if ( _status == 0 )
   {
      _initialised = true;
   }
}


void ST7036::setDelay (int cmdDelay,int charDelay) 
{
    _cmdDelay = cmdDelay;
    _charDelay = charDelay;
}


void ST7036::command(uint8_t value) 
{
   // If the LCD has been initialised correctly, write to it
   if ( _initialised )
   {
      Wire.beginTransmission ( _i2cAddress );
      Wire.write ( DISP_CMD );
      Wire.write ( value );
      _status = Wire.endTransmission ();
      delay(_cmdDelay);
   }
}


size_t ST7036::write(uint8_t value) 
{
   // If the LCD has been initialised correctly write to it
   // -----------------------------------------------------
   if ( _initialised )
   {
      
      // If it is a new line, set the cursor to the next line (1,0)
      // ----------------------------------------------------------
      if ( value == '\n' )
      {
         setCursor (1,0);
      }
      else
      {
         Wire.beginTransmission ( _i2cAddress );
         Wire.write ( RAM_WRITE_CMD );
         Wire.write ( value );
         _status = Wire.endTransmission ();
         delay(_charDelay);
      }
   }
}

size_t ST7036::write(const uint8_t *buffer, size_t size)
{
   // If the LCD has been initialised correctly, write to it
   // ------------------------------------------------------
   if ( _initialised )
   {
      Wire.beginTransmission ( _i2cAddress );
      Wire.write ( RAM_WRITE_CMD );
      Wire.write ( (uint8_t *)buffer, size );
      _status = Wire.endTransmission ();
      delay(_charDelay);
   }
}


void ST7036::clear()
{
   command (CLEAR_DISP_CMD);
}


void ST7036::home()
{
   command ( HOME_CMD );
}


void ST7036::on()
{   
   command ( DISP_ON_CMD );
}


void ST7036::off()
{
   command ( DISP_OFF_CMD );        
}


void ST7036::cursor_on()
{
   command ( DISP_ON_CMD | CURSOR_ON_BIT );
}

void ST7036::cursor_off()
{
   command ( DISP_ON_CMD & ~(CURSOR_ON_BIT) );
}

void ST7036::blink_on()
{
   command ( DISP_ON_CMD | BLINK_ON_BIT );
}

void ST7036::blink_off()
{
   command ( DISP_ON_CMD & ~(BLINK_ON_BIT) ); 
}


void ST7036::setCursor(uint8_t line_num, uint8_t x)
{
   uint8_t base = 0x00;
   
   // If the LCD has been initialised correctly, write to it
   // ------------------------------------------------------
   if ( _initialised )
   {
      // set the baseline address with respect to the number of lines of
      // the display 
      base = dram_dispAddr[_num_lines-1][line_num];
      base = SET_DDRAM_CMD + base + x;
      command ( base );
   }
}

#ifdef _LCDEXPANDED
uint8_t ST7036::status(){
    
    return _status;
}


uint8_t ST7036::keypad ()
{
   // NOT SUPPORTED
   return 0;
}


void ST7036::load_custom_character (uint8_t char_num, uint8_t *rows)
{
   // If the LCD has been initialised correctly start writing to it
   // --------------------------------------------------------------------------
   if ( _initialised )
   {
      // If it is a valid place holder for the character, write it into the
      // display's CGRAM
      // --------------------------------------------------------------------------
      if ( char_num < MAX_USER_CHARS )
      {
         // Set up the display to write into CGRAM - configure LCD to use func table 0
         Wire.beginTransmission ( _i2cAddress );
         Wire.write ( DISP_CMD );
         Wire.write ( FUNC_SET_TBL0 ); // Function set: 8 bit, 2 line display 5x8, funct tab 0
         delay ( _cmdDelay );
         
         // Set CGRAM position to write
         Wire.write ( RAM_WRITE_CMD + (PIXEL_ROWS_PER_CHAR * char_num) ); 
         _status = Wire.endTransmission ();
         
         // If we have changed the function table and configured the CGRAM position
         // write the new character to the LCD's CGRAM
         // -----------------------------------------------------------------------
         if ( _status == 0 )
         {
            write ( rows, PIXEL_ROWS_PER_CHAR ); // write the character to CGRAM 
            
            // Leave the LCD as it was - function table 1 DDRAM and set the cursor 
            // position to (0, 0) to start writing.
            command ( FUNC_SET_TBL1 );
            setCursor ( 0,0 );
         }
      }
   }
}


void ST7036::setBacklight(uint8_t new_val)
{
   // Set analog write to the pin, the routine already checks if it can
   // set a PWM or not.
   // -----------------------------------------------------------------
    if ( _backlightPin != -1 )
   {
      analogWrite ( _backlightPin, new_val );
   }
}


void ST7036::setContrast(uint8_t new_val)
{
   // Only allow 15 levels of contrast
    new_val = map ( new_val, 0, 255, 0, 15 );
   
   command(CONTRAST_CMD + new_val);
}
#endif // _LCDEXPANDED   <-----   */


