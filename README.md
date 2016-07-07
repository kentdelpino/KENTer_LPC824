# KENTer_LPC824
Distribution for the LPC824 unit including HAL(CMSIS), newlib retarget, RTOS and makefile with focus on GNU linker optionLink time optimization (LTO).

  -- early stage --  
  This is published while listening to ('Changed The Way You Kiss Me'): https://www.youtube.com/watch?v=CLXt3yh2g0s 

The System setup (reset and init), Hardware Abstraction Layer (HAL; baseed on LPCopen and CMSIS) and newlib retarget-stubs for a small micro-controller based on ARM Cortex-M0+ and produced by NXP, named LPC812. 

This distribution is focusing on ANCI C programmers environment on bare-metal, meaning inclusion of newlib formatted IO functions, such as scanf and printf for very small micro-controller, mewlib-nano. Second agenda has been to take full advantage of the GNU linker optionLink time optimization (LTO) enabled.

Included is also some minimum-functions, see minimum_hw.h. The file contains som functions for initializing of a UART (stdin, stdout) and some IO-pins, plus is support direct IO control and communication with minimumPut and minimumGet on UART0 and buffer-handling via minimumIOCTL. This minimum-functions are indirectly a part of newlib-nano retarget-stubs. 

Sometimes these minimum-functions can be usefully, rather than going through the newlib-nano OI-functions which among other things are buffered and have to be flushed, except the stderr-file (see note. 1.). It should also be mentioned here that even the "nano" version of newlib fills up in a small micro-comtroller. 

   In this version only stdout, stdin and stderr are implemented and go via the 
   minimum-functions to UART0.     

# Concerning the used of RTOS, please read .\system\mick\_readme.txt

# Needed tools: 
  a/ Compiler: ARM Embedded (https://launchpad.net/gcc-arm-embedded). We have 
  tested on GCC 4.9.3-2015q1. See ARM Compiler toolchain Compiler Reference: 
  http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0491c/BABCIGHE.html

  b/ Setup: I use stand-alone makefile and script file, public domain files from, 
  me. (Include in the root of the source code). Can be used directly from the
  command line or treated as a project in IDE, such as eclipse.

  c/ I use a In-System Programming (ISP) tool to flash the unit, on UART_0. 
  One program that can be used is FlashMagic [ http://www.flashmagictool.com ]
  or a open source vrsion like, lpc21isp.


Note. 1.:
It is a known issue (it's a standard), that functions like printf() and 
fwrite() buffer-up until a text-stream is ended by '\n', '\r', '\r\n' or flush-
file command is given. This setup is optimize for strings, not single chars. A 
tested solution to unwanted buffering could be to call setbuf(stdout, NULL) before 
the first call to any IO-functions like printf(), but very inefficient.


# -- > 
CMSIS violates the following MISRA-C:2004 rules:

    Required Rule 8.5, object/function definition in header file.
    Function definitions in header files are used to allow 'inlining'.

    Required Rule 18.4, declaration of union type or object of union type: '{...}'.
    Unions are used for effective representation of core registers.

    Advisory Rule 19.7, Function-like macro defined.
    Function-like macros are used to allow more efficient code.





