                                                       -*- mode: text -*-

                            Newlib-nano
                  Bin Cheng <bin.cheng@arm.com>

Introduction

Newlib is a C library that can be used on embedded systems.  However, as the
feature set has been increased it has become too bloated for use on the very
smallest systems where the amount of memory can be very limited.  In order to
provide a C library with a very small footprint, suited for use with micro-
controllers, we have created newlib-nano based on newlib 1.19.0.

This document explains the difference between newlib-nano and newlib, and
how to use it.


What has changed from newlib 1.19.0?

From the view of interface, the following user-visible changes exist between
standard newlib and newlib-nano:

1) The formatted printing and parsing routines (the printf and scanf family
   of routines) have been re-implemented to:
   a) remove a direct dependency on the floating-point input/output handling
      code.  Programs that need to handle floating-point values using these
      functions must now explicitly request the feature during linking.  See
      Usage below.
   b) support only conversion specifiers defined in C89 standard.  This
      brings us good balance between small memory footprint and full feature
      formatted input/output.
   c) remove now redundant integer-only implementations of the printf/scanf
      family of routines (iprintf/iscanf, etc).  The functions now alias the
      standard routines.  This avoids the risk of getting duplicated
      functionality when these operations are needed.  The affected functions
      are:
           
        diprintf vdiprintf
 
        siprintf fiprintf iprintf sniprintf asiprintf asniprintf
 
        siscanf fiscanf iscanf
 
        viprintf vfiprintf vsiprintf vsniprintf vasiprintf vasniprintf
 
        viscanf vfiscanf vsiscanf
 
        _diprintf_r _vdiprintf_r
  
        _siprintf_r _fiprintf_r _iprintf_r _sniprintf_r _asiprintf_r
        _asniprintf_r

        _siscanf_r _fiscanf_r _iscanf_r

        _viprintf_r _vfiprintf_r _vsiprintf_r _asniprintf_r _vasiprintf_r
        _vasniprintf_r

        _viscanf_r _vfiscanf_r _vsiscanf_r

   With the re-implemented formatted input/output routines, the following
   newlib configuration options are not supported in newlib-nano:

     enable-newlib-io-pos-args
     enable-newlib-io-c99-formats
     enable-newlib-io-long-long
     enable-newlib-io-long-double

   Additionally, "enable-newlib-io-float" is no longer needed because of
   changes to the handling of floating-point input/output in newlib-nano.

2) Newlib-nano has a hard limit of at most 32 functions being registered
   with atexit().  The standard newlib configuration option
     enable-newlib-atexit-dynamic-alloc
   is not supported in newlib-nano.

3) The C standard has the following requirement for the exit function:

     "all open streams with unwritten buffered data are flushed, all open
     streams are closed."

   In newlib-nano, only unwritten buffered data is flushed on exit.  Open
   streams are not closed.

4) The dynamic memory allocator has been re-implemented.  The newlib
   configuration option "enable-malloc-debugging" is not supported in
   newlib-nano.

Usage

Newlib-nano works in exactly the same way as newlib works, you can configure,
build, install and use it in the same way as newlib with the following
exceptions:

1) The configuration options previously described as not supported or needed
   in newlib-nano should not be used.

2) Programs that require formatted floating-point input/output must explicitly
   reference the relevant support function during linking.  For the scanf()
   family of routines the support function is "_scanf_float".  For the
   printf() family of routines the support function is "_printf_float".
   There are two ways to achieve this:
   a) Explicitly reference the support function during linking.  For example,
      by adding "-u _printf_float" to the link command:

        $ cat > tst.c <<EOF
        > #include <stdio.h>
        > int
        > main (int argc, char **argv)
        > {
        >   puts ("floating point number is: %f\n", 1.01);
        >   return 0;
        > }
        > EOF
        $ arm-none-eabi-gcc -mthumb -mcpu=cortex-m3 tst.c -o tst-1.out \
                            -lc -lnosys -lc
        $ arm-none-eabi-readelf -s tst-1.out | grep "_printf_float"
           317: 00000000     0 NOTYPE  WEAK   DEFAULT  UND _printf_loat

        $ arm-none-eabi-gcc -mthumb -mcpu=cortex-m3 tst.c -o tst-2.out \
                            -lc -lnosys -lc -u _printf_float
        $ arm-none-eabi-readelf -s tst-2.out | grep "_printf_float"
           413: 000083c5  1230 FUNC    WEAK   DEFAULT    2 _printf_float

      We can see "_printf_float" is pulled in binary by "-u" option.  The same
      story stands for scanf, only need to replace symbol "_printf_float" with
      "_scanf_float".

   b) Put an explicit reference to the support function into one of the object
      files in your program.  One way to do this is to add a statement such as
        asm (".global _scanf_float");
      to one or more of the source files in your program.

3) Newlib-nano is published as a stand alone C library and it is also
   published in "GNU Tools for ARM Embedded Processors" as pre-built binaries.
   The usage of newlib-nano in that tool is a little different.  Please refer
   to documents in that tool for more information.  You can find the tool at
   link: "https://launchpad.net/gcc-arm-embedded".