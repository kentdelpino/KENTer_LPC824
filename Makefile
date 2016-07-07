#############################################################################
#                                      Link www.linkedin.com/in/kentdelpino
#                                      Author Kent DEL PINO
# File:     Makefile
# Compiler: GCC ARM Embedded,          https://launchpad.net/gcc-arm-embedded
# Version:  1.5.5
# Date:     13 Nov. 2015;  Valencia/ Spain
# 
#  -- This source-code is released into the public domain, by the author --
#
# This file contains, is Unlicensed material:
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# The author of this material /source-code kindly ask all users and distribu-
# tors of it, to leave this notice / text in full, in all copies of it, for 
# all time to come.
#############################################################################

#############################################################################
# For the future development of makerfiles and link-script files, the 
# templates and doc. from the compiler distribution (GCC ARM Embedded in 
# Launchpad) should be used as INSPIRATION. The templates/samples 
# can be found: _installation_folder_\share\gcc-arm-none-eabi\samples
#
# * The options -mapcs, -mapcs-frame, -mtpcs-frame and -mtpcs-leaf-frame
#   will be deprecated in gcc 5.0, hence recommend to avoid them. 
#" 
#  - No problem, We do not use these options- by Kent DEL PINO; 16 Oct. 2015
#
#############################################################################


#############################################################################
# Note: 
#   Set CLEANCOMMAND = del, for Windows inveriment, which the Author use.
#
#   -DENABLE_CODE_PROTECTION Code Read Protection (CRP) read more about this 
#    in the files startup_lpc824.c, protect.h and linkLPC824.ld.
#
#  Compiling:
#    -fno-exceptions, For now, just to be sure no-exceptions is inserted.
#
#  Linker:
#    -Wl,-allow-multiple-definition, is needed in this version.
#
#############################################################################

# Name on output-files.hex and .bin
APP_NAME = NODEv05
# The file with the main() function
MAINFILE = _THE_MAIN
# Name of RTOS
RT_KERNEL = MICK_OS

    # NOTE: Task-files(Threads) is added down the APPLICATION_OBJS


# This project is very much about using Link time optimization(LTO), but 
# sometimes it is nice NOT using it. To disable it, put in 'YES' here.
#DISABLE_LTO = YES


# LPC82X(-M0+, small communications unit) and LPC134X(M3, data processing 
# unit) is in focus on this distribution, yes, the LPC812 is supported.
MPU_VERSION = CHIP_LPC82X


# Determine the language standard. The default is gnu90(ISO C90 plus GNU 
# ext.). NOTE: We target the gnu99 and we use NON standard C library or 
# retarget_stubs for newlib, it can generate warnings. The target is gnu99.  
LANGUAGE_STD = gnu90


# MPU difference.
ifeq ($(MPU_VERSION),CHIP_LPC134X)
 #TODO: or shold it be a SAM4S (M4 without FPU)
else
ifeq ($(MPU_VERSION),CHIP_LPC82X)
# The LPC824 is based on M0+ r0p1. It have ROM-based integer divide functions
# and We need a patch in ASM to use it.
SOURCE_DEF = -D$(MPU_VERSION)
SOURCE_DEF += -DCORE_M0PLUS
SOURCE_DEF += -D__USE_ROMDIVIDE
ASM_COMPONENTS = YES
else
# If no MPU is defined, We assume it is a LPC812 (M0+ r0p0).
SOURCE_DEF = -DCORE_M0PLUS
endif
endif

# Code-protection for LPC-units see settings: startup_lpc824_812.c
SOURCE_DEF += -DENABLE_CODE_PROTECTION
# Can Interrupt Service Routines signaling the kernel!.
ifeq ($(RT_KERNEL),MICK_OS)
SOURCE_DEF += -D$(RT_KERNEL)
endif

# Link script with different memory sizes, among others.
ifeq ($(MPU_VERSION),CHIP_LPC134X)
 #TODO: or shold it be a SAM4S (M4 without FPU)
else
ifeq ($(MPU_VERSION),CHIP_LPC82X)
LINKERSCRIPT = ./controller/linkLPC824.ld
else
LINKERSCRIPT = ./controller/linkLPC812.ld
endif
endif

# The command for deleting files, while using "clean" 
CLEANCOMMAND = del


# Our compiled application ends here. 
OUT_DIR = ./_out/
# Point to retarget-stubs and kernel. 
NEW_RETARGET = ./system/retarget_stubs/
ifeq ($(RT_KERNEL),MICK_OS)
SYS_KERNEL = ./system/mick/
endif
# Hardware Abstraction Layer (HAL) for our micro-controller 
MCU_DRIVER = ./controller/chip_8xx/


INC = -I. -I./controller/startup_code -I./controller/chip_common -I./controller
INC += -I$(MCU_DRIVER)config_8xx -I./controller/cmsis/inc -I$(MCU_DRIVER) 
INC += -I./system -I$(NEW_RETARGET)
ifeq ($(RT_KERNEL),MICK_OS)
INC += -I$(SYS_KERNEL) -I$(SYS_KERNEL)/lib
endif
INC += -I./protocols -I./graphics -I./functions
INC += -I./protocols/JSON -I./graphics/Newhaven_I2C


# Assembler source to-do first, such as lib-patchs or system parts. 
ifeq ($(ASM_COMPONENTS),YES)
ifeq ($(MPU_VERSION),CHIP_LPC82X)
# Software-patch for LPC824 ROM-based integer divide function.
ASM_OBJS = ./controller/chip_patch/aeabi_romdiv_patch.o
else
ASM_OBJS = 
endif
endif


# Board-files, retarget-files, hardware drivers and protocols.
SYSTEM_OBJS = ./controller/startup_code/startup_lpc824_812.o ./controller/minimum_board.o \
			./controller/min_chip_on_board.o $(NEW_RETARGET)_sbrk.o $(NEW_RETARGET)retarget.o \
			$(NEW_RETARGET)_ttywrch.o ./protocols/JSON/jsmn.o ./graphics/Newhaven_I2C/ST7036i.o

#RTOS components to use
ifeq ($(RT_KERNEL),MICK_OS)
KERNEL_OBJS = $(SYS_KERNEL)/lib/tcblink.o $(SYS_KERNEL)mick.o
endif

# HAl-layer, We use LPCopen 2_19.
ifeq ($(MPU_VERSION),CHIP_LPC82X)
CHIP_OBJS = $(MCU_DRIVER)syscon_8xx.o $(MCU_DRIVER)clock_8xx.o $(MCU_DRIVER)iocon_8xx.o $(MCU_DRIVER)swm_8xx.o \
			$(MCU_DRIVER)pinint_8xx.o $(MCU_DRIVER)pmu_8xx.o $(MCU_DRIVER)acmp_8xx.o $(MCU_DRIVER)crc_8xx.o \
			$(MCU_DRIVER)gpio_8xx.o $(MCU_DRIVER)uart_8xx.o $(MCU_DRIVER)wkt_8xx.o $(MCU_DRIVER)wwdt_8xx.o \
			$(MCU_DRIVER)sct_8xx.o $(MCU_DRIVER)sct_pwm_8xx.o $(MCU_DRIVER)i2cm_8xx.o $(MCU_DRIVER)i2cs_8xx.o \
			$(MCU_DRIVER)i2c_common_8xx.o $(MCU_DRIVER)spi_8xx.o $(MCU_DRIVER)spim_8xx.o $(MCU_DRIVER)spis_8xx.o \
			$(MCU_DRIVER)chip_8xx.o $(MCU_DRIVER)sysinit_8xx.o  $(MCU_DRIVER)irc_8xx.o $(MCU_DRIVER)stopwatch_8xx.o \
            $(MCU_DRIVER)adc_8xx.o $(MCU_DRIVER)dma_8xx.o ./controller/chip_common/ring_buffer.o 
else
CHIP_OBJS = $(MCU_DRIVER)syscon_8xx.o $(MCU_DRIVER)clock_8xx.o $(MCU_DRIVER)iocon_8xx.o $(MCU_DRIVER)swm_8xx.o \
			$(MCU_DRIVER)pinint_8xx.o $(MCU_DRIVER)pmu_8xx.o $(MCU_DRIVER)acmp_8xx.o $(MCU_DRIVER)crc_8xx.o \
			$(MCU_DRIVER)gpio_8xx.o $(MCU_DRIVER)uart_8xx.o $(MCU_DRIVER)wkt_8xx.o $(MCU_DRIVER)wwdt_8xx.o \
			$(MCU_DRIVER)sct_8xx.o $(MCU_DRIVER)sct_pwm_8xx.o $(MCU_DRIVER)i2cm_8xx.o $(MCU_DRIVER)i2cs_8xx.o \
			$(MCU_DRIVER)i2c_common_8xx.o $(MCU_DRIVER)spi_8xx.o $(MCU_DRIVER)spim_8xx.o $(MCU_DRIVER)spis_8xx.o \
			$(MCU_DRIVER)chip_8xx.o $(MCU_DRIVER)sysinit_8xx.o  $(MCU_DRIVER)irc_8xx.o $(MCU_DRIVER)stopwatch_8xx.o \
            ./controller/chip_common/ring_buffer.o
endif

# Different task-files and main. 
APPLICATION_OBJS = ./protocols/task_stdin_json.o ./protocols/task_stdout_json.o ./system/task_sys_health.o \
			./graphics/task_display.o ./task_our_app.o ./$(MAINFILE).o

# -THE ORDER- in which source-code is compiled and objects linked, matters.
LIST_OF_ALL_OBJS = $(SYSTEM_OBJS) $(KERNEL_OBJS) $(CHIP_OBJS) $(APPLICATION_OBJS)


# ARM-core compiler and tools
CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
AS = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size


# Set instruction-set and special CPU properties, little-endian just to be sure
ifeq ($(MPU_VERSION),CHIP_LPC82X)
# Target Thumb1 instructions for ARM, enable Cortex-m0+ r0p1 1-cycle multiplier
CPU = -mthumb -mlittle-endian -mtune=cortex-m0plus.small-multiply -mcpu=cortex-m0plus
else
CPU = -mthumb -mlittle-endian -mcpu=cortex-m0plus
endif
FPU = 

# Compiler options, small code-size in focus, see info at: 
#                      https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html
#
CFLAGS = $(CPU) $(FPU) $(SOURCE_DEF) -std=$(LANGUAGE_STD)
ifeq ($(DISABLE_LTO),YES)
CFLAGS += -Os -fno-exceptions -ffunction-sections -fdata-sections
else
CFLAGS += -Os -flto -fno-exceptions -ffunction-sections -fdata-sections
endif
CFLAGS += $(INC)
CXXFLAGS = $(CFLAGS)
ASFLAGS = -mthumb -mcpu=cortex-m0plus

# Link for small code-size
GC= -Wl,--gc-sections -Wl,-allow-multiple-definition

# Create map file to view symbols
MAP=-Wl,-Map=$(MAINFILE).map

# C-libs to link-to (includes), is used by the linker, only if needed:
LIBC = ${shell ${CC} ${CFLAGS} --print-file-name=libc_nano.a}
LIBG = ${shell ${CC} ${CFLAGS} --print-file-name=libg_nano.a}
LIBM = ${shell ${CC} ${CFLAGS} --print-file-name=libm.a}
LIBGCC = ${shell ${CC} ${CFLAGS} --print-libgcc-file-name}

LIB_TO_USE = $(LIBC) $(LIBG) $(LIBM) $(LIBGCC)


# We supply the linker script file, by using the `-T' 
LDSCRIPTS = -L. -L./ -T $(LINKERSCRIPT)
LFLAGS = --specs=nano.specs --specs=nosys.specs $(LDSCRIPTS) $(GC) $(LIB_TO_USE) $(MAP)


# ALL - make us an application. 
ifeq ($(ASM_COMPONENTS),YES)
all: asm $(OUT_DIR)$(MAINFILE).bin
else
all: $(OUT_DIR)$(MAINFILE).bin
endif
$(OUT_DIR)$(MAINFILE).axf: $(MAINFILE).c $(ASM_OBJS) $(LIST_OF_ALL_OBJS)
	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@
	-@echo ""
	@$(SIZE) $(OUT_DIR)$(MAINFILE).axf

%.bin:%.axf
	-@echo ""
ifeq ($(DISABLE_LTO),YES)
	-@echo "Generating $(OUT_DIR)$(APP_NAME).bin for MPU $(MPU_VERSION); Linked with-OUT -LTO-"
else	
	-@echo "Generating $(OUT_DIR)$(APP_NAME).bin for MPU $(MPU_VERSION)"
endif
	@$(OBJCOPY) --strip-unneeded -O binary $(OUT_DIR)$(MAINFILE).axf $(OUT_DIR)$(APP_NAME).bin
	-@echo "Generating $(OUT_DIR)$(APP_NAME).hex (FlashMagic, etc.)"
	@$(OBJCOPY) --strip-unneeded -O ihex $(OUT_DIR)$(MAINFILE).axf $(OUT_DIR)$(APP_NAME).hex


asm: $(ASM_OBJS)
$(AS) $(ASFLAGS) -Wa,-mimplicit-it=thumb -o $(ASM_OBJS)


clean:
ifeq ($(CLEANCOMMAND),del)
	del /S *.o
	del *.map
	del .\_out\$(MAINFILE).axf .\_out\$(APP_NAME).hex .\_out\$(APP_NAME).bin
else
	@rm -f $(LIST_OF_ALL_OBJS)
	@rm -f *.o *.map
	@rm -f $(OUT_DIR)$(MAINFILE).axf $(OUT_DIR)$(MAINFILE).hex $(OUT_DIR)$(MAINFILE).bin
endif

