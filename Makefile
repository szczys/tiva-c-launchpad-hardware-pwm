# Makefile which can be included in open source packages, more details:
# https://github.com/szczys/tiva-c-launchpad-template


# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME=main

#Part Declaration
PART=TM4C123GH6PM

# Location of the TivaWare root directory
TIVAWARE_LIB = /home/mike/compile/TivaWare

# put your *.o targets here, make should handle the rest!
#SRCS = $(PROJ_NAME).c $(TIVAWARE_LIB)/examples/boards/ek-tm4c123gxl/hello/startup_gcc.c

# location of OpenOCD Board .cfg files (only used with 'make program')
OPENOCD_BOARD_DIR=/usr/share/openocd/scripts/board

# Configuration (cfg) file containing programming directives for OpenOCD
OPENOCD_PROC_FILE=extra/stm32f0-openocd.cfg

# that's it, no need to change anything below this line!

#Create symbolic links to startup and linker
#These files can't be included in Open Source Repos
#Because of TI's restrictive licensing.
HACK:=$(shell mkdir -p gcc)
HACK:=$(shell ln -sf $(TIVAWARE_LIB)/examples/boards/ek-tm4c123gxl/hello/hello.ld gcc/$(PROJ_NAME).ld)
HACK:=$(shell ln -sf $(TIVAWARE_LIB)/examples/boards/ek-tm4c123gxl/hello/startup_gcc.c gcc/.)

include $(TIVAWARE_LIB)/makedefs

VPATH=$(TIVAWARE_LIB)/utils
IPATH=$(TIVAWARE_LIB)/

all: gcc/$(PROJ_NAME).axf

gcc/$(PROJ_NAME).axf: gcc/$(PROJ_NAME).o
gcc/$(PROJ_NAME).axf: gcc/startup_gcc.o
gcc/$(PROJ_NAME).axf: gcc/uartstdio.o
gcc/$(PROJ_NAME).axf: $(TIVAWARE_LIB)/driverlib/gcc/libdriver.a
gcc/$(PROJ_NAME).axf: gcc/$(PROJ_NAME).ld

SCATTERgcc_$(PROJ_NAME)=gcc/$(PROJ_NAME).ld
ENTRY_$(PROJ_NAME)=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C123_RB1

ifneq (${MAKECMDGOALS},clean)
-include ${wildcard gcc/*.d} __dummy__
endif

program: gcc/$(PROJ_NAME).bin 
	openocd --file /usr/local/share/openocd/scripts/board/ek-tm4c123gxl.cfg -f extra/tiva-launchpad.cfg -c "tiva_flash `basename $(CURID)` gcc/$(PROJ_NAME).bin" -c shutdown

clean:
	@rm -vf gcc/*
	@rmdir -v gcc

