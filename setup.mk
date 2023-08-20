PREFIX = mipsel-none-elf-

PSN00B_BASE	= ../psn00bsdk/
GCC_BASE	= ../mipsel-none-elf

LIBDIRS		= -L$(PSN00B_BASE)lib/libpsn00b/release
INCLUDE	 	= -I$(PSN00B_BASE)include/libpsn00b

ELF2X		= $(PSN00B_BASE)bin/elf2x
MKPSXISO    = $(PSN00B_BASE)bin/mkpsxiso
NOPS		= tools/nops
MCOMMS		= tools/mcomms

GCC_BIN		= $(GCC_BASE)/bin/

CC			= $(GCC_BIN)$(PREFIX)gcc
CXX			= $(GCC_BIN)$(PREFIX)g++
AS			= $(GCC_BIN)$(PREFIX)as
AR			= $(GCC_BIN)$(PREFIX)ar
RANLIB		= $(GCC_BIN)$(PREFIX)ranlib
LD			= $(GCC_BIN)$(PREFIX)ld

#TOOLS

MKPSXISO_XML= iso_build/iso_build.xml
BIN_FOLDER  = bin
ISO_FOLDER  = iso