#------------------------------------------------------------------------------
#                           ENVIRONMENT VARIABLES                              
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
#                         The adjust-path macro
#
# If Make is launched from Windows through
# Windows Subsystem for Linux (WSL).  The adjust-path macro converts absolute windows 
# paths into unix style paths (Example: c:/dir -> /c/dir). 
# The adjust_path_mixed function converts WSL path to Windows path.
# This will ensure paths are readable by GNU Make.
#------------------------------------------------------------------------------

ifeq ($(OS), Windows_NT)
	ifneq ($(strip $(USE_LEGAY)),'y')
		WSL = wsl
		adjust_path = $(shell wsl wslpath $1)
		adjust_path_mixed = $(if $(call eq,$(shell echo $1 | wsl head -c 5),/mnt/),$(shell echo $1 | wsl sed 's/\/mnt\///g;s/\//:\//1'),$1)
	else
		adjust_path = $1
		adjust_path_mixed = $1
	endif
else # !Windows_NT
	adjust_path = $1
	adjust_path_mixed = $1
endif

PLATFORM_RELATIVE_PATH = $1
PLATFORM_FULL_PATH = $1

OBJECTS_DIR	= $(BUILD_DIR)/obj
TEMP_DIR	= $(BUILD_DIR)/tmp
ELF		= $(BUILD_DIR)/$(PROJECT_NAME).elf
BINARY          = $(BUILD_DIR)/$(PROJECT_NAME).bin

# Define the platform compiler switch
CFLAGS += -D ALTERA_PLATFORM

#LSCRIPT = $(SOCEDS_DEST_ROOT)/host_tools/mentor/gnu/arm/baremetal/arm-altera-eabi/lib/cortex-a9/cycloneV-dk-ram.ld
LSCRIPT = $(NO-OS)/tools/scripts/platform/altera/cycloneV-dk-ram-modified.ld
#LIB_PATHS += -L$(BUILD_DIR)/bsp

CROSS_COMPILE=arm-altera-eabi-
CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)g++
NM := $(CROSS_COMPILE)nm
OC := $(CROSS_COMPILE)objcopy
OD := $(CROSS_COMPILE)objdump


CFLAGS += -xc								\
	  -pipe								\
	  -O3								\
	  -g								\
          -mcpu=cortex-a9                                               \
          -mfloat-abi=softfp                                            \
          -mfpu=neon


LDFLAGS += -lm

