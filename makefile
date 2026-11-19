# VEXcode makefile 2019_03_26_01

# Guard: if this file's contents ever get pasted in twice, the second copy is skipped
# instead of loading the build rules again.
ifndef MAKEFILE_LOADED
MAKEFILE_LOADED = 1

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# warn if this code is behind GitHub (see custom/update-check.mk)
-include custom/update-check.mk

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*.c)
SRC_C += $(wildcard custom/src/*.cpp)
SRC_C += $(wildcard custom/src/*.c)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include

# build targets
all: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk

endif
