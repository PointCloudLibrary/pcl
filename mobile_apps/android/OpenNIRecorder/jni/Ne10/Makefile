#
#  Copyright 2011-12 ARM Limited
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

C_TOOL    = gcc
EXE_TOOL    = gcc
ASM_TOOL    = as

#BJ_FLAGS   = -mthumb-interwork -march=armv7-a -mcpu=cortex-a9 -mfpu=vfp3
ARM_FLAGS   = -mthumb-interwork -march=armv7-a -mcpu=cortex-a9
C_FLAGS     = -lm -lrt -I./inc/
#DEBUG_FLAGS = -gstabs
OPTIMIZE_FLAGS = -O3
# -save-temps -O3

LDFLAGS+=-L.  -L/usr/local/lib -L/client/lib -L/lib/arm-linux-gnueabi
LDFLAGS+=-lm

ALLFILES = \
    NE10_addc.c_r.o        NE10_subc.c_r.o       NE10_rsbc.c_r.o      \
    NE10_mulc.c_r.o        NE10_divc.c_r.o       NE10_mlac.c_r.o      \
    NE10_setc.c_r.o        NE10_add.c_r.o        NE10_sub.c_r.o       \
    NE10_mul.c_r.o         NE10_div.c_r.o        NE10_mla.c_r.o       \
    NE10_abs.c_r.o         NE10_len.c_r.o        NE10_normalize.c_r.o \
    NE10_addc.neon_r.o     NE10_subc.neon_r.o    NE10_rsbc.neon_r.o   \
    NE10_mulc.neon_r.o     NE10_divc.neon_r.o    NE10_mlac.neon_r.o   \
    NE10_setc.neon_r.o     NE10_add.neon_r.o     NE10_sub.neon_r.o    \
    NE10_mul.neon_r.o      NE10_div.neon_r.o     NE10_mla.neon_r.o    \
    NE10_abs.neon_r.o      NE10_len.neon_r.o     NE10_normalize.neon_r.o \
    NE10_dot.c_r.o         NE10_dot.neon_r.o     NE10_cross.c_r.o     \
    NE10_cross.neon_r.o    NE10_addmat.c_r.o     NE10_addmat.neon_r.o \
    NE10_submat.c_r.o      NE10_submat.neon_r.o  NE10_mulmat.c_r.o    \
    NE10_mulmat.neon_r.o   NE10_mulcmatvec.c_r.o NE10_mulcmatvec.neon_r.o \
    NE10_detmat.c_r.o      NE10_detmat.neon_r.o  NE10_invmat.c_r.o    \
    NE10_invmat.neon_r.o   NE10_transmat.c_r.o   NE10_transmat.neon_r.o \
    NE10_identitymat.c_r.o NE10_identitymat.neon_r.o

#TARGET_ARCH = stdc

.PHONY: all clean

all: NE10_test_static.ex NE10_test_dynamic.ex

clean:
	./cleanall.sh

NE10_test_static.ex : libNE10.a NE10_init.h NE10_test.c
		$(EXE_TOOL) $(OPTIMIZE_FLAGS) $(ARM_FLAGS) ./NE10_init.c ./NE10_test.c -o $@ -l:libNE10.a $(C_FLAGS) -L/lib/arm-linux-gnueabi

NE10_test_dynamic.ex : libNE10.so NE10_init.h NE10_test.c
		$(EXE_TOOL) $(OPTIMIZE_FLAGS) $(ARM_FLAGS) ./NE10_init.c ./NE10_test.c -o $@ -l:libNE10.so $(C_FLAGS) -L/lib/arm-linux-gnueabi

libNE10.a : $(ALLFILES) NE10_init.h NE10_init.c
		ar rcs libNE10.a $(ALLFILES)

libNE10.so : $(ALLFILES) NE10_init.h NE10_init.c
		gcc -shared -o $@ $(C_FLAGS) $(ALLFILES) 

%mat.test_r.ex : %.asm_r.o %.c_r.o %.neon_r.o %mat.c_r.o %mat.neon_r.o ./source/%mat_test.c  ./inc/NE10.h
		$(EXE_TOOL) $(OPTIMIZE_FLAGS) $(ARM_FLAGS) $^ -o $@ $(C_FLAGS) -L/lib/arm-linux-gnueabi

%.test_r.ex : %.asm_r.o %.c_r.o %.neon_r.o ./source/%_test.c  ./inc/NE10.h
		$(EXE_TOOL) $(OPTIMIZE_FLAGS) $(ARM_FLAGS) $^ -o $@ $(C_FLAGS) -L/lib/arm-linux-gnueabi
 
%.c_r.o : ./source/%.c ./inc/NE10.h
		$(C_TOOL) $(OPTIMIZE_FLAGS) $(ARM_FLAGS) -mfpu=vfp3 -c $< -o $@ $(C_FLAGS) -L/lib/arm-linux-gnueabi 

%.asm_r.o : ./source/%.asm.s
		$(ASM_TOOL) $(ARM_FLAGS) -mfpu=vfp3 $< -o $@

# Either use the C version or use the Assembly version for compiling the NEON routines

# Rules for the Assembly version
%.neon_r.o : ./source/%.neon.s
		$(ASM_TOOL) $(ARM_FLAGS) -mfpu=neon $< -o $@

# Rules for the C version
%.neon_r.o : ./source/%.neon.c ./inc/NE10.h
		$(C_TOOL) $(OPTIMIZE_FLAGS) $(ARM_FLAGS) -mfpu=neon -c $< -o $@ $(C_FLAGS)
