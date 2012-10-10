LOCAL_PATH:= $(call my-dir)

ne10_neon_source := \
    Ne10/source/NE10_abs.neon.s \
    Ne10/source/NE10_addc.neon.c \
    Ne10/source/NE10_addmat.neon.c \
    Ne10/source/NE10_add.neon.s \
    Ne10/source/NE10_cross.neon.s \
    Ne10/source/NE10_detmat.neon.s \
    Ne10/source/NE10_divc.neon.c \
    Ne10/source/NE10_div.neon.s \
    Ne10/source/NE10_dot.neon.s \
    Ne10/source/NE10_identitymat.neon.s \
    Ne10/source/NE10_invmat.neon.s \
    Ne10/source/NE10_len.neon.s \
    Ne10/source/NE10_mla.neon.s \
    Ne10/source/NE10_mlac.neon.c \
    Ne10/source/NE10_mulcmatvec.neon.s \
    Ne10/source/NE10_mulc.neon.c \
    Ne10/source/NE10_mulmat.neon.s \
    Ne10/source/NE10_mul.neon.c \
    Ne10/source/NE10_normalize.neon.s \
    Ne10/source/NE10_rsbc.neon.c \
    Ne10/source/NE10_setc.neon.c \
    Ne10/source/NE10_subc.neon.c \
    Ne10/source/NE10_submat.neon.c \
    Ne10/source/NE10_sub.neon.s \
    Ne10/source/NE10_transmat.neon.s \

ne10_source_files := \
    Ne10/source/NE10_abs.asm.s \
    Ne10/source/NE10_addc.asm.s \
    Ne10/source/NE10_addmat.asm.s \
    Ne10/source/NE10_add.asm.s \
    Ne10/source/NE10_cross.asm.s \
    Ne10/source/NE10_detmat.asm.s \
    Ne10/source/NE10_divc.asm.s \
    Ne10/source/NE10_div.asm.s \
    Ne10/source/NE10_dot.asm.s \
    Ne10/source/NE10_identitymat.asm.s \
    Ne10/source/NE10_invmat.asm.s \
    Ne10/source/NE10_len.asm.s \
    Ne10/source/NE10_mla.asm.s \
    Ne10/source/NE10_mlac.asm.s \
    Ne10/source/NE10_mulcmatvec.asm.s \
    Ne10/source/NE10_mulc.asm.s \
    Ne10/source/NE10_mulmat.asm.s \
    Ne10/source/NE10_mul.asm.s \
    Ne10/source/NE10_normalize.asm.s \
    Ne10/source/NE10_rsbc.asm.s \
    Ne10/source/NE10_setc.asm.s \
    Ne10/source/NE10_subc.asm.s \
    Ne10/source/NE10_submat.asm.s \
    Ne10/source/NE10_sub.asm.s \
    Ne10/source/NE10_transmat.asm.s \
    Ne10/source/NE10_abs.c \
    Ne10/source/NE10_addc.c \
    Ne10/source/NE10_addmat.c \
    Ne10/source/NE10_add.c \
    Ne10/source/NE10_cross.c \
    Ne10/source/NE10_detmat.c \
    Ne10/source/NE10_divc.c \
    Ne10/source/NE10_div.c \
    Ne10/source/NE10_dot.c \
    Ne10/source/NE10_identitymat.c \
    Ne10/source/NE10_invmat.c \
    Ne10/source/NE10_len.c \
    Ne10/source/NE10_mla.c \
    Ne10/source/NE10_mlac.c \
    Ne10/source/NE10_mulcmatvec.c \
    Ne10/source/NE10_mulc.c \
    Ne10/source/NE10_mulmat.c \
    Ne10/source/NE10_mul.c \
    Ne10/source/NE10_normalize.c \
    Ne10/source/NE10_rsbc.c \
    Ne10/source/NE10_setc.c \
    Ne10/source/NE10_subc.c \
    Ne10/source/NE10_submat.c \
    Ne10/source/NE10_sub.c \
    Ne10/source/NE10_transmat.c \
    
LOCAL_C_INCLUDES :=     $(LOCAL_PATH)/Ne10/headers/ \
                        $(LOCAL_PATH)/Ne10/inc 

LOCAL_SRC_FILES :=  \
    $(ne10_source_files)

ifeq ($(ARCH_ARM_HAVE_NEON),true)
LOCAL_SRC_FILES += $(ne10_neon_source)
endif

LOCAL_CFLAGS := -D_ARM_ASSEM_

LOCAL_ARM_MODE := arm

LOCAL_MODULE_TAGS := eng
LOCAL_MODULE := libne10

include $(BUILD_STATIC_LIBRARY)
#include $(PREBUILT_SHARED_LIBRARY)