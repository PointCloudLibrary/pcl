#----------------------------------------------------------------------------------
# File:            apps\multitouch\jni\Android.mk
# Samples Version: Android NVIDIA samples 2 
# Email:           tegradev@nvidia.com
# Forum:           http://developer.nvidia.com/tegra/forums/tegra-forums/android-development
    
#PCL library linkers
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

#PCL libraries - full support
PCL_STATIC_LIB_DIR := ../../pcl-superbuild/build/CMakeExternals/Install/pcl-android/lib
BOOST_STATIC_LIB_DIR := ../../pcl-superbuild/build/CMakeExternals/Install/boost-android/lib
FLANN_STATIC_LIB_DIR := ../../pcl-superbuild/build/CMakeExternals/Install/flann-android/lib
					
PCL_STATIC_LIBRARIES := pcl_common pcl_geometry pcl_kdtree pcl_octree pcl_sample_consensus \
							pcl_surface pcl_features pcl_io pcl_keypoints pcl_recognition \
							pcl_search pcl_tracking pcl_filters pcl_io_ply pcl_ml \
							pcl_registration pcl_segmentation 
BOOST_STATIC_LIBRARIES := boost_date_time boost_iostreams boost_regex boost_system \
							boost_filesystem boost_program_options boost_signals \
							boost_thread
FLANN_STATIC_LIBRARIES := flann_cpp_s flann_s

										
define build_pcl_static
	include $(CLEAR_VARS)
	LOCAL_MODULE:=$1
	LOCAL_SRC_FILES:=$(PCL_STATIC_LIB_DIR)/lib$1.a
	include $(PREBUILT_STATIC_LIBRARY)
endef

define build_boost_static
	include $(CLEAR_VARS)
	LOCAL_MODULE:=$1
	LOCAL_SRC_FILES:=$(BOOST_STATIC_LIB_DIR)/lib$1.a
	include $(PREBUILT_STATIC_LIBRARY)
endef

define build_flann_static
	include $(CLEAR_VARS)
	LOCAL_MODULE:=$1
	LOCAL_SRC_FILES:=$(FLANN_STATIC_LIB_DIR)/lib$1.a
	include $(PREBUILT_STATIC_LIBRARY)
endef

$(foreach module,$(PCL_STATIC_LIBRARIES),$(eval $(call build_pcl_static,$(module))))
$(foreach module,$(BOOST_STATIC_LIBRARIES),$(eval $(call build_boost_static,$(module))))
$(foreach module,$(FLANN_STATIC_LIBRARIES),$(eval $(call build_flann_static,$(module))))					

#opencv libraries - static links
include $(CLEAR_VARS)
OPENCV_LIB_TYPE := STATIC
include ../../OpenCV-2.4.2-android-sdk/sdk/native/jni/OpenCV.mk			


#define USE_KINECT_DRIVER 0

LOCAL_MODULE := pcl_openni_recorder 

#libusb files
LOCAL_SRC_FILES := libusb/core_usb.c libusb/io.c libusb/descriptor.c libusb/linux_usbfs.c 

#libfreenect files
LOCAL_SRC_FILES += libfreenect/cameras.c libfreenect/core.c libfreenect/loader.c \
				libfreenect/sync.c libfreenect/tilt.c libfreenect/usb_libusb10.c

#quicklz
LOCAL_SRC_FILES += quicklz/quicklz.c

#threadpool
LOCAL_SRC_FILES += threadpool/threadpool.c


#pivate api		
#LOCAL_SRC_FILES += gpgpu/GpuImageProcessor.cpp gpgpu/GpuImageProcessorBuffer.cpp
	
#the rest of the directory
LOCAL_SRC_FILES += $(wildcard *.cpp)
LOCAL_SRC_FILES += $(wildcard *.c)

#not sure which one is better yet? thumb is 0.01ms faster right now.
LOCAL_LDFLAGS += \
	-L../../../libs/obj/local/$(TARGET_ARCH_ABI) \
	-Lobj/local/$(TARGET_ARCH_ABI)/ \
	-L../../../build/platforms/tegra/lib 
	
LOCAL_C_INCLUDES += ../../../libs/jni/ ../../../build/platforms/tegra/include .. \
				 libusb libfreenect quicklz threadpool
				
#png library 
LOCAL_C_INCLUDES += ../../lpng1512
 
 
#pcl library
LOCAL_LDFLAGS += -L../../pcl-superbuild/build/CMakeExternals/Install/pcl-android/lib \
				 -L../../pcl-superbuild/build/CMakeExternals/Install/boost-android/lib \
				 -L../../pcl-superbuild/build/CMakeExternals/Install/flann-android/lib
				  				
LOCAL_C_INCLUDES += ../../pcl-superbuild/build/CMakeExternals/Install/pcl-android/include/pcl-1.6 \
					../../pcl-superbuild/build/CMakeExternals/Install/boost-android/include \
					../../pcl-superbuild/build/CMakeExternals/Install/eigen \
					../../pcl-superbuild/build/CMakeExternals/Install/flann-android/include  		
			
LOCAL_STATIC_LIBRARIES   += pcl_common pcl_geometry pcl_kdtree pcl_octree pcl_sample_consensus \
							pcl_surface pcl_features pcl_io pcl_keypoints pcl_recognition \
							pcl_search pcl_tracking pcl_filters pcl_io_ply pcl_ml \
							pcl_registration pcl_segmentation 
							
LOCAL_STATIC_LIBRARIES   += boost_date_time boost_iostreams boost_regex boost_system \
							boost_filesystem boost_program_options boost_signals \
							boost_thread
								
#ifeq (USE_KINECT_DRIVER,1)
LOCAL_LDFLAGS += -L../../openni_drivers/openni_kinect/Platform/Android/libs/$(TARGET_ARCH_ABI) \
	-L../../openni_drivers/kinect_engine/Platform/Android/libs/$(TARGET_ARCH_ABI) 
LOCAL_C_INCLUDES += ../../openni_drivers/openni_kinect/Include \
				 ../../openni_drivers/kinect_engine/Include
#else
LOCAL_LDFLAGS += -L../../openni_drivers/openni/build/openni/Platform/Android/libs/$(TARGET_ARCH_ABI) \
	-L../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/libs/$(TARGET_ARCH_ABI) 
LOCAL_C_INCLUDES += ../../openni_drivers/openni/build/openni/Include \
				 ../../openni_drivers/ps_engine/build/ps_engine/Include
#endif			 

#OPENNI sensors and drivers
LOCAL_LDLIBS += -lstdc++ -lc -lm -llog -landroid -ldl -lGLESv2 -lEGL -lOpenNI

LOCAL_C_INCLUDES += pivate/include

LOCAL_STATIC_LIBRARIES += nv_and_util nv_egl_util nv_bitfont nv_math nv_glesutil nv_hhdds nv_log nv_shader nv_file nv_thread

#for the Ne10
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
    Ne10/source/NE10_mul.neon.s \
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
    Ne10/NE10_init.c \
    Ne10/NE10_test.c
    
LOCAL_CFLAGS := -D_ARM_ASSEM_
LOCAL_ARM_MODE := thumb
LOCAL_C_INCLUDES +=     $(LOCAL_PATH)/Ne10/headers/ \
                        $(LOCAL_PATH)/Ne10/inc \
                        $(LOCAL_PATH)/Ne10
#require NEON support!                        
LOCAL_SRC_FILES += $(ne10_source_files) $(ne10_neon_source)
#If you don't want the compiler to automatically make you code faster and !#parallel remove "-O2 -ftree-vectorize"
LOCAL_CFLAGS += -mfloat-abi=softfp -mfpu=neon -march=armv7 -mthumb -O3

include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, ../../../libs/jni)
$(call import-add-path, ../../libs/jni)
$(call import-module,nv_and_util)
$(call import-module,nv_egl_util)
$(call import-module,nv_bitfont)
$(call import-module,nv_math)
$(call import-module,nv_glesutil)
$(call import-module,nv_hhdds)
$(call import-module,nv_log)
$(call import-module,nv_shader)
$(call import-module,nv_file)
$(call import-module,nv_thread)

