/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */
 
#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


// We define these calls here, so the user doesn't need to include __FILE__ and __LINE__
// The advantage is the developers gets to use the inline function so they can debug
#define cutilDrvSafeCallNoSync(err)     __cuSafeCallNoSync  (err, __FILE__, __LINE__)
#define cutilDrvSafeCall(err)           __cuSafeCall        (err, __FILE__, __LINE__)
#define cutilDrvCtxSync()               __cuCtxSync         (__FILE__, __LINE__)
#define cutilDrvCheckMsg(msg)           __cuCheckMsg        (msg, __FILE__, __LINE__)
#define cutilDrvAlignOffset(offset, alignment)  ( offset = (offset + (alignment-1)) & ~((alignment-1)) )

// These are the inline versions for all of the CUTIL functions
inline void __cuSafeCallNoSync( CUresult err, const char *file, const int line )
{
    if( CUDA_SUCCESS != err) {
        fprintf(stderr, "cuSafeCallNoSync() Driver API error = %04d from file <%s>, line %i.\n",
                err, file, line );
        exit(-1);
    }
}
inline void __cuSafeCall( CUresult err, const char *file, const int line )
{
    __cuSafeCallNoSync( err, file, line );
}

inline void __cuCtxSync(const char *file, const int line )
{
    CUresult err = cuCtxSynchronize();
    if( CUDA_SUCCESS != err ) {
        fprintf(stderr, "cuCtxSynchronize() API error = %04d in file <%s>, line %i.\n",
                err, file, line );
        exit(-1);
    }
}

#define MIN(a,b) ((a < b) ? a : b)
#define MAX(a,b) ((a > b) ? a : b)

// Beginning of GPU Architecture definitions
inline int _ConvertSMVer2CoresDrvApi(int major, int minor)
{
	// Defines for GPU Architecture types (using the SM version to determine the # of cores per SM
	struct sSMtoCores{
		int SM; // 0xMm (hexadecimal notation), M = SM Major version, and m = SM minor version
		int Cores;
	};

        sSMtoCores nGpuArchCoresPerSM[] =
        { { 0x10,  8 },
          { 0x11,  8 },
          { 0x12,  8 },
          { 0x13,  8 },
          { 0x20, 32 },
          { 0x21, 48 },
          {   -1, -1 }
        };

	int index = 0;
	while (nGpuArchCoresPerSM[index].SM != -1) {
		if (nGpuArchCoresPerSM[index].SM == ((major << 4) + minor) ) {
			return nGpuArchCoresPerSM[index].Cores;
		}
		index++;
	}
	printf("MapSMtoCores undefined SMversion %d.%d!\n", major, minor);
	return -1;
}
// end of GPU Architecture definitions

// This function returns the best GPU based on performance
inline int cutilDrvGetMaxGflopsDeviceId()
{
    CUdevice current_device = 0;
    CUdevice max_perf_device = 0;
    int device_count     = 0;
    int max_compute_perf = 0;
    int best_SM_arch     = 0;

    cuInit(0);
    cutilDrvSafeCallNoSync(cuDeviceGetCount(&device_count));

	// Find the best major SM Architecture GPU device
	while ( current_device < device_count ) {
		int major = 0;
	        int minor = 0;
		cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, current_device));
		cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, current_device));
		if (major > 0 && major < 9999) {
			best_SM_arch = MAX(best_SM_arch, major);
		}
		current_device++;
	}

    	// Find the best CUDA capable GPU device
	current_device = 0;
	while( current_device < device_count ) {
                int multiProcessorCount;
                int clockRate;
                int major = 0;
                int minor = 0;              
		cutilDrvSafeCallNoSync( cuDeviceGetAttribute( &multiProcessorCount, 
                                                            CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, 
                                                            current_device ) );
        	cutilDrvSafeCallNoSync( cuDeviceGetAttribute( &clockRate, 
                                                            CU_DEVICE_ATTRIBUTE_CLOCK_RATE, 
                                                            current_device ) );
        	cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, current_device));
        	cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, current_device));

        	int sm_per_multiproc = (major == 9999 && minor == 9999) ? 1 : _ConvertSMVer2CoresDrvApi(major, minor);

		int compute_perf  = multiProcessorCount * sm_per_multiproc * clockRate;
		if( compute_perf  > max_compute_perf ) {
            		// If we find GPU with SM major > 2, search only these
			if ( best_SM_arch > 2 ) {
				// If our device==dest_SM_arch, choose this, or else pass
				if (major == best_SM_arch) {	
                    			max_compute_perf  = compute_perf;
                    			max_perf_device   = current_device;
				}
			} 
			else {
				max_compute_perf  = compute_perf;
				max_perf_device   = current_device;
			}
		}
		++current_device;
	}
	return max_perf_device;
}

// This function returns the best Graphics GPU based on performance
inline int cutilDrvGetMaxGflopsGraphicsDeviceId()
{
    CUdevice current_device = 0;
    CUdevice max_perf_device = 0;
    int device_count     = 0;
    int max_compute_perf = 0;
    int best_SM_arch     = 0;

    cuInit(0);
    cutilDrvSafeCallNoSync(cuDeviceGetCount(&device_count));

	// Find the best major SM Architecture GPU device that are graphics devices
	while ( current_device < device_count ) {
              	char deviceName[256];
              	int major = 0;
              	int minor = 0;
              	int bTCC = 0;
		cutilDrvSafeCallNoSync( cuDeviceGetName(deviceName, 256, current_device) );
		cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, current_device));
		cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, current_device));
		cutilDrvSafeCallNoSync( cuDeviceGetAttribute(&bTCC, CU_DEVICE_ATTRIBUTE_TCC_DRIVER, current_device) );

		if (!bTCC) {
			if (major > 0 && major < 9999) {
				best_SM_arch = MAX(best_SM_arch, major);
			}
		}
		current_device++;
	}

    // Find the best CUDA capable GPU device
	current_device = 0;
	while( current_device < device_count ) {
              	int multiProcessorCount;
              	int clockRate;
              	int major = 0;
              	int minor = 0;
              	int bTCC = 0;
		cutilDrvSafeCallNoSync( cuDeviceGetAttribute( &multiProcessorCount, 
                                                            CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, 
                                                            current_device ) );
        	cutilDrvSafeCallNoSync( cuDeviceGetAttribute( &clockRate, 
                                                            CU_DEVICE_ATTRIBUTE_CLOCK_RATE, 
                                                            current_device ) );
        	cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, current_device));
        	cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, current_device));

		cutilDrvSafeCallNoSync( cuDeviceGetAttribute( &bTCC,  CU_DEVICE_ATTRIBUTE_TCC_DRIVER, current_device ) );

              	int sm_per_multiproc = (major == 9999 && minor == 9999) ? 1 : _ConvertSMVer2CoresDrvApi(major, minor);

		// If this is a Tesla based GPU and SM 2.0, and TCC is disabled, this is a contender
		if (!bTCC) // Is this GPU running the TCC driver?  If so we pass on this
		{
			int compute_perf  = multiProcessorCount * sm_per_multiproc * clockRate;
			if( compute_perf  > max_compute_perf ) {
				// If we find GPU with SM major > 2, search only these
				if ( best_SM_arch > 2 ) {
					// If our device = dest_SM_arch, then we pick this one
					if (major == best_SM_arch) {	
                        			max_compute_perf  = compute_perf;
                        			max_perf_device   = current_device;
					}
				} 
				else {
					max_compute_perf  = compute_perf;
					max_perf_device   = current_device;
				}
			}
		}
		++current_device;
	}
	return max_perf_device;
}

inline void __cuCheckMsg( const char * msg, const char *file, const int line )
{
    CUresult err = cuCtxSynchronize();
    if( CUDA_SUCCESS != err) {
	fprintf(stderr, "cutilDrvCheckMsg -> %s", msg);
        fprintf(stderr, "cutilDrvCheckMsg -> cuCtxSynchronize API error = %04d in file <%s>, line %i.\n",
                err, file, line );
        exit(-1);
    }
}


#if __DEVICE_EMULATION__
    inline int cutilDeviceInitDrv(int ARGC, char **ARGV) { } 
#else
    inline int cutilDeviceInitDrv(int ARGC, char ** ARGV) 
    {
        int cuDevice = 0;
        int deviceCount = 0;
        CUresult err = cuInit(0);
        if (CUDA_SUCCESS == err)
            cutilDrvSafeCallNoSync(cuDeviceGetCount(&deviceCount));
        if (deviceCount == 0) {
            fprintf(stderr, "CUTIL DeviceInitDrv error: no devices supporting CUDA\n");
            exit(-1);
        }
        int dev = 0;
        cutGetCmdLineArgumenti(ARGC, (const char **) ARGV, "device", &dev);
        if (dev < 0) dev = 0;
        if (dev > deviceCount-1) {
		fprintf(stderr, "\n");
		fprintf(stderr, ">> %d CUDA capable GPU device(s) detected. <<\n", deviceCount);
            	fprintf(stderr, ">> cutilDeviceInit (-device=%d) is not a valid GPU device. <<\n", dev);
		fprintf(stderr, "\n");
            	return -dev;
        }
        cutilDrvSafeCallNoSync(cuDeviceGet(&cuDevice, dev));
        char name[100];
        cuDeviceGetName(name, 100, cuDevice);
        if (cutCheckCmdLineFlag(ARGC, (const char **) ARGV, "quiet") == CUTFalse) {
           printf("> Using CUDA Device [%d]: %s\n", dev, name);
       	}
        return dev;
    }
#endif

    // General initialization call to pick the best CUDA Device
#if __DEVICE_EMULATION__
    inline CUdevice cutilChooseCudaDeviceDrv(int argc, char **argv, int *p_devID)
#else
    inline CUdevice cutilChooseCudaDeviceDrv(int argc, char **argv, int *p_devID)
    {
        CUdevice cuDevice;
        int devID = 0;
        // If the command-line has a device number specified, use it
        if( cutCheckCmdLineFlag(argc, (const char**)argv, "device") ) {
            devID = cutilDeviceInitDrv(argc, argv);
            if (devID < 0) {
                printf("exiting...\n");
                exit(0);
            }
        } else {
            // Otherwise pick the device with highest Gflops/s
            char name[100];
            devID = cutilDrvGetMaxGflopsDeviceId();
            cutilDrvSafeCallNoSync(cuDeviceGet(&cuDevice, devID));
            cuDeviceGetName(name, 100, cuDevice);
            printf("> Using CUDA Device [%d]: %s\n", devID, name);
        }
        cuDeviceGet(&cuDevice, devID);
        if (p_devID) *p_devID = devID;
        return cuDevice;
    }
#endif


//! Check for CUDA context lost
inline void cutilDrvCudaCheckCtxLost(const char *errorMessage, const char *file, const int line ) 
{
    CUresult err = cuCtxSynchronize();
    if( CUDA_ERROR_INVALID_CONTEXT != err) {
        fprintf(stderr, "Cuda error: %s in file '%s' in line %i\n",
        errorMessage, file, line );
        exit(-1);
    }
    err = cuCtxSynchronize();
    if( CUDA_SUCCESS != err) {
        fprintf(stderr, "Cuda error: %s in file '%s' in line %i\n",
        errorMessage, file, line );
        exit(-1);
    } 
}

#ifndef STRCASECMP
#ifdef _WIN32
#define STRCASECMP  _stricmp
#else
#define STRCASECMP  strcasecmp
#endif
#endif

#ifndef STRNCASECMP
#ifdef _WIN32
#define STRNCASECMP _strnicmp
#else
#define STRNCASECMP strncasecmp
#endif
#endif

inline void __cutilDrvQAFinish(int argc, char **argv, bool bStatus)
{
    const char *sStatus[] = { "FAILED", "PASSED", "WAIVED", NULL };

    bool bFlag = false;
    for (int i=1; i < argc; i++) {
        if (!STRCASECMP(argv[i], "-qatest") || !STRCASECMP(argv[i], "-noprompt")) {
            bFlag |= true;
        }
    }

    if (bFlag) {
        printf("&&&& %s %s", sStatus[bStatus], argv[0]);
        for (int i=1; i < argc; i++) printf(" %s", argv[i]);
    } else {
        printf("[%s] test result\n%s\n", argv[0], sStatus[bStatus]);
    }
}

// General check for CUDA GPU SM Capabilities for a specific device #
inline bool cutilDrvCudaDevCapabilities(int major_version, int minor_version, int deviceNum, int argc, char** argv)
{
    int major, minor, dev;
    char device_name[256];

#ifdef __DEVICE_EMULATION__
    printf("> Compute Device Emulation Mode \n");
#endif

    cutilDrvSafeCallNoSync( cuDeviceGet(&dev, deviceNum) );
    cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, dev));
    cutilDrvSafeCallNoSync (cuDeviceGetAttribute (&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, dev));
    cutilDrvSafeCallNoSync( cuDeviceGetName(device_name, 256, dev) ); 

    if((major > major_version) ||
	   (major == major_version && minor >= minor_version))
    {
        printf("> Device %d: < %s >, Compute SM %d.%d detected\n", dev, device_name, major, minor);
        return true;
    }
    else
    {
        printf("There is no device supporting CUDA compute capability %d.%d.\n", major_version, minor_version);
        __cutilDrvQAFinish(argc, argv, true);
        return false;
    }
}

// General check for CUDA GPU SM Capabilities
inline bool cutilDrvCudaCapabilities(int major_version, int minor_version, int argc, char **argv)
{
	return cutilDrvCudaDevCapabilities(major_version, minor_version, 0, argc, argv);
}
