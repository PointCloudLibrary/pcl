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

#ifdef _WIN32
#ifdef _DEBUG // Do this only in debug mode...
#  define WINDOWS_LEAN_AND_MEAN
#  include <windows.h>
#  include <stdlib.h>
#  undef min
#  undef max
#endif
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <cufft.h>

// We define these calls here, so the user doesn't need to include __FILE__ and __LINE__
// The advantage is the developers gets to use the inline function so they can debug
#define cutilSafeCallNoSync(err)     __cudaSafeCallNoSync(err, __FILE__, __LINE__)
#define cutilSafeCall(err)           __cudaSafeCall      (err, __FILE__, __LINE__)
#define cutilSafeThreadSync()        __cudaSafeThreadSync(__FILE__, __LINE__)
#define cufftSafeCall(err)           __cufftSafeCall     (err, __FILE__, __LINE__)
#define cutilCheckError(err)         __cutilCheckError   (err, __FILE__, __LINE__)
#define cutilCheckMsg(msg)           __cutilGetLastError (msg, __FILE__, __LINE__)
#define cutilCheckMsgAndSync(msg)    __cutilGetLastErrorAndSync (msg, __FILE__, __LINE__)
#define cutilSafeMalloc(mallocCall)  __cutilSafeMalloc   ((mallocCall), __FILE__, __LINE__)
#define cutilCondition(val)          __cutilCondition    (val, __FILE__, __LINE__)
#define cutilExit(argc, argv)        __cutilExit         (argc, argv)

inline cudaError cutilDeviceSynchronize()
{
	return cudaDeviceSynchronize();
}

inline cudaError cutilDeviceReset()
{
	return cudaDeviceReset();
}

inline void __cutilCondition(int val, char *file, int line) 
{
    if( CUTFalse == cutCheckCondition( val, file, line ) ) {
        exit(EXIT_FAILURE);
    }
}

inline void __cutilExit(int argc, char **argv)
{     
    if (!cutCheckCmdLineFlag(argc, (const char**)argv, "noprompt")) {
        printf("\nPress ENTER to exit...\n");
        fflush( stdout);
        fflush( stderr);
        getchar();
    }
    exit(EXIT_SUCCESS);
}

#define MIN(a,b) ((a < b) ? a : b)
#define MAX(a,b) ((a > b) ? a : b)

// Beginning of GPU Architecture definitions
inline int _ConvertSMVer2Cores(int major, int minor)
{
	// Defines for GPU Architecture types (using the SM version to determine the # of cores per SM
	struct sSMtoCores {
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

// This function returns the best GPU (with maximum GFLOPS)
inline int cutGetMaxGflopsDeviceId()
{
    int current_device   = 0;
    int max_compute_perf = 0;
    int max_perf_device  = 0;
    int device_count     = 0;
    int best_SM_arch     = 0;

	cudaGetDeviceCount( &device_count );
	// Find the best major SM Architecture GPU device
	while ( current_device < device_count ) {
              cudaDeviceProp deviceProp;
		cudaGetDeviceProperties( &deviceProp, current_device );
		if (deviceProp.major > 0 && deviceProp.major < 9999) {
			best_SM_arch = MAX(best_SM_arch, deviceProp.major);
		}
		current_device++;
	}

    // Find the best CUDA capable GPU device
	current_device = 0;
	while( current_device < device_count ) {
              cudaDeviceProp deviceProp;
		cudaGetDeviceProperties( &deviceProp, current_device );
              int sm_per_multiproc = (deviceProp.major == 9999 && deviceProp.minor == 9999) ? 1 : _ConvertSMVer2Cores(deviceProp.major, deviceProp.minor);

		int compute_perf  = deviceProp.multiProcessorCount * sm_per_multiproc * deviceProp.clockRate;
		if( compute_perf  > max_compute_perf ) {
            // If we find GPU with SM major > 2, search only these
			if ( best_SM_arch > 2 ) {
				// If our device==dest_SM_arch, choose this, or else pass
				if (deviceProp.major == best_SM_arch) {	
					max_compute_perf  = compute_perf;
					max_perf_device   = current_device;
				}
			} else {
				max_compute_perf  = compute_perf;
				max_perf_device   = current_device;
			}
		}
		++current_device;
	}
	return max_perf_device;
}

// This function returns the best GPU (with maximum GFLOPS)
inline int cutGetMaxGflopsGraphicsDeviceId()
{
    int current_device   = 0;
    int max_compute_perf = 0;
    int max_perf_device  = 0;
    int device_count     = 0;
    int best_SM_arch     = 0;
    int bTCC = 0;

	cudaGetDeviceCount( &device_count );
	// Find the best major SM Architecture GPU device that is graphics capable
	while ( current_device < device_count ) {
              	cudaDeviceProp deviceProp;
		cudaGetDeviceProperties( &deviceProp, current_device );

		if (deviceProp.tccDriver) bTCC = 1;

		if (!bTCC) {
			if (deviceProp.major > 0 && deviceProp.major < 9999) {
				best_SM_arch = MAX(best_SM_arch, deviceProp.major);
			}
		}
		current_device++;
	}

    // Find the best CUDA capable GPU device
	current_device = 0;
	while( current_device < device_count ) {
              	cudaDeviceProp deviceProp;
		cudaGetDeviceProperties( &deviceProp, current_device );
              	int sm_per_multiproc = (deviceProp.major == 9999 && deviceProp.minor == 9999) ? 1 : _ConvertSMVer2Cores(deviceProp.major, deviceProp.minor);

		if (deviceProp.tccDriver) bTCC = 1;

		if (!bTCC) // Is this GPU running the TCC driver?  If so we pass on this
		{
			int compute_perf  = deviceProp.multiProcessorCount * sm_per_multiproc * deviceProp.clockRate;
			if( compute_perf  > max_compute_perf ) {
				// If we find GPU with SM major > 2, search only these
				if ( best_SM_arch > 2 ) {
					// If our device==dest_SM_arch, choose this, or else pass
					if (deviceProp.major == best_SM_arch) {	
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

// Give a little more for Windows : the console window often disappears before we can read the message
#ifdef _WIN32
# if 1//ndef UNICODE
#  ifdef _DEBUG // Do this only in debug mode...
	inline void VSPrintf(FILE *file, LPCSTR fmt, ...)
	{
		std::size_t fmt2_sz = 2048;
		char *fmt2 = (char*)malloc(fmt2_sz);
		va_list  vlist;
		va_start(vlist, fmt);
		while((_vsnprintf(fmt2, fmt2_sz, fmt, vlist)) < 0) // means there wasn't anough room
		{
			fmt2_sz *= 2;
			if(fmt2) free(fmt2);
			fmt2 = (char*)malloc(fmt2_sz);
		}
		OutputDebugStringA(fmt2);
		fprintf(file, fmt2);
		free(fmt2);
	}
#define FPRINTF(a) VSPrintf a
#else //debug
#define FPRINTF(a) fprintf a
// For other than Win32
#endif //debug
#else //unicode
// Unicode case... let's give-up for now and keep basic printf
#define FPRINTF(a) fprintf a
#endif //unicode
#else //win32
#define FPRINTF(a) fprintf a
#endif //win32

// NOTE: "%s(%i) : " allows Visual Studio to directly jump to the file at the right line
// when the user double clicks on the error line in the Output pane. Like any compile error.

inline void __cudaSafeCallNoSync( cudaError err, const char *file, const int line )
{
    if( cudaSuccess != err) {
        FPRINTF((stderr, "%s(%i) : cudaSafeCallNoSync() Runtime API error : %s.\n",
                file, line, cudaGetErrorString( err) ));
        exit(-1);
    }
}

inline void __cudaSafeCall( cudaError err, const char *file, const int line )
{
    if( cudaSuccess != err) {
	FPRINTF((stderr, "%s(%i) : cudaSafeCall() Runtime API error : %s.\n",
                file, line, cudaGetErrorString( err) ));
        exit(-1);
    }
}

inline void __cudaSafeThreadSync( const char *file, const int line )
{
    cudaError err = cutilDeviceSynchronize();
    if ( cudaSuccess != err) {
        FPRINTF((stderr, "%s(%i) : cudaDeviceSynchronize() Runtime API error : %s.\n",
                file, line, cudaGetErrorString( err) ));
        exit(-1);
    }
}

inline void __cufftSafeCall( cufftResult err, const char *file, const int line )
{
    if( CUFFT_SUCCESS != err) {
        FPRINTF((stderr, "%s(%i) : cufftSafeCall() CUFFT error.\n",
                file, line));
        exit(-1);
    }
}

inline void __cutilCheckError( CUTBoolean err, const char *file, const int line )
{
    if( CUTTrue != err) {
        FPRINTF((stderr, "%s(%i) : CUTIL CUDA error.\n",
                file, line));
        exit(-1);
    }
}

inline void __cutilGetLastError( const char *errorMessage, const char *file, const int line )
{
    cudaError_t err = cudaGetLastError();
    if( cudaSuccess != err) {
        FPRINTF((stderr, "%s(%i) : cutilCheckMsg() CUTIL CUDA error : %s : %s.\n",
                file, line, errorMessage, cudaGetErrorString( err) ));
        exit(-1);
    }
}

inline void __cutilGetLastErrorAndSync( const char *errorMessage, const char *file, const int line )
{
    cudaError_t err = cudaGetLastError();
    if( cudaSuccess != err) {
        FPRINTF((stderr, "%s(%i) : cutilCheckMsg() CUTIL CUDA error : %s : %s.\n",
                file, line, errorMessage, cudaGetErrorString( err) ));
        exit(-1);
    }

	err = cutilDeviceSynchronize();
    if( cudaSuccess != err) {
	FPRINTF((stderr, "%s(%i) : cutilCheckMsg cudaDeviceSynchronize error: %s : %s.\n",
                file, line, errorMessage, cudaGetErrorString( err) ));
        exit(-1);
    }
}

inline void __cutilSafeMalloc( void *pointer, const char *file, const int line )
{
    if( !(pointer)) {
        FPRINTF((stderr, "%s(%i) : cutilSafeMalloc host malloc failure\n",
                file, line));
        exit(-1);
    }
}

#if __DEVICE_EMULATION__
    inline int cutilDeviceInit(int ARGC, char **ARGV) { }
    inline int cutilChooseCudaDevice(int ARGC, char **ARGV) { }
#else
    inline int cutilDeviceInit(int ARGC, char **ARGV)
    {
        int deviceCount;
        cutilSafeCallNoSync(cudaGetDeviceCount(&deviceCount));
        if (deviceCount == 0) {
            FPRINTF((stderr, "CUTIL CUDA error: no devices supporting CUDA.\n"));
            exit(-1);
        }
        int dev = 0;
        cutGetCmdLineArgumenti(ARGC, (const char **) ARGV, "device", &dev);
        if (dev < 0) 
            dev = 0;
        if (dev > deviceCount-1) {
			fprintf(stderr, "\n");
			fprintf(stderr, ">> %d CUDA capable GPU device(s) detected. <<\n", deviceCount);
            fprintf(stderr, ">> cutilDeviceInit (-device=%d) is not a valid GPU device. <<\n", dev);
			fprintf(stderr, "\n");
            return -dev;
        }  
        cudaDeviceProp deviceProp;
        cutilSafeCallNoSync(cudaGetDeviceProperties(&deviceProp, dev));
        if (deviceProp.major < 1) {
            FPRINTF((stderr, "cutil error: GPU device does not support CUDA.\n"));
            exit(-1);                                                  \
        }
        printf("> Using CUDA device [%d]: %s\n", dev, deviceProp.name);
        cutilSafeCall(cudaSetDevice(dev));

        return dev;
    }

    // General initialization call to pick the best CUDA Device
    inline int cutilChooseCudaDevice(int argc, char **argv)
    {
        cudaDeviceProp deviceProp;
        int devID = 0;
        // If the command-line has a device number specified, use it
        if( cutCheckCmdLineFlag(argc, (const char**)argv, "device") ) {
            devID = cutilDeviceInit(argc, argv);
            if (devID < 0) {
               printf("exiting...\n");
               cutilExit(argc, argv);
               exit(0);
            }
        } else {
            // Otherwise pick the device with highest Gflops/s
            devID = cutGetMaxGflopsDeviceId();
            cutilSafeCallNoSync( cudaSetDevice( devID ) );
            cutilSafeCallNoSync( cudaGetDeviceProperties(&deviceProp, devID) );
            printf("> Using CUDA device [%d]: %s\n", devID, deviceProp.name);
        }
        return devID;
    }
#endif


//! Check for CUDA context lost
inline void cutilCudaCheckCtxLost(const char *errorMessage, const char *file, const int line ) 
{
    cudaError_t err = cudaGetLastError();
    if( cudaSuccess != err) {
        FPRINTF((stderr, "%s(%i) : CUDA error: %s : %s.\n",
        file, line, errorMessage, cudaGetErrorString( err) ));
        exit(-1);
    }
    err = cutilDeviceSynchronize();
    if( cudaSuccess != err) {
        FPRINTF((stderr, "%s(%i) : CUDA error: %s : %s.\n",
        file, line, errorMessage, cudaGetErrorString( err) ));
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

inline void __cutilQAFinish(int argc, char **argv, bool bStatus)
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
    } 
    else {
        printf("[%s] test result\n%s\n", argv[0], sStatus[bStatus]);
    }
}

// General check for CUDA GPU SM Capabilities
inline bool cutilCudaCapabilities(int major_version, int minor_version, int argc, char **argv)
{
    cudaDeviceProp deviceProp;
    deviceProp.major = 0;
    deviceProp.minor = 0;
    int dev;

#ifdef __DEVICE_EMULATION__
    printf("> Compute Device Emulation Mode \n");
#endif

    cutilSafeCall( cudaGetDevice(&dev) );
    cutilSafeCall( cudaGetDeviceProperties(&deviceProp, dev));

    if((deviceProp.major > major_version) ||
	   (deviceProp.major == major_version && deviceProp.minor >= minor_version)) {
        printf("> Device %d: <%16s >, Compute SM %d.%d detected\n", dev, deviceProp.name, deviceProp.major, deviceProp.minor);
        return true;
    }
    else {
        printf("There is no device supporting CUDA compute capability %d.%d.\n", major_version, minor_version);
        __cutilQAFinish(argc, argv, true);
        return false;
    }
}
