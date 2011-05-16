/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCL_MACROS_H_
#define PCL_MACROS_H_

#if defined __INTEL_COMPILER
  #pragma warning disable 2196 2536 279
#endif

#include <iostream>
#include <stdarg.h>
#include <stdio.h>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

#ifndef PVAR
  #define PVAR(s) \
    #s << " = " << (s) << std::flush
#endif
#ifndef PVARN
#define PVARN(s) \
  #s << " = " << (s) << "\n"
#endif
#ifndef PVARC
#define PVARC(s) \
  #s << " = " << (s) << ", " << std::flush
#endif
#ifndef PVARS
#define PVARS(s) \
  #s << " = " << (s) << " " << std::flush
#endif
#ifndef PVARA
#define PVARA(s) \
  #s << " = " << RAD2DEG(s) << "deg" << std::flush
#endif
#ifndef PVARAN
#define PVARAN(s) \
  #s << " = " << RAD2DEG(s) << "deg\n"
#endif
#ifndef PVARAC
#define PVARAC(s) \
  #s << " = " << RAD2DEG(s) << "deg, " << std::flush
#endif
#ifndef PVARAS
#define PVARAS(s) \
  #s << " = " << RAD2DEG(s) << "deg " << std::flush
#endif

#define FIXED(s) \
  std::fixed << s << std::resetiosflags(std::ios_base::fixed)

#ifndef ERASE_STRUCT
#define ERASE_STRUCT(var) memset(&var, 0, sizeof(var))
#endif

#ifndef ERASE_ARRAY
#define ERASE_ARRAY(var, size) memset(var, 0, size*sizeof(*var))
#endif

#ifndef SET_ARRAY
#define SET_ARRAY(var, value, size) {for (int i=0; i<(int)size; ++i) var[i]=value;}
#endif

/* //This is copy/paste from http://gcc.gnu.org/wiki/Visibility */
/* #if defined _WIN32 || defined __CYGWIN__ */
/*   #ifdef BUILDING_DLL */
/*     #ifdef __GNUC__ */
/* #define DLL_PUBLIC __attribute__((dllexport)) */
/*     #else */
/* #define DLL_PUBLIC __declspec(dllexport) // Note: actually gcc seems to also supports this syntax. */
/*     #endif */
/*   #else */
/*     #ifdef __GNUC__ */
/* #define DLL_PUBLIC __attribute__((dllimport)) */
/*     #else */
/* #define DLL_PUBLIC __declspec(dllimport) // Note: actually gcc seems to also supports this syntax. */
/*     #endif */
/*   #endif */
/*   #define DLL_LOCAL */
/* #else */
/*   #if __GNUC__ >= 4 */
/* #define DLL_PUBLIC __attribute__ ((visibility("default"))) */
/* #define DLL_LOCAL  __attribute__ ((visibility("hidden"))) */
/*   #else */
/*     #define DLL_PUBLIC */
/*     #define DLL_LOCAL */
/*   #endif */
/* #endif */

#ifndef PCL_EXTERN_C
    #ifdef __cplusplus
        #define PCL_EXTERN_C extern "C"
    #else
        #define PCL_EXTERN_C
    #endif
#endif

#if defined WIN32 || defined _WIN32 || defined WINCE || defined __MINGW32__
    #ifdef PCLAPI_EXPORTS
        #define PCL_EXPORTS __declspec(dllexport)
    #else
        #define PCL_EXPORTS
    #endif
#else
    #define PCL_EXPORTS
#endif

#if defined WIN32 || defined _WIN32
    #define PCL_CDECL __cdecl
    #define PCL_STDCALL __stdcall
#else
    #define PCL_CDECL
    #define PCL_STDCALL
#endif

#ifndef PCLAPI
    #define PCLAPI(rettype) PCL_EXTERN_C PCL_EXPORTS rettype PCL_CDECL
#endif

#endif  //#ifndef PCL_MACROS_H_
