/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */


#if defined(_MSC_VER)

// This file is specific to Micrsoft's compiler.
// It contains linking pragmas for building the opennurbs examples.

#pragma once

#if defined(ON_DLL_EXPORTS)
// If you get the following error, your compiler settings
// indicate you are building opennurbs as a DLL. This file
// is used for linking with opennurbs.
#error This file contains linking pragmas for using opennurbs.
#endif

#if !defined(ON_MSC_SOLUTION_DIR)
#define ON_MSC_SOLUTION_DIR ".."
#endif

#if !defined(ON_MSC_LIB_DIR)

#if defined(WIN64)

// x64 (64 bit) static libraries

#if defined(NDEBUG)

// Release x64 (64 bit) libs
#define ON_MSC_LIB_DIR "x64/Release"

#else // _DEBUG

// Debug x64 (64 bit) libs
#define ON_MSC_LIB_DIR "x64/Debug"

#endif // NDEBUG else _DEBUG

#else // WIN32

// x86 (32 bit) static libraries

#if defined(NDEBUG)

// Release x86 (32 bit) libs
#define ON_MSC_LIB_DIR "Release"

#else // _DEBUG

// Debug x86 (32 bit) libs
#define ON_MSC_LIB_DIR "Debug"

#endif // NDEBUG else _DEBUG

#endif // WIN64 else WIN32

#endif //  !defined(ON_MSC_LIB_DIR)

#if defined(ON_DLL_IMPORTS)
#pragma message( " --- dynamically linking opennurbs (DLL)." )
#pragma comment(lib, "\"" ON_MSC_SOLUTION_DIR "/" ON_MSC_LIB_DIR "/" "opennurbs.lib" "\"")
#else
#pragma message( " --- statically linking opennurbs." )
#pragma comment(lib, "\"" ON_MSC_SOLUTION_DIR "/" ON_MSC_LIB_DIR "/" "zlib.lib" "\"")
#pragma comment(lib, "\"" ON_MSC_SOLUTION_DIR "/" ON_MSC_LIB_DIR "/" "opennurbs_staticlib.lib" "\"")
#endif


#endif
