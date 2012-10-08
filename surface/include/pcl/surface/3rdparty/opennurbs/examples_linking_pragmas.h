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
