# Useful information for the Intel compiler specifically to determine if we can
# use the -x<code> or /Qx<code> versions, or if we need to use -m<code> or
# /arch<code>.  Using -x<code> on a non-Intel CPU is not allowed.
cmake_host_system_information(
    RESULT PCL_HOST_PROCESSOR_DESCRIPTION
    QUERY  PROCESSOR_DESCRIPTION
)
if(PCL_HOST_PROCESSOR_DESCRIPTION MATCHES "Intel")
    set(PCL_HOST_PROCESSOR_IS_INTEL TRUE)
else()
    set(PCL_HOST_PROCESSOR_IS_INTEL FALSE)
endif()

# Helper macro for pcl_check_for_sse, *NOT* intended to be called anywhere else.
# No error checking is performed!
#
# output: Where to store the compiler specific flag.
# arch: the architecture flag to test.  Inputs _must_ be lower-case, the only
#       acceptable values are:
#
#       sse, sse2, sse3, sse4.1, sse4.2
#
# NOTE: ${output} set to the empty string in cases where the compiler does not
#       have a flag for this ${arch} (e.g., Visual Studio and sse4.2).
#
# Example: pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS sse4.2)
function(PCL_SSE_COMPILER_ARCH_FLAG output arch)
    if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
        # sse -> -msse, sse4.2 -> -msse4.2
        set(${output} "-m${arch}" PARENT_SCOPE)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
        # Intel Compiler rules:
        #
        # If compiling for Intel CPU (https://software.intel.com/en-us/node/522845):
        #   Windows: /Qx<code>
        #   Unix:    -x<code>
        #
        # Otherwise:
        #   Windows: /arch:<code>
        #            https://software.intel.com/en-us/node/522822
        #   Unix:    -m<code>
        #            https://software.intel.com/en-us/node/522834
        #
        # As such, since we are doing all of the tests anyway, we will avoid -xHost
        # and /QxHost because those are only for Intel CPUs (their equivalent of
        # -march=native).
        #
        # The logic is included because the -x or /Qx versions include additional
        # optimizations that -m or /arch do not.
        if(arch STREQUAL "sse")
            set(${output} "" PARENT_SCOPE)
        else()
            string(TOUPPER "${arch}" arch)
            if (WIN32)
                if(PCL_HOST_PROCESSOR_IS_INTEL)
                    set(${output} "/Qx${arch}" PARENT_SCOPE)
                else()
                    set(${output} "/arch:${arch}" PARENT_SCOPE)
                endif()
            else()
                if(PCL_HOST_PROCESSOR_IS_INTEL)
                    set(${output} "-x${arch}" PARENT_SCOPE)
                else()
                    set(${output} "-m${arch}" PARENT_SCOPE)
                endif()
            endif()
        endif()
    elseif(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8) # 64 bit Visual Studio
        # MSVC only supports SSE, SSE2, AVX, and AVX2
        string(TOUPPER "${arch}" arch)
        set(msvc_archs SSE SSE2 AVX AVX2)
        if(arch IN_LIST msvc_archs)
            set(${output} "/arch:${arch}" PARENT_SCOPE)
        else()
            set(${output} "" PARENT_SCOPE)
        endif()
    else()
        set(${output} "" PARENT_SCOPE)
    endif()
endfunction()

###############################################################################
# Check for the presence of SSE and figure out the flags to use for it.
macro(PCL_CHECK_FOR_SSE)
    set(SSE_FLAGS)
    set(SSE_DEFINITIONS)

    if (NOT CMAKE_CROSSCOMPILING)
        if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
            list(APPEND SSE_FLAGS "-march=native")
            message(STATUS "Using CPU native flags for SSE optimization: ${SSE_FLAGS}")
        endif()
    endif()

    # Unfortunately we need to check for SSE to enable "-mfpmath=sse" alongside
    # "-march=native". The reason for this is that by default, 32bit architectures
    # tend to use the x87 FPU (which has 80 bit internal precision), thus leading
    # to different results than 64bit architectures which are using SSE2 (64 bit internal
    # precision). One solution would be to use "-ffloat-store" on 32bit (see
    # http://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html), but that slows code down,
    # so the preferred solution is to try "-mpfmath=sse" first.
    include(CheckCXXSourceRuns)
    set(CMAKE_REQUIRED_FLAGS)

    check_cxx_source_runs("
        // Intel compiler defines an incompatible _mm_malloc signature
        #if defined(__INTEL_COMPILER)
            #include <malloc.h>
        #else
            #include <mm_malloc.h>
        #endif
        int main()
        {
          void* mem = _mm_malloc (100, 16);
          return 0;
        }"
        HAVE_MM_MALLOC)

    check_cxx_source_runs("
        #include <stdlib.h>
        int main()
        {
          void* mem;
          return posix_memalign (&mem, 16, 100);
        }"
        HAVE_POSIX_MEMALIGN)

    pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS "avx2")
    check_cxx_source_runs("
        #include <immintrin.h>
        int main ()
        {
          /* simple test: subtract result = left - right    */
          /* using _mm256_sub_epi32 is the 'test' (new in AVX2) */
          __m256i left   = _mm256_set_epi32 (0, 1, 2, 3, 4, 5, 6, 7);
          __m256i right  = _mm256_set_epi32 (7, 6, 5, 4, 3, 2, 1, 0);
          __m256i result = _mm256_sub_epi32 (right, left);

          // result: {-7, -5, -3, -1, 1, 3, 5, 7}
          // could check using int *i = (int *)&result;
          return (0);
        }"
        HAVE_AVX2_EXTENSIONS)

    pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS "avx")
    check_cxx_source_runs("
        #include <immintrin.h>
        int main ()
        {
          /* simple test: subtract result = left - right    */
          /* using _mm256_sub_ps is the 'test' (new in AVX) */
          __m256 left   = _mm256_set_ps (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
          __m256 right  = _mm256_set_ps (7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0);
          __m256 result = _mm256_sub_ps (right, left);

          // result: {-7, -5, -3, -1, 1, 3, 5, 7}
          // could check using float *f = (float *)&result;
          return (0);
        }"
        HAVE_AVX_EXTENSIONS)

    pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS "sse4.2")
    check_cxx_source_runs("
        #include <emmintrin.h>
        #include <nmmintrin.h>
        int main ()
        {
          long long a[2] = {  1, 2 };
          long long b[2] = { -1, 3 };
          long long c[2];
          __m128i va = _mm_loadu_si128 ((__m128i*)a);
          __m128i vb = _mm_loadu_si128 ((__m128i*)b);
          __m128i vc = _mm_cmpgt_epi64 (va, vb);

          _mm_storeu_si128 ((__m128i*)c, vc);
          if (c[0] == -1LL && c[1] == 0LL)
            return (0);
          else
            return (1);
        }"
        HAVE_SSE4_2_EXTENSIONS)

    pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS "sse4.1")
    check_cxx_source_runs("
        #include <smmintrin.h>
        int main ()
        {
          __m128 a, b;
          float vals[4] = {1, 2, 3, 4};
          const int mask = 123;
          a = _mm_loadu_ps (vals);
          b = a;
          b = _mm_dp_ps (a, a, mask);
          _mm_storeu_ps (vals,b);
          return (0);
        }"
        HAVE_SSE4_1_EXTENSIONS)

    pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS "ssse3")
    check_cxx_source_runs("
        #include <tmmintrin.h>
        int main ()
        {
          __m128i a, b;
          int vals[4] = {-1, -2, -3, -4};
          a = _mm_loadu_si128 ((const __m128i*)vals);
          b = _mm_abs_epi32 (a);
          _mm_storeu_si128 ((__m128i*)vals, b);
          return (0);
        }"
        HAVE_SSSE3_EXTENSIONS)

    pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS "sse3")
    check_cxx_source_runs("
        #include <pmmintrin.h>
        int main ()
        {
            __m128d a, b;
            double vals[2] = {0};
            a = _mm_loadu_pd (vals);
            b = _mm_hadd_pd (a,a);
            _mm_storeu_pd (vals, b);
            return (0);
        }"
        HAVE_SSE3_EXTENSIONS)

    pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS "sse2")
    check_cxx_source_runs("
        #include <emmintrin.h>
        int main ()
        {
            __m128d a, b;
            double vals[2] = {0};
            a = _mm_loadu_pd (vals);
            b = _mm_add_pd (a,a);
            _mm_storeu_pd (vals,b);
            return (0);
        }"
        HAVE_SSE2_EXTENSIONS)

    pcl_sse_compiler_arch_flag(CMAKE_REQUIRED_FLAGS "sse")
    check_cxx_source_runs("
        #include <xmmintrin.h>
        int main ()
        {
            __m128 a, b;
            float vals[4] = {0};
            a = _mm_loadu_ps (vals);
            b = a;
            b = _mm_add_ps (a,b);
            _mm_storeu_ps (vals,b);
            return (0);
        }"
        HAVE_SSE_EXTENSIONS)

    # Make sure to un-set this variable so later code is not affected.
    set(CMAKE_REQUIRED_FLAGS)

    # Start: empty, if empty after all if-elseif signals no SSE/AVX support.
    # Order matters: make sure to do "highest" vectorization checks first.
    # NOTE: helper function sets to empty string on 32-bit MSVC build always.
    set(architecture_flag)
    if(HAVE_AVX2_EXTENSIONS)
        pcl_sse_compiler_arch_flag(architecture_flag "avx2")
    elseif(HAVE_AVX_EXTENSIONS)
        pcl_sse_compiler_arch_flag(architecture_flag "avx")
    elseif(HAVE_SSE4_2_EXTENSIONS)
        pcl_sse_compiler_arch_flag(architecture_flag "sse4.2")
    elseif(HAVE_SSE4_1_EXTENSIONS)
        pcl_sse_compiler_arch_flag(architecture_flag "sse4.1")
    elseif(HAVE_SSSE3_EXTENSIONS)
        pcl_sse_compiler_arch_flag(architecture_flag "ssse3")
    elseif(HAVE_SSE3_EXTENSIONS)
        pcl_sse_compiler_arch_flag(architecture_flag "sse3")
    elseif(HAVE_SSE2_EXTENSIONS)
        pcl_sse_compiler_arch_flag(architecture_flag "sse2")
    elseif(HAVE_SSE_EXTENSIONS)
        pcl_sse_compiler_arch_flag(architecture_flag "sse")
    endif()

    if(architecture_flag)
        list(APPEND SSE_FLAGS "${architecture_flag}")
        # GCC, Clang, or Intel on Non-Windows (AKA Intel backed by GCC or Clang)
        if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
            list(APPEND SSE_FLAGS "-mfpmath=sse")
        endif()
    else()
        if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
            # Setting -ffloat-store to alleviate 32bit vs 64bit discrepancies on non-SSE
            # platforms.
            list(APPEND SSE_FLAGS "-ffloat-store")
        endif()
    endif()

    # Erase architecture_flag
    set(architecture_flag)

    if(MSVC)
        # TODO: why are these definitions here and what are the SSE4, AVX, and AVX2 ones?
        if(HAVE_SSSE3_EXTENSIONS)
            SET(SSE_DEFINITIONS "${SSE_DEFINITIONS} -D__SSSE3__")
        endif()
        if(HAVE_SSE2_EXTENSIONS)
            SET(SSE_DEFINITIONS "${SSE_DEFINITIONS} -D__SSE2__")
        endif()
        if(HAVE_SSE_EXTENSIONS)
            SET(SSE_DEFINITIONS "${SSE_DEFINITIONS} -D__SSE__")
        endif()
    endif()
    string(REPLACE ";" " " SSE_FLAGS_STR "${SSE_FLAGS}")
endmacro(PCL_CHECK_FOR_SSE)
