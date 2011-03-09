###############################################################################
# Check for the presence of SSE and figure out the flags to use for it.
macro(PCL_CHECK_FOR_SSE)
    include(CheckCXXSourceRuns)
    if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        set(SSE_FLAGS)

        set(CMAKE_REQUIRED_FLAGS "-msse3")
        check_cxx_source_runs("
            #include <pmmintrin.h>

            int main()
            {
                __m128d a, b;
                double vals[2] = {0};
                a = _mm_loadu_pd(vals);
                b = _mm_hadd_pd(a,a);
                _mm_storeu_pd(vals, b);
                return 0;
            }"
            HAVE_SSE3_EXTENSIONS)

        set(CMAKE_REQUIRED_FLAGS "-msse2")
        check_cxx_source_runs("
            #include <emmintrin.h>

            int main()
            {
                __m128d a, b;
                double vals[2] = {0};
                a = _mm_loadu_pd(vals);
                b = _mm_add_pd(a,a);
                _mm_storeu_pd(vals,b);
                return 0;
            }"
            HAVE_SSE2_EXTENSIONS)

        set(CMAKE_REQUIRED_FLAGS "-msse")
        check_cxx_source_runs("
            #include <xmmintrin.h>
            int main()
            {
                __m128 a, b;
                float vals[4] = {0};
                a = _mm_loadu_ps(vals);
                b = a;
                b = _mm_add_ps(a,b);
                _mm_storeu_ps(vals,b);
                return 0;
            }"
            HAVE_SSE_EXTENSIONS)

       set(CMAKE_REQUIRED_FLAGS)

       if(HAVE_SSE3_EXTENSIONS)
           set(SSE_FLAGS "-msse3 -mfpmath=sse")
           message(STATUS "Found SSE3 extensions, using flags: ${SSE_FLAGS}")
       elseif(HAVE_SSE2_EXTENSIONS)
           set(SSE_FLAGS "-msse2 -mfpmath=sse")
           message(STATUS "Found SSE2 extensions, using flags: ${SSE_FLAGS}")
       elseif(HAVE_SSE_EXTENSIONS)
           set(SSE_FLAGS "-msse -mfpmath=sse")
           message(STATUS "Found SSE extensions, using flags: ${SSE_FLAGS}")
       endif(HAVE_SSE3_EXTENSIONS)
    elseif(MSVC)
        check_cxx_source_runs("
            #include <emmintrin.h>

            int main()
            {
                __m128d a, b;
                double vals[2] = {0};
                a = _mm_loadu_pd(vals);
                b = _mm_add_pd(a,a);
                _mm_storeu_pd(vals,b);
                return 0;
            }"
            HAVE_SSE2_EXTENSIONS)

        if(HAVE_SSE2_EXTENSIONS)
            set(SSE_FLAGS "/arch:SSE2 /fp:fast -D__SSE__ -D__SSE2__" )
            message(STATUS "Found SSE2 extensions, using flags: ${SSE_FLAGS}")
        endif(HAVE_SSE2_EXTENSIONS)
    endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
endmacro(PCL_CHECK_FOR_SSE)


###############################################################################
# Add the SSE flags to a target.
# _name The name of the target to add the flags to.
macro(PCL_ADD_SSE_FLAGS _name)
    if(SSE_FLAGS)
        PCL_ADD_CFLAGS(${_name} ${SSE_FLAGS})
    endif(SSE_FLAGS)
endmacro(PCL_ADD_SSE_FLAGS)

