###############################################################################
# Check for the presence of AVX and figure out the flags to use for it.
function(PCL_CHECK_FOR_AVX)
  set(AVX_FLAGS)

  include(CheckCXXSourceRuns)
  
  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
    # Setting -march & -mtune just as required flags for check_cxx_source_runs,
    # and CMAKE_REQUIRED_FLAGS will be restored after test runs.
    set(CMAKE_REQUIRED_FLAGS "-march=native -mtune=native")
  endif()

  check_cxx_source_runs("    
    #include <immintrin.h>
    int main()
    {
      __m256i a = {0};
      a = _mm256_abs_epi16(a);
      return 0;
    }"
  HAVE_AVX2)

  if(NOT HAVE_AVX2)
    check_cxx_source_runs("
      #include <immintrin.h>
      int main()
      {
        __m256 a;
        a = _mm256_set1_ps(0);
        return 0;
      }"
    HAVE_AVX)
  endif()

  set(CMAKE_REQUIRED_FLAGS)

# Setting the -mavx/-mavx2 defines __AVX(2)__, see here https://stackoverflow.com/a/28939692
# and this allows the compiler to use the codes for AVX behind code guards.
  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
    if(HAVE_AVX2)
      set(AVX_FLAGS "-mavx2" PARENT_SCOPE)
    elseif(HAVE_AVX)
      set(AVX_FLAGS "-mavx" PARENT_SCOPE)
    endif()
  endif()

# Setting the /arch defines __AVX(2)__, see here https://docs.microsoft.com/en-us/cpp/build/reference/arch-x64?view=msvc-160
# AVX2 extends and includes AVX.
# Setting these defines allows the compiler to use AVX instructions as well as code guarded with the defines.
# TODO: Add AVX512 variant if needed.
  if(MSVC)
    if(HAVE_AVX2)
      set(AVX_FLAGS "/arch:AVX2" PARENT_SCOPE)
    elseif(HAVE_AVX)
      set(AVX_FLAGS "/arch:AVX" PARENT_SCOPE)
    endif()
  endif()
endfunction()
