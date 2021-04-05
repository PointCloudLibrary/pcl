###############################################################################
# Check for the presence of AVX and figure out the flags to use for it.
function(PCL_CHECK_FOR_AVX)
  set(AVX_FLAGS)

  include(CheckCXXSourceRuns)
  
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
