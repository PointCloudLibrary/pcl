###############################################################################
# Check for the presence of _mm_malloc() and posix_memalign()
function(PCL_CHECK_FOR_ALIGNMENT)
  include(CheckCXXSourceCompiles)
  if(NOT DEFINED HAVE_MM_MALLOC)
    check_cxx_source_compiles("
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
  endif()

  if(NOT DEFINED HAVE_POSIX_MEMALIGN)
    check_cxx_source_compiles("
      #include <stdlib.h>
      int main()
      {
        void* mem;
        return posix_memalign (&mem, 16, 100);
      }"
      HAVE_POSIX_MEMALIGN)
  endif()
endfunction()
