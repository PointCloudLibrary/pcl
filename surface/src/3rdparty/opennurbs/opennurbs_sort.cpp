#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

#include <pcl/common/utils.h> // pcl::utils::ignore


/*
If the speed of ON_qsort() functions on arrays that
are nearly sorted is as good as heap sort, then define
ON__QSORT_FASTER_THAN_HSORT.
*/

#define ON_COMPILING_OPENNURBS_SORT_CPP

#if defined(ON_OS_WINDOWS) && defined(ON_COMPILER_MSC)

#pragma optimize("t", on)

// have a reliable thread safe CRT qsort.
#define ON__HAVE_RELIABLE_SYSTEM_QSORT
#define ON__QSORT_FASTER_THAN_HSORT

#elif defined(_GNU_SOURCE)

// have a reliable thread safe CRT qsort.
#define ON__HAVE_RELIABLE_SYSTEM_QSORT
#define ON__QSORT_FASTER_THAN_HSORT

#endif

#if defined(ON_OS_WINDOWS) && defined(ON_COMPILER_MSC)

// have a reliable thread safe CRT qsort with context that is
// faster than the functions below.
#define ON__HAVE_RELIABLE_SYSTEM_CONTEXT_QSORT
#define ON__QSORT_FASTER_THAN_HSORT

#elif defined(_GNU_SOURCE)

#define ON__HAVE_RELIABLE_SYSTEM_CONTEXT_QSORT
#define ON__QSORT_FASTER_THAN_HSORT

#endif

#define work_size 64

void 
ON_qsort( void *base, std::size_t nel, std::size_t width, int (*compar)(void*,const void *, const void *),void* context)
{
  pcl::utils::ignore(base, nel, width, compar, context);
#if defined(ON__HAVE_RELIABLE_SYSTEM_CONTEXT_QSORT)
  // The call here must be a thread safe system qsort
  // that is faster than the alternative code in this function.
  // In particular, if it uses a random number generator to
  // find pivots, that calculation must be thread safe.
#if defined(ON_COMPILER_MSC)
  qsort_s(base,nel,width,compar,context);
#elif defined(ON_COMPILER_XCODE)
  qsort_r(base,nel,width,context,compar);
#endif
#else
  ON_hsort(base, nel, width, compar, context);
#endif
}

void 
ON_qsort( void *base, std::size_t nel, std::size_t width, int (*compar)(const void *, const void *))
{
#if defined(ON__HAVE_RELIABLE_SYSTEM_QSORT)
  // The call here must be a thread safe system qsort
  // that is faster than the alternative code in this function.
  // In particular, if it uses a random number generator to
  // find pivots, that calculation must be thread safe.
  qsort(base,nel,width,compar);
#else
  ON_hsort(base, nel, width, compar);
#endif
}

void
ON_hsort(void *base, std::size_t nel, std::size_t width, int (*compar)(const void*,const void*))
{
  std::size_t
    i_end,k;
  unsigned char
    work_memory[work_size], *e_tmp, *e_end;

  if (nel < 2) return;
  k = nel >> 1;
  i_end = nel-1;
  e_end = ((unsigned char*)base) + i_end*width;
  e_tmp = (width > work_size) ? (unsigned char*)onmalloc(width) : work_memory;
  for (;;) {
    if (k) {
      --k;
      memcpy(e_tmp,((unsigned char*)base)+k*width,width); /* e_tmp = e[k]; */
    } 
    else {      
      memcpy(e_tmp,e_end,width);     /* e_tmp = e[i_end]; */
      memcpy(e_end,base,width);      /* e[i_end] = e[0];  */
      if (!(--i_end)) {
        memcpy(base,e_tmp,width);    /* e[0] = e_tmp;     */
        break;
      }
      e_end -= width;
    }
    { std::size_t i, j;
      unsigned char *e_i, *e_j;
      i = k;
      j = (k<<1) + 1;
      e_i = ((unsigned char*)base) + i*width;
      while (j <= i_end) {
        e_j = ((unsigned char*)base) + j*width;
        if (j < i_end && compar(e_j,e_j+width)<0 /*e[j] < e[j + 1] */)
          {j++; e_j += width;}
        if (compar(e_tmp,e_j)<0 /* e_tmp < e[j] */) {
          memcpy(e_i,e_j,width); /* e[i] = e[j]; */
          i = j;
          e_i = e_j;
          j = (j<<1) + 1;
        } else j = i_end + 1;
      }
      memcpy(e_i,e_tmp,width); /* e[i] = e_tmp; */
    }
  }
  if (width > work_size) onfree(e_tmp); 
}

void
ON_hsort(void *base, std::size_t nel, std::size_t width, int (*compar)(void*,const void*,const void*), void* context)
{
  std::size_t
    i_end,k;
  unsigned char
    work_memory[work_size], *e_tmp, *e_end;

  if (nel < 2) return;
  k = nel >> 1;
  i_end = nel-1;
  e_end = ((unsigned char*)base) + i_end*width;
  e_tmp = (width > work_size) ? (unsigned char*)onmalloc(width) : work_memory;
  for (;;) {
    if (k) {
      --k;
      memcpy(e_tmp,((unsigned char*)base)+k*width,width); /* e_tmp = e[k]; */
    } 
    else {      
      memcpy(e_tmp,e_end,width);     /* e_tmp = e[i_end]; */
      memcpy(e_end,base,width);      /* e[i_end] = e[0];  */
      if (!(--i_end)) {
        memcpy(base,e_tmp,width);    /* e[0] = e_tmp;     */
        break;
      }
      e_end -= width;
    }
    { std::size_t i, j;
      unsigned char *e_i, *e_j;
      i = k;
      j = (k<<1) + 1;
      e_i = ((unsigned char*)base) + i*width;
      while (j <= i_end) {
        e_j = ((unsigned char*)base) + j*width;
        if (j < i_end && compar(context,e_j,e_j+width)<0 /*e[j] < e[j + 1] */)
          {j++; e_j += width;}
        if (compar(context,e_tmp,e_j)<0 /* e_tmp < e[j] */) {
          memcpy(e_i,e_j,width); /* e[i] = e[j]; */
          i = j;
          e_i = e_j;
          j = (j<<1) + 1;
        } else j = i_end + 1;
      }
      memcpy(e_i,e_tmp,width); /* e[i] = e_tmp; */
    }
  }
  if (width > work_size) onfree(e_tmp); 
}

#undef work_size  

#define ON_COMPILING_OPENNURBS_QSORT_FUNCTIONS
#define ON_COMPILING_OPENNURBS_HSORT_FUNCTIONS
#define ON_SORT_TEMPLATE_STATIC_FUNCTION

#define ON_SORT_TEMPLATE_TYPE double
#define ON_QSORT_FNAME ON_qsort_double
#define ON_HSORT_FNAME ON_hsort_double
#include "pcl/surface/3rdparty/opennurbs/opennurbs_qsort_template.h"
#include "pcl/surface/3rdparty/opennurbs/opennurbs_hsort_template.h"
#undef ON_SORT_TEMPLATE_TYPE
#undef ON_QSORT_FNAME
#undef ON_HSORT_FNAME

void ON_SortDoubleArray( 
        ON::sort_algorithm sort_algorithm,
        double* a,
        std::size_t nel
        )
{
  if ( ON::heap_sort == sort_algorithm )
    ON_hsort_double(a,nel);
  else
    ON_qsort_double(a,nel);
}

#define ON_SORT_TEMPLATE_TYPE float
#define ON_QSORT_FNAME ON_qsort_float
#define ON_HSORT_FNAME ON_hsort_float
#include "pcl/surface/3rdparty/opennurbs/opennurbs_qsort_template.h"
#include "pcl/surface/3rdparty/opennurbs/opennurbs_hsort_template.h"
#undef ON_SORT_TEMPLATE_TYPE
#undef ON_QSORT_FNAME
#undef ON_HSORT_FNAME

void ON_SortFloatArray( 
        ON::sort_algorithm sort_algorithm,
        float* a,
        std::size_t nel
        )
{
  if ( ON::heap_sort == sort_algorithm )
    ON_hsort_float(a,nel);
  else
    ON_qsort_float(a,nel);
}


#define ON_SORT_TEMPLATE_TYPE int
#define ON_QSORT_FNAME ON_qsort_int
#define ON_HSORT_FNAME ON_hsort_int
#include "pcl/surface/3rdparty/opennurbs/opennurbs_qsort_template.h"
#include "pcl/surface/3rdparty/opennurbs/opennurbs_hsort_template.h"
#undef ON_SORT_TEMPLATE_TYPE
#undef ON_QSORT_FNAME
#undef ON_HSORT_FNAME

void ON_SortIntArray(
        ON::sort_algorithm sort_algorithm,
        int* a,
        std::size_t nel
        )
{
  if ( ON::heap_sort == sort_algorithm )
    ON_hsort_int(a,nel);
  else
    ON_qsort_int(a,nel);
}


#define ON_SORT_TEMPLATE_TYPE unsigned int
#define ON_QSORT_FNAME ON_qsort_uint
#define ON_HSORT_FNAME ON_hsort_uint
#include "pcl/surface/3rdparty/opennurbs/opennurbs_qsort_template.h"
#include "pcl/surface/3rdparty/opennurbs/opennurbs_hsort_template.h"
#undef ON_SORT_TEMPLATE_TYPE
#undef ON_QSORT_FNAME
#undef ON_HSORT_FNAME

void ON_SortUnsignedIntArray(
        ON::sort_algorithm sort_algorithm,
        unsigned int* a,
        std::size_t nel
        )
{
  if ( ON::heap_sort == sort_algorithm )
    ON_hsort_uint(a,nel);
  else
    ON_qsort_uint(a,nel);
}

