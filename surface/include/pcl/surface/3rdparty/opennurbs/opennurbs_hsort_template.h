#if !defined(ON_COMPILING_OPENNURBS_HSORT_FUNCTIONS)
/*
See opennurbs_sort.cpp for examples of using openurbs_hsort_template.c
to define type specific heap sort functions.
*/
#error Do not compile openurbs_hsort_template.c directly.
#endif

// ON_SORT_TEMPLATE_TYPE -> double, int, ....
#if !defined(ON_SORT_TEMPLATE_TYPE)
#error Define ON_SORT_TEMPLATE_TYPE macro before including opennurbs_qsort_template.c
#endif

#if !defined(ON_HSORT_FNAME)
#error Define ON_HSORT_FNAME macro before including opennurbs_qsort_template.c
#endif

#if defined(ON_SORT_TEMPLATE_COMPARE)
// use a compare function like strcmp for char* strings
#define ON_HSORT_GT(A,B) ON_SORT_TEMPLATE_COMPARE(A,B) > 0
#define ON_HSORT_GT_TMP(A) ON_SORT_TEMPLATE_COMPARE(A,&tmp) > 0
#else
// use type compares
#define ON_HSORT_GT(A,B) *A > *B
#define ON_HSORT_GT_TMP(A) *A > tmp
#endif

#if defined(ON_SORT_TEMPLATE_USE_MEMCPY)
#define ON_HSORT_TO_TMP(A) memcpy(&tmp,A,sizeof(tmp))
#define ON_HSORT_FROM_TMP(A) memcpy(A,&tmp,sizeof(tmp))
#define ON_HSORT_COPY(dst,src) memcpy(dst,src,sizeof(tmp))
#else
#define ON_HSORT_TO_TMP(A) tmp = *A
#define ON_HSORT_FROM_TMP(A) *A = tmp
#define ON_HSORT_COPY(dst,src) *dst = *src
#endif

#if defined(ON_SORT_TEMPLATE_STATIC_FUNCTION)
static
#endif
void
ON_HSORT_FNAME( ON_SORT_TEMPLATE_TYPE* base, std::size_t nel )
{
  std::size_t i_end,k,i,j;
  ON_SORT_TEMPLATE_TYPE* e_end;
  ON_SORT_TEMPLATE_TYPE* e_i;
  ON_SORT_TEMPLATE_TYPE* e_j;
  ON_SORT_TEMPLATE_TYPE tmp;

  if (0 == base || nel < 2)
    return;

  k = nel >> 1;
  i_end = nel-1;
  e_end = base + i_end;
  for (;;) 
  {
    if (k)
    {
      --k;
      ON_HSORT_TO_TMP((base+k));  /* e_tmp = e[k]; */
    } 
    else
    {      
      ON_HSORT_TO_TMP(e_end);     /* e_tmp = e[i_end]; */
      ON_HSORT_COPY(e_end,base);  /* e[i_end] = e[0];  */
      if (!(--i_end))
      {
        ON_HSORT_FROM_TMP(base);  /* e[0] = e_tmp;     */
        break;
      }
      e_end--;
    }

    i = k;
    j = (k<<1) + 1;
    e_i = base + i;
    while (j <= i_end) 
    {
      e_j = base + j;
      if (j < i_end && ON_HSORT_GT((e_j+1),e_j) /*e[j] < e[j + 1] */)
      {
        j++;
        e_j++;
      }
      if (ON_HSORT_GT_TMP(e_j) /* tmp < e[j] */) 
      {
        ON_HSORT_COPY(e_i,e_j);  /* e[i] = e[j]; */
        i = j;
        e_i = e_j;
        j = (j<<1) + 1;
      }
      else
        j = i_end + 1;
    }

    ON_HSORT_FROM_TMP(e_i); /* e[i] = e_tmp; */
  }
}

#undef ON_HSORT_GT
#undef ON_HSORT_GT_TMP
#undef ON_HSORT_TO_TMP
#undef ON_HSORT_FROM_TMP
#undef ON_HSORT_COPY
#undef ON_HSORT_FROM_TMP
