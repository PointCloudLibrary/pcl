// NOTE: 14 April 2011 Dale Lear:
//    Replace this code with Mikko's "quacksort", once "quacksort" is fully debugged
//    This code is based ont the VC 2010 crt qsort.c file and must not be released
//    with public opennurbs.

#if !defined(ON_COMPILING_OPENNURBS_QSORT_FUNCTIONS)
/*
See opennurbs_sort.cpp for examples of using openurbs_qsort_template.c
to define type specific quick sort functions.
*/
#error Do not compile openurbs_qsort_template.c directly.
#endif

#define ON_QSORT_CUTOFF 8            /* testing shows that this is good value */

/* Note: the theoretical number of stack entries required is
   no more than 1 + log2(num).  But we switch to insertion
   sort for CUTOFF elements or less, so we really only need
   1 + log2(num) - log2(CUTOFF) stack entries.  For a CUTOFF
   of 8, that means we need no more than 30 stack entries for
   32 bit platforms, and 62 for 64-bit platforms. */
#define ON_QSORT_STKSIZ (8*sizeof(void*) - 2)


// ON_SORT_TEMPLATE_TYPE -> double, int, ....
#if !defined(ON_SORT_TEMPLATE_TYPE)
#error Define ON_SORT_TEMPLATE_TYPE macro before including opennurbs_qsort_template.c
#endif

#if !defined(ON_QSORT_FNAME)
#error Define ON_QSORT_FNAME macro before including opennurbs_qsort_template.c
#endif

#if defined(ON_SORT_TEMPLATE_COMPARE)
// use a compare function like strcmp for char* strings
#define ON_QSORT_GT(A,B) ON_SORT_TEMPLATE_COMPARE(A,B) > 0
#define ON_QSORT_LE(A,B) ON_SORT_TEMPLATE_COMPARE(A,B) <= 0
#define ON_QSORT_EQ(A,B) ON_SORT_TEMPLATE_COMPARE(A,B) == 0
#else
// use type compares
#define ON_QSORT_GT(A,B) *A > *B
#define ON_QSORT_LE(A,B) *A <= *B
#define ON_QSORT_EQ(A,B) *A == *B
#endif

#if defined(ON_SORT_TEMPLATE_USE_MEMCPY)
#define ON_QSORT_SWAP(A,B) memcpy(&tmp,A,sizeof(tmp));memcpy(A,B,sizeof(tmp));memcpy(B,&tmp,sizeof(tmp))
#else
#define ON_QSORT_SWAP(A,B) tmp = *A; *A = *B; *B = tmp
#endif

static void ON_shortsort(ON_SORT_TEMPLATE_TYPE *, ON_SORT_TEMPLATE_TYPE *);
static void ON_shortsort(ON_SORT_TEMPLATE_TYPE *lo, ON_SORT_TEMPLATE_TYPE *hi)
{
  ON_SORT_TEMPLATE_TYPE *p;
  ON_SORT_TEMPLATE_TYPE *max;
  ON_SORT_TEMPLATE_TYPE tmp;

  /* Note: in assertions below, i and j are alway inside original bound of
      array to sort. */

  while (hi > lo)
  {
    /* A[i] <= A[j] for i <= j, j > hi */
    max = lo;
    for (p = lo+1; p <= hi; p++)
    {
      /* A[i] <= A[max] for lo <= i < p */
      if ( ON_QSORT_GT(p,max) )
      {
          max = p;
      }
      /* A[i] <= A[max] for lo <= i <= p */
    }

    /* A[i] <= A[max] for lo <= i <= hi */

    ON_QSORT_SWAP(max,hi);

    /* A[i] <= A[hi] for i <= hi, so A[i] <= A[j] for i <= j, j >= hi */

    hi--;

    /* A[i] <= A[j] for i <= j, j > hi, loop top condition established */
  }
  /* A[i] <= A[j] for i <= j, j > lo, which implies A[i] <= A[j] for i < j,
      so array is sorted */
}

/* this parameter defines the cutoff between using quick sort and
   insertion sort for arrays; arrays with lengths shorter or equal to the
   below value use insertion sort */

#if defined(ON_SORT_TEMPLATE_STATIC_FUNCTION)
static
#endif
void 
ON_QSORT_FNAME (
    ON_SORT_TEMPLATE_TYPE *base,
    std::size_t num
    )
{
  ON_SORT_TEMPLATE_TYPE *lo;                   /* start of sub-array currently sorting */
  ON_SORT_TEMPLATE_TYPE *hi;                   /* end of sub-array currently sorting */
  ON_SORT_TEMPLATE_TYPE *mid;                  /* points to middle of subarray */
  ON_SORT_TEMPLATE_TYPE *loguy;                /* traveling pointers for partition step */
  ON_SORT_TEMPLATE_TYPE *higuy;                /* traveling pointers for partition step */
  ON_SORT_TEMPLATE_TYPE *lostk[ON_QSORT_STKSIZ];
  ON_SORT_TEMPLATE_TYPE *histk[ON_QSORT_STKSIZ];
  std::size_t size;                /* size of the sub-array */
  int stkptr;                 /* stack for saving sub-array to be processed */
  ON_SORT_TEMPLATE_TYPE tmp;

  if ( 0 == base || num < 2 )
    return;

  stkptr = 0;                 /* initialize stack */

  lo = base;
  hi = base + (num-1);        /* initialize limits */

  /* this entry point is for pseudo-recursion calling: setting
      lo and hi and jumping to here is like recursion, but stkptr is
      preserved, locals aren't, so we preserve stuff on the stack */
recurse:

  size = (hi - lo) + 1;        /* number of el's to sort */

  /* below a certain size, it is faster to use a O(n^2) sorting method */
  if (size <= ON_QSORT_CUTOFF) 
  {
      ON_shortsort(lo, hi);
  }
  else {
    /* First we pick a partitioning element.  The efficiency of the
        algorithm demands that we find one that is approximately the median
        of the values, but also that we select one fast.  We choose the
        median of the first, middle, and last elements, to avoid bad
        performance in the face of already sorted data, or data that is made
        up of multiple sorted runs appended together.  Testing shows that a
        median-of-three algorithm provides better performance than simply
        picking the middle element for the latter case. */

    mid = lo + (size / 2);      /* find middle element */

    /* Sort the first, middle, last elements into order */
    if ( ON_QSORT_GT(lo,mid) ) {ON_QSORT_SWAP(lo,mid);}
    if ( ON_QSORT_GT(lo,hi)  ) {ON_QSORT_SWAP(lo,hi);}
    if ( ON_QSORT_GT(mid,hi) ) {ON_QSORT_SWAP(mid,hi);}

    /* We now wish to partition the array into three pieces, one consisting
        of elements <= partition element, one of elements equal to the
        partition element, and one of elements > than it.  This is done
        below; comments indicate conditions established at every step. */

    loguy = lo;
    higuy = hi;

    /* Note that higuy decreases and loguy increases on every iteration,
        so loop must terminate. */
    for (;;)
    {
      /* lo <= loguy < hi, lo < higuy <= hi,
          A[i] <= A[mid] for lo <= i <= loguy,
          A[i] > A[mid] for higuy <= i < hi,
          A[hi] >= A[mid] */

      /* The doubled loop is to avoid calling comp(mid,mid), since some
          existing comparison funcs don't work when passed the same
          value for both pointers. */

      if (mid > loguy) 
      {
          do  {
              loguy++;
          } while (loguy < mid && ON_QSORT_LE(loguy,mid));
      }
      if (mid <= loguy) 
      {
          do  {
              loguy++;
          } while (loguy <= hi && ON_QSORT_LE(loguy,mid));
      }

      /* lo < loguy <= hi+1, A[i] <= A[mid] for lo <= i < loguy,
          either loguy > hi or A[loguy] > A[mid] */

      do  {
          higuy--;
      } while (higuy > mid && ON_QSORT_GT(higuy,mid));

      /* lo <= higuy < hi, A[i] > A[mid] for higuy < i < hi,
          either higuy == lo or A[higuy] <= A[mid] */

      if (higuy < loguy)
          break;

      /* if loguy > hi or higuy == lo, then we would have exited, so
          A[loguy] > A[mid], A[higuy] <= A[mid],
          loguy <= hi, higuy > lo */

      ON_QSORT_SWAP(loguy,higuy);

      /* If the partition element was moved, follow it.  Only need
          to check for mid == higuy, since before the swap,
          A[loguy] > A[mid] implies loguy != mid. */

      if (mid == higuy)
          mid = loguy;

      /* A[loguy] <= A[mid], A[higuy] > A[mid]; so condition at top
          of loop is re-established */
    }

    /*     A[i] <= A[mid] for lo <= i < loguy,
            A[i] > A[mid] for higuy < i < hi,
            A[hi] >= A[mid]
            higuy < loguy
        implying:
            higuy == loguy-1
            or higuy == hi - 1, loguy == hi + 1, A[hi] == A[mid] */

    /* Find adjacent elements equal to the partition element.  The
        doubled loop is to avoid calling comp(mid,mid), since some
        existing comparison funcs don't work when passed the same value
        for both pointers. */

    higuy++;
    if (mid < higuy) {
        do  {
            higuy--;
        } while (higuy > mid && ON_QSORT_EQ(higuy,mid));
    }
    if (mid >= higuy) {
        do  {
            higuy--;
        } while (higuy > lo && ON_QSORT_EQ(higuy,mid));
    }

    /* OK, now we have the following:
          higuy < loguy
          lo <= higuy <= hi
          A[i]  <= A[mid] for lo <= i <= higuy
          A[i]  == A[mid] for higuy < i < loguy
          A[i]  >  A[mid] for loguy <= i < hi
          A[hi] >= A[mid] */

    /* We've finished the partition, now we want to sort the subarrays
        [lo, higuy] and [loguy, hi].
        We do the smaller one first to minimize stack usage.
        We only sort arrays of length 2 or more.*/

    if ( higuy - lo >= hi - loguy ) {
        if (lo < higuy) {
            lostk[stkptr] = lo;
            histk[stkptr] = higuy;
            ++stkptr;
        }                           /* save big recursion for later */

        if (loguy < hi) {
            lo = loguy;
            goto recurse;           /* do small recursion */
        }
    }
    else {
        if (loguy < hi) {
            lostk[stkptr] = loguy;
            histk[stkptr] = hi;
            ++stkptr;               /* save big recursion for later */
        }

        if (lo < higuy) {
            hi = higuy;
            goto recurse;           /* do small recursion */
        }
    }
  }

  /* We have sorted the array, except for any pending sorts on the stack.
      Check if there are any, and do them. */

  --stkptr;
  if (stkptr >= 0) {
      lo = lostk[stkptr];
      hi = histk[stkptr];
      goto recurse;           /* pop subarray from stack */
  }
  else
      return;                 /* all subarrays done */
}

#undef ON_QSORT_GT
#undef ON_QSORT_LE
#undef ON_QSORT_EQ
#undef ON_QSORT_SWAP
#undef ON_QSORT_CUTOFF
#undef ON_QSORT_STKSIZ
