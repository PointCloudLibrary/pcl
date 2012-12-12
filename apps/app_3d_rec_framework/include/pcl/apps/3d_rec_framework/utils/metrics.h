/*
 * metrics.h
 *
 *  Created on: Jun 22, 2011
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_METRICS_H_
#define REC_FRAMEWORK_METRICS_H_

#include <cmath>
#include <cstdlib>

namespace Metrics
{
  using ::std::abs;

  template<typename T>
    struct Accumulator
    {
      typedef T Type;
    };

  template<>
    struct Accumulator<unsigned char>
    {
      typedef float Type;
    };
  template<>
    struct Accumulator<unsigned short>
    {
      typedef float Type;
    };
  template<>
    struct Accumulator<unsigned int>
    {
      typedef float Type;
    };
  template<>
    struct Accumulator<char>
    {
      typedef float Type;
    };
  template<>
    struct Accumulator<short>
    {
      typedef float Type;
    };
  template<>
    struct Accumulator<int>
    {
      typedef float Type;
    };

  template<class T>
    struct HistIntersectionUnionDistance
    {
      typedef T ElementType;
      typedef typename Accumulator<T>::Type ResultType;

      /**
       *  Compute a distance between two vectors using (1 - (1 + sum(min(a_i,b_i))) / (1 + sum(max(a_i, b_i))) )
       *
       *  This distance is not a valid kdtree distance, it's not dimensionwise additive
       *  and ignores worst_dist parameter.
       */

      template<typename Iterator1, typename Iterator2>
        ResultType
        operator() (Iterator1 a, Iterator2 b, size_t size, ResultType worst_dist = -1) const
        {
          (void)worst_dist;
          ResultType result = ResultType ();
          ResultType min0, min1, min2, min3;
          ResultType max0, max1, max2, max3;
          Iterator1 last = a + size;
          Iterator1 lastgroup = last - 3;

          ResultType sum_min, sum_max;
          sum_min = 0;
          sum_max = 0;

          while (a < lastgroup)
          {
            min0 = (a[0] < b[0] ? a[0] : b[0]);
            max0 = (a[0] > b[0] ? a[0] : b[0]);
            min1 = (a[1] < b[1] ? a[1] : b[1]);
            max1 = (a[1] > b[1] ? a[1] : b[1]);
            min2 = (a[2] < b[2] ? a[2] : b[2]);
            max2 = (a[2] > b[2] ? a[2] : b[2]);
            min3 = (a[3] < b[3] ? a[3] : b[3]);
            max3 = (a[3] > b[3] ? a[3] : b[3]);
            sum_min += min0 + min1 + min2 + min3;
            sum_max += max0 + max1 + max2 + max3;
            a += 4;
            b += 4;
            /*if (worst_dist>0 && result>worst_dist) {
             return result;
             }*/
          }

          while (a < last)
          {
            min0 = *a < *b ? *a : *b;
            max0 = *a > *b ? *a : *b;
            sum_min += min0;
            sum_max += max0;
            a++;
            b++;
            //std::cout << a << " " << last << std::endl;
          }

          result = static_cast<ResultType> (1.0 - ((1 + sum_min) / (1 + sum_max)));
          return result;
        }

      /* This distance functor is not dimension-wise additive, which
       * makes it an invalid kd-tree distance, not implementing the accum_dist method */
      /**
       * Partial distance, used by the kd-tree.
       */
      template<typename U, typename V>
        inline ResultType
        accum_dist (const U& a, const V& b, int dim) const
        {
          //printf("New code being used, accum_dist\n");
          ResultType min0;
          ResultType max0;
          min0 = (a < b ? a : b) + 1.0;
          max0 = (a > b ? a : b) + 1.0;
          return (1 - (min0 / max0));
          //return (a+b-2*(a<b?a:b));
        }
    };
}

#endif /* REC_FRAMEWORK_METRICS_H_ */
