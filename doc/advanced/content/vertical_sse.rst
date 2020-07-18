.. _vertical_sse:

Vertical SSE for PCL2.0
-----------------------

.. note::

    All code is available from
    https://kforge.ros.org/projects/mihelich/services/pcl_simd/.

Representing point data
=======================

In PCL currently, points are stored with their fields interleaved. For the
simplest :pcl:`PointXYZ <pcl::PointXYZ>` type, this looks like::

    XYZ_XYZ_XYZ_XYZ_ ...

where ``_`` denotes an extra padding ``float`` so that each point is 16-byte
aligned. Operating on ``XYZ_`` data efficiently often requires the use of
*horizontal* SSE instructions, which perform computations using multiple
elements of the same SSE register.

This representation is also known as *Array-of-Structures* (AoS). ``PointXYZ``
is defined as a struct, and all fields for an individual point are stored
together in memory.

Instead a *vertical* representation, aka *Structure-of-Arrays* (SoA), can be
used::

    XXXXXXXX ...
    YYYYYYYY ...
    ZZZZZZZZ ...

This layout fits traditional vertical SSE processing better. Most arithmetic
SSE instructions are binary operations on corresponding elements of two SSE
registers.

Why does PCL use AoS?
=====================

PCL's use of AoS, normally non-optimal, does have its logic. In PCL, frequently
we wish to process only some (indexed / valid) subset of a point cloud. Besides
dense processing of all points, we then have two other cases.

Indexed subsets
^^^^^^^^^^^^^^^

PCL operators routinely provide a :pcl:`setIndices()
<pcl::PCLBase::setIndices>` method, ordering them to use only certain points
identified by index. With the AoS representation, each individual point can be
used in an SSE register via a simple aligned load. Indexed access therefore
does not much complicate an SSE-optimized implementation.

Vertical SSE (in the dense case) processes four adjacent points simultaneously,
and indexed access breaks the adjacency requirement. Instead of an aligned
load, the implementation must "gather" the data for the next four indexed
points (spread out in memory).

Organized point clouds
^^^^^^^^^^^^^^^^^^^^^^

PCL permits point clouds with missing data. For imager-based 3D sensors, this
allows point clouds to retain their 2D structure, making it trivial to identify
nearby points. Invalid points have each field set to NaN, so that it is clear
when invalid data is accidentally used in a computation.

Handling invalid points in PCL (with AoS) is again rather simple. For each
point, check if ``X`` is NaN; if so, ignore it.

The SoA situation is much more complicated. Since we operate on four points at
a time, we have to check if any of the four points are invalid. If so, it
becomes very tricky to use SSE at all without destroying our result. Masking
tricks are possible, but imply some overhead over the simple dense code.

Horizontal or vertical?
=======================

Both representations have pros and cons.

**Horizontal**

* Pros

  * More intuitive, easier to write code for
  * Handling indexed subsets is simple - can still use aligned loads
  * Handling NaNs also simple

* Cons

  * Clearly slower at dense processing
  * Waste space and computation on padding elements

**Vertical**

* Pros

  * Clearly faster at dense processing
  * No wasted space - only 3/4 as many loads required
  * No wasted computation
  * May have less loop overhead, since you process 4 points per iteration
    instead of 1

* Cons

  * Less intuitive
  * Indexed subsets require gathering data for non-adjacent points
  * Handling NaNs is complicated

Data structures
===============

For benchmarking, we define two very simple point cloud representations:

.. code-block:: cpp

    // Array-of-Structures
    struct AOS
    {
      float x;
      float y;
      float z;
      float h;
    };

    // Structure-of-Arrays
    struct SOA
    {
      float* x;
      float* y;
      float* z;
      std::size_t size;
    };

Computations considered
=======================

We benchmark two basic operations:

* Compute the dot product of every point in a cloud with a given point
* Compute the centroid of a point cloud

For both operations, we implemented several versions covering the space of:

* Horizontal (AoS) or vertical (SoA)
* Dense or indexed
* SSE instruction set

Representative examples are listed below.

Dot product
^^^^^^^^^^^

Vertical (SoA), SSE2-optimized:

.. code-block:: cpp

    void dotSSE2 (const SOA& vectors, const AOS& vector,
                  float* result, unsigned long size)
    {
      float x = vector.x, y = vector.y, z = vector.z;

      // Broadcast X, Y, Z of constant vector into 3 SSE registers
      __m128 vX  = _mm_set_ps1(x);
      __m128 vY  = _mm_set_ps1(y);
      __m128 vZ  = _mm_set_ps1(z);
      __m128 X, Y, Z;

      unsigned i = 0;
      for ( ; i < size - 3; i += 4)
      {
        // Load data for next 4 points
        X = _mm_load_ps (vectors.x + i);
        Y = _mm_load_ps (vectors.y + i);
        Z = _mm_load_ps (vectors.z + i);

        // Compute X*X'+Y*Y'+Z*Z' for each point
        X = _mm_mul_ps (X, vX);
        Y = _mm_mul_ps (Y, vY);
        Z = _mm_mul_ps (Z, vZ);
        X = _mm_add_ps (X, Y);
        X = _mm_add_ps (X, Z);

        // Store results
        _mm_store_ps(result + i, X);
      }

      // Handle any leftovers at the end
      for ( ; i < size; ++i)
      {
        result[i] = vectors.x[i]*x + vectors.y[i]*y + vectors.z[i]*z;
      }
    }

Horizontal (AoS), SSE4.1-optimized (with horizontal DPPS instruction):

.. code-block:: cpp

    void dotSSE4_1 (const AOS* vectors, const AOS& vector,
                    float* result, unsigned long size)
    {
      // Load constant vector into an SSE register
      __m128 vec = _mm_load_ps ((const float*) &vector);
      __m128 XYZH;

      // Set mask to ignore the padding elements
      const int mask = 123;
      for (unsigned i = 0; i < size; ++i)
      {
        // Load next point
        XYZH = _mm_load_ps ((const float*)(vectors + i));

        // Dot product from SSE4.1
        XYZH = _mm_dp_ps (XYZH, vec, mask);

        // Store single result (the bottom register element)
        _mm_store_ss (&(result [i]), XYZH);
      }
    }

Centroid
^^^^^^^^

Vertical (SoA), SSE2-optimized:

.. code-block:: cpp

    void centroidSSE2 (const SOA& vectors, AOS& result, std::size_t size)
    {
      __m128 X_sum = _mm_setzero_ps();
      __m128 Y_sum = _mm_setzero_ps();
      __m128 Z_sum = _mm_setzero_ps();
      __m128 X, Y, Z;

      std::size_t i = 0;
      for ( ; i < size - 3; i += 4)
      {
        // Load next 4 points
        X = _mm_load_ps (vectors.x + i);
        Y = _mm_load_ps (vectors.y + i);
        Z = _mm_load_ps (vectors.z + i);

        // Accumulate 4 sums in each dimension
        X_sum = _mm_add_ps(X_sum, X);
        Y_sum = _mm_add_ps(Y_sum, Y);
        Z_sum = _mm_add_ps(Z_sum, Z);
      }

      // Horizontal adds (HADD from SSE3 could help slightly)
      float* pX = reinterpret_cast<float*>(&X_sum);
      float* pY = reinterpret_cast<float*>(&Y_sum);
      float* pZ = reinterpret_cast<float*>(&Z_sum);
      result.x = pX[0] + pX[1] + pX[2] + pX[3];
      result.y = pY[0] + pY[1] + pY[2] + pY[3];
      result.z = pZ[0] + pZ[1] + pZ[2] + pZ[3];

      // Leftover points
      for ( ; i < size; ++i)
      {
        result.x += vectors.x[i];
        result.y += vectors.y[i];
        result.z += vectors.z[i];
      }

      // Average
      float inv_size = 1.0f / size;
      result.x *= inv_size;
      result.y *= inv_size;
      result.z *= inv_size;
    }

Horizontal (AoS), SSE2-optimized:

.. code-block:: cpp

    void centroidSSE2 (const AOS* vectors, AOS& result, std::size_t size)
    {
      __m128 sum = _mm_setzero_ps();

      for (unsigned i = 0; i < size; ++i)
      {
        __m128 XYZH = _mm_load_ps ((const float*)(vectors + i));
        sum = _mm_add_ps(sum, XYZH);
      }
      _mm_store_ps((float*)&result, sum);

      float inv_size = 1.0f / size;
      result.x *= inv_size;
      result.y *= inv_size;
      result.z *= inv_size;
    }

Indexed
^^^^^^^

When using point indices, the vertical implementation can no longer use aligned
loads. Instead it's best to use the ``_mm_set_ps`` intrinsic to gather the next
four points.

Vertical (SoA) dot product, SSE2-optimized:

.. code-block:: cpp

    void dotIndexedSSE2 (const SOA& vectors, const AOS& vector,
                         const int* indices, float* result, unsigned long size)
    {
      float x = vector.x, y = vector.y, z = vector.z;

      __m128 vX  = _mm_set_ps1(x);
      __m128 vY  = _mm_set_ps1(y);
      __m128 vZ  = _mm_set_ps1(z);
      __m128 X, Y, Z;

      unsigned i = 0;
      for ( ; i < size - 3; i += 4)
      {
        int i0 = indices[i + 0];
        int i1 = indices[i + 1];
        int i2 = indices[i + 2];
        int i3 = indices[i + 3];

        // Gather next four indexed points
        X = _mm_set_ps(vectors.x[i3], vectors.x[i2], vectors.x[i1], vectors.x[i0]);
        Y = _mm_set_ps(vectors.y[i3], vectors.y[i2], vectors.y[i1], vectors.y[i0]);
        Z = _mm_set_ps(vectors.z[i3], vectors.z[i2], vectors.z[i1], vectors.z[i0]);

        // Computation
        X = _mm_mul_ps (X, vX);
        Y = _mm_mul_ps (Y, vY);
        Z = _mm_mul_ps (Z, vZ);
        X = _mm_add_ps (X, Y);
        X = _mm_add_ps (X, Z);

        // Store result
        _mm_store_ps(result + i, X);
      }

      for ( ; i < size; ++i)
      {
        int idx = indices[i];
        result[i] = vectors.x[idx]*x + vectors.y[idx]*x + vectors.z[idx]*z;
      }
    }

Benchmarks (random data)
========================

The test point cloud is randomly generated, 640x480, dense. Each operation is
repeated 1000 times.

For indexed tests, the indices list every 4th point. More random index patterns
would change execution time by affecting caching and prefetching, but I'd
expect such effects to be similar for horizontal and vertical code.

"Scalar" code uses no vector instructions, otherwise the instruction set is
listed. A trailing u# means the code was unrolled by factor #.

Dot product
^^^^^^^^^^^

Dense
"""""

::

    Horizontal (AOS)
      Scalar:   0.621674 seconds 
      SSE2:     0.756300 seconds 
      SSE4.1:   0.532441 seconds 
      SSE4.1u4: 0.476841 seconds 
    Vertical (SOA)
      Scalar:   0.519625 seconds 
      SSE2:     0.215499 seconds

The vertical SSE2 code is the clear winner, more than twice as fast as
horizontal code even with the special horizontal dot product from SSE4.1.

On the first i7 I used, horizontal SSE4.1 was actually the *slowest*
implementation. Unrolling it x4 helped significantly, although it was still
much worse than vertical SSE2. I attributed this to the very high latency of
the DPPS instruction; it takes 11 cycles before the result can be stored.
Unrolling helps hide the latency by providing more computation to do during
that time. I don't know why the results from my office i7 (shown above) are so
different.

Indexed
"""""""

::

    Horizontal (AOS)
      Scalar:   0.271768 seconds
      SSE2:     0.276114 seconds
      SSE4.1:   0.259613 seconds
    Vertical (SOA)
      Scalar:   0.193394 seconds
      SSE2:     0.177262 seconds

SSE optimization actually gives meager benefits in both the horizontal and
vertical cases. However vertical SSE2 is still the winner.

Centroid
^^^^^^^^

The story for centroid is similar; vertical SSE2 is fastest, significantly so
for dense data.

Dense
"""""

::

    Horizontal (AOS)
      Scalar:  0.628597 seconds 
      SSE2:    0.326645 seconds 
      SSE2u2:  0.247539 seconds 
      SSE2u4:  0.236474 seconds 
    Vertical (SOA)
      Scalar:  0.711040 seconds 
      SSE2:    0.149806 seconds 

Indexed
"""""""

::

    Horizontal (AOS)
      Scalar:  0.256237 seconds 
      SSE2:    0.195724 seconds 
    Vertical (SOA)
      Scalar:  0.194030 seconds 
      SSE2:    0.166639 seconds 

Vertical SSE for organized point clouds
=======================================

We still need a way to effectively use vertical SSE for organized point clouds
(containing NaNs). A promising approach is to compute a *run-length encoding*
(RLE) of the valid points as a preprocessing step. The data structure is very
simple:

.. code-block:: cpp

    struct RlePair
    {
      std::size_t good;
      std::size_t skip;
    };
    typedef std::vector<RlePair> RLE;

The RLE counts the length of alternating runs of valid and invalid points. Once
computed, it allows us to process only valid points without explicitly checking
each one for NaNs. In fact, operations become ``O(#valid points)`` instead of
``O(#total points)``, which can itself be a win if many points are invalid.

In real scenes, valid points are clustered together (into objects), so valid
(and invalid) runs should be lengthy on average. A long run of valid points can
be split into <4 beginning points, <4 final points, and a run of aligned, valid
point data which can be safely processed with vertical SSE.

Abstracting point iteration
===========================

We are still left with three distinct cases for processing point clouds,
requiring different methods of iterating over point data:

* Dense (no NaNs)
* Indexed
* Organized (contains NaNs)

Writing and maintaining three copies of each PCL algorithm is a huge burden.
The RLE for organized data in particular imposes a relatively complicated
iteration method. Ideally we should be able to write the computational core of
an algorithm only once, and have it work efficiently in each of the three cases.

Currently PCL does not meet this goal. In fact, core algorithms tend to have
four near-identical implementations:

* Dense
* Dense indexed
* Organized
* Organized indexed

I think it's unnecessary to distinguish between "dense indexed" and "organized
indexed", if we require that indices point to valid data.

Writing algorithms as computational kernels
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As an experiment, I rewrote the vertical centroid as a *kernel* class. This
implements only the computation, without worrying about the memory layout of
the whole cloud:

.. code-block:: cpp

    struct CentroidKernel
    {
      // State
      float x_sum, y_sum, z_sum;
      __m128 X_sum, Y_sum, Z_sum;
      std::size_t count;
      AOS result;

      void init()
      {
        // Initialization
        x_sum = y_sum = z_sum = 0.0f;
        X_sum = _mm_setzero_ps();
        Y_sum = _mm_setzero_ps();
        Z_sum = _mm_setzero_ps();
        count = 0;
      }

      // Scalar operator
      inline void operator() (float x, float y, float z)
      {
        x_sum += x;
        y_sum += y;
        z_sum += z;
        ++count;
      }

      // SIMD operator
      inline void operator() (__m128 X, __m128 Y, __m128 Z)
      {
        X_sum = _mm_add_ps(X_sum, X);
        Y_sum = _mm_add_ps(Y_sum, Y);
        Z_sum = _mm_add_ps(Z_sum, Z);
        count += 4;
      }

      void reduce()
      {
        float* pX = reinterpret_cast<float*>(&X_sum);
        float* pY = reinterpret_cast<float*>(&Y_sum);
        float* pZ = reinterpret_cast<float*>(&Z_sum);
        result.x = pX[0] + pX[1] + pX[2] + pX[3] + x_sum;
        result.y = pY[0] + pY[1] + pY[2] + pY[3] + y_sum;
        result.z = pZ[0] + pZ[1] + pZ[2] + pZ[3] + z_sum;

        float inv_count = 1.0f / count;
        result.x *= inv_count;
        result.y *= inv_count;
        result.z *= inv_count;
      }
    };

Kernel applicators
^^^^^^^^^^^^^^^^^^

We can then define *applicator* functions that apply a kernel to a particular
case of point cloud. The dense version simply uses aligned loads:

.. code-block:: cpp

    template <typename Kernel>
    void applyDense (Kernel& kernel, const SOA& pts)
    {
      kernel.init();

      std::size_t i = 0;
      for ( ; i < pts.size - 3; i += 4)
      {
        __m128 X = _mm_load_ps (pts.x + i);
        __m128 Y = _mm_load_ps (pts.y + i);
        __m128 Z = _mm_load_ps (pts.z + i);

        kernel(X, Y, Z);
      }
      for ( ; i < pts.size; ++i)
      {
        kernel(pts.x[i], pts.y[i], pts.z[i]);
      }

      kernel.reduce();
    }

The indexed version performs the necessary data gathering:

.. code-block:: cpp

    template <typename Kernel>
    void applySparse (Kernel& kernel, const SOA& pts,
                      const std::vector<int>& indices)
    {
      kernel.init();

      std::size_t i = 0;
      for ( ; i < indices.size() - 3; i += 4)
      {
        int i0 = indices[i + 0];
        int i1 = indices[i + 1];
        int i2 = indices[i + 2];
        int i3 = indices[i + 3];

        // Gather next four indexed points
        __m128 X = _mm_set_ps(pts.x[i3], pts.x[i2], pts.x[i1], pts.x[i0]);
        __m128 Y = _mm_set_ps(pts.y[i3], pts.y[i2], pts.y[i1], pts.y[i0]);
        __m128 Z = _mm_set_ps(pts.z[i3], pts.z[i2], pts.z[i1], pts.z[i0]);

        kernel(X, Y, Z);
      }
      for ( ; i < indices.size(); ++i)
      {
        int idx = indices[i];
        kernel(pts.x[idx], pts.y[idx], pts.z[idx]);
      }

      kernel.reduce();
    }

The organized version is most complicated, and uses the RLE to vectorize as
much of the computation as possible:

.. code-block:: cpp

    template <typename Kernel>
    void applyOrganized (Kernel& kernel, const SOA& pts, const RLE& rle)
    {
      kernel.init();

      std::size_t i = 0;
      for (RLE::const_iterator rle_it = rle.begin(); rle_it != rle.end(); ++rle_it)
      {
        // Process current stretch of good pixels
        std::size_t good = rle_it->good;
        std::size_t skip = rle_it->skip;
        std::size_t good_end = i + good;

        // Any unaligned points at start
        std::size_t unaligned_end = std::min( (i + 3) & ~3, good_end );
        for ( ; i < unaligned_end; ++i)
          kernel(pts.x[i], pts.y[i], pts.z[i]);
        // Aligned SIMD point data
        for ( ; i + 4 <= good_end; i += 4)
        {
          __m128 X = _mm_load_ps (pts.x + i);
          __m128 Y = _mm_load_ps (pts.y + i);
          __m128 Z = _mm_load_ps (pts.z + i);

          kernel(X, Y, Z);
        }
        // <4 remaining points
        for ( ; i < good_end; ++i)
          kernel(pts.x[i], pts.y[i], pts.z[i]);

        // Skip the following stretch of NaNs
        i += skip;
      }

      kernel.reduce();
    }

The kernel + applicator combinations for the dense and indexed cases were added
to the centroid benchmark for random point data, and show identical performance
to the hand-written vertical SSE2 code.

The above code is written with simplicity in mind. The biggest improvement
would be to combine the scalar and SSE ``operator() (...)`` functions; this
could possibly be achieved by using ``Eigen::Array`` as an SSE backend (similar
to how ``Eigen::Matrix`` maps are currently used), something like:

.. code-block:: cpp

    // N can be 1 or 4
    template <int N>
    void operator() (const Eigen::Array<float, N, 1>& x,
                     const Eigen::Array<float, N, 1>& y,
                     const Eigen::Array<float, N, 1>& z);

Benchmarks (real point clouds)
==============================

Finally, we compare ``CentroidKernel`` + applicator to
``pcl::compute3DCentroid()`` for several real organized (and one dense) point
clouds.

The point clouds used are:

* `capture000X.pcd <https://github.com/PointCloudLibrary/data/tree/master/tutorials/pairwise>`_
* `table_scene_mug_stereo_textured.pcd <https://github.com/PointCloudLibrary/data/blob/master/tutorials/table_scene_lms400.pcd?raw=true>`_
* `table_scene_lms400.pcd <https://github.com/PointCloudLibrary/data/blob/master/tutorials/table_scene_lms400.pcd?raw=true>`_

``capture0001.pcd`` (organized, 640x480, 57553 NaNs)::

    PCL:    0.926901 seconds

    RLE:    0.348173 seconds
    Kernel: 0.174194 seconds

``capture0002.pcd`` (organized, 640x480, 57269 NaNs)::

    PCL:    0.931111 seconds

    RLE:    0.345437 seconds
    Kernel: 0.171373 seconds

Even if you include the RLE computation time (which could be amortized over
several operations, and perhaps optimized) in the total, the vertical kernel
beats the current PCL implementation. Discounting RLE, it's more than 5x faster.

``table_scene_mug_stereo_textured.pcd`` (organized, 640x480, 97920 NaNs)::

    PCL:    3.36001 seconds

    RLE:    0.379737 seconds
    Kernel: 0.183159 seconds

The very poor performance of PCL on the mug scene is a mystery to me. Perhaps
the larger number of NaNs has an effect?

``table_scene_lms400.pcd`` (dense, 460400 pts)::

    PCL:    0.678805 seconds

    RLE:    N/A
    Kernel: 0.242546 seconds

Conclusions
===========

For the simple operations considered here, vertical SSE is a huge win. In the
best case, this suggests that much of PCL could get at least a 3x speedup by
switching to the more SSE-friendly memory layout.

Vertical SSE presents some complications in usage and implementation for PCL,
but good solutions (RLE, kernel abstraction) are possible.

Looking at instruction sets, vertical SSE is especially advantageous both on
older and very new processors. On older processors, because it makes excellent
use of SSE2 instructions, whereas horizontal SSE may require horizontal
instructions (introduced in SSE3 and later) for good performance. On new
processors, because the latest AVX extensions expand SSE register to 256 bits,
allowing 8 floating point operations at a time instead of 4. The vertical SSE
techniques shown here trivially extend to AVX, and future instruction sets will
likely expand SSE registers even further. The upcoming AVX2 extensions add
dedicated *gather* instructions, which should improve performance with indices.

Remaining questions
===================

Are there PCL algorithms that aren't easily implementable in the proposed
kernel style?

How to handle nearest neighbor searches? These may be hard to vectorize.
