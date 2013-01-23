/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2004, Sylvain Paris and Francois Sillion

 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: fast_bilateral_omp.h 8379 2013-01-02 23:12:21Z sdmiller $
 *
 */


#ifndef PCL_FILTERS_FAST_BILATERAL_OMP_H_
#define PCL_FILTERS_FAST_BILATERAL_OMP_H_

#include <pcl/filters/filter.h>
#include <pcl/filters/fast_bilateral.h>

namespace pcl
{
  /** \brief Implementation of a fast bilateral filter for smoothing depth information in organized point clouds
   *  Based on the following paper:
   *    * Sylvain Paris and Frdo Durand
   *      "A Fast Approximation of the Bilateral Filter using a Signal Processing Approach"
   *       European Conference on Computer Vision (ECCV'06)
   *
   *  More details on the webpage: http://people.csail.mit.edu/sparis/bf/
   */
  template<typename PointT>
  class FastBilateralFilterOMP : public FastBilateralFilter<PointT>
  {
  protected:
    using FastBilateralFilter<PointT>::input_;
    using FastBilateralFilter<PointT>::sigma_s_;
    using FastBilateralFilter<PointT>::sigma_r_;
    using FastBilateralFilter<PointT>::early_division_;
    typedef typename FastBilateralFilter<PointT>::Array3D Array3D;

    typedef typename Filter<PointT>::PointCloud PointCloud;

    public:
    
      typedef boost::shared_ptr< FastBilateralFilterOMP<PointT> > Ptr;
      typedef boost::shared_ptr< const FastBilateralFilterOMP<PointT> > ConstPtr;

      /** \brief Empty constructor. */
      FastBilateralFilterOMP (unsigned int nr_threads = 0)
        : threads_ (nr_threads)
      { }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      inline void 
      setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }

      /** \brief Filter the input data and store the results into output.
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);
    
    protected:
      /** \brief The number of threads the scheduler should use. */
      unsigned int threads_;

  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/fast_bilateral_omp.hpp>
#else
#define PCL_INSTANTIATE_FastBilateralFilterOMP(T) template class PCL_EXPORTS pcl::FastBilateralFilterOMP<T>;
#endif


#endif /* PCL_FILTERS_FAST_BILATERAL_OMP_H_ */

