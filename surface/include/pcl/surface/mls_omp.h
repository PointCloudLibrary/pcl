/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: mls_omp.h 5823 2012-06-01 20:25:35Z aichim $
 *
 */

#ifndef PCL_MLS_OMP_H_
#define PCL_MLS_OMP_H_

#include <pcl/surface/mls.h>

namespace pcl
{
  /** \brief MovingLeastSquaresOMP represent an OpenMP implementation of the MLS (Moving Least Squares) algorithm for 
    * data smoothing and improved normal estimation.
    * \author Radu B. Rusu
    * \ingroup surface
    */
  template <typename PointInT, typename PointOutT>
  class MovingLeastSquaresOMP : public MovingLeastSquares<PointInT, PointOutT>
  {
    using MovingLeastSquares<PointInT, PointOutT>::input_;
    using MovingLeastSquares<PointInT, PointOutT>::indices_;
    using MovingLeastSquares<PointInT, PointOutT>::fake_indices_;
    using MovingLeastSquares<PointInT, PointOutT>::initCompute;
    using MovingLeastSquares<PointInT, PointOutT>::deinitCompute;
    using MovingLeastSquares<PointInT, PointOutT>::nr_coeff_;
    using MovingLeastSquares<PointInT, PointOutT>::order_;
    using MovingLeastSquares<PointInT, PointOutT>::normals_;
    using MovingLeastSquares<PointInT, PointOutT>::upsample_method_;
    using MovingLeastSquares<PointInT, PointOutT>::voxel_size_;
    using MovingLeastSquares<PointInT, PointOutT>::dilation_iteration_num_;
    using MovingLeastSquares<PointInT, PointOutT>::tree_;
    using MovingLeastSquares<PointInT, PointOutT>::mls_results_;
    using MovingLeastSquares<PointInT, PointOutT>::search_radius_;
    using MovingLeastSquares<PointInT, PointOutT>::compute_normals_;
    using MovingLeastSquares<PointInT, PointOutT>::searchForNeighbors;
      
    typedef typename MovingLeastSquares<PointInT, PointOutT>::PointCloudIn PointCloudIn;
    typedef typename MovingLeastSquares<PointInT, PointOutT>::PointCloudOut PointCloudOut;
    typedef typename MovingLeastSquares<PointInT, PointOutT>::NormalCloud NormalCloud;
    typedef typename MovingLeastSquares<PointInT, PointOutT>::MLSVoxelGrid MLSVoxelGrid;

    public:
      /** \brief Empty constructor. */
      MovingLeastSquaresOMP () : threads_ (1)
      {};

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      MovingLeastSquaresOMP (unsigned int nr_threads) : threads_ (0)
      {
        setNumberOfThreads (nr_threads);
      }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      inline void 
      setNumberOfThreads (unsigned int nr_threads)
      { 
        if (nr_threads == 0)
          nr_threads = 1;
        threads_ = nr_threads; 
      }

    private:
      /** \brief The number of threads the scheduler should use. */
      unsigned int threads_;

      /** \brief Abstract surface reconstruction method. 
        * \param[out] output the result of the reconstruction 
        */
      virtual void 
      performProcessing (PointCloudOut &output);

      /** \brief Abstract class get name method. */
      std::string getClassName () const { return ("MovingLeastSquaresOMP"); }
  };
}

#endif  //#ifndef PCL_MLS_H_
