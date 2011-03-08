/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: normal_3d_tbb.h 35659 2011-02-01 05:57:51Z rusu $
 *
 */

#ifndef PCL_NORMAL_3D_TBB_H_
#define PCL_NORMAL_3D_TBB_H_

//#include "pcl/pcl_config.h"
//#if defined(HAVE_TBB)

#include <pcl/features/normal_3d.h>

// TBB includes
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/task_scheduler_init.h>

namespace pcl
{
  template <typename PointInT, typename PointOutT> class TBB_NormalEstimationTBB;

  /** \brief @b NormalEstimationTBB estimates local surface properties at each 3D point, such as surface normals and
    * curvatures, in parallel, using Intel's Threading Building Blocks library.
    * \author Radu Bogdan Rusu
    */
  template <typename PointInT, typename PointOutT>
  class NormalEstimationTBB: public NormalEstimation<PointInT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;

      typedef typename NormalEstimation<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef boost::shared_ptr <Feature<PointInT, PointOutT> > FeaturePtr;

      /** \brief Empty constructor. */
      NormalEstimationTBB ()
      {
        scheduler_.initialize (tbb::task_scheduler_init::automatic);
        feature_name_ = "NormalEstimationTBB";
      }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      NormalEstimationTBB (int nr_threads)
      {
        setNumberOfThreads (nr_threads);
        feature_name_ = "NormalEstimationTBB";
      }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      inline void setNumberOfThreads (int nr_threads) { scheduler_.initialize (nr_threads); }

      /** \brief Empty destructor. */
      virtual ~NormalEstimationTBB ()
      {
        scheduler_.terminate ();
      }

    private:

      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      inline void
      computeFeature (PointCloudOut &output)
      {
        tbb::parallel_for (tbb::blocked_range <size_t> (0, indices_->size ()),
                           TBB_NormalEstimationTBB<PointInT, PointOutT> (this, output));
      }

    private:
      /** \brief The TBB scheduler. */
      tbb::task_scheduler_init scheduler_;
  };

  template <typename PointInT, typename PointOutT>
  class TBB_NormalEstimationTBB
  {
    typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    NormalEstimationTBB<PointInT, PointOutT> *feature_;
    PointCloudOut &output_;

    public:
      /** \brief Method holding the actual computational loop. */
      void operator () (const tbb::blocked_range <size_t> &r) const;

      /** \brief Empty constructor. */
      TBB_NormalEstimationTBB (NormalEstimationTBB<PointInT, PointOutT> *feature, PointCloudOut &output): feature_ (feature), output_ (output) {}
  };
}

//#endif  // HAVE_TBB

#endif  //#ifndef PCL_NORMAL_3D_TBB_H_


