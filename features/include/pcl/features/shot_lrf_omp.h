/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id$
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/features/shot_lrf.h>

namespace pcl
{
  /** \brief SHOTLocalReferenceFrameEstimation estimates the Local Reference Frame used in the calculation
    * of the (SHOT) descriptor.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template<typename PointInT, typename PointOutT = ReferenceFrame>
  class SHOTLocalReferenceFrameEstimationOMP : public SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<SHOTLocalReferenceFrameEstimationOMP<PointInT, PointOutT> >;
      using ConstPtr = shared_ptr<const SHOTLocalReferenceFrameEstimationOMP<PointInT, PointOutT> >;
      /** \brief Constructor */
    SHOTLocalReferenceFrameEstimationOMP ()
      {
        feature_name_ = "SHOTLocalReferenceFrameEstimationOMP";

        setNumberOfThreads(0);
      }

    /** \brief Empty destructor */
    ~SHOTLocalReferenceFrameEstimationOMP () override = default;

    /** \brief Initialize the scheduler and set the number of threads to use.
     * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
     */
    void
    setNumberOfThreads (unsigned int nr_threads = 0);

    protected:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      //using Feature<PointInT, PointOutT>::searchForNeighbors;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF;
      using PointCloudIn = typename Feature<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;

      /** \brief Feature estimation method.
        * \param[out] output the resultant features
        */
      void
      computeFeature (PointCloudOut &output) override;

      /** \brief The number of threads the scheduler should use. */
      unsigned int threads_;

  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/shot_lrf_omp.hpp>
#endif
