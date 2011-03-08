/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_DISTANCE_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_DISTANCE_H_

#include <pcl/registration/correspondence_rejection.h>

namespace pcl
{
  namespace registration
  {

    /**
     * @b CorrespondenceRejectorDistance implements a simple correspondence
     * rejection method based on thresholding the distances between the
     * correspondences.
     * \author Dirk Holz
     */
    class CorrespondenceRejectorDistance: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

    public:
      CorrespondenceRejectorDistance() : max_distance_(std::numeric_limits<float>::max())
      {
        rejection_name_ = "CorrespondenceRejectorDistance";
      }

      inline void getCorrespondences(const pcl::registration::Correspondences& original_correspondences, pcl::registration::Correspondences& remaining_correspondences);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the maximum distance used for thresholding in correspondence rejection.
       * \param distance Distance to be used as maximum distance between correspondences. Correspondences with larger distance are rejected.
       * */
      virtual inline void setMaximumDistance(float distance) { max_distance_ = distance; };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the maximum distance used for thresholding in correspondence rejection. */
      inline float getMaxmimumDistance() { return max_distance_; };

    protected:

      void applyRejection(pcl::registration::Correspondences &correspondences);

      float max_distance_;
    };

  }
}

#include "pcl/registration/impl/correspondence_rejection_distance.hpp"

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_DISTANCE_H_ */
