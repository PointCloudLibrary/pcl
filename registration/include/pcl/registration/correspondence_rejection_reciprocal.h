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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_RECIPROCAL_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_RECIPROCAL_H_

#include <pcl/registration/correspondence_rejection.h>

namespace pcl
{
  namespace registration
  {

    /**
     * @b CorrespondenceRejectorReciprocal implements a reciprocal
     * correspondence rejection method for ICP-like registration algorithms.
     * Given the correspondences between target and source (reciprocal search),
     * it removes all correspondences, where the target point (match index) has
     * a different corresponding point in the source than the query point
     * (query index) and the difference in distances exceeds a given threshold.
     * \author Dirk Holz
     */
    class CorrespondenceRejectorReciprocal: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

    public:
      CorrespondenceRejectorReciprocal() : distance_threshold_(std::numeric_limits<float>::max())
      {
        rejection_name_ = "CorrespondenceRejectorReciprocal";
      }

      inline void getCorrespondences(const pcl::registration::Correspondences& original_correspondences, pcl::registration::Correspondences& remaining_correspondences);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set maximally allowable distance between a correspondence and its reciprocal correspondence. */
      virtual inline void setDistanceThreshold(float threshold) { distance_threshold_ = threshold; };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set maximally allowable distance between a correspondence and its reciprocal correspondence. */
      inline float getDistanceThreshold() { return distance_threshold_; };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Provide a pointer to the vector of the input correspondences.
        * \param correspondences the const boost shared pointer to a std::vector of correspondences
        */
      virtual inline void setReciprocalCorrespondences(const CorrespondencesConstPtr &correspondences) { reciprocal_correspondences_ = correspondences; };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a pointer to the vector of the input correspondences. */
      inline CorrespondencesConstPtr getReciprocalCorrespondences() { return reciprocal_correspondences_; };


    protected:

      void applyRejection(pcl::registration::Correspondences &correspondences);

      float distance_threshold_;

      /** \brief The reciprocal correspondences. */
      CorrespondencesConstPtr reciprocal_correspondences_;
    };

  }
}

#include "pcl/registration/impl/correspondence_rejection_reciprocal.hpp"

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_RECIPROCAL_H_ */
