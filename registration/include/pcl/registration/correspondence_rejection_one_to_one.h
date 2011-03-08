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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_ONE_TO_ONE_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_ONE_TO_ONE_H_

#include <pcl/registration/correspondence_rejection.h>

namespace pcl
{
  namespace registration
  {

    /**
     * @b CorrespondenceRejectorOneToOne implements a correspondence
     * rejection method based on eliminating duplicate match indices in
     * the correspondences. Correspondences with the same match index are
     * removed and only the one with smallest distance between query and
     * match are kept. That is, considering match->query 1-m correspondences
     * are removed leaving only 1-1 correspondences.
     * \author Dirk Holz
     */
    class CorrespondenceRejectorOneToOne: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

    public:
      CorrespondenceRejectorOneToOne()
      {
        rejection_name_ = "CorrespondenceRejectorOneToOne";
      }

      inline void getCorrespondences(const pcl::registration::Correspondences& original_correspondences, pcl::registration::Correspondences& remaining_correspondences);


    protected:

      void applyRejection(pcl::registration::Correspondences &correspondences);

    };

  }
}

#include "pcl/registration/impl/correspondence_rejection_one_to_one.hpp"

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_ONE_TO_ONE_H_ */
