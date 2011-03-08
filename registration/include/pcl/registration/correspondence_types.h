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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_TYPES_H_
#define PCL_REGISTRATION_CORRESPONDENCE_TYPES_H_

namespace pcl
{
  namespace registration
  {
    /** @b Correspondence represents point correspondences (similar to OpenCV's DMATCH)
     * \author Dirk Holz
     */
    struct Correspondence;
    // Members: int indexQuery, int indexMatch (-1 is used in case there is no match), float distance

    typedef std::vector<pcl::registration::Correspondence> Correspondences;
    typedef boost::shared_ptr<std::vector<pcl::registration::Correspondence> > CorrespondencesPtr;
    typedef boost::shared_ptr<const std::vector<pcl::registration::Correspondence> > CorrespondencesConstPtr;

    /** get mean and standard deviation of the correspondence distances */
    inline void getCorDistMeanStd(const Correspondences& correspondences, double &mean, double &stddev);

    /** get a vector containing only the query indices */
    inline void getQueryIndices(const Correspondences& correspondences, std::vector<int>& indices);

    /** get a vector containing only the match indices */
    inline void getMatchIndices(const Correspondences& correspondences, std::vector<int>& indices);

//    inline void getNonMatchingQueryIndices()
//    {
//      // Get the diference
//      std::vector<int> remaining_indices;
//      set_difference (all_indices.begin (), all_indices.end (), indices.begin (), indices.end (),
//                      inserter (remaining_indices, remaining_indices.begin ()));
//    }

  }
}

#include "pcl/registration/impl/correspondence_types.hpp"

#endif /* PCL_REGISTRATION_CORRESPONDENCE_TYPES_H_ */
