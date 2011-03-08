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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_SORTING_H_
#define PCL_REGISTRATION_CORRESPONDENCE_SORTING_H_

#include <pcl/registration/correspondence_types.h>

namespace pcl
{
  namespace registration
  {

    /** @b sortCorrespondencesByQueryIndex : a functor for sorting correspondences by query index
     * \author Dirk Holz
     */
    struct sortCorrespondencesByQueryIndex : public std::binary_function<pcl::registration::Correspondence, pcl::registration::Correspondence, bool>
    {
      bool operator()( pcl::registration::Correspondence a, pcl::registration::Correspondence b)
      {
        return (a.indexQuery < b.indexQuery);
      }
    };

    /** @b sortCorrespondencesByMatchIndex : a functor for sorting correspondences by match index
     * \author Dirk Holz
     */
    struct sortCorrespondencesByMatchIndex : public std::binary_function<pcl::registration::Correspondence, pcl::registration::Correspondence, bool>
    {
      bool operator()( pcl::registration::Correspondence a, pcl::registration::Correspondence b)
      {
        return (a.indexMatch < b.indexMatch);
      }
    };

    /** @b sortCorrespondencesByDistance : a functor for sorting correspondences by distance
     * \author Dirk Holz
     */
    struct sortCorrespondencesByDistance : public std::binary_function<pcl::registration::Correspondence, pcl::registration::Correspondence, bool>
    {
      bool operator()( pcl::registration::Correspondence a, pcl::registration::Correspondence b)
      {
        return (a.distance < b.distance);
      }
    };

    /** @b sortCorrespondencesByQueryIndexAndDistance : a functor for sorting correspondences by query index _and_ distance
     * \author Dirk Holz
     */
    struct sortCorrespondencesByQueryIndexAndDistance : public std::binary_function<pcl::registration::Correspondence, pcl::registration::Correspondence, bool>
    {
      bool operator()( pcl::registration::Correspondence a, pcl::registration::Correspondence b)
      {
        if (a.indexQuery < b.indexQuery)
          return true;
        else if ( (a.indexQuery == b.indexQuery) && (a.distance < b.distance) )
          return true;
        return false;
      }
    };

    /** @b sortCorrespondencesByMatchIndexAndDistance : a functor for sorting correspondences by match index _and_ distance
     * \author Dirk Holz
     */
    struct sortCorrespondencesByMatchIndexAndDistance : public std::binary_function<pcl::registration::Correspondence, pcl::registration::Correspondence, bool>
    {
      bool operator()( pcl::registration::Correspondence a, pcl::registration::Correspondence b)
      {
        if (a.indexMatch < b.indexMatch)
          return true;
        else if ( (a.indexMatch == b.indexMatch) && (a.distance < b.distance) )
          return true;
        return false;
      }
    };


  }
}

#endif /* PCL_REGISTRATION_CORRESPONDENCE_SORTING_H_ */
