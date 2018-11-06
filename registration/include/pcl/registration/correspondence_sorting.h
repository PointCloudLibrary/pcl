/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_REGISTRATION_CORRESPONDENCE_SORTING_H_
#define PCL_REGISTRATION_CORRESPONDENCE_SORTING_H_

#if defined __GNUC__
#  pragma GCC system_header
#endif

#include <pcl/registration/correspondence_types.h>

namespace pcl
{
  namespace registration
  {
    /** @b sortCorrespondencesByQueryIndex : a functor for sorting correspondences by query index
      * \author Dirk Holz
      * \ingroup registration
      */
    struct sortCorrespondencesByQueryIndex
    {
      typedef pcl::Correspondence first_argument_type;
      typedef pcl::Correspondence second_argument_type;
      typedef bool result_type;
      bool
      operator()( pcl::Correspondence a, pcl::Correspondence b)
      {
        return (a.index_query < b.index_query);
      }
    };

    /** @b sortCorrespondencesByMatchIndex : a functor for sorting correspondences by match index
      * \author Dirk Holz
      * \ingroup registration
      */
    struct sortCorrespondencesByMatchIndex
    {
      typedef pcl::Correspondence first_argument_type;
      typedef pcl::Correspondence second_argument_type;
      typedef bool result_type;
      bool 
      operator()( pcl::Correspondence a, pcl::Correspondence b)
      {
        return (a.index_match < b.index_match);
      }
    };

    /** @b sortCorrespondencesByDistance : a functor for sorting correspondences by distance
      * \author Dirk Holz
      * \ingroup registration
      */
    struct sortCorrespondencesByDistance
    {
      typedef pcl::Correspondence first_argument_type;
      typedef pcl::Correspondence second_argument_type;
      typedef bool result_type;
      bool 
      operator()( pcl::Correspondence a, pcl::Correspondence b)
      {
        return (a.distance < b.distance);
      }
    };

    /** @b sortCorrespondencesByQueryIndexAndDistance : a functor for sorting correspondences by query index _and_ distance
      * \author Dirk Holz
      * \ingroup registration
      */
    struct sortCorrespondencesByQueryIndexAndDistance
    {
      typedef pcl::Correspondence first_argument_type;
      typedef pcl::Correspondence second_argument_type;
      typedef bool result_type;
      inline bool 
      operator()( pcl::Correspondence a, pcl::Correspondence b)
      {
        if (a.index_query < b.index_query)
          return (true);
        else if ( (a.index_query == b.index_query) && (a.distance < b.distance) )
          return (true);
        return (false);
      }
    };

    /** @b sortCorrespondencesByMatchIndexAndDistance : a functor for sorting correspondences by match index _and_ distance
      * \author Dirk Holz
      * \ingroup registration
      */
    struct sortCorrespondencesByMatchIndexAndDistance
    {
      typedef pcl::Correspondence first_argument_type;
      typedef pcl::Correspondence second_argument_type;
      typedef bool result_type;
      inline bool 
      operator()( pcl::Correspondence a, pcl::Correspondence b)
      {
        if (a.index_match < b.index_match)
          return (true);
        else if ( (a.index_match == b.index_match) && (a.distance < b.distance) )
          return (true);
        return (false);
      }
    };

  }
}

#endif /* PCL_REGISTRATION_CORRESPONDENCE_SORTING_H_ */
