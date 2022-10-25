/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/pcl_macros.h>

#include <map>

namespace pcl
{
  namespace visualization
  {
    /** /brief Class representing 3D area picking events. */
    class PCL_EXPORTS AreaPickingEvent
    {
      public:
        AreaPickingEvent (std::map<std::string, pcl::Indices> cloud_indices)
        : cloud_indices_ (std::move(cloud_indices))
        {}

        PCL_DEPRECATED(1,16,"This constructor is deprecated!")
        AreaPickingEvent(int /*nb_points*/, const pcl::Indices& indices)
          : AreaPickingEvent ({{"",indices}}) {}

        /** \brief For situations where a whole area is selected, return the points indices.
          * \param[out] indices indices of the points under the area selected by user.
          * \return true, if the area selected by the user contains points, false otherwise
          */
        inline bool
        getPointsIndices (pcl::Indices& indices) const
        {
          if (cloud_indices_.empty())
            return (false);

          for (const auto& i : cloud_indices_)
            indices.insert(indices.cend (), i.second.cbegin (), i.second.cend ());

          return (true);
        }
        /** \brief For situations where a whole area is selected, return the names
          * of the selected point clouds.
          * \return The names of selected point clouds
          */
        inline std::vector<std::string>
        getCloudNames () const
        {
          std::vector<std::string> names;
          for (const auto& i : cloud_indices_)
            names.push_back (i.first);
          return names;
        }
        /** \brief For situations where a whole area is selected, return the points indices
          * for a given point cloud
          * \param[in] name of selected clouds.
          * \return The indices for the selected cloud.
          */
        inline Indices
        getPointsIndices (const std::string& name) const
        {
          const auto cloud = cloud_indices_.find (name);
          if(cloud == cloud_indices_.cend ())
            return Indices ();

          return cloud->second;
        }

      private:
        std::map<std::string, pcl::Indices> cloud_indices_;
    };
  } //namespace visualization
} //namespace pcl
