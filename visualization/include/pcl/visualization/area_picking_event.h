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

#include <vector>

namespace pcl
{
  namespace visualization
  {
    /** /brief Class representing 3D area picking events. */
    class PCL_EXPORTS AreaPickingEvent
    {
      public:
       PCL_DEPRECATED AreaPickingEvent (int nb_points, const Indices& indices)
        : nb_points_ (nb_points)
        , indices_ (indices)
        {cloudIndices_.insert({"all_clouds",indices});}
        
        AreaPickingEvent (int nb_points, std::map<std::string, std::vector<int>> cloudIndices)
          : nb_points_ (nb_points)
          , cloudIndices_ (cloudIndices)
        {}

        /** \brief For situations where a whole are is selected, return the points indices.
          * \param[out] indices indices of the points under the area selected by user.
          * \return true, if the area selected by the user contains points, false otherwise
          */
        inline bool
        getPointsIndices (std::vector<int>& indices) const
        {
          if (nb_points_ <= 0)
            return (false);
            for (const auto& i : cloudIndices_)
              indices.insert(indices.end(), i.second.cbegin(), i.second.cend());
          return (true);
        }
        /** \brief For situations where a whole area is selected, return the points indices.
          * \param[out] names of selected clouds.
          * \return true, if the area selected by the user contains points, false otherwise
          */
        inline int
        getCloudNames (std::vector<std::string>& names) const
        {
          if (nb_points_ <= 0)
            return (false);
          for (const auto& i : cloudIndices_)
            names.insert(names.end(), i.first);
          return names.size();
        }
        /** \brief For situations where a whole area is selected, return the points indices. for given cloud
          * \param[in] name of selected clouds.
          * \param[out] indices of given cloud.
        * \return true, if the area selected by the user contains points, false otherwise
        */
        inline bool
        getCloudIndices (std::string& name, std::vector<int>& indices) const
        {
          if (nb_points_ <= 0)
            return (false);
          auto cloud = std::find_if(cloudIndices_.begin(), cloudIndices_.end(), [&name](const auto& cloud_Indices) { return cloud_Indices.first == name; });
          if(cloud == cloudIndices_.end())
            return (false);
          indices = cloud->second;
          return (true);
        }

      private:
        int nb_points_;
        std::vector<int> indices_;
        std::map<std::string, std::vector<int>> cloudIndices_;
    };
  } //namespace visualization
} //namespace pcl
