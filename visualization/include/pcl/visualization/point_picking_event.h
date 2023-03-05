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

#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/types.h> // for pcl::Indices
#include <pcl/visualization/common/actor_map.h>

#include <vtkCommand.h>
#include <vtkActor.h>

#include <map>
#include <vector>


class vtkRenderWindowInteractor;

namespace pcl
{
  namespace visualization
  {
    class PCL_EXPORTS PointPickingCallback : public vtkCommand
    {
      public:
        static PointPickingCallback *New () 
        { 
          return (new PointPickingCallback); 
        }
      
        /** \brief Empty destructor */
        ~PointPickingCallback () override = default;

        void
        Execute (vtkObject *caller, unsigned long eventid, void*) override;

        pcl::index_t
        performSinglePick (vtkRenderWindowInteractor *iren);

        pcl::index_t
        performSinglePick (vtkRenderWindowInteractor *iren, float &x, float &y, float &z);

        pcl::index_t
        performAreaPick (vtkRenderWindowInteractor *iren,
                         CloudActorMapPtr cam_ptr,
                         std::map<std::string, pcl::Indices>& cloud_indices) const;


      private:
        float x_{0}, y_{0}, z_{0};
        pcl::index_t idx_{pcl::UNAVAILABLE};
        bool pick_first_{false};
        const vtkActor* actor_{nullptr};
     };

    /** /brief Class representing 3D point picking events. */
    class PCL_EXPORTS PointPickingEvent
    {
      public:
        PointPickingEvent (pcl::index_t idx) : PointPickingEvent ( idx, -1,-1, -1) {}
        PointPickingEvent (pcl::index_t idx, float x, float y, float z, const std::string& name = "") : idx_ (idx), idx2_ (-1), x_ (x), y_ (y), z_ (z), x2_ (), y2_ (), z2_ (), name_ (name) {}
        PointPickingEvent (pcl::index_t idx1, pcl::index_t idx2, float x1, float y1, float z1, float x2, float y2, float z2) :
          idx_ (idx1), idx2_ (idx2), x_ (x1), y_ (y1), z_ (z1), x2_ (x2), y2_ (y2), z2_ (z2) 
        {}

        /** \brief Obtain the ID of a point that the user just clicked on.
          *  \warning If the cloud contains NaNs the index returned by this function will not correspond to the
          * original indices. To get the correct index either sanitize the input cloud to remove NaNs or use the 
          * PointPickingEvent::getPoint function to get the x,y,z of the picked point and then search the original 
          * cloud for the correct index. An example of how to do this can be found in the pp_callback function in
          * visualization/tools/pcd_viewer.cpp
          */
        inline pcl::index_t
        getPointIndex () const
        {
          return (idx_);
        }

        /** \brief Obtain the XYZ point coordinates of a point that the user just clicked on.
          * \param[out] x the x coordinate of the point that got selected by the user
          * \param[out] y the y coordinate of the point that got selected by the user
          * \param[out] z the z coordinate of the point that got selected by the user
          */
        inline void
        getPoint (float &x, float &y, float &z) const
        {
          x = x_; y = y_; z = z_;
        }

        /** \brief For situations when multiple points are selected in a sequence, return the point coordinates.
          * \param[out] x1 the x coordinate of the first point that got selected by the user
          * \param[out] y1 the y coordinate of the first point that got selected by the user
          * \param[out] z1 the z coordinate of the first point that got selected by the user
          * \param[out] x2 the x coordinate of the second point that got selected by the user
          * \param[out] y2 the y coordinate of the second point that got selected by the user
          * \param[out] z2 the z coordinate of the second point that got selected by the user
          * \return true, if two points are available and have been clicked by the user, false otherwise
          */
        inline bool
        getPoints (float &x1, float &y1, float &z1, float &x2, float &y2, float &z2) const
        {
          if (idx2_ == pcl::UNAVAILABLE)
            return (false);
          x1 = x_; y1 = y_; z1 = z_;
          x2 = x2_; y2 = y2_; z2 = z2_;
          return (true);
        }

        /** \brief For situations where multiple points are selected in a sequence, return the points indices.
          * \param[out] index_1 index of the first point selected by user
          * \param[out] index_2 index of the second point selected by user
          * \return true, if two points are available and have been clicked by the user, false otherwise
          * \warning If the cloud contains NaNs the index returned by this function will not correspond to the
          * original indices. To get the correct index either sanitize the input cloud to remove NaNs or use the 
          * PointPickingEvent::getPoint function to get the x,y,z of the picked point and then search the original 
          * cloud for the correct index. An example of how to do this can be found in the pp_callback function in
          * visualization/tools/pcd_viewer.cpp
          */
        inline bool
        getPointIndices(pcl::index_t& index_1, pcl::index_t& index_2) const
        {
          if (idx2_ == pcl::UNAVAILABLE)
            return (false);
          index_1 = idx_;
          index_2 = idx2_;
          return (true);
        }

        /** \brief Get name of selected cloud.
          * \return name of the cloud selected by the user
          */
        inline const std::string&
        getCloudName () const { return name_; }

      private:
        pcl::index_t idx_, idx2_;

        float x_, y_, z_;
        float x2_, y2_, z2_;
        std::string name_;
    };
  } //namespace visualization
} //namespace pcl
