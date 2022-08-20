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
 * $Id$
 *
 */

#pragma once

#include <pcl/visualization/point_cloud_geometry_handlers.h> // for PointCloudGeometryHandler
#include <pcl/visualization/point_cloud_color_handlers.h> // for PointCloudColorHandler
#include <pcl/PCLPointCloud2.h>

#include <vtkLODActor.h>
#include <vtkSmartPointer.h>
#include <vtkIdTypeArray.h>

#include <unordered_map>
#include <vector>

template <typename T> class vtkSmartPointer;
class vtkLODActor;
class vtkProp;

namespace pcl
{
  namespace visualization
  {
    class PCL_EXPORTS CloudActor
    {
      using GeometryHandler = PointCloudGeometryHandler<pcl::PCLPointCloud2>;
      using GeometryHandlerPtr = GeometryHandler::Ptr;
      using GeometryHandlerConstPtr = GeometryHandler::ConstPtr;

      using ColorHandler = PointCloudColorHandler<pcl::PCLPointCloud2>;
      using ColorHandlerPtr = ColorHandler::Ptr;
      using ColorHandlerConstPtr = ColorHandler::ConstPtr;

      public:
        CloudActor () : color_handler_index_ (0), geometry_handler_index_ (0) {}

        virtual ~CloudActor () = default;

        /** \brief The actor holding the data to render. */
        vtkSmartPointer<vtkLODActor> actor;

        /** \brief A vector of geometry handlers that can be used for rendering the data. */
        std::vector<GeometryHandlerConstPtr> geometry_handlers;

        /** \brief A vector of color handlers that can be used for rendering the data. */
        std::vector<ColorHandlerConstPtr> color_handlers;

        /** \brief The active color handler. */
        int color_handler_index_;

        /** \brief The active geometry handler. */
        int geometry_handler_index_;

        /** \brief The viewpoint transformation matrix. */
        vtkSmartPointer<vtkMatrix4x4> viewpoint_transformation_;

        /** \brief Internal cell array. Used for optimizing updatePointCloud. */
        vtkSmartPointer<vtkIdTypeArray> cells;
    };

    using CloudActorMap = std::unordered_map<std::string, CloudActor>;
    using CloudActorMapPtr = shared_ptr<CloudActorMap>;

    using ShapeActorMap = std::unordered_map<std::string, vtkSmartPointer<vtkProp> >;
    using ShapeActorMapPtr = shared_ptr<ShapeActorMap>;

    using CoordinateActorMap = std::unordered_map<std::string, vtkSmartPointer<vtkProp> >;
    using CoordinateActorMapPtr = shared_ptr<CoordinateActorMap>;
  }
}
