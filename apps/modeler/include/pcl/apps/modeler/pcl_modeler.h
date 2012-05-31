/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */

#ifndef PCL_MODELER_PCLMODELER_H_
#define PCL_MODELER_PCLMODELER_H_

#include <map>
#include <vtkLODActor.h>
#include <pcl/visualization/point_cloud_handlers.h>

namespace pcl
{
  namespace modeler
  {
    class CloudActor;
    class MainWindow;

    /** \brief PCL Visualizer main class.
      * \author Yangyan Li
      * \ingroup apps
      */
    class PCL_EXPORTS PCLModeler
    {
      public:
        typedef pcl::visualization::PointCloudGeometryHandler<sensor_msgs::PointCloud2> GeometryHandler;
        typedef GeometryHandler::Ptr GeometryHandlerPtr;
        typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

        typedef pcl::visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
        typedef ColorHandler::Ptr ColorHandlerPtr;
        typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

        /** \brief PCL Visualizer constructor.
          * \param[in] style interactor style (defaults to pcl::visualization::PCLVisualizerInteractorStyle)
          * \param[in] create_interactor if true (default), create an interactor, false otherwise
          */
        PCLModeler (MainWindow* main_window);

        /** \brief PCL Visualizer destructor. */
        virtual ~PCLModeler ();

        bool
        openPointCloud(const std::string& filename);

        typedef std::map<vtkSmartPointer<vtkActor>, boost::shared_ptr<CloudActor> > CloudActorMap;
        CloudActorMap&
        getCloudActorMap() {return cloud_actor_map_;}
        const CloudActorMap&
        getCloudActorMap() const {return cloud_actor_map_;}
      private:
        /** \brief Internal. actor to cloud actor map*/
        CloudActorMap   cloud_actor_map_;

        /** \brief Internal. pointer to the MainWindow*/
        MainWindow*     main_window_;
    };
  }
}

#include <pcl/apps/modeler/impl/pcl_modeler.hpp>

#endif // PCL_MODELER_PCLMODELER_H_
