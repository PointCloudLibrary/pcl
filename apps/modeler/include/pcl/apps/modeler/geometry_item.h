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
#ifndef PCL_MODELER_GEOMETRY_ITEM_H_
#define PCL_MODELER_GEOMETRY_ITEM_H_

#include <pcl/apps/modeler/tree_item.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <vtkSmartPointer.h>

class vtkActor;

namespace pcl
{
  namespace modeler
  {
    class RenderWidget;

    class GeometryItem : public TreeItem
    {
      public:
        typedef sensor_msgs::PointCloud2  PointCloud2;
        typedef PointCloud2::Ptr          PointCloud2Ptr;
        typedef PointCloud2::ConstPtr     PointCloud2ConstPtr;

        typedef pcl::visualization::PointCloudGeometryHandler<PointCloud2> GeometryHandler;
        typedef GeometryHandler::Ptr GeometryHandlerPtr;
        typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

        typedef pcl::visualization::PointCloudColorHandler<PointCloud2> ColorHandler;
        typedef ColorHandler::Ptr ColorHandlerPtr;
        typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

        GeometryItem(MainWindow* main_window);
        GeometryItem(MainWindow* main_window, const QString & text);
        GeometryItem(MainWindow* main_window, const QIcon & icon, const QString & text);
        ~GeometryItem ();

        virtual void
        updateOnInserted();

        virtual void
        updateOnAboutToBeRemoved();

        void
        setColorHandler(double r, double g, double b);
        void
        setColorHandler(const std::string& field_name);

        GeometryHandlerConstPtr
        getGeometryHandler() const {return geometry_handler_;}

        ColorHandlerConstPtr
        getColorHandler() const {return color_handler_;}

        bool
        isCapable();

      protected:
        RenderWidget*
        getRenderWidget();

        virtual void
        prepareContextMenu(QMenu* menu) const;

        virtual void
        initHandlers() = 0;

        virtual bool
        updateActor() { return (createActor());}

        virtual bool
        createActor() = 0;

        virtual void
        handleDataChange();

      protected:
        /** \brief The actor holding the data to render. */
        vtkSmartPointer<vtkActor> actor_;

        /** \brief geometry handler that can be used for rendering the data. */
        GeometryHandlerConstPtr geometry_handler_;

        /** \brief color handler that can be used for rendering the data. */
        ColorHandlerConstPtr color_handler_;
    };
  }
}

#endif // PCL_MODELER_GEOMETRY_ITEM_H_
