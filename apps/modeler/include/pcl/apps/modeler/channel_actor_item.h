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
#ifndef PCL_MODELER_CHANNEL_ACTOR_ITEM_H_
#define PCL_MODELER_CHANNEL_ACTOR_ITEM_H_

#include <vtkSmartPointer.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/eigen.h>
#include <pcl/apps/modeler/qt.h>
#include <pcl/apps/modeler/abstract_item.h>

class vtkActor;
class vtkMatrix4x4;
class vtkRenderWindow;


namespace pcl
{
  namespace modeler
  {
    class CloudMesh;

    class ChannelActorItem : public QTreeWidgetItem, public AbstractItem
    {
      public:
        ChannelActorItem(QTreeWidgetItem* parent,
                         const boost::shared_ptr<CloudMesh>& cloud_mesh,
                         const vtkSmartPointer<vtkRenderWindow>& render_window,
                         const vtkSmartPointer<vtkActor>& actor,
                         const std::string& channel_name);
        ~ChannelActorItem();

        void
        createActor();

      protected:
        void
        attachActor();

        void
        detachActor();

        virtual void
        createActorImpl() = 0;

        virtual void
        prepareContextMenu(QMenu* menu) const;

        boost::shared_ptr<CloudMesh>      cloud_mesh_;
        vtkSmartPointer<vtkActor>         actor_;
        vtkSmartPointer<vtkRenderWindow>  render_window_;
        std::string                       color_scheme_;
        double                            r_, g_, b_;

        /** \brief Internal method. Convert origin and orientation to vtkMatrix4x4.
          * \param[in] origin the point cloud origin
          * \param[in] orientation the point cloud orientation
          * \param[out] vtk_matrix the resultant VTK 4x4 matrix
          */
        static void
        convertToVtkMatrix(const Eigen::Vector4f& origin,
                           const Eigen::Quaternion<float>& orientation,
                           vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);
        /** \brief The viewpoint transformation matrix. */
        vtkSmartPointer<vtkMatrix4x4> viewpoint_transformation_;

      private:
    };
  }
}

#endif // PCL_MODELER_CHANNEL_ACTOR_ITEM_H_