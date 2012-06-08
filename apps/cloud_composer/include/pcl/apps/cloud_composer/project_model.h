/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright  (c) 2012, Jeremie Papon.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PROJECT_MODEL_H_
#define PROJECT_MODEL_H_

#include <QStandardItemModel>
#include <QStandardItem>
#include <QItemSelectionModel>
#include <QModelIndex>
#include <QVariant>

#include <vtkSmartPointer.h>
#include <vtkCamera.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//Define user roles
#ifndef USER_ROLES
#define USER_ROLES
enum DATAELEMENTS { 
  CLOUD = Qt::UserRole+1,
  GEOMETRY, 
  COLOR,
  ORIGIN,
  ORIENTATION
};
#endif

//Typedefs to make things sane
typedef pcl::visualization::PointCloudGeometryHandler<sensor_msgs::PointCloud2> GeometryHandler;
typedef pcl::visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;

class QItemSelectionModel;

namespace pcl
{
  namespace cloud_composer
  {

    class ProjectModel : public QStandardItemModel
    {
        Q_OBJECT

      public:
        ProjectModel (QObject *parent = 0);
        ProjectModel (const ProjectModel& to_copy);
        virtual ~ProjectModel ();
        
        ProjectModel (QString project_name, QObject *parent = 0);
        
        inline const QString
        getName ()
        { 
          return horizontalHeaderItem (0)->text ();
        }
        
        /** \brief Sets the name of the project using the horizontalHeaderItem         */
        void 
        setName (QString new_name);        
      
        inline QItemSelectionModel* const
        getSelectionModel()
        {
          return selection_model_;
        }
        
        void 
        insertNewCloudFromFile (QString filename);

        QStandardItem*
        createNewCloudItem (sensor_msgs::PointCloud2::Ptr cloud_ptr, 
                            const QString cloud_name,
                            const Eigen::Vector4f origin,
                            const Eigen::Quaternionf orientation);
      private:
        QItemSelectionModel* selection_model_;
        vtkSmartPointer<vtkCamera> camera_; 
    };
  }
}

Q_DECLARE_METATYPE (pcl::cloud_composer::ProjectModel);
//Add PointCloud types to QT MetaType System
Q_DECLARE_METATYPE (sensor_msgs::PointCloud2::Ptr);
Q_DECLARE_METATYPE (GeometryHandler::ConstPtr);
Q_DECLARE_METATYPE (ColorHandler::ConstPtr);
Q_DECLARE_METATYPE (Eigen::Vector4f);
Q_DECLARE_METATYPE (Eigen::Quaternionf);

#endif //PROJECT_MODEL_H

