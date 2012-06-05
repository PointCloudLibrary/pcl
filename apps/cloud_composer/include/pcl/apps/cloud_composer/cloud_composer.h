/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Jeremie Papon.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CLOUD_COMPOSER_H_
#define CLOUD_COMPOSER_H_

//Qt
#include <QMainWindow>
#include <QMetaType>
#include <QDir>
#include <Qt>

#include <ui_cloud_composer_main_window.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

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

namespace Ui
{
  class MainWindow;
}

class QTreeView;
    
//Typedefs to make things sane
typedef pcl::visualization::PointCloudGeometryHandler<sensor_msgs::PointCloud2> GeometryHandler;
typedef pcl::visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
     
class QStandardItem;

namespace pcl
{
  namespace cloud_composer
  {

    
    class ProjectModel;
    class CloudViewer;
    
    
    class ComposerMainWindow : public QMainWindow
    {
      Q_OBJECT
      public:
        explicit ComposerMainWindow (QWidget *parent = 0);
        ~ComposerMainWindow ();
  
      signals:

      public slots:
      //Slots for File Menu Actions
        void
        slotNewProject ();
        void
        slotOpenCloudAsNewProject ();
        void
        slotOpenProject ();
        void
        slotSaveProject ();
        void
        slotSaveProjectAs ();
        void
        slotExit ();

        //Slots for Edit Menu Actions
        void
        slotInsertFromFile ();
        void
        slotInsertFromOpenNiSource ();

      private:
        void
        connectFileActionsToSlots ();
        void
        connectEditActionsToSlots ();
        
        void 
        initializeCloudBrowser ();
        void 
        setCurrentModel (ProjectModel* model);
        QStandardItem*
        createNewCloudItem (sensor_msgs::PointCloud2::Ptr cloud_ptr, 
                            QString cloud_name,
                            Eigen::Vector4f origin,
                            Eigen::Quaternionf orientation);
        
        Ui::MainWindow* ui_;
        ProjectModel* current_model_;
        CloudViewer* cloud_viewer_;
        
        QDir last_directory_;
    };
    
  }
}

//Add PointCloud types to QT MetaType System
Q_DECLARE_METATYPE (sensor_msgs::PointCloud2::Ptr);
Q_DECLARE_METATYPE (GeometryHandler::ConstPtr);
Q_DECLARE_METATYPE (ColorHandler::ConstPtr);
Q_DECLARE_METATYPE (Eigen::Vector4f);
Q_DECLARE_METATYPE (Eigen::Quaternionf);


#endif // CLOUD_COMPOSER_H
