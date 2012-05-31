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

#ifndef PCL_MODELER_MAIN_WINDOW_H_
#define PCL_MODELER_MAIN_WINDOW_H_

#include <boost/shared_ptr.hpp>
#include <vtkSmartPointer.h>
#include <QMainWindow>

// Forward Qt class declarations
namespace Ui
{
  class MainWindow;
}

class QStandardItemModel;
class vtkActor;
class vtkRenderer;
class vtkRenderWindow;

namespace pcl
{
  namespace modeler
  {
    class RenderWidget;
    class DockWidget;
    class PCLModeler;

    class MainWindow : public QMainWindow
    {
      Q_OBJECT

      public:
        MainWindow();
        ~MainWindow();

        void
        setActiveDockWidget(RenderWidget* render_widget);

        vtkSmartPointer<vtkRenderer>
        getActiveRender();

        void
        addItemToSceneTree(vtkSmartPointer<vtkRenderer> renderer, vtkSmartPointer<vtkActor> actor);
      public slots:
        // slots for file menu
        void 
        slotOpenPointCloud();
        void 
        slotImportPointCloud();
        void 
        slotSavePointCloud();
        void 
        slotClosePointCloud();
        void 
        slotOpenProject();
        void 
        slotSaveProject();
        void 
        slotCloseProject();
        void 
        slotExit();

        // slots for view menu
        void
        slotCreateRenderWindow();

      protected:

      protected slots:

      private:
        // methods for file Menu
        void 
        connectFileMenuActions();
        void 
        createRecentPointCloudActions();
        void 
        updateRecentPointCloudActions();
        bool
        openPointCloudImpl(const QStringList& filenames);
        bool 
        openPointCloudImpl(const QString& filename);
        void 
        createRecentProjectActions();
        void 
        updateRecentProjectActions();
        bool 
        openProjectImpl(const QString& filename);
        static void 
        updateRecentActions(std::vector<boost::shared_ptr<QAction> >& recent_actions, QStringList& recent_items);
        QString 
        getRecentFolder();


        // methods for view menu
        void 
        connectViewMenuActions();

        // methods for global settings
        void 
        loadGlobalSettings();
        void 
        saveGlobalSettings();

        RenderWidget*
        getActiveRenderWidget();

        void
        addRenderWidget(RenderWidget* render_widget);
      private slots:
        void 
        slotOpenRecentPointCloud();
        void 
        slotOpenRecentProject();

      private:
        Ui::MainWindow                    *ui_; // Designer form

        // shortcuts for recent point clouds/projects
        QStringList                       recent_pointclouds_;
        QStringList                       recent_projects_;
        static const size_t               MAX_RECENT_NUMBER = 8;
        std::vector<boost::shared_ptr<QAction> >  recent_pointcloud_actions_;
        std::vector<boost::shared_ptr<QAction> >  recent_project_actions_;

        // data
        boost::shared_ptr<PCLModeler>  pcl_modeler_;

        // scene tree
        boost::shared_ptr<QStandardItemModel>     scene_tree_;

        // render widgets
        typedef std::vector<RenderWidget*> RenderWidgets;
        RenderWidgets      render_widgets_;
    };
  }
}

#endif // PCL_MODELER_MAIN_WINDOW_H_
