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

#include <pcl/apps/cloud_composer/qt.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ui_cloud_composer_main_window.h>
class QTreeView;

namespace pcl
{
  namespace cloud_composer
  {
    class ProjectModel;
    class CloudViewer;
    class CloudCommand;
    class ToolFactory;
    class ToolBoxModel;
    class SignalMultiplexer;
    
    /** \brief MainWindow of cloud_composer application
     * \author Jeremie Papon
     * \ingroup cloud_composer
     * The following member objects are defined in the ui file and can be manipulated:
     *    * cloud_viewer_ is the view which contains the PCLVisualizer & QVTKWidget
     *    * cloud_browser_ is the tree view in the left dock
     *    * item_inspector_ is the details view in the left dock
     *    * tool_box_view_ is the tool box in right dock
     *    * tool_parameter_view_ shows adjustable parameters for currently selected tool
     *    * undo_view_ is the undo stack view in the right dock
     */
    class ComposerMainWindow : public QMainWindow, private Ui::ComposerMainWindow
    {
      Q_OBJECT
      public:
        explicit ComposerMainWindow (QWidget *parent = 0);
        ~ComposerMainWindow ();
  
      Q_SIGNALS:
        /** \brief Signal emitted when the active project is switched - ie a different project tab is selected */
        void
        activeProjectChanged (ProjectModel* new_model, ProjectModel* previous_model);
        
        /** \brief This signal tells the current project to insert a cloud using a file dialog box */
        void 
        insertNewCloudFromFile ();
        
        /** \brief This signal tells the current project to insert a cloud from and RGB and Depth image using a file dialog box */
        void 
        insertNewCloudFromRGBandDepth ();
        
        /** \brief This signal tells the current project to save currently selected cloud to file */
        void 
        saveSelectedCloudToFile ();
        
      public Q_SLOTS:
      //Slots for File Menu Actions
        void
        on_action_new_project__triggered (/*QString name = "unsaved project"*/);
        void
        on_action_open_cloud_as_new_project__triggered ();
        void
        on_action_open_project__triggered ();
        void
        on_action_save_project__triggered ();
        void
        on_action_save_project_as__triggered ();
        void 
        on_action_save_selected_cloud__triggered ();
        void
        on_action_exit__triggered ();

        //Slots for Edit Menu Actions
        void
        on_action_insert_from_file__triggered ();
        void
        on_action_insert_from_openNi_source__triggered ();
        void
        on_action_insert_from_rgb_depth__triggered ();

          
        
        void 
        setCurrentModel (ProjectModel* model);
        
        void 
        setMouseStyleAction (interactor_styles::INTERACTOR_STYLES selected_style);
        
        void
        enqueueToolAction (AbstractTool* tool);
       
      private:
        void
        connectFileActions ();
        void
        connectEditActions ();
        
        void 
        connectViewActions ();
        
        void 
        initializeCloudBrowser ();
        void
        initializeCloudViewer ();
        void 
        initializeItemInspector ();
        
        void 
        initializeToolBox ();
        void 
        initializePlugins ();

        
        /** \brief Pointer to the model which is currently being viewed  */
        ProjectModel* current_model_;
        QItemSelectionModel* current_selection_model_;

        QMap <QString, ProjectModel*> name_model_map_;
        
        QUndoGroup *undo_group_;
        
        QItemSelectionModel* tool_selection_model_;
        ToolBoxModel* tool_box_model_;
        
        SignalMultiplexer* multiplexer_;
        
        QActionGroup* mouse_style_group_;
    };
    
  }
}




#endif // CLOUD_COMPOSER_H
