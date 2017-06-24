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


#include <vtkSmartPointer.h>
#include <vtkCamera.h>

#include <pcl/io/pcd_io.h>

#include <pcl/apps/cloud_composer/commands.h>
#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/point_selectors/selection_event.h> 
#include <pcl/apps/cloud_composer/point_selectors/manipulation_event.h>
#include <pcl/apps/cloud_composer/point_selectors/interactor_style_switch.h>

class QItemSelectionModel;

namespace pcl
{
  namespace cloud_composer
  {
    class CloudCommand;
    class AbstractTool;
    class WorkQueue;
    class CloudComposerItem;
    class CloudView;
    class InteractorStyleSwitch;
    
    class ProjectModel : public QStandardItemModel
    {
        Q_OBJECT

      public:
        ProjectModel (QObject *parent = 0);
        ProjectModel (const ProjectModel& to_copy);
        virtual ~ProjectModel ();
        
        ProjectModel (QString project_name, QObject *parent = 0);
        
        inline const QString
        getName () { return horizontalHeaderItem (0)->text (); }
        
        inline QUndoStack*
        getUndoStack () { return undo_stack_; }
        
        /** \brief Sets the name of the project using the horizontalHeaderItem         */
        void 
        setName (QString new_name);     
        
        /** \brief Returns the selection model which is used for this project */
        inline QItemSelectionModel*
        getSelectionModel ()
        {
          return selection_model_;
        }
        
        
        
        /** \brief Takes tool object issues signal to work queue to take control of it */
        void
        enqueueToolAction (AbstractTool* tool);
               
        /** \brief Helper function which inserts the item into this model and makes connections for properties */
        void 
        insertNewCloudComposerItem (CloudComposerItem* new_item, QStandardItem* parent_item);
        
        /** \brief Sets the CloudView that this project is rendering in */
        void
        setCloudView (CloudView* view);
        
        /** \brief This sets the selection for points which have been selected in the QVTKWindow */
        void 
        setPointSelection (boost::shared_ptr<SelectionEvent> selected_event);
        
        /** \brief This is invoked to perform the manipulations specified on the model */
        void
        manipulateClouds (boost::shared_ptr<ManipulationEvent> manip_event);
      public Q_SLOTS:
        void 
        commandCompleted (CloudCommand* command);
        
        void
        clearSelection ();
        
        void 
        deleteSelectedItems ();
        
        /** \brief Loads from file and inserts a new pointcloud into the model   */
        void 
        insertNewCloudFromFile ();
        
        /** \brief Loads from rgb and depth file and inserts a new pointcloud into the model   */
        void 
        insertNewCloudFromRGBandDepth ();
        
        /** \brief Opens a file dialog and saves selected cloud to file   */
        void 
        saveSelectedCloudToFile ();
        
        /** \brief This emits all the state signals, which updates the GUI action button status (enabled/disabled)" */
        void
        emitAllStateSignals ();
        
        /** \brief This sets whether the CloudView for this project shows axes */
        void
        setAxisVisibility (bool visible);
              
        /** \brief Slot called when the mouse style selected in the GUI changes */
        void 
        mouseStyleChanged (QAction* new_style_action);
        
        /** \brief Slot Called whenever the item selection_model_ changes */
        void
        itemSelectionChanged ( const QItemSelection &, const QItemSelection &);
        
        /** \brief Creates a new cloud from the selected items and points */
        void 
        createNewCloudFromSelection ();
        
        /** \brief Selects all items in the model */
        void 
        selectAllItems (QStandardItem* item = 0 );
      Q_SIGNALS:
        void
        enqueueNewAction (AbstractTool* tool, ConstItemList data);
        
        /** \brief Catch-all signal emitted whenever the model changes */
        void
        modelChanged ();
        
        void 
        axisVisible (const bool axis_visible);
        
        void
        deleteAvailable (bool can_delete);
        
        void
        newCloudFromSelectionAvailable (bool can_create);
        
        void
        mouseStyleState (interactor_styles::INTERACTOR_STYLES);
        
      private:
        /** \brief Checks to see if selection contains only CloudItem s */
        bool
        onlyCloudItemsSelected ();
        
        QItemSelectionModel* selection_model_;
        QMap <QString, int> name_to_type_map_;
        QUndoStack* undo_stack_;
        WorkQueue* work_queue_; 
        QThread* work_thread_;
        CloudView* cloud_view_;
        
        /** \brief Stores last directory used in file read/write operations */
        QDir last_directory_;
                
        //Variables for toggle action status
        bool axis_visible_;
        QMap <interactor_styles::INTERACTOR_STYLES, bool> selected_style_map_; 
        /** \brief Internal helper function for updating map */
        void
        setSelectedStyle (interactor_styles::INTERACTOR_STYLES style);
        
        /** \brief Internal pointer storing the last selection event arriving from vtk */
        boost::shared_ptr<SelectionEvent> selection_event_;
        /** \brief Map which stores which cloud items and indices were selected in the selection_event_ */
        QMap <CloudItem*, pcl::PointIndices::Ptr > selected_item_index_map_;
    };
  }
}

Q_DECLARE_METATYPE (pcl::cloud_composer::ProjectModel);
Q_DECLARE_METATYPE (pcl::cloud_composer::interactor_styles::INTERACTOR_STYLES);

#endif //PROJECT_MODEL_H

