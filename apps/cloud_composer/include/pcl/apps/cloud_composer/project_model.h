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
#include <QItemSelectionModel>
#include <QModelIndex>
#include <QVariant>
#include <QUndoStack>

#include <vtkSmartPointer.h>
#include <vtkCamera.h>

#include <pcl/io/pcd_io.h>

#include <pcl/apps/cloud_composer/commands.h>


class QItemSelectionModel;

namespace pcl
{
  namespace cloud_composer
  {
    class CloudCommand;
    class AbstractTool;
    class WorkQueue;
    class CloudComposerItem;
    
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
        inline QItemSelectionModel* const
        getSelectionModel ()
        {
          return selection_model_;
        }
        
        /** \brief Loads from file and inserts a new pointcloud into the model   */
        void 
        insertNewCloudFromFile (const QString filename);
        
        /** \brief Takes tool object issues signal to work queue to take control of it */
        void
        enqueueToolAction (AbstractTool* tool);
      public slots:
        void 
        commandCompleted (CloudCommand* command);
      signals:  
        void
        enqueueNewAction (AbstractTool* tool, ConstItemList data);
        
      private:
        QItemSelectionModel* selection_model_;
        vtkSmartPointer<vtkCamera> camera_; 
        QMap <QString, int> name_to_type_map_;
        QUndoStack* undo_stack_;
        WorkQueue* work_queue_; 
        QThread* work_thread_;

    };
  }
}

Q_DECLARE_METATYPE (pcl::cloud_composer::ProjectModel);


#endif //PROJECT_MODEL_H

