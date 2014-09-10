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

#ifndef CLOUD_VIEW_H_
#define CLOUD_VIEW_H_

#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/cloud_composer/point_selectors/interactor_style_switch.h>
#include <vtkEventQtSlotConnect.h>

namespace pcl
{
  namespace cloud_composer
  {
    class ProjectModel;
    /** \brief View class for displaying ProjectModel data using PCLVisualizer
     * \author Jeremie Papon
     * \ingroup cloud_composer
     */
    class CloudView : public QWidget
    {
      Q_OBJECT
      
    public:
      CloudView (QWidget* parent = 0);
      CloudView (const CloudView& to_copy);
      CloudView (ProjectModel* model, QWidget* parent = 0);
      virtual ~CloudView ();
      
      void 
      setModel (ProjectModel* new_model);
      ProjectModel* 
      getModel () const { return model_; }
      
      QVTKWidget* 
      getQVTK() const {return qvtk_; }
      
      boost::shared_ptr<pcl::visualization::PCLVisualizer>
      getPCLVisualizer () const { return vis_; }
      
      void 
      setAxisVisibility (bool visible);
      
      void 
      setInteractorStyle (interactor_styles::INTERACTOR_STYLES style);
    public slots:
      void 
      refresh ();
      
      /** \brief Slot called when the item selected in cloud browser changes */
      void 
      selectedItemChanged (const QItemSelection & selected, const QItemSelection & deselected);
      
      /** \brief Slot called when the data in model changes */
      void 
      dataChanged ( const QModelIndex & topLeft, const QModelIndex & bottomRight );
      
    protected slots:
      /** \brief Slot called when an item in the model changes
       * \param topLeft 
       * \param bottomRight
       */
      void
      itemChanged (QStandardItem* item);
      
      /** \brief Slot called when rows inserted to model
       * \param start Start of new rows (inclusive)
       * \param end End of new rows (inclusive)
       */
      void
      rowsInserted (const QModelIndex& parent, int start, int end);
      
      void
      rowsAboutToBeRemoved (const QModelIndex& parent, int start, int end);
      
      void
      selectionCompleted (vtkObject* caller, unsigned long event_id, void* client_data, void* call_data);
      
      void
      manipulationCompleted (vtkObject* caller, unsigned long event_id, void* client_data, void* call_data);
      
    protected:
      void
      paintEvent (QPaintEvent* event);
      void 
      resizeEvent (QResizeEvent* event);
      //   void scrollContentsBy (int dx, int dy);
      
      
      
    private:
      void
      connectSignalsAndSlots ();
      
      /** \brief Internal function for setting up the style_switch_ */
      void 
      initializeInteractorSwitch ();
      
      void
      addOrientationMarkerWidgetAxes ();
      void
      removeOrientationMarkerWidgetAxes ();
      
      boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
      ProjectModel* model_;
      QVTKWidget* qvtk_;
      vtkSmartPointer<InteractorStyleSwitch> style_switch_;
      
      vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget_;
      vtkSmartPointer<vtkAxesActor> axes_;
      
      /** \brief Manages VTK events by connecting them to QT slots */
      vtkSmartPointer<vtkEventQtSlotConnect> connections_;
    };
  }
}

Q_DECLARE_METATYPE (pcl::cloud_composer::CloudView);
#endif

