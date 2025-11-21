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

#pragma once

#include <QMap>

#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/common/actor_map.h>
#include <pcl/visualization/common/ren_win_interact_map.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>
#include <vtkAreaPicker.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkRendererCollection.h>
#include <vtkInteractorStyle.h>

class QVTKWidget;

namespace pcl
{
  namespace cloud_composer
  {
    namespace interactor_styles
    {
      enum INTERACTOR_STYLES
      { 
        PCL_VISUALIZER = 0,
        RECTANGULAR_FRUSTUM,
        SELECTED_TRACKBALL,
        CLICK_TRACKBALL
      };
    }
    namespace interactor_events
    {
      enum 
      {
        SELECTION_COMPLETE_EVENT = vtkCommand::UserEvent + 1,
        MANIPULATION_COMPLETE_EVENT
      };
    };

    class RectangularFrustumSelector;  
    class SelectedTrackballStyleInteractor;
    class ClickTrackballStyleInteractor;
    class ProjectModel;
    
    class InteractorStyleSwitch : public vtkInteractorStyle 
    {
      public:
        static InteractorStyleSwitch *New();
        vtkTypeMacro(InteractorStyleSwitch, vtkInteractorStyle);
        
        InteractorStyleSwitch();
  
        void 
        SetInteractor(vtkRenderWindowInteractor *iren) override;
        
        vtkGetObjectMacro(current_style_, vtkInteractorStyle);
        
        void 
        initializeInteractorStyles (pcl::visualization::PCLVisualizer::Ptr vis, ProjectModel* model);
        
        inline void 
        setQVTKWidget (QVTKWidget* qvtk) { qvtk_ = qvtk; }
                
        void
        setCurrentInteractorStyle (interactor_styles::INTERACTOR_STYLES interactor_style);
        
      //  vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle>
      //  getPCLVisInteractorStyle () { return pcl_vis_style_; }
        
        inline vtkSmartPointer <vtkInteractorStyle>
        getInteractorStyle (const interactor_styles::INTERACTOR_STYLES interactor_style) const 
          { return name_to_style_map_.value (interactor_style); }
        
        
        void SetDefaultRenderer(vtkRenderer*) override;
        
        void SetCurrentRenderer(vtkRenderer*) override;
        
        void
        OnLeave () override;
                  
      protected:
        void 
        setCurrentStyle();
        
        QMap <interactor_styles::INTERACTOR_STYLES, vtkSmartPointer <vtkInteractorStyle> > name_to_style_map_;
        
        
        vtkRenderWindowInteractor* render_window_interactor_;
        vtkSmartPointer<vtkRendererCollection> rens_;
        
        vtkInteractorStyle* current_style_;
        vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle> pcl_vis_style_;
        vtkSmartPointer<RectangularFrustumSelector> rectangular_frustum_selector_;
        
        vtkSmartPointer<SelectedTrackballStyleInteractor> selected_trackball_interactor_style_;
        
        vtkSmartPointer<ClickTrackballStyleInteractor> click_trackball_interactor_style_;
        vtkSmartPointer<vtkAreaPicker> area_picker_;
        vtkSmartPointer<vtkPointPicker> point_picker_;
        
        /** \brief Internal pointer to QVTKWidget that this Switch works with */
        QVTKWidget* qvtk_;
        /** \brief Internal pointer to PCLVisualizer that this Switch works with */
        pcl::visualization::PCLVisualizer::Ptr vis_;
      private:
        InteractorStyleSwitch(const InteractorStyleSwitch&);  // Not implemented.
        void operator=(const InteractorStyleSwitch&);  // Not implemented.
        ProjectModel* project_model_;
    };
    
  }
  
}
