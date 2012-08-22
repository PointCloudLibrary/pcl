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

#ifndef SELECTED_TRACKBALL_STYLE_INTERACTOR_H_
#define SELECTED_TRACKBALL_STYLE_INTERACTOR_H_

#include <pcl/apps/cloud_composer/point_selectors/interactor_style_switch.h>
#include <pcl/apps/cloud_composer/qt.h>




namespace pcl
{
  namespace cloud_composer
  {
      
    class PCL_EXPORTS SelectedTrackballStyleInteractor : public vtkInteractorStyleTrackballActor
    {     
      public:
        static SelectedTrackballStyleInteractor* New();
        vtkTypeMacro(SelectedTrackballStyleInteractor,vtkInteractorStyleTrackballActor);
        
        SelectedTrackballStyleInteractor ();
        
        virtual ~SelectedTrackballStyleInteractor ();
               
        /** \brief Pass a pointer to the actor map
          * \param[in] actors the actor map that will be used with this style
          */
        inline void 
        setCloudActorMap (const pcl::visualization::CloudActorMapPtr &actors) { actors_ = actors; }

        /** \brief Get the cloud actor map pointer. */
        inline pcl::visualization::CloudActorMapPtr 
        getCloudActorMap () const { return (actors_); }

        /** \brief Pass a set of renderers to the interactor style. 
          * \param[in] rens the vtkRendererCollection to use
          */
        void 
        setRendererCollection (vtkSmartPointer<vtkRendererCollection> &rens) { renderers_ = rens; }

        /** \brief Function called on left mouse button click, ie, beginning of trackball */
        virtual void
        OnLeftButtonDown ();
        
        virtual void
        OnRightButtonDown ();
        
        /** \brief Function called on left mouse button release, ie, end of trackball*/
        virtual void
        OnLeftButtonUp ();
        
        virtual void
        OnRightButtonUp ();

        virtual void 
        Rotate();
        virtual void 
        Spin();
        virtual void 
        Pan();
        virtual void 
        UniformScale();
        
        /** \brief Event emitted once a valid selection has been made */
        int manipulation_complete_event_;
        
        inline void 
        setProjectModel (ProjectModel* model) { model_ = model; }
      private:

        void
        setSelectedActors ();
        
        /** \brief Actor map stored internally. */
        pcl::visualization::CloudActorMapPtr actors_;
        
        /** \brief Collection of vtkRenderers stored internally. */
        vtkSmartPointer<vtkRendererCollection> renderers_;
        
        /** \brief Internal Pointer to Project Model */
        ProjectModel* model_;
        
        QMap <QString, vtkSmartPointer<vtkMatrix4x4> > start_matrix_map_;
        QMap <QString, vtkLODActor*> selected_actors_map_;
        
    };
    
  }
  
}

#endif // SELECTED_TRACKBALL_STYLE_INTERACTOR_H_
        
        