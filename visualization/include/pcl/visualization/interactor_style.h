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
 * $Id$
 *
 */
#ifndef PCL_PCL_VISUALIZER_INTERACTOR_STYLE_H_
#define PCL_PCL_VISUALIZER_INTERACTOR_STYLE_H_

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkLegendScaleActor.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkAssemblyPath.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkObjectFactory.h>
#include <vtkScalarBarActor.h>
#include <vtkScalarsToColors.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget.h>
#include <vtkBoxWidget2.h>
#include <vtkClipPolyData.h>
#include <vtkPlanes.h>

#include <pcl/console/print.h>
#include <pcl/visualization/common/actor_map.h>
#include <pcl/visualization/common/ren_win_interact_map.h>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
//#include <boost/signals2/slot.hpp>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>

namespace pcl
{
  namespace visualization
  {
    /** \brief PCL Visualizer interactory style class. 
      * \author Radu Bogdan Rusu
      * \ingroup visualization
      */
    class PCL_EXPORTS PCLVisualizerInteractorStyle : public vtkInteractorStyleTrackballCamera
    {
      typedef boost::shared_ptr<CloudActorMap> CloudActorMapPtr;

      public:
        static PCLVisualizerInteractorStyle *New ();
        // this macro defines Superclass, the isA functionality and the safe downcast method
//        vtkTypeMacro(PCLVisualizerInteractorStyle,vtkInteractorStyleTrackballCamera);
        
        /** \brief Initialization routine. Must be called before anything else. */
        virtual void Initialize ();
        
        /** \brief Pass a pointer to the actor map
          * \param actors the actor map that will be used with this style
          */
        inline void setCloudActorMap (const CloudActorMapPtr &actors) { actors_ = actors; }
        inline CloudActorMapPtr getCloudActorMap () { return (actors_); }

        /** \brief Pass a set of renderers to the interactor style. 
          * \param rens the vtkRendererCollection to use
          */
        void setRendererCollection (vtkSmartPointer<vtkRendererCollection> &rens) { rens_ = rens; }

        boost::signals2::connection registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> );
        boost::signals2::connection registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> );
      protected:
        /** \brief Set to true after initialization is complete. */
        bool init_;

        /** \brief Collection of vtkRenderers stored internally. */
        vtkSmartPointer<vtkRendererCollection> rens_;

        /** \brief Actor map stored internally. */
        CloudActorMapPtr actors_;

        /** \brief The current window width/height. */
        int win_height_, win_width_;

        /** \brief The current window position x/y. */
        int win_pos_x_, win_pos_y_;

        /** \brief The maximum resizeable window width/height. */
        int max_win_height_, max_win_width_;

        /** \brief Set to true if the grid actor is enabled. */
        bool grid_enabled_;
        /** \brief Actor for 2D grid on screen. */
        vtkSmartPointer<vtkLegendScaleActor> grid_actor_;

        /** \brief Set to true if the LUT actor is enabled. */
        bool lut_enabled_;
        /** \brief Actor for 2D lookup table on screen. */
        vtkSmartPointer<vtkScalarBarActor> lut_actor_;

        /** \brief A PNG writer for screenshot captures. */
        vtkSmartPointer<vtkPNGWriter> snapshot_writer_;
        /** \brief Internal window to image filter. Needed by \a snapshot_writer_. */
        vtkSmartPointer<vtkWindowToImageFilter> wif_;

        boost::signals2::signal<void (const pcl::visualization::MouseEvent&)> mouse_signal_;
        boost::signals2::signal<void (const pcl::visualization::KeyboardEvent&)> keyboard_signal_;
        /** \brief Interactor style internal method. Gets called whenever a key is pressed. */
        virtual void OnChar ();

        // Keyboard events
        virtual void OnKeyDown ();
        virtual void OnKeyUp ();
        
        // mouse button events
        virtual void 	OnMouseMove ();
        virtual void 	OnLeftButtonDown ();
        virtual void 	OnLeftButtonUp ();
        virtual void 	OnMiddleButtonDown ();
        virtual void 	OnMiddleButtonUp ();
        virtual void 	OnRightButtonDown ();
        virtual void 	OnRightButtonUp ();
        virtual void 	OnMouseWheelForward ();
        virtual void 	OnMouseWheelBackward ();
        
        // mouse move event
        /** \brief Interactor style internal method. Gets called periodically if a timer is set. */
        virtual void OnTimer ();

        /** \brief Interactor style internal method. Zoom in. */
        void zoomIn ();

        /** \brief Interactor style internal method. Zoom out. */
        void zoomOut ();

        /** \brief True if we're using red-blue colors for anaglyphic stereo, false if magenta-green. */
        bool stereo_anaglyph_mask_default_;
    };

    /** \brief PCL histogram visualizer interactory style class.
      * \author Radu Bogdan Rusu
      */
    class PCLHistogramVisualizerInteractorStyle : public vtkInteractorStyleTrackballCamera
    {
      public:
        static PCLHistogramVisualizerInteractorStyle *New ();

        /** \brief Initialization routine. Must be called before anything else. */
        void Initialize ();
        
        /** \brief Pass a map of render/window/interactors to the interactor style. 
          * \param wins the RenWinInteract map to use
          */
        void setRenWinInteractMap (const RenWinInteractMap &wins) { wins_ = wins; }
      private:
        /** \brief A map of all windows on screen (with their renderers and interactors). */
        RenWinInteractMap wins_;

        /** \brief Set to true after initialization is complete. */
        bool init_;

        /** \brief The current window width/height. */
        int win_height_, win_width_;

        /** \brief The current window position x/y. */
        int win_pos_x_, win_pos_y_;

        /** \brief The maximum resizeable window width/height. */
        int max_win_height_, max_win_width_;

        /** \brief A PNG writer for screenshot captures. */
        vtkSmartPointer<vtkPNGWriter> snapshot_writer_;
        /** \brief Internal window to image filter. Needed by \a snapshot_writer_. */
        vtkSmartPointer<vtkWindowToImageFilter> wif_;

        /** \brief Interactor style internal method. Gets called whenever a key is pressed. */
        void OnChar ();

        /** \brief Interactor style internal method. Gets called periodically if a timer is set. */
        void OnTimer ();
    };
  }
}

#endif
