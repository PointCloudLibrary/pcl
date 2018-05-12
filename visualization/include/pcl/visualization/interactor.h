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
 * $Id: interactor.h 2050 2011-08-12 15:26:19Z bouffa $
 *
 */
#ifndef PCL_PCL_VISUALIZER_INTERACTOR_H_
#define PCL_PCL_VISUALIZER_INTERACTOR_H_

#include <pcl/pcl_macros.h>

#include <vtkConfigure.h>
#include <vtkObjectFactory.h>
#ifdef _WIN32
#  include <vtkWin32RenderWindowInteractor.h>
#else
#include <vtkConfigure.h>
#if (VTK_MAJOR_VERSION <= 5 && defined VTK_USE_COCOA) || defined __APPLE__
#  include <vtkCocoaRenderWindowInteractor.h>
#elif VTK_MAJOR_VERSION <= 5 && defined VTK_USE_CARBON
#  include <vtkCarbonRenderWindowInteractor.h>
#else
// Stupid X.h defines Complex, Bool, Success globally (!)
#  include <vtkXRenderWindowInteractor.h>
#  undef Complex
#  undef Bool
#  undef Success
#  undef Status
#endif
#endif

namespace pcl
{
  namespace visualization
  {
    /** \brief The PCLVisualizer interactor */
#ifdef _WIN32
    class PCL_EXPORTS PCLVisualizerInteractor : public vtkWin32RenderWindowInteractor
#elif (VTK_MAJOR_VERSION <= 5 && defined VTK_USE_COCOA) || defined __APPLE__
    class PCLVisualizerInteractor : public vtkCocoaRenderWindowInteractor
#elif VTK_MAJOR_VERSION <= 5 && defined VTK_USE_CARBON
    class PCLVisualizerInteractor : public vtkCarbonRenderWindowInteractor
#else
    class PCLVisualizerInteractor : public vtkXRenderWindowInteractor
#endif
    {
      public:
        static PCLVisualizerInteractor *New ();
        
//        void TerminateApp (void); // do nothing -> disable exit(0) on keys 'q', 'Q' or Esc
        void stopLoop ();
        
        bool stopped;
        int timer_id_;

#if defined (_WIN32) && ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
        int BreakLoopFlag;                // if true quit the GetMessage loop

        virtual void 
        Start ();                         // Redefine the vtkWin32RenderWindowInteractor::Start method...

        vtkGetMacro (BreakLoopFlag, int);

        void 
        SetBreakLoopFlag (int);           // Change the value of BreakLoopFlag

        void 
        BreakLoopFlagOff ();              // set BreakLoopFlag to 0
        
        void 
        BreakLoopFlagOn ();               // set BreakLoopFlag to 1 (quit)
#endif
    };
  }
}

#endif
