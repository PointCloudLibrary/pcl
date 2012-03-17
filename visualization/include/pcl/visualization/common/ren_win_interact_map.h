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
#ifndef PCL_PCL_VISUALIZER_REN_WIN_INTERACT_MAP_H_
#define PCL_PCL_VISUALIZER_REN_WIN_INTERACT_MAP_H_

#include <vtkSmartPointer.h>
#include <vtkXYPlotActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
#include <pcl/visualization/interactor.h>
#else
#include <vtkRenderWindowInteractor.h>
#endif
#include <map>

namespace pcl
{
  namespace visualization
  {
    class RenWinInteract
    {
      public:

        RenWinInteract () : xy_plot_ (vtkSmartPointer<vtkXYPlotActor>::New ()), 
                            ren_ (vtkSmartPointer<vtkRenderer>::New ()), 
                            win_ (vtkSmartPointer<vtkRenderWindow>::New ()),
                            interactor_ (),
                            style_ ()
        {}

        /** \brief The XY plot actor holding the actual data. */
        vtkSmartPointer<vtkXYPlotActor> xy_plot_;

        /** \brief The renderer used. */
        vtkSmartPointer<vtkRenderer> ren_;

        /** \brief The render window. */
        vtkSmartPointer<vtkRenderWindow> win_;

        /** \brief The render window interactor. */

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
        vtkSmartPointer<PCLVisualizerInteractor> interactor_;
#else
        vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
#endif
        /** \brief The render window interactor style. */
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style_;
    };
    typedef std::map<std::string, RenWinInteract> RenWinInteractMap;
  }
}

#endif
