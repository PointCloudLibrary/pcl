/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_PCL_VISUALIZER_REN_WIN_INTERACT_MAP_H_
#define PCL_PCL_VISUALIZER_REN_WIN_INTERACT_MAP_H_

#include <map>
#include <string>

template <typename T> class vtkSmartPointer;
class vtkXYPlotActor;
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkInteractorStyleTrackballCamera;
class PCLVisualizerInteractor;

namespace pcl
{
  namespace visualization
  {
    class RenWinInteract
    {
      public:

        RenWinInteract ();

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
