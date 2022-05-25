 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
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
#include <pcl/console/print.h> // for PCL_DEBUG
#include <pcl/visualization/vtk/vtkRenderWindowInteractorFix.h>
#include <vtkVersion.h>
#if __unix__ && VTK_MAJOR_VERSION == 9 && ((VTK_MINOR_VERSION == 0 && (VTK_BUILD_VERSION == 2 || VTK_BUILD_VERSION == 3)) || (VTK_MINOR_VERSION == 1 && VTK_BUILD_VERSION == 0))
#include <pcl/visualization/vtk/vtkFixedXRenderWindowInteractor.h>
#endif

#ifndef __APPLE__
vtkRenderWindowInteractor* vtkRenderWindowInteractorFixNew ()
{
#if __unix__ && VTK_MAJOR_VERSION == 9 && ((VTK_MINOR_VERSION == 0 && (VTK_BUILD_VERSION == 2 || VTK_BUILD_VERSION == 3)) || (VTK_MINOR_VERSION == 1 && VTK_BUILD_VERSION == 0))
// VTK versions 9.0.2, 9.0.3, 9.1.0
  vtkRenderWindowInteractor* interactor = vtkRenderWindowInteractor::New ();
  if(interactor->IsA("vtkXRenderWindowInteractor")) {
    PCL_DEBUG("Using a fixed version of the vtkXRenderWindowInteractor!\n");
    interactor->Delete ();
    interactor = pcl::vtkXRenderWindowInteractor::New ();
  }
  return (interactor);
#else
  return (vtkRenderWindowInteractor::New ());
#endif
}
#endif


