/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#pragma once
#include <pcl/visualization/qvtk_compatibility.h>

#include <vtkSmartPointer.h>

class vtkCubeAxesActor;

namespace pcl {
namespace modeler {

class RenderWindowItem;

class RenderWindow : public PCLQVTKWidget {
public:
  RenderWindow(RenderWindowItem* render_window_item,
               QWidget* parent = nullptr,
               Qt::WindowFlags flags = {});
  ~RenderWindow();

  QSize
  sizeHint() const override
  {
    return {512, 512};
  }

  void
  setActive(bool flag);

  void
  setTitle(const QString& title);

  void
  render();

  void
  resetCamera();

  void
  updateAxes();

  void
  getBackground(double& r, double& g, double& b);

  void
  setBackground(double r, double g, double b);

  void
  setShowAxes(bool flag);

protected:
  void
  focusInEvent(QFocusEvent* event) override;

private:
  void
  initRenderer();

  vtkSmartPointer<vtkCubeAxesActor> axes_;
  RenderWindowItem* render_window_item_;
};

} // namespace modeler
} // namespace pcl
