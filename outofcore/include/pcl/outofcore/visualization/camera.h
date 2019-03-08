/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#pragma once

// C++
#include <iostream>
#include <string>

// PCL
#include <pcl/outofcore/visualization/object.h>
#include <pcl/common/eigen.h>

// VTK
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class Camera : public Object
{
public:

  // Operators
  // -----------------------------------------------------------------------------
  Camera (std::string name);
  Camera (std::string name, vtkSmartPointer<vtkCamera> camera);

private:
//  friend std::ostream & operator<<(std::ostream &os, const Camera& camera);

public:

  // Accessors
  // -----------------------------------------------------------------------------
  inline vtkSmartPointer<vtkCamera>
  getCamera () const
  {
    return camera_;
  }

  inline vtkSmartPointer<vtkCameraActor>
  getCameraActor () const
  {
    return camera_actor_;
  }

  inline vtkSmartPointer<vtkActor>
  getHullActor () const
  {
    return hull_actor_;
  }

  inline bool
  getDisplay () const
  {
    return display_;
  }

  void
  setDisplay (bool display)
  {
    this->display_ = display;
  }

  void
  getFrustum (double frustum[])
  {
    for (int i = 0; i < 24; i++)
      frustum[i] = frustum_[i];
  }

  void
  setProjectionMatrix (const Eigen::Matrix4d &projection_matrix)
  {
    projection_matrix_ = projection_matrix;
  }

  Eigen::Matrix4d
  getProjectionMatrix ()
  {
    return projection_matrix_;
  }

  void
  setModelViewMatrix (const Eigen::Matrix4d &model_view_matrix)
  {
    model_view_matrix_ = model_view_matrix;
  }

  Eigen::Matrix4d
  getModelViewMatrix ()
  {
    return model_view_matrix_;
  }

  Eigen::Matrix4d
  getViewProjectionMatrix ()
  {
    return Eigen::Matrix4d (projection_matrix_ * model_view_matrix_);
  }

  Eigen::Vector3d
  getPosition ()
  {
    //Compute eye or position from model view matrix
    Eigen::Matrix4d inverse_model_view_matrix = model_view_matrix_.inverse ();
    Eigen::Vector3d position;
    for (int i = 0; i < 3; i++)
    {
      position (i) = inverse_model_view_matrix (i, 3);
    }

    return position;
  }

  inline void
  setClippingRange (float near_value = 0.0001f, float far_value = 100000.f)
  {
    camera_->SetClippingRange (near_value, far_value);
  }

  void
  render (vtkRenderer* renderer) override;

  // Methods
  // -----------------------------------------------------------------------------
  //void computeFrustum(double aspect);
  void
  computeFrustum ();
  //computeFrustum(double aspect);
  void
  printFrustum ();

private:

  // Members
  // -----------------------------------------------------------------------------
  vtkSmartPointer<vtkCamera> camera_;
  vtkSmartPointer<vtkCameraActor> camera_actor_;
  vtkSmartPointer<vtkActor> hull_actor_;

  bool display_;

  double frustum_[24];
  Eigen::Matrix4d projection_matrix_;
  Eigen::Matrix4d model_view_matrix_;

  double prevUp_[3];
  double prevFocal_[3];
  double prevPos_[3];
};
