#ifndef PCL_OUTOFCORE_CAMERA_H_
#define PCL_OUTOFCORE_CAMERA_H_

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

  virtual void
  render (vtkRenderer* renderer);

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

#endif
