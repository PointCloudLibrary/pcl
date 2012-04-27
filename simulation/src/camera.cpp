#include <iostream>
#include <pcl/simulation/camera.h>

using namespace Eigen;
using namespace pcl::simulation;

void
pcl::simulation::Camera::move (double vx, double vy, double vz)
{
  Vector3d v;
  v << vx, vy, vz;
  pose_.pretranslate (pose_.rotation ()*v);
  x_ = pose_.translation ().x ();
  y_ = pose_.translation ().y ();
  z_ = pose_.translation ().z ();
}

void
pcl::simulation::Camera::updatePose ()
{
  Matrix3d m;
  m = AngleAxisd (yaw_, Vector3d::UnitZ ())
    * AngleAxisd (pitch_, Vector3d::UnitY ())
    * AngleAxisd (roll_, Vector3d::UnitX ());

  pose_.setIdentity ();
  pose_ *= m;
  
  Vector3d v;
  v << x_, y_, z_;
  pose_.translation () = v;
}

void
pcl::simulation::Camera::setParameters (int width, int height,
                                        float fx, float fy,
                                        float cx, float cy,
                                        float z_near, float z_far)
{
  width_ = width;
  height_ = height;
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  z_near_ = z_near;
  z_far_ = z_far;

  float z_nf = (z_near_-z_far_);
  projection_matrix_ <<  2.0f*fx_/width_,  0,                 1.0f-(2.0f*cx_/width_),     0,
                         0,                2.0f*fy_/height_,  1.0f-(2.0f*cy_/height_),    0,
                         0,                0,                (z_far_+z_near_)/z_nf,  2.0f*z_near_*z_far_/z_nf,
                         0,                0,                -1.0f,                  0;
}

void
pcl::simulation::Camera::initializeCameraParameters ()
{
  setParameters (640, 480,
                 576.09757860f, 576.09757860f,
                 321.06398107f, 242.97676897f,
                 0.7f, 20.0f);
}
