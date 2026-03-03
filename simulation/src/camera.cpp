#include <pcl/simulation/camera.h>

using namespace Eigen;
using namespace pcl::simulation;

void
pcl::simulation::Camera::move(double vx, double vy, double vz)
{
  const Vector3d v(vx, vy, vz);
  pose_.pretranslate(pose_.rotation() * v);

  const Vector3d& t = pose_.translation();
  x_ = t.x();
  y_ = t.y();
  z_ = t.z();
}

void
pcl::simulation::Camera::updatePose()
{
  const Matrix3d m =
      AngleAxisd(yaw_,   Vector3d::UnitZ()) *
      AngleAxisd(pitch_, Vector3d::UnitY()) *
      AngleAxisd(roll_,  Vector3d::UnitX());

  pose_.setIdentity();
  pose_.linear() = m;

  pose_.translation() = Vector3d(x_, y_, z_);
}

void
pcl::simulation::Camera::setParameters(int width,
                                       int height,
                                       float fx,
                                       float fy,
                                       float cx,
                                       float cy,
                                       float z_near,
                                       float z_far)
{
  width_ = width;
  height_ = height;
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  z_near_ = z_near;
  z_far_ = z_far;

  const float inv_width  = 1.0f / static_cast<float>(width_);
  const float inv_height = 1.0f / static_cast<float>(height_);
  const float z_nf       = (z_near_ - z_far_);

  // clang-format off
  projection_matrix_ <<  2.0f * fx_ * inv_width,   0,                       1.0f - (2.0f * cx_ * inv_width),   0,
                         0,                       2.0f * fy_ * inv_height, 1.0f - (2.0f * cy_ * inv_height),  0,
                         0,                       0,                       (z_far_ + z_near_) / z_nf,         2.0f * z_near_ * z_far_ / z_nf,
                         0,                       0,                       -1.0f,                             0;
  // clang-format on
}

void
pcl::simulation::Camera::initializeCameraParameters()
{
  setParameters(640,
                480,
                576.09757860f,
                576.09757860f,
                321.06398107f,
                242.97676897f,
                0.7f,
                20.0f);
}
