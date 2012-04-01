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
