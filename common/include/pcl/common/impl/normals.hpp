#ifndef PCL_COMMON_NORMALS_IMPL_HPP_
#define PCL_COMMON_NORMALS_IMPL_HPP_

#include <pcl/common/normals.h>

bool
pcl::alignNormals (Eigen::Ref<Eigen::Vector3f> pn, const Eigen::Ref<const Eigen::Vector3f> &av)
{
  if (pn.dot (av) < 0.0f)
  {
    pn *= -1.0;
    return (true);
  }

  return (false);
}

bool
pcl::checkNormalsEqual (const Eigen::Vector3f &n1, const Eigen::Vector3f &n2, const double &angle_threshold)
{
  double denom = n1.norm () * n2.norm ();
  if (denom == 0.0)
  {
    PCL_ERROR("Normal has length zero.\n");
    return (false);
  }
  else
  {
    return (std::acos (n1.dot (n2) / denom) <= angle_threshold);
  }
}

#endif  // PCL_COMMON_NORMALS_IMPL_H_
