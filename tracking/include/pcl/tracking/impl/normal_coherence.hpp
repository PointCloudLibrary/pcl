#ifndef PCL_TRACKING_IMPL_NORMAL_COHERENCE_H_
#define PCL_TRACKING_IMPL_NORMAL_COHERENCE_H_

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/tracking/normal_coherence.h>

namespace pcl {
namespace tracking {
template <typename PointInT>
double
NormalCoherence<PointInT>::computeCoherence(PointInT& source, PointInT& target)
{
  Eigen::Vector4f n = source.getNormalVector4fMap();
  Eigen::Vector4f n_dash = target.getNormalVector4fMap();
  if (n.norm() <= 1e-5 || n_dash.norm() <= 1e-5) {
    PCL_ERROR_STREAM("[NormalCoherence::computeCoherence] normal of source and/or "
                     "target is zero! source: "
                     << source << "target: " << target << std::endl);
    return 0.0;
  }
  n.normalize();
  n_dash.normalize();
  double theta = pcl::getAngle3D(n, n_dash);
  if (!std::isnan(theta))
    return 1.0 / (1.0 + weight_ * theta * theta);
  return 0.0;
}
} // namespace tracking
} // namespace pcl

#define PCL_INSTANTIATE_NormalCoherence(T)                                             \
  template class PCL_EXPORTS pcl::tracking::NormalCoherence<T>;

#endif
