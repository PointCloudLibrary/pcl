#ifndef PCL_TRACKING_IMPL_DISTANCE_COHERENCE_H_
#define PCL_TRACKING_IMPL_DISTANCE_COHERENCE_H_

#include <Eigen/Dense>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> double
    DistanceCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
    {
      Eigen::Vector4f p (source.x, source.y, source.z, 0.0f);
      Eigen::Vector4f p_dash (target.x, target.y, target.z, 0.0f);
      double d = (p - p_dash).norm ();
      return 1.0 / (1.0 + d * d * weight_);
    }
  }
}
#endif
