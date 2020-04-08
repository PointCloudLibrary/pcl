#pragma once

#include <Eigen/Dense>
#include <pcl/tracking/distance_coherence.h>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> double
    DistanceCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
    {
       Eigen::Vector4f p = source.getVector4fMap ();
       Eigen::Vector4f p_dash = target.getVector4fMap ();
       double d = (p - p_dash).norm ();
       return 1.0 / (1.0 + d * d * weight_);
    }
  }
}

#define PCL_INSTANTIATE_DistanceCoherence(T) template class PCL_EXPORTS pcl::tracking::DistanceCoherence<T>;
