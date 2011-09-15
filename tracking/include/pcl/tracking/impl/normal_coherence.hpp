#ifndef PCL_TRACKING_IMPL_NORMAL_COHERENCE_H_
#define PCL_TRACKING_IMPL_NORMAL_COHERENCE_H_

#include <pcl/common/common.h>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> double 
    NormalCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
    {
      Eigen::Vector4f n (source.normal[0],
                         source.normal[1],
                         source.normal[2],
                         0.0f);
      Eigen::Vector4f n_dash (target.normal[0],
                              target.normal[1],
                              target.normal[2],
                              0.0f);
      if ( n.norm () <= 1e-5 || n_dash.norm () <= 1e-5 )
      {
        PCL_ERROR("norm might be ZERO!\n");
        std::cout << "source: " << source << std::endl;
        std::cout << "target: " << target << std::endl;
        exit (1);
        return 0.0;
      }
      else
      {
        n.normalize ();
        n_dash.normalize ();
        double theta = pcl::getAngle3D (n, n_dash);
        if (!pcl_isnan (theta))
        {
          return 1.0 / (1.0 + weight_ * theta * theta);
        }
        else
        {
          return 0.0;
        }
      }
    }
  }
}
#endif
