#ifndef PCL_TRACKING_IMPL_COHERENCE_H_
#define PCL_TRACKING_IMPL_COHERENCE_H_

namespace pcl
{
  namespace tracking
  {
    
    template <typename PointInT> double
    PointCoherence<PointInT>::compute (PointInT &source, PointInT &target)
    {
      double ret = computeCoherence (source, target);
      return ret;
    }
  }
}

#endif
