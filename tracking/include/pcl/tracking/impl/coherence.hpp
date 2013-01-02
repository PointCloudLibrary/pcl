#ifndef PCL_TRACKING_IMPL_COHERENCE_H_
#define PCL_TRACKING_IMPL_COHERENCE_H_

#include <pcl/console/print.h>
#include <pcl/tracking/coherence.h>

namespace pcl
{
  namespace tracking
  {
    
    template <typename PointInT> double
    PointCoherence<PointInT>::compute (PointInT &source, PointInT &target)
    {
      return computeCoherence (source, target);
    }

    template <typename PointInT> double
    PointCloudCoherence<PointInT>::calcPointCoherence (PointInT &source, PointInT &target)
    {
      double val = 0.0;
      for (size_t i = 0; i < point_coherences_.size (); i++)
      {
        PointCoherencePtr coherence = point_coherences_[i];
        double d = log(coherence->compute (source, target));
        //double d = coherence->compute (source, target);
        if (! pcl_isnan(d))
          val += d;
        else
          PCL_WARN ("nan!\n");
      }
      return val;
    }
    
    template <typename PointInT> bool
    PointCloudCoherence<PointInT>::initCompute ()
    {
      if (!target_input_ || target_input_->points.empty ())
      {
        PCL_ERROR ("[pcl::%s::compute] target_input_ is empty!\n", getClassName ().c_str ());
        return false;
      }

      return true;
      
    }
    
    template <typename PointInT> void
    PointCloudCoherence<PointInT>::compute (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices, float &w)
    {
      if (!initCompute ())
      {
        PCL_ERROR ("[pcl::%s::compute] Init failed.\n", getClassName ().c_str ());
        return;
      }
      computeCoherence (cloud, indices, w);
    }
  }
}

#endif
