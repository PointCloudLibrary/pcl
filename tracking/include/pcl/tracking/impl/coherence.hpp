#ifndef PCL_TRACKING_IMPL_COHERENCE_H_
#define PCL_TRACKING_IMPL_COHERENCE_H_

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
      double val = 1.0;
      for (size_t i = 0; i < point_coherences_.size (); i++)\
      {
        PointCoherencePtr coherence = point_coherences_[i];
        val *= coherence->comput (source, target);
      }
      return val;
    }
    
    template <typename PointInT> bool
    PointCloudCoherence<PointInT>::initCompute ()
    {
      if (PCLBase<PointInT>::initCompute ())
      {
        PCL_ERROR ("[pcl::%s::initCompute] PCLBase::Init failed.\n", getClassName ().c_str ());
        deinitCompute ();
        return (false);
      }

      if (!target_input_ || target_input_->points.empty ())
      {
        PCL_ERROR ("[pcl::%s::compute] target_input_ is empty!\n", getClassName ().c_str ());
        deinitCompute ();
        return false;
      }

      return true;
      
    }
    
    template <typename PointInT> double
    PointCloudCoherence<PointInT>::compute ()
    {
      if (initCompute ())
      {
        PCL_ERROR ("[pcl::%s::compute] Init failed.\n", getClassName ().c_str ());
        return (false);
      }

      return computeCoherence ();
    }
  }
}

#endif
