#ifndef PCL_TRACKING_IMPL_APPROX_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_IMPL_APPROX_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include <pcl/search/octree.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> void
    ApproxNearestPairPointCloudCoherence<PointInT>::computeCoherence (
        const PointCloudInConstPtr &cloud, const IndicesConstPtr &, float &w)
    {
      double val = 0.0;
      //for (size_t i = 0; i < indices->size (); i++)
      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        int k_index = 0;
        float k_distance = 0.0;
        //PointInT input_point = cloud->points[(*indices)[i]];
        PointInT input_point = cloud->points[i];
        search_->approxNearestSearch(input_point, k_index, k_distance);
        if (k_distance < maximum_distance_ * maximum_distance_)
        {
          PointInT target_point = target_input_->points[k_index];
          double coherence_val = 1.0;
          for (size_t i = 0; i < point_coherences_.size (); i++)
          {
            PointCoherencePtr coherence = point_coherences_[i];  
            double w = coherence->compute (input_point, target_point);
            coherence_val *= w;
          }
          val += coherence_val;
        }
      }
      w = - static_cast<float> (val);
    }

    template <typename PointInT> bool
    ApproxNearestPairPointCloudCoherence<PointInT>::initCompute ()
    {
      if (!PointCloudCoherence<PointInT>::initCompute ())
      {
        PCL_ERROR ("[pcl::%s::initCompute] PointCloudCoherence::Init failed.\n", getClassName ().c_str ());
        //deinitCompute ();
        return (false);
      }
      
      // initialize tree
      if (!search_)
        search_.reset (new pcl::search::Octree<PointInT> (0.01));
      
      if (new_target_ && target_input_)
      {
        search_->setInputCloud (target_input_);
        new_target_ = false;
      }
      
      return true;
    }
    
  }
}

#define PCL_INSTANTIATE_ApproxNearestPairPointCloudCoherence(T) template class PCL_EXPORTS pcl::tracking::ApproxNearestPairPointCloudCoherence<T>;

#endif
