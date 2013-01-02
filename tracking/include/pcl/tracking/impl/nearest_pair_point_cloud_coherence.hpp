#ifndef PCL_TRACKING_IMPL_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_IMPL_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> void 
    NearestPairPointCloudCoherence<PointInT>::computeCoherence (
        const PointCloudInConstPtr &cloud, const IndicesConstPtr &, float &w)
    {
      double val = 0.0;
      //for (size_t i = 0; i < indices->size (); i++)
      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        PointInT input_point = cloud->points[i];
        std::vector<int> k_indices(1);
        std::vector<float> k_distances(1);
        search_->nearestKSearch (input_point, 1, k_indices, k_distances);
        int k_index = k_indices[0];
        float k_distance = k_distances[0];
        if (k_distance < maximum_distance_ * maximum_distance_)
        {
          // nearest_targets.push_back (k_index);
          // nearest_inputs.push_back (i);
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
    NearestPairPointCloudCoherence<PointInT>::initCompute ()
    {
      if (!PointCloudCoherence<PointInT>::initCompute ())
      {
        PCL_ERROR ("[pcl::%s::initCompute] PointCloudCoherence::Init failed.\n", getClassName ().c_str ());
        //deinitCompute ();
        return (false);
      }
      
      // initialize tree
      if (!search_)
        search_.reset (new pcl::search::KdTree<PointInT> (false));
      
      if (new_target_ && target_input_)
      {
        search_->setInputCloud (target_input_);
        new_target_ = false;
      }
      
      return true;
    }
  }
}

#define PCL_INSTANTIATE_NearestPairPointCloudCoherence(T) template class PCL_EXPORTS pcl::tracking::NearestPairPointCloudCoherence<T>;

#endif
