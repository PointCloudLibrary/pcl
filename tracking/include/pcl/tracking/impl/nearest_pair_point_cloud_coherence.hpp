#ifndef PCL_TRACKING_IMPL_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_IMPL_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include <pcl/search/kdtree.h>
#include <pcl/search/organized_neighbor.h>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> void
    NearestPairPointCloudCoherence<PointInT>::computeCoherence
    (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices, float &w)
    {
      std::vector<size_t> nearest_targets;
      std::vector<size_t> nearest_inputs;
      
      size_t num;
      if (indices == NULL)
        num = cloud->points.size ();
      else
        num = indices->size ();
      
      for (size_t i = 0; i < num; i++)
      {
        std::vector<int> k_indices(1);
        std::vector<float> k_distances(1);
        PointInT input_point = cloud->points[(*indices)[i]];
        if (search_method_policy_ == NEAREST_NEIGHBOR)
        {
          search_->nearestKSearch (input_point, 1, k_indices, k_distances);
        }
        else if (search_method_policy_ == APPROXIMATE_NEIGHBOR)
        {
          int k_index;
          float k_distance;
          search_->approxNearestSearch(input_point, k_index, k_distance);
          k_indices[0] = k_index;
          k_distances[0] = k_distance;
        }
        
        if (k_distances[0] < maximum_distance_ * maximum_distance_)
        {
          nearest_targets.push_back (k_indices[0]);
          nearest_inputs.push_back (i);
        }
      }

      double val = 0.0;
      for (size_t i = 0; i < nearest_targets.size (); i++)
      {
          int input_index = nearest_inputs[i];
          int target_index = nearest_targets[i];
          PointInT target_point = target_input_->points[target_index];
          PointInT input_point = cloud->points[(*indices)[input_index]];
          double coherence_val = 1.0;
          for (size_t i = 0; i < point_coherences_.size (); i++)
          {
              PointCoherencePtr coherence = point_coherences_[i];  
              double w = coherence->compute (input_point, target_point);
              coherence_val *= w;
          }
          val += coherence_val;
      }
      w = - val;
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
