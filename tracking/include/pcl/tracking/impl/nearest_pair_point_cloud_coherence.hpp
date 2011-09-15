#ifndef PCL_TRACKING_IMPL_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_IMPL_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/organized_data.h>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> double
    NearestPairPointCloudCoherence<PointInT>::computeCoherence ()
    {
      std::vector<int> k_indices(1);
      std::vector<float> k_distances(1);
      double val = 1.0;
      for ( size_t i = 0; i < indices_->size (); i++ )
      {
        PointInT input_point = input_->points[(*indices_)[i]];
        tree_->nearestKSearch (input_point, 1, k_indices, k_distances);
        PointInT target_point = target_input_->points[k_indices[0]];

        val *= calcPointCoherence(input_point, target_point);
      }
      return val;
    }

    template <typename PointInT> bool
    NearestPairPointCloudCoherence<PointInT>::initCompute ()
    {
      if (!PointCloudCoherence<PointInT>::initCompute ())
      {
        PCL_ERROR ("[pcl::%s::initCompute] PointCloudCoherence::Init failed.\n", getClassName ().c_str ());
        deinitCompute ();
        return (false);
      }

      // initialize tree
      if (!tree_)
        tree_.reset (new pcl::KdTreeFLANN<PointInT> (false));
      
      if (new_target_ && target_input_)
      {
        tree_->setInputCloud (target_input_);
        new_target_ = false;
      }
      
      return true;
    }
  }
}

#endif
