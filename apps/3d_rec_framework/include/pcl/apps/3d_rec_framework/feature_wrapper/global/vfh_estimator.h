/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_VFH_ESTIMATOR_H_
#define REC_FRAMEWORK_VFH_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/features/vfh.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
      class VFHEstimation : public GlobalEstimator<PointInT, FeatureT>
      {

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        using GlobalEstimator<PointInT, FeatureT>::normal_estimator_;

      public:
        void
        estimate (PointInTPtr & in, PointInTPtr & processed,
                  std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                  std::vector<Eigen::Vector3f> & centroids)
        {

          if (!normal_estimator_)
          {
            PCL_ERROR("This feature needs normals... please provide a normal estimator\n");
            return;
          }

          pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
          normal_estimator_->estimate (in, processed, normals);

          typedef typename pcl::VFHEstimation<PointInT, pcl::Normal, FeatureT> VFHEstimation;
          pcl::PointCloud<FeatureT> vfh_signature;

          VFHEstimation vfh;
          typename pcl::search::KdTree<PointInT>::Ptr vfh_tree (new pcl::search::KdTree<PointInT>);
          vfh.setSearchMethod (vfh_tree);
          vfh.setInputCloud (processed);
          vfh.setInputNormals (normals);
          vfh.setNormalizeBins (true);
          vfh.setNormalizeDistance (true);
          vfh.compute (vfh_signature);

          signatures.resize (1);
          centroids.resize (1);

          signatures[0] = vfh_signature;

          Eigen::Vector4f centroid4f;
          pcl::compute3DCentroid (*in, centroid4f);
          centroids[0] = Eigen::Vector3f (centroid4f[0], centroid4f[1], centroid4f[2]);

        }

        bool
        computedNormals ()
        {
          return true;
        }
      };
  }
}

#endif /* REC_FRAMEWORK_VFH_ESTIMATOR_H_ */
