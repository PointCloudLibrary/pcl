/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_ESF_ESTIMATOR_H_
#define REC_FRAMEWORK_ESF_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/features/esf.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
      class ESFEstimation : public GlobalEstimator<PointInT, FeatureT>
      {

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;

      public:
        void
        estimate (PointInTPtr & in, PointInTPtr & processed,
                  typename pcl::PointCloud<FeatureT>::CloudVectorType & signatures,
                  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > & centroids)
        {

          typedef typename pcl::ESFEstimation<PointInT, FeatureT> ESFEstimation;
          pcl::PointCloud<FeatureT> ESF_signature;

          ESFEstimation esf;
          esf.setInputCloud (in);
          esf.compute (ESF_signature);

          signatures.resize (1);
          centroids.resize (1);

          signatures[0] = ESF_signature;

          Eigen::Vector4f centroid4f;
          pcl::compute3DCentroid (*in, centroid4f);
          centroids[0] = Eigen::Vector3f (centroid4f[0], centroid4f[1], centroid4f[2]);

          pcl::copyPointCloud(*in, *processed);
          //processed = in;
        }

        bool
        computedNormals ()
        {
          return false;
        }
      };
  }
}

#endif /* REC_FRAMEWORK_ESF_ESTIMATOR_H_ */
