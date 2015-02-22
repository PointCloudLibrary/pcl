/*
 * estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_GLOBAL_ESTIMATOR_H_
#define REC_FRAMEWORK_GLOBAL_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template <typename PointInT, typename FeatureT>
    class GlobalEstimator {
      protected:
        bool computed_normals_;
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;

        typename boost::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > normal_estimator_;

        pcl::PointCloud<pcl::Normal>::Ptr normals_;

      public:
        virtual void
        estimate (PointInTPtr & in, PointInTPtr & processed, std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<
            pcl::PointCloud<FeatureT> > > & signatures, std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > & centroids)=0;

        virtual bool computedNormals() = 0;

        void setNormalEstimator(boost::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > & ne) {
          normal_estimator_ = ne;
        }

        void getNormals(pcl::PointCloud<pcl::Normal>::Ptr & normals) {
          normals = normals_;
        }

    };
  }
}


#endif /* REC_FRAMEWORK_ESTIMATOR_H_ */
