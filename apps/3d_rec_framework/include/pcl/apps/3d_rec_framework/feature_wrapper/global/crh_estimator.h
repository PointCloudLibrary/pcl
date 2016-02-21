/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_CRH_ESTIMATOR_H_
#define REC_FRAMEWORK_CRH_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/features/crh.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
    class CRHEstimation : public GlobalEstimator<PointInT, FeatureT>
    {

      typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
      using GlobalEstimator<PointInT, FeatureT>::normal_estimator_;
      using GlobalEstimator<PointInT, FeatureT>::normals_;

      typename boost::shared_ptr<GlobalEstimator<PointInT, FeatureT> > feature_estimator_;
      typedef pcl::PointCloud<pcl::Histogram<90> > CRHPointCloud;
      std::vector< CRHPointCloud::Ptr > crh_histograms_;

    public:

      CRHEstimation ()
      {

      }

      void
      setFeatureEstimator(typename boost::shared_ptr<GlobalEstimator<PointInT, FeatureT> > & feature_estimator) {
        feature_estimator_ = feature_estimator;
      }

      void
      estimate (PointInTPtr & in, PointInTPtr & processed,
                std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > & centroids)
      {

        if (!feature_estimator_)
        {
          PCL_ERROR("CRHEstimation needs a global estimator... please provide one\n");
          return;
        }

        feature_estimator_->estimate(in, processed, signatures, centroids);

        if(!feature_estimator_->computedNormals()) {
          normals_.reset(new pcl::PointCloud<pcl::Normal>);
          normal_estimator_->estimate (in, processed, normals_);
        } else {
          feature_estimator_->getNormals(normals_);
        }

        crh_histograms_.resize(signatures.size());

        typedef typename pcl::CRHEstimation<PointInT, pcl::Normal, pcl::Histogram<90> > CRHEstimation;
        CRHEstimation crh;
        crh.setInputCloud(processed);
        crh.setInputNormals(normals_);

        for (size_t idx = 0; idx < signatures.size (); idx++)
        {
          Eigen::Vector4f centroid4f(centroids[idx][0],centroids[idx][1],centroids[idx][2],0);
          crh.setCentroid(centroid4f);
          crh_histograms_[idx].reset(new CRHPointCloud());
          crh.compute (*crh_histograms_[idx]);
        }

      }

      void getCRHHistograms(std::vector< CRHPointCloud::Ptr > & crh_histograms) {
        crh_histograms = crh_histograms_;
      }

      bool
      computedNormals ()
      {
        return true;
      }
    };
  }
}

#endif /* REC_FRAMEWORK_CVFH_ESTIMATOR_H_ */
