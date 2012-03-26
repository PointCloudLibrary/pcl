/*
 * shot_local_estimator.h
 *
 *  Created on: Mar 24, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_SHOT_LOCAL_ESTIMATOR_H_
#define REC_FRAMEWORK_SHOT_LOCAL_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/local/local_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/features/shot.h>
#include <pcl/io/pcd_io.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
      class SHOTLocalEstimation : public LocalEstimator<PointInT, FeatureT>
      {

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;
        typedef pcl::PointCloud<int> KeypointCloud;

        using LocalEstimator<PointInT, FeatureT>::support_radius_;
        using LocalEstimator<PointInT, FeatureT>::normal_estimator_;
        using LocalEstimator<PointInT, FeatureT>::keypoint_extractor_;

      public:
        bool
        estimate (PointInTPtr & in, PointInTPtr & processed, KeypointCloud & keypoints, FeatureTPtr & signatures)
        {

          if (!normal_estimator_)
          {
            PCL_ERROR("SHOTLocalEstimation :: This feature needs normals... please provide a normal estimator\n");
            return false;
          }

          if (!keypoint_extractor_)
          {
            PCL_ERROR("SHOTLocalEstimation :: This feature needs a keypoint extractor... please provide one\n");
            return false;
          }

          //compute normals
          pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
          normal_estimator_->estimate (in, processed, normals);

          //compute keypoints
          keypoint_extractor_->setInputCloud (processed);
          keypoint_extractor_->compute (keypoints);
          if (keypoints.points.size () == 0)
          {
            PCL_WARN("SHOTLocalEstimation :: No keypoints were found\n");
            return false;
          }

          //compute signatures
          typedef typename pcl::SHOTEstimation<PointInT, pcl::Normal, pcl::SHOT> SHOTEstimator;
          typename pcl::search::KdTree<PointInT>::Ptr tree (new pcl::search::KdTree<PointInT>);

          boost::shared_ptr < std::vector<int> > indices (new std::vector<int> ());
          indices->resize (keypoints.points.size ());
          for (size_t i = 0; i < indices->size (); i++)
            (*indices)[i] = keypoints.points[i];

          pcl::PointCloud<pcl::SHOT>::Ptr shots (new pcl::PointCloud<pcl::SHOT>);
          SHOTEstimator shot_estimate;
          shot_estimate.setSearchMethod (tree);
          shot_estimate.setIndices (indices);
          shot_estimate.setInputCloud (processed);
          shot_estimate.setInputNormals (normals);
          shot_estimate.setRadiusSearch (support_radius_);
          shot_estimate.compute (*shots);
          signatures->resize (shots->points.size ());
          signatures->width = static_cast<int> (shots->points.size ());
          signatures->height = 1;

          int size_feat = sizeof(signatures->points[0].histogram) / sizeof(float);

          for (size_t k = 0; k < shots->points.size (); k++)
            for (int i = 0; i < size_feat; i++)
              signatures->points[k].histogram[i] = shots->points[k].descriptor[i];

          return true;

        }

      };
  }
}

#endif /* REC_FRAMEWORK_SHOT_LOCAL_ESTIMATOR_H_ */
