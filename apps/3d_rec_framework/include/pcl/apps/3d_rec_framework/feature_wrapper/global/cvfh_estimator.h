/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_CVFH_ESTIMATOR_H_
#define REC_FRAMEWORK_CVFH_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/features/cvfh.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
    class CVFHEstimation : public GlobalEstimator<PointInT, FeatureT>
    {

      typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
      using GlobalEstimator<PointInT, FeatureT>::normal_estimator_;
      float eps_angle_threshold_;
      float curvature_threshold_;
      bool normalize_bins_;

    public:

      CVFHEstimation ()
      {
        eps_angle_threshold_ = 0.13f;
        curvature_threshold_ = 0.035f;
      }

      void
      estimate (PointInTPtr & in, PointInTPtr & processed,
                typename pcl::PointCloud<FeatureT>::CloudVectorType & signatures,
                std::vector<Eigen::Vector3f> & centroids)
      {

        if (!normal_estimator_)
        {
          PCL_ERROR("This feature needs normals... please provide a normal estimator\n");
          return;
        }

        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        normal_estimator_->estimate (in, processed, normals);

        typedef typename pcl::CVFHEstimation<PointInT, pcl::Normal, FeatureT> CVFHEstimation;
        pcl::PointCloud<FeatureT> cvfh_signatures;
        typename pcl::search::KdTree<PointInT>::Ptr cvfh_tree (new pcl::search::KdTree<PointInT>);

        CVFHEstimation cvfh;
        cvfh.setSearchMethod (cvfh_tree);
        cvfh.setInputCloud (processed);
        cvfh.setInputNormals (normals);

        cvfh.setEPSAngleThreshold (eps_angle_threshold_);
        cvfh.setCurvatureThreshold (curvature_threshold_);
        cvfh.setNormalizeBins (normalize_bins_);

        float radius = normal_estimator_->normal_radius_;
        float cluster_tolerance_radius = normal_estimator_->grid_resolution_ * 3.f;

        if (normal_estimator_->compute_mesh_resolution_)
        {
          radius = normal_estimator_->mesh_resolution_ * normal_estimator_->factor_normals_;
          cluster_tolerance_radius = normal_estimator_->mesh_resolution_ * 3.f;

          if (normal_estimator_->do_voxel_grid_)
          {
            radius *= normal_estimator_->factor_voxel_grid_;
            cluster_tolerance_radius *= normal_estimator_->factor_voxel_grid_;
          }
        }

        cvfh.setClusterTolerance (cluster_tolerance_radius);
        cvfh.setRadiusNormals (radius);
        cvfh.setMinPoints (50);

        cvfh.compute (cvfh_signatures);

        for (size_t i = 0; i < cvfh_signatures.points.size (); i++)
        {
          pcl::PointCloud<FeatureT> vfh_signature;
          vfh_signature.points.resize (1);
          vfh_signature.width = vfh_signature.height = 1;
          for (int d = 0; d < 308; ++d)
            vfh_signature.points[0].histogram[d] = cvfh_signatures.points[i].histogram[d];

          signatures.push_back (vfh_signature);
        }

        cvfh.getCentroidClusters (centroids);

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
