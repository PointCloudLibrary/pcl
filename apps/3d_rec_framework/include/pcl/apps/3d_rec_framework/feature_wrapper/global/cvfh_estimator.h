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
#include <pcl/surface/mls.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
    class CVFHEstimation : public GlobalEstimator<PointInT, FeatureT>
    {

      typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
      using GlobalEstimator<PointInT, FeatureT>::normal_estimator_;
      using GlobalEstimator<PointInT, FeatureT>::normals_;
      float eps_angle_threshold_;
      float curvature_threshold_;
      float cluster_tolerance_factor_;
      bool normalize_bins_;
      bool adaptative_MLS_;

    public:

      CVFHEstimation ()
      {
        eps_angle_threshold_ = 0.13f;
        curvature_threshold_ = 0.035f;
        normalize_bins_ = true;
        cluster_tolerance_factor_ = 3.f;
      }

      void setCVFHParams(float p1, float p2, float p3) {
        eps_angle_threshold_ = p1;
        curvature_threshold_ = p2;
        cluster_tolerance_factor_ = p3;
      }

      void setAdaptativeMLS(bool b) {
        adaptative_MLS_ = b;
      }

      void
      estimate (PointInTPtr & in, PointInTPtr & processed,
                typename pcl::PointCloud<FeatureT>::CloudVectorType & signatures,
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > & centroids)
      {

        if (!normal_estimator_)
        {
          PCL_ERROR("This feature needs normals... please provide a normal estimator\n");
          return;
        }

        pcl::MovingLeastSquares<PointInT, PointInT> mls;
        if(adaptative_MLS_) {
          typename search::KdTree<PointInT>::Ptr tree;
          Eigen::Vector4f centroid_cluster;
          pcl::compute3DCentroid (*in, centroid_cluster);
          float dist_to_sensor = centroid_cluster.norm();
          float sigma = dist_to_sensor * 0.01f;
          mls.setSearchMethod(tree);
          mls.setSearchRadius (sigma);
          mls.setUpsamplingMethod (mls.SAMPLE_LOCAL_PLANE);
          mls.setUpsamplingRadius (0.002);
          mls.setUpsamplingStepSize (0.001);
        }

        normals_.reset (new pcl::PointCloud<pcl::Normal>);
        {
          normal_estimator_->estimate (in, processed, normals_);
        }

        if(adaptative_MLS_) {
          mls.setInputCloud(processed);

          PointInTPtr filtered(new pcl::PointCloud<PointInT>);
          mls.process(*filtered);

          processed.reset(new pcl::PointCloud<PointInT>);
          normals_.reset (new pcl::PointCloud<pcl::Normal>);
          {
            filtered->is_dense = false;
            normal_estimator_->estimate (filtered, processed, normals_);
          }
        }

        /*normals_.reset(new pcl::PointCloud<pcl::Normal>);
        normal_estimator_->estimate (in, processed, normals_);*/

        typedef typename pcl::CVFHEstimation<PointInT, pcl::Normal, FeatureT> CVFHEstimation;
        pcl::PointCloud<FeatureT> cvfh_signatures;
        typename pcl::search::KdTree<PointInT>::Ptr cvfh_tree (new pcl::search::KdTree<PointInT>);

        CVFHEstimation cvfh;
        cvfh.setSearchMethod (cvfh_tree);
        cvfh.setInputCloud (processed);
        cvfh.setInputNormals (normals_);

        cvfh.setEPSAngleThreshold (eps_angle_threshold_);
        cvfh.setCurvatureThreshold (curvature_threshold_);
        cvfh.setNormalizeBins (normalize_bins_);

        float radius = normal_estimator_->normal_radius_;
        float cluster_tolerance_radius = normal_estimator_->grid_resolution_ * cluster_tolerance_factor_;

        if (normal_estimator_->compute_mesh_resolution_)
        {
          radius = normal_estimator_->mesh_resolution_ * normal_estimator_->factor_normals_;
          cluster_tolerance_radius = normal_estimator_->mesh_resolution_ * cluster_tolerance_factor_;

          if (normal_estimator_->do_voxel_grid_)
          {
            radius *= normal_estimator_->factor_voxel_grid_;
            cluster_tolerance_radius *= normal_estimator_->factor_voxel_grid_;
          }
        }

        cvfh.setClusterTolerance (cluster_tolerance_radius);
        cvfh.setRadiusNormals (radius);
        cvfh.setMinPoints (100);

        //std::cout << "Res:" << normal_estimator_->mesh_resolution_ << " Radius normals:" << radius << " Cluster tolerance:" << cluster_tolerance_radius << " " << eps_angle_threshold_ << " " << curvature_threshold_ << std::endl;

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
        //std::cout << "centroids size:" << centroids.size () << std::endl;

      }

      bool
      computedNormals ()
      {
        return true;
      }

      void setNormalizeBins(bool b) {
        normalize_bins_ = b;
      }
    };
  }
}

#endif /* REC_FRAMEWORK_CVFH_ESTIMATOR_H_ */
