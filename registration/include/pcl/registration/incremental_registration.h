#ifndef PCL_REGISTRATION_INCREMENTAL_REGISTRATION_H_
#define PCL_REGISTRATION_INCREMENTAL_REGISTRATION_H_

#include <pcl/pcl_base.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/registration/transformation_estimation_svd.h>

namespace pcl
{
  namespace registration
  {

    template <typename PointT>
    class IncrementalRegistration : public PCLBase<PointT>
    {
      using PCLBase<PointT>::initCompute;
      using PCLBase<PointT>::deinitCompute;

    public:

      using PCLBase<PointT>::indices_;
      using PCLBase<PointT>::input_;

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      IncrementalRegistration()
      {
        transform_incremental_ = Eigen::Matrix4f::Identity();

        downsampling_leaf_size_input_ = 0.05;
        downsampling_leaf_size_model_ = 0.05;
        registration_distance_threshold_ = 1.0;

        downsample_input_cloud = true;
        downsample_model_cloud = true;

        number_clouds_processed_ = 0;
      };

      virtual ~IncrementalRegistration(){};

      inline void
      align(PointCloud &output, bool use_vanilla_ICP = false)
      {

        /// INIT /////////////////////////////////////////////////////////////
        if ( !initCompute() )
          return;


        /// DOWNSAMPLING /////////////////////////////////////////////////////
        if ( downsample_input_cloud )
        {
          sor_.setInputCloud(input_);
          sor_.setIndices(indices_);
          sor_.setLeafSize(downsampling_leaf_size_input_, downsampling_leaf_size_input_, downsampling_leaf_size_input_);
          sor_.filter(cloud_input_);
          cloud_input_ptr_ = cloud_input_.makeShared();
        }
        else
        {
          cloud_input_ = *input_;
          cloud_input_ptr_ = cloud_input_.makeShared();
//          cloud_input_ptr_ = input_;
        }

        /// REGISTRATION /////////////////////////////////////////////////////
        if (number_clouds_processed_ == 0)
        {
          // no registration needed
          output = cloud_input_;
          cloud_model_ = cloud_input_;
          cloud_model_ptr_ = cloud_model_.makeShared();
          transform_incremental_.setIdentity();
        }
        else
        {
          pcl::transformPointCloud(cloud_input_, cloud_input_, transform_incremental_);

          Eigen::Matrix4f transform_current;
          transform_current.setIdentity();

          bool registration_successful = true;

          if ( !use_vanilla_ICP )
          {
            /// use GICP for registration //////////////////////////////////////
            reg_.setInputCloud(cloud_input_ptr_);
            reg_.setInputTarget(cloud_model_ptr_);
            reg_.setMaxCorrespondenceDistance(registration_distance_threshold_);
            reg_.setMaximumIterations(25);
            reg_.align(output);

            transform_current = reg_.getFinalTransformation();

            registration_successful = true;
            /// use GICP for registration //////////////////////////////////////
          }
          else
          {
            /// use normal icp /////////////////////////////////////////////////
            output = cloud_input_;
            unsigned int max_iterations = 30, n_iter = 0, n_iter_linear = max_iterations - 5;

            registration_successful = true;
            while ( ( n_iter++ < max_iterations ) && registration_successful )
            {
              float max_dist = registration_distance_threshold_;

              bool use_linear_distance_threshold = true;
              if ( use_linear_distance_threshold )
              {
                float dist_start = registration_distance_threshold_;
                float dist_stop = 1.5f * downsampling_leaf_size_model_;
                if ( n_iter < n_iter_linear )
                  max_dist = dist_start - n_iter * (dist_start - dist_stop) / (float)(max_iterations);
                else
                  max_dist = dist_stop;
              }

              // determine correspondences
              PointCloudConstPtr cloud_output_ptr = output.makeShared();

              pcl::CorrespondencesPtr correspondences_ptr (new pcl::Correspondences);
              corr_est_.setInputTarget(cloud_model_ptr_);
              corr_est_.setInputCloud(cloud_output_ptr);
              corr_est_.determineCorrespondences (*correspondences_ptr, max_dist);

              // remove one-to-n correspondences
              pcl::CorrespondencesPtr correspondeces_one_to_one_ptr (new pcl::Correspondences);
              cor_rej_one_to_one_.setInputCorrespondences (correspondences_ptr);
              cor_rej_one_to_one_.getCorrespondences (*correspondeces_one_to_one_ptr);

              // SAC-based correspondence rejection
              double sac_threshold = max_dist;
              int sac_max_iterations = 100;
              pcl::Correspondences correspondences_sac;
              cor_rej_sac_.setInputCloud (cloud_output_ptr);
              cor_rej_sac_.setTargetCloud (cloud_model_ptr_);
              cor_rej_sac_.setInlierThreshold (sac_threshold);
              cor_rej_sac_.setMaxIterations (sac_max_iterations);
              cor_rej_sac_.setInputCorrespondences (correspondeces_one_to_one_ptr);
              cor_rej_sac_.getCorrespondences (correspondences_sac);

              unsigned int nr_min_correspondences = 10;
              if (correspondences_sac.size() < nr_min_correspondences)
              {
                registration_successful = false;
                break;
              }

              Eigen::Matrix4f transform_svd;
              trans_est_.estimateRigidTransformation(output, cloud_model_, correspondences_sac, transform_svd);

              pcl::transformPointCloud(output, output, transform_svd);

              transform_current = transform_svd * transform_current;
            }

          }

          if ( registration_successful )
          {
            transform_incremental_ = transform_current * transform_incremental_;

            // setting up new model
            addCloudToModel(output);
            if ( downsample_model_cloud )
              subsampleModel();
          }

        }

        ++number_clouds_processed_;

        /// DEINIT ///////////////////////////////////////////////////////////
        deinitCompute ();
      }

      /** get the incrementally point map */
      inline PointCloud* getModel() { return &cloud_model_; };

      /** reset the incremental registration (re-empty point model and reset transformation matrix) */
      inline void reset() { number_clouds_processed_ = 0; };

      /** get latest (global) transformation */
      inline Eigen::Matrix4f getTransformation() { return transform_incremental_; };
      


      /** set the leaf size used for downsampling the input cloud */
      inline void setDownsamplingLeafSizeInput(double leaf_size) { downsampling_leaf_size_input_ = leaf_size; };
      /** get the leaf size used for downsampling the input cloud */
      inline double getDownsamplingLeafSizeInput() { return downsampling_leaf_size_input_; };

      /** set the leaf size used for downsampling the incrementally built model */
      inline void setDownsamplingLeafSizeModel(double leaf_size) { downsampling_leaf_size_model_ = leaf_size; };
      /** get the leaf size used for downsampling the incrementally built model */
      inline void getDownsamplingLeafSizeModel() { return downsampling_leaf_size_model_; };

      /** set the distance threshold for the ICP-based registration */
      inline void setRegistrationDistanceThreshold(double threshold) { registration_distance_threshold_ = threshold; };
      /** get the distance threshold for the ICP-based registration */
      inline void getRegistrationDistanceThreshold() { return registration_distance_threshold_; };

      /** enable downsampling of the input cloud. The leaf size is set using \a setDownsamplingLeafSizeInput */
      inline void downsampleInputCloud(bool enable) { downsample_input_cloud = enable; };
      /** enable downsampling of the incrementally built model cloud. The leaf size is set using \a setDownsamplingLeafSizeModel */
      inline void downsampleModelCloud(bool enable) { downsample_model_cloud = enable; };

    protected:

      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> reg_;
      pcl::VoxelGrid<PointT> sor_;
      PointCloud cloud_input_, cloud_model_;
      PointCloudConstPtr cloud_input_ptr_, cloud_model_ptr_;

      Eigen::Matrix4f transform_incremental_;

      inline void addCloudToModel(PointCloud& cloud)
      {
        cloud_model_ += cloud;
        cloud_model_ptr_ = cloud_model_.makeShared();
      }

      inline void subsampleModel()
      {
        PointCloud cloud_temp;
        sor_.setInputCloud(cloud_model_ptr_);
        sor_.setLeafSize (static_cast<float> (downsampling_leaf_size_model_), 
                          static_cast<float> (downsampling_leaf_size_model_), 
                          static_cast<float> (downsampling_leaf_size_model_));
        sor_.filter(cloud_temp);
        cloud_model_ = cloud_temp;
        cloud_model_ptr_ = cloud_model_.makeShared();
      }

      bool downsample_input_cloud;
      bool downsample_model_cloud;
      double downsampling_leaf_size_input_;
      double downsampling_leaf_size_model_;
      double registration_distance_threshold_;

      unsigned int number_clouds_processed_;

      pcl::registration::CorrespondenceEstimation<PointT, PointT> corr_est_;
      pcl::registration::CorrespondenceRejectorOneToOne cor_rej_one_to_one_;
      pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> cor_rej_sac_;
      pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est_;
    };
  }
}

#endif /* PCL_REGISTRATION_INCREMENTAL_REGISTRATION_H_ */
