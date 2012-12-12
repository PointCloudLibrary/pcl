/*
 * local_recognizer.h
 *
 *  Created on: Mar 24, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_LOCAL_RECOGNIZER_H_
#define REC_FRAMEWORK_LOCAL_RECOGNIZER_H_

//#include <opencv2/opencv.hpp>
#include <flann/flann.h>
#include <pcl/common/common.h>
#include <pcl/apps/3d_rec_framework/pc_source/source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/local/local_estimator.h>
#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/recognition/hv/hypotheses_verification.h>
#include <pcl/visualization/pcl_visualizer.h>

inline bool
correspSorter (const pcl::Correspondence & i, const pcl::Correspondence & j)
{
  return (i.distance < j.distance);
}

namespace pcl
{
  namespace rec_3d_framework
  {
    /**
     * \brief Object recognition + 6DOF pose based on local features, GC and HV
     * Contains keypoints/local features computation, matching using FLANN,
     * point-to-point correspondence grouping, pose refinement and hypotheses verification
     * Available features: SHOT, FPFH
     * See apps/3d_rec_framework/tools/apps for usage
     * \author Aitor Aldoma, Federico Tombari
     */

    template<template<class > class Distance, typename PointInT, typename FeatureT>
      class PCL_EXPORTS LocalRecognitionPipeline
      {

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<PointInT>::ConstPtr ConstPointInTPtr;

        typedef Distance<float> DistT;
        typedef Model<PointInT> ModelT;

        /** \brief Directory where the trained structure will be saved */
        std::string training_dir_;

        /** \brief Point cloud to be classified */
        PointInTPtr input_;

        /** \brief Model data source */
        typename boost::shared_ptr<Source<PointInT> > source_;

        /** \brief Computes a feature */
        typename boost::shared_ptr<LocalEstimator<PointInT, FeatureT> > estimator_;

        /** \brief Point-to-point correspondence grouping algorithm */
        typename boost::shared_ptr<CorrespondenceGrouping<PointInT, PointInT> > cg_algorithm_;

        /** \brief Hypotheses verification algorithm */
        typename boost::shared_ptr<HypothesisVerification<PointInT, PointInT> > hv_algorithm_;

        /** \brief Descriptor name */
        std::string descr_name_;

        /** \brief Id of the model to be used */
        std::string search_model_;

        bool compute_table_plane_;

        class flann_model
        {
        public:
          ModelT model;
          int view_id;
          int keypoint_id;
          std::vector<float> descr;
        };

        flann::Matrix<float> flann_data_;
        flann::Index<DistT> * flann_index_;
        std::vector<flann_model> flann_models_;

        std::vector<int> indices_;

        bool use_cache_;
        std::map<std::pair<std::string, int>, Eigen::Matrix4f, std::less<std::pair<std::string, int> >, Eigen::aligned_allocator<std::pair<std::pair<
            std::string, int>, Eigen::Matrix4f> > > poses_cache_;
        std::map<std::pair<std::string, int>, typename pcl::PointCloud<PointInT>::Ptr> keypoints_cache_;

        float threshold_accept_model_hypothesis_;
        int ICP_iterations_;

        boost::shared_ptr<std::vector<ModelT> > models_;
        boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_;

        int kdtree_splits_;
        float VOXEL_SIZE_ICP_;

        PointInTPtr keypoints_input_;
        PointInTPtr processed_;
        typename pcl::PointCloud<FeatureT>::Ptr signatures_;

        //load features from disk and create flann structure
        void
        loadFeaturesAndCreateFLANN ();

        inline void
        convertToFLANN (const std::vector<flann_model> &models, flann::Matrix<float> &data)
        {
          data.rows = models.size ();
          data.cols = models[0].descr.size (); // number of histogram bins

          flann::Matrix<float> flann_data (new float[models.size () * models[0].descr.size ()], models.size (), models[0].descr.size ());

          for (size_t i = 0; i < data.rows; ++i)
            for (size_t j = 0; j < data.cols; ++j)
            {
              flann_data.ptr ()[i * data.cols + j] = models[i].descr[j];
            }

          data = flann_data;
        }

        void
        nearestKSearch (flann::Index<DistT> * index, const flann_model &model, int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

        class ObjectHypothesis
        {
        public:
          ModelT model_;
          typename pcl::PointCloud<PointInT>::Ptr correspondences_pointcloud; //points in model coordinates
          boost::shared_ptr<std::vector<float> > feature_distances_;
          pcl::CorrespondencesPtr correspondences_to_inputcloud; //indices between correspondences_pointcloud and scene cloud
        };

        void
        getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix);

        void
        getKeypoints (ModelT & model, int view_id, typename pcl::PointCloud<PointInT>::Ptr & keypoints_cloud);

        void
        drawCorrespondences (PointInTPtr & cloud, ObjectHypothesis & oh, PointInTPtr & keypoints_pointcloud, pcl::Correspondences & correspondences)
        {
          pcl::visualization::PCLVisualizer vis_corresp_;
          vis_corresp_.setWindowName("correspondences...");
          pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler (cloud, 255, 0, 0);
          vis_corresp_.addPointCloud<PointInT> (cloud, random_handler, "points");

          typename pcl::PointCloud<PointInT>::ConstPtr cloud_sampled;
          cloud_sampled = oh.model_.getAssembled (0.0025f);

          pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler_sampled (cloud_sampled, 0, 0, 255);
          vis_corresp_.addPointCloud<PointInT> (cloud_sampled, random_handler_sampled, "sampled");

          for (size_t kk = 0; kk < correspondences.size (); kk++)
          {
            pcl::PointXYZ p;
            p.getVector4fMap () = oh.correspondences_pointcloud->points[correspondences[kk].index_query].getVector4fMap ();
            pcl::PointXYZ p_scene;
            p_scene.getVector4fMap () = keypoints_pointcloud->points[correspondences[kk].index_match].getVector4fMap ();

            std::stringstream line_name;
            line_name << "line_" << kk;

            vis_corresp_.addLine<pcl::PointXYZ, pcl::PointXYZ> (p_scene, p, line_name.str ());
          }

          vis_corresp_.spin ();
          vis_corresp_.removeAllPointClouds();
          vis_corresp_.removeAllShapes();
          vis_corresp_.close();
        }

      public:

        LocalRecognitionPipeline ()
        {
          use_cache_ = false;
          threshold_accept_model_hypothesis_ = 0.2f;
          ICP_iterations_ = 30;
          kdtree_splits_ = 512;
          search_model_ = "";
          VOXEL_SIZE_ICP_ = 0.0025f;
          compute_table_plane_ = false;
        }

        void setISPK(typename pcl::PointCloud<FeatureT>::Ptr & signatures, PointInTPtr & p, PointInTPtr & keypoints)
        {
          keypoints_input_ = keypoints;
          signatures_ = signatures;
          processed_ = p;
        }

        void setVoxelSizeICP(float s) {
          VOXEL_SIZE_ICP_ = s;
        }
        void
        setSearchModel (std::string & id)
        {
          search_model_ = id;
        }

        void
        setThresholdAcceptHyp (float t)
        {
          threshold_accept_model_hypothesis_ = t;
        }

        ~LocalRecognitionPipeline ()
        {

        }

        void
        setKdtreeSplits (int n)
        {
          kdtree_splits_ = n;
        }

        void
        setIndices (std::vector<int> & indices)
        {
          indices_ = indices;
        }

        void
        setICPIterations (int it)
        {
          ICP_iterations_ = it;
        }

        void
        setUseCache (bool u)
        {
          use_cache_ = u;
        }

        boost::shared_ptr<std::vector<ModelT> >
        getModels ()
        {
          return models_;
        }

        boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > >
        getTransforms ()
        {
          return transforms_;
        }

        /**
         * \brief Sets the model data source_
         */
        void
        setDataSource (typename boost::shared_ptr<Source<PointInT> > & source)
        {
          source_ = source;
        }

        typename boost::shared_ptr<Source<PointInT> >
        getDataSource ()
        {
          return source_;
        }

        /**
         * \brief Sets the local feature estimator
         */
        void
        setFeatureEstimator (typename boost::shared_ptr<LocalEstimator<PointInT, FeatureT> > & feat)
        {
          estimator_ = feat;
        }

        /**
         * \brief Sets the CG algorithm
         */
        void
        setCGAlgorithm (typename boost::shared_ptr<CorrespondenceGrouping<PointInT, PointInT> > & alg)
        {
          cg_algorithm_ = alg;
        }

        /**
         * \brief Sets the HV algorithm
         */
        void
        setHVAlgorithm (typename boost::shared_ptr<HypothesisVerification<PointInT, PointInT> > & alg)
        {
          hv_algorithm_ = alg;
        }

        /**
         * \brief Sets the input cloud to be classified
         */
        void
        setInputCloud (const PointInTPtr & cloud)
        {
          input_ = cloud;
        }

        /**
         * \brief Sets the descriptor name
         */
        void
        setDescriptorName (std::string & name)
        {
          descr_name_ = name;
        }

        /**
         * \brief Filesystem dir where to keep the generated training data
         */
        void
        setTrainingDir (std::string & dir)
        {
          training_dir_ = dir;
        }

        void
        setComputeTablePlane(bool b) {
          compute_table_plane_ = b;
        }

        void
        getProcessed(PointInTPtr & cloud) {
          cloud = processed_;
        }

        /**
         * \brief Initializes the FLANN structure from the provided source
         * It does training for the models that havent been trained yet
         */

        void
        initialize (bool force_retrain = false);

        /**
         * \brief Performs recognition and pose estimation on the input cloud
         */

        void
        recognize ();
      };
  }
}

#endif /* REC_FRAMEWORK_LOCAL_RECOGNIZER_H_ */
