/*
 * global.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_GLOBAL_RECOGNIZER_CRH_H_
#define REC_FRAMEWORK_GLOBAL_RECOGNIZER_CRH_H_

#include <flann/flann.h>
#include <pcl/common/common.h>
#include <pcl/apps/3d_rec_framework/pc_source/source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/crh_estimator.h>
#include <pcl/recognition/hv/hypotheses_verification.h>

namespace pcl
{
  namespace rec_3d_framework
  {

    /**
     * \brief Nearest neighbor search based classification of PCL point type features.
     * FLANN is used to identify a neighborhood, based on which different scoring schemes
     * can be employed to obtain likelihood values for a specified list of classes.
     * Available features: ESF, VFH, CVFH
     * \author Aitor Aldoma
     */

    template<template<class > class Distance, typename PointInT, typename FeatureT>
    class PCL_EXPORTS GlobalNNCRHRecognizer
    {

    protected:

      struct index_score
      {
        int idx_models_;
        int idx_input_;
        double score_;
      };

      struct sortIndexScores
      {
        bool
        operator() (const index_score& d1, const index_score& d2)
        {
          return d1.score_ < d2.score_;
        }
      } sortIndexScoresOp;

      typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
      typedef typename pcl::PointCloud<PointInT>::ConstPtr ConstPointInTPtr;

      typedef Distance<float> DistT;
      typedef Model<PointInT> ModelT;
      typedef pcl::PointCloud<pcl::Histogram<90> > CRHPointCloud;

      /** \brief Directory where the trained structure will be saved */
      std::string training_dir_;

      /** \brief Point cloud to be classified */
      PointInTPtr input_;

      /** \brief Model data source */
      typename boost::shared_ptr<pcl::rec_3d_framework::Source<PointInT> > source_;

      /** \brief Computes a feature */
      typename boost::shared_ptr<CRHEstimation<PointInT, FeatureT> > crh_estimator_;

      /** \brief Hypotheses verification algorithm */
      typename boost::shared_ptr<HypothesisVerification<PointInT, PointInT> > hv_algorithm_;

      /** \brief Descriptor name */
      std::string descr_name_;

      int ICP_iterations_;

      bool noisify_;
      float noise_;

      class flann_model
      {
      public:
        ModelT model;
        int view_id;
        int descriptor_id;
        std::vector<float> descr;
      };

      flann::Matrix<float> flann_data_;
      flann::Index<DistT> * flann_index_;
      std::vector<flann_model> flann_models_;


      bool use_cache_;
      std::map<std::pair<std::string, int>, Eigen::Matrix4f, std::less<std::pair<std::string, int> >, Eigen::aligned_allocator<std::pair<std::pair<
          std::string, int>, Eigen::Matrix4f> > > poses_cache_;
      std::map<std::pair<std::string, int>, Eigen::Vector3f > centroids_cache_;

      std::vector<int> indices_;

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

      void
      getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix);

      void
      getCRH (ModelT & model, int view_id, int d_id, CRHPointCloud::Ptr & hist);

      void
      getCentroid (ModelT & model, int view_id, int d_id, Eigen::Vector3f & centroid);

      void
      getView (ModelT & model, int view_id, PointInTPtr & view);

      int NN_;

      boost::shared_ptr<std::vector<ModelT> > models_;
      boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_;

    public:

      GlobalNNCRHRecognizer ()
      {
        ICP_iterations_ = 0;
        noisify_ = false;
        do_CRH_ = true;
      }

      ~GlobalNNCRHRecognizer ()
      {
      }

      void setNoise(float n) {
        noisify_ = true;
        noise_ = n;
      }

      void setDOCRH(bool b) {
        do_CRH_ = b;
      }

      void
      setNN (int nn)
      {
        NN_ = nn;
      }

      void
      setICPIterations (int it)
      {
        ICP_iterations_ = it;
      }

      /**
       * \brief Initializes the FLANN structure from the provided source
       */

      void
      initialize (bool force_retrain = false);

      /**
       * \brief Sets the model data source_
       */
      void
      setDataSource (typename boost::shared_ptr<Source<PointInT> > & source)
      {
        source_ = source;
      }

      /**
       * \brief Sets the model data source_
       */

      void
      setFeatureEstimator (typename boost::shared_ptr<CRHEstimation<PointInT, FeatureT> > & feat)
      {
        crh_estimator_ = feat;
      }

      /**
       * \brief Sets the HV algorithm
       */
      void
      setHVAlgorithm (typename boost::shared_ptr<HypothesisVerification<PointInT, PointInT> > & alg)
      {
        hv_algorithm_ = alg;
      }

      void
      setIndices (std::vector<int> & indices)
      {
        indices_ = indices;
      }

      /**
       * \brief Sets the input cloud to be classified
       */
      void
      setInputCloud (const PointInTPtr & cloud)
      {
        input_ = cloud;
      }

      void
      setDescriptorName (std::string & name)
      {
        descr_name_ = name;
      }

      void
      setTrainingDir (std::string & dir)
      {
        training_dir_ = dir;
      }

      /**
       * \brief Performs recognition on the input cloud
       */

      void
      recognize ();

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

      void
      setUseCache (bool u)
      {
        use_cache_ = u;
      }

      bool do_CRH_;

    };
  }
}
#endif /* REC_FRAMEWORK_GLOBAL_PIPELINE_H_ */
