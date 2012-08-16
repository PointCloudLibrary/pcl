/*
 * global.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_GLOBAL_PIPELINE_H_
#define REC_FRAMEWORK_GLOBAL_PIPELINE_H_

#include <flann/flann.h>
#include <pcl/common/common.h>
#include <pcl/apps/3d_rec_framework/pc_source/source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>

namespace pcl
{
  namespace rec_3d_framework
  {

    template<typename PointInT>
    class PCL_EXPORTS GlobalClassifier {
      public:
      typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;

      virtual void
      setNN (int nn) = 0;

      virtual void
      getCategory (std::vector<std::string> & categories) = 0;

      virtual void
      getConfidence (std::vector<float> & conf) = 0;

      virtual void
      classify () = 0;

      virtual void
      setIndices (std::vector<int> & indices) = 0;

      virtual void
      setInputCloud (const PointInTPtr & cloud) = 0;
    };

    /**
     * \brief Nearest neighbor search based classification of PCL point type features.
     * FLANN is used to identify a neighborhood, based on which different scoring schemes
     * can be employed to obtain likelihood values for a specified list of classes.
     * Available features: ESF, VFH, CVFH
     * See apps/3d_rec_framework/tools/apps/global_classification.cpp for usage
     * \author Aitor Aldoma
     */

    template<template<class > class Distance, typename PointInT, typename FeatureT>
      class PCL_EXPORTS GlobalNNPipeline : public pcl::rec_3d_framework::GlobalClassifier<PointInT>
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
        typedef Distance<float> DistT;
        typedef Model<PointInT> ModelT;

        /** \brief Directory where the trained structure will be saved */
        std::string training_dir_;

        /** \brief Point cloud to be classified */
        PointInTPtr input_;

        /** \brief Model data source */
        typename boost::shared_ptr<pcl::rec_3d_framework::Source<PointInT> > source_;

        /** \brief Computes a feature */
        typename boost::shared_ptr<GlobalEstimator<PointInT, FeatureT> > estimator_;

        /** \brief Descriptor name */
        std::string descr_name_;

        typedef std::pair<ModelT, std::vector<float> > flann_model;
        flann::Matrix<float> flann_data_;
        flann::Index<DistT> * flann_index_;
        std::vector<flann_model> flann_models_;

        std::vector<int> indices_;

        //load features from disk and create flann structure
        void
        loadFeaturesAndCreateFLANN ();

        inline void
        convertToFLANN (const std::vector<flann_model> &models, flann::Matrix<float> &data)
        {
          data.rows = models.size ();
          data.cols = models[0].second.size (); // number of histogram bins

          flann::Matrix<float> flann_data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

          for (size_t i = 0; i < data.rows; ++i)
            for (size_t j = 0; j < data.cols; ++j)
            {
              flann_data.ptr ()[i * data.cols + j] = models[i].second[j];
            }

          data = flann_data;
        }

        void
        nearestKSearch (flann::Index<DistT> * index, const flann_model &model, int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

        int NN_;
        std::vector<std::string> categories_;
        std::vector<float> confidences_;

        std::string first_nn_category_;

      public:

        GlobalNNPipeline ()
        {
          NN_ = 1;
        }

        ~GlobalNNPipeline ()
        {
        }

        void
        setNN (int nn)
        {
          NN_ = nn;
        }

        void
        getCategory (std::vector<std::string> & categories)
        {
          categories = categories_;
        }

        void
        getConfidence (std::vector<float> & conf)
        {
          conf = confidences_;
        }

        /**
         * \brief Initializes the FLANN structure from the provided source
         */

        void
        initialize (bool force_retrain = false);

        /**
         * \brief Performs classification
         */

        void
        classify ();

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
        setFeatureEstimator (typename boost::shared_ptr<GlobalEstimator<PointInT, FeatureT> > & feat)
        {
          estimator_ = feat;
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
      };
  }
}
#endif /* REC_FRAMEWORK_GLOBAL_PIPELINE_H_ */
