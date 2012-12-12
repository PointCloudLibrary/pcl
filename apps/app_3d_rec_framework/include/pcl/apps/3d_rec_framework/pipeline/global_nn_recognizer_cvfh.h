/*
 * global.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_GLOBAL_RECOGNIZER_CVFH_H_
#define REC_FRAMEWORK_GLOBAL_RECOGNIZER_CVFH_H_

#include <flann/flann.h>
#include <pcl/common/common.h>
#include <pcl/apps/3d_rec_framework/pc_source/source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/ourcvfh_estimator.h>
#include <pcl/recognition/hv/hypotheses_verification.h>

namespace pcl
{
  namespace rec_3d_framework
  {

    /**
     * \brief Nearest neighbor search based classification of PCL point type features.
     * Available features: CVFH
     * \author Aitor Aldoma
     */

    template<template<class > class Distance, typename PointInT, typename FeatureT = pcl::VFHSignature308>
      class PCL_EXPORTS GlobalNNCVFHRecognizer
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

        /** \brief Directory where the trained structure will be saved */
        std::string training_dir_;

        /** \brief Point cloud to be classified */
        PointInTPtr input_;

        /** \brief Model data source */
        typename boost::shared_ptr<pcl::rec_3d_framework::Source<PointInT> > source_;

        /** \brief Computes a feature */
        typename boost::shared_ptr<OURCVFHEstimator<PointInT, FeatureT> > micvfh_estimator_;

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

          bool
          operator< (const flann_model &other) const
          {
            if ((this->model.id_.compare (other.model.id_) < 0))
            {
              return true;
            }
            else
            {

              if (this->model.id_.compare (other.model.id_) == 0)
              {
                //check view id
                if ((this->view_id < other.view_id))
                {
                  return true;
                }
                else
                {
                  if (this->view_id == other.view_id)
                  {
                    if (this->descriptor_id < other.descriptor_id)
                    {
                      return true;
                    }
                  }
                }
              }
            }

            return false;
          }

          bool
          operator== (const flann_model &other) const
          {
            return (model.id_ == other.model.id_) && (view_id == other.view_id) && (descriptor_id == other.descriptor_id);
          }

        };

        flann::Matrix<float> flann_data_;
        flann::Index<DistT> * flann_index_;
        std::vector<flann_model> flann_models_;

        std::vector<flann::Matrix<float> > single_categories_data_;
        std::vector<flann::Index<DistT> *> single_categories_index_;
        std::vector<boost::shared_ptr<std::vector<int> > > single_categories_pointers_to_models_;
        std::map<std::string, int> category_to_vectors_indices_;
        std::vector<std::string> categories_to_be_searched_;
        bool use_single_categories_;

        bool use_cache_;
        std::map<std::pair<std::string, int>, Eigen::Matrix4f, std::less<std::pair<std::string, int> >, Eigen::aligned_allocator<std::pair<std::pair<
            std::string, int>, Eigen::Matrix4f> > > poses_cache_;
        std::map<std::pair<std::string, int>, Eigen::Vector3f> centroids_cache_;

        std::vector<int> indices_;

        bool compute_scale_;

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

        inline void
        convertToFLANN (const std::vector<flann_model> &models, boost::shared_ptr<std::vector<int> > & indices, flann::Matrix<float> &data)
        {
          data.rows = indices->size ();
          data.cols = models[0].descr.size (); // number of histogram bins

          flann::Matrix<float> flann_data(new float[indices->size () * models[0].descr.size ()],indices->size(),models[0].descr.size ());

          for (size_t i = 0; i < data.rows; ++i)
            for (size_t j = 0; j < data.cols; ++j)
            {
              flann_data.ptr()[i * data.cols + j] = models[indices->at(i)].descr[j];
            }

          data = flann_data;
        }

        void
        nearestKSearch (flann::Index<DistT> * index, const flann_model &model, int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

        void
        getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix);

        bool
        getRollPose (ModelT & model, int view_id, int d_id, Eigen::Matrix4f & pose_matrix);

        void
        getCentroid (ModelT & model, int view_id, int d_id, Eigen::Vector3f & centroid);

        void
        getView (ModelT & model, int view_id, PointInTPtr & view);

        int NN_;

        boost::shared_ptr<std::vector<ModelT> > models_;
        boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_;

        std::vector<float> descriptor_distances_;

      public:

        GlobalNNCVFHRecognizer ()
        {
          ICP_iterations_ = 0;
          noisify_ = false;
          compute_scale_ = false;
          use_single_categories_ = false;
        }

        ~GlobalNNCVFHRecognizer ()
        {
        }

        void
        getDescriptorDistances (std::vector<float> & ds)
        {
          ds = descriptor_distances_;
        }

        void
        setComputeScale (bool d)
        {
          compute_scale_ = d;
        }

        void
        setCategoriesToUseForRecognition (std::vector<std::string> & cats_to_use)
        {
          categories_to_be_searched_.clear ();
          categories_to_be_searched_ = cats_to_use;
        }

        void setUseSingleCategories(bool b) {
          use_single_categories_ = b;
        }

        void
        setNoise (float n)
        {
          noisify_ = true;
          noise_ = n;
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
        setFeatureEstimator (typename boost::shared_ptr<OURCVFHEstimator<PointInT, FeatureT> > & feat)
        {
          micvfh_estimator_ = feat;
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

      };
  }
}
#endif /* REC_FRAMEWORK_GLOBAL_PIPELINE_H_ */
