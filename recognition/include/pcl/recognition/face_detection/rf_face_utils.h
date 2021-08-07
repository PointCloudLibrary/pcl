/*
 * fanellis_face_detector.h
 *
 *  Created on: 22 Sep 2012
 *      Author: Aitor Aldoma
 */

#pragma once

#include "pcl/recognition/face_detection/face_common.h"
#include <pcl/ml/feature_handler.h>
#include <pcl/ml/stats_estimator.h>
#include <pcl/ml/branch_estimator.h>

namespace pcl
{
  namespace face_detection
  {
    template<class FT, class DataSet, class ExampleIndex>
    class FeatureHandlerDepthAverage: public pcl::FeatureHandler<FT, DataSet, ExampleIndex>
    {

      private:
        int wsize_; //size of the window
        int max_patch_size_; //max size of the smaller patches
        int num_channels_; //the number of feature channels
        float min_valid_small_patch_depth_; //percentage of valid depth in a small patch
      public:

        FeatureHandlerDepthAverage()
        {
          wsize_ = 80;
          max_patch_size_ = 40;
          num_channels_ = 1;
          min_valid_small_patch_depth_ = 0.5f;
        }

        /** \brief Sets the size of the window to extract features.
         * \param[in] w Window size.
         */
        void setWSize(int w)
        {
          wsize_ = w;
        }

        /** \brief Sets the number of channels a feature has (i.e. 1 - depth, 4 - depth + normals)
         * \param[in] nf Number of channels.
         */
        void setNumChannels(int nf)
        {
          num_channels_ = nf;
        }

        /** \brief Create a set of random tests to evaluate examples.
         * \param[in] w Number features to generate.
         */
        void setMaxPatchSize(int w)
        {
          max_patch_size_ = w;
        }

        /** \brief Create a set of random tests to evaluate examples.
         * \param[in] num_of_features Number features to generated.
         * \param[out] features Generated features.
         */
        /*void createRandomFeatures(const std::size_t num_of_features, std::vector<FT> & features)
         {
         srand (time(NULL));
         int min_s = 10;
         float range_d = 0.03f;
         for (std::size_t i = 0; i < num_of_features; i++)
         {
         FT f;

         f.row1_ = rand () % (wsize_ - max_patch_size_ - 1);
         f.col1_ = rand () % (wsize_ / 2 - max_patch_size_ - 1);
         f.wsizex1_ = min_s + (rand () % (max_patch_size_ - min_s - 1));
         f.wsizey1_ = min_s + (rand () % (max_patch_size_ - min_s - 1));

         f.row2_ = rand () % (wsize_ - max_patch_size_ - 1);
         f.col2_ = wsize_ / 2 + rand () % (wsize_ / 2 - max_patch_size_ - 1);
         f.wsizex2_ = min_s + (rand () % (max_patch_size_ - 1 - min_s));
         f.wsizey2_ = min_s + (rand () % (max_patch_size_ - 1 - min_s));

         f.used_ii_ = 0;
         if(num_channels_ > 1)
         f.used_ii_ = rand() % num_channels_;

         f.threshold_ = -range_d + (rand () / static_cast<float> (RAND_MAX)) * (range_d * 2.f);
         features.push_back (f);
         }
         }*/

        void createRandomFeatures(const std::size_t num_of_features, std::vector<FT> & features) override
        {
          srand (static_cast<unsigned int>(time (nullptr)));
          int min_s = 20;
          float range_d = 0.05f;
          float incr_d = 0.01f;

          std::vector < FT > windows_and_functions;

          for (std::size_t i = 0; i < num_of_features; i++)
          {
            FT f;

            f.row1_ = rand () % (wsize_ - max_patch_size_ - 1);
            f.col1_ = rand () % (wsize_ / 2 - max_patch_size_ - 1);
            f.wsizex1_ = min_s + (rand () % (max_patch_size_ - min_s - 1));
            f.wsizey1_ = min_s + (rand () % (max_patch_size_ - min_s - 1));

            f.row2_ = rand () % (wsize_ - max_patch_size_ - 1);
            f.col2_ = wsize_ / 2 + rand () % (wsize_ / 2 - max_patch_size_ - 1);
            f.wsizex2_ = min_s + (rand () % (max_patch_size_ - 1 - min_s));
            f.wsizey2_ = min_s + (rand () % (max_patch_size_ - 1 - min_s));

            f.used_ii_ = 0;
            if (num_channels_ > 1)
              f.used_ii_ = rand () % num_channels_;

            windows_and_functions.push_back (f);
          }

          for (std::size_t i = 0; i < windows_and_functions.size (); i++)
          {
            FT f = windows_and_functions[i];
            for (std::size_t j = 0; j <= 10; j++)
            {
              f.threshold_ = -range_d + static_cast<float> (j) * incr_d;
              features.push_back (f);
            }
          }
        }

        /** \brief Evaluates a feature on the specified set of examples.
         * \param[in] feature The feature to evaluate.
         * \param[in] data_set The data set on which the feature is evaluated.
         * \param[in] examples The set of examples of the data set the feature is evaluated on.
         * \param[out] results The destination for the results of the feature evaluation.
         * \param[out] flags Flags that are supplied together with the results.
         */
        void evaluateFeature(const FT & feature, DataSet & data_set, std::vector<ExampleIndex> & examples, std::vector<float> & results,
            std::vector<unsigned char> & flags) const override
        {
          results.resize (examples.size ());
          for (std::size_t i = 0; i < examples.size (); i++)
          {
            evaluateFeature (feature, data_set, examples[i], results[i], flags[i]);
          }
        }

        /** \brief Evaluates a feature on the specified example.
         * \param[in] feature The feature to evaluate.
         * \param[in] data_set The data set on which the feature is evaluated.
         * \param[in] example The example of the data set the feature is evaluated on.
         * \param[out] result The destination for the result of the feature evaluation.
         * \param[out] flag Flags that are supplied together with the results.
         */
        void evaluateFeature(const FT & feature, DataSet & data_set, const ExampleIndex & example, float & result, unsigned char & flag) const override
        {
          TrainingExample te = data_set[example];
          int el_f1 = te.iimages_[feature.used_ii_]->getFiniteElementsCount (te.col_ + feature.col1_, te.row_ + feature.row1_, feature.wsizex1_,
              feature.wsizey1_);
          int el_f2 = te.iimages_[feature.used_ii_]->getFiniteElementsCount (te.col_ + feature.col2_, te.row_ + feature.row2_, feature.wsizex2_,
              feature.wsizey2_);

          float sum_f1 = static_cast<float>(te.iimages_[feature.used_ii_]->getFirstOrderSum (te.col_ + feature.col1_, te.row_ + feature.row1_, feature.wsizex1_, feature.wsizey1_));
          float sum_f2 = static_cast<float>(te.iimages_[feature.used_ii_]->getFirstOrderSum (te.col_ + feature.col2_, te.row_ + feature.row2_, feature.wsizex2_, feature.wsizey2_));

          float f = min_valid_small_patch_depth_;
          if (el_f1 == 0 || el_f2 == 0 || (el_f1 <= static_cast<int> (f * static_cast<float>(feature.wsizex1_ * feature.wsizey1_)))
              || (el_f2 <= static_cast<int> (f * static_cast<float>(feature.wsizex2_ * feature.wsizey2_))))
          {
            result = static_cast<float> (pcl_round (static_cast<float>(rand ()) / static_cast<float> (RAND_MAX)));
            flag = 1;
          } else
          {
            result = static_cast<float> ((sum_f1 / static_cast<float>(el_f1) - sum_f2 / static_cast<float>(el_f2)) > feature.threshold_);
            flag = 0;
          }

        }

        /** \brief Generates evaluation code for the specified feature and writes it to the specified stream.
         */
         // param[in] feature The feature for which code is generated.
         // param[out] stream The destination for the code.
        void generateCodeForEvaluation(const FT &/*feature*/, ::std::ostream &/*stream*/) const override
        {

        }
    };

    /** \brief Statistics estimator for regression trees which optimizes information gain and pose parameters error. */
    template<class LabelDataType, class NodeType, class DataSet, class ExampleIndex>
    class PoseClassRegressionVarianceStatsEstimator: public pcl::StatsEstimator<LabelDataType, NodeType, DataSet, ExampleIndex>
    {

      public:
        /** \brief Constructor. */
        PoseClassRegressionVarianceStatsEstimator(BranchEstimator * branch_estimator) :
            branch_estimator_ (branch_estimator)
        {
        }

        /** \brief Destructor. */
        ~PoseClassRegressionVarianceStatsEstimator()
        {
        }

        /** \brief Returns the number of branches the corresponding tree has. */
        inline std::size_t getNumOfBranches() const override
        {
          return branch_estimator_->getNumOfBranches ();
        }

        /** \brief Returns the label of the specified node.
         * \param[in] node The node which label is returned.
         */
        inline LabelDataType getLabelOfNode(NodeType & node) const override
        {
          return node.value;
        }

        /** \brief Computes the covariance matrix for translation offsets.
         * \param[in] data_set The corresponding data set.
         * \param[in] examples A set of examples from the dataset.
         * \param[out] covariance_matrix The covariance matrix.
         * \param[out] centroid The mean of the data.
         */
        inline unsigned int computeMeanAndCovarianceOffset(DataSet & data_set, std::vector<ExampleIndex> & examples, Eigen::Matrix3d & covariance_matrix,
            Eigen::Vector3d & centroid) const
        {
          Eigen::Matrix<double, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<double, 1, 9, Eigen::RowMajor>::Zero ();
          unsigned int point_count = static_cast<unsigned int> (examples.size ());

          for (std::size_t i = 0; i < point_count; ++i)
          {
            TrainingExample te = data_set[examples[i]];
            accu[0] += te.trans_[0] * te.trans_[0];
            accu[1] += te.trans_[0] * te.trans_[1];
            accu[2] += te.trans_[0] * te.trans_[2];
            accu[3] += te.trans_[1] * te.trans_[1];
            accu[4] += te.trans_[1] * te.trans_[2];
            accu[5] += te.trans_[2] * te.trans_[2];
            accu[6] += te.trans_[0];
            accu[7] += te.trans_[1];
            accu[8] += te.trans_[2];
          }

          if (point_count != 0)
          {
            accu /= static_cast<double> (point_count);
            centroid.head<3> ().matrix () = accu.tail<3> ();
            covariance_matrix.coeffRef (0) = accu[0] - accu[6] * accu[6];
            covariance_matrix.coeffRef (1) = accu[1] - accu[6] * accu[7];
            covariance_matrix.coeffRef (2) = accu[2] - accu[6] * accu[8];
            covariance_matrix.coeffRef (4) = accu[3] - accu[7] * accu[7];
            covariance_matrix.coeffRef (5) = accu[4] - accu[7] * accu[8];
            covariance_matrix.coeffRef (8) = accu[5] - accu[8] * accu[8];
            covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
            covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
            covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);
          }

          return point_count;
        }

        /** \brief Computes the covariance matrix for rotation values.
         * \param[in] data_set The corresponding data set.
         * \param[in] examples A set of examples from the dataset.
         * \param[out] covariance_matrix The covariance matrix.
         * \param[out] centroid The mean of the data.
         */
        inline unsigned int computeMeanAndCovarianceAngles(DataSet & data_set, std::vector<ExampleIndex> & examples, Eigen::Matrix3d & covariance_matrix,
            Eigen::Vector3d & centroid) const
        {
          Eigen::Matrix<double, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<double, 1, 9, Eigen::RowMajor>::Zero ();
          unsigned int point_count = static_cast<unsigned int> (examples.size ());

          for (std::size_t i = 0; i < point_count; ++i)
          {
            TrainingExample te = data_set[examples[i]];
            accu[0] += te.rot_[0] * te.rot_[0];
            accu[1] += te.rot_[0] * te.rot_[1];
            accu[2] += te.rot_[0] * te.rot_[2];
            accu[3] += te.rot_[1] * te.rot_[1];
            accu[4] += te.rot_[1] * te.rot_[2];
            accu[5] += te.rot_[2] * te.rot_[2];
            accu[6] += te.rot_[0];
            accu[7] += te.rot_[1];
            accu[8] += te.rot_[2];
          }

          if (point_count != 0)
          {
            accu /= static_cast<double> (point_count);
            centroid.head<3> ().matrix () = accu.tail<3> ();
            covariance_matrix.coeffRef (0) = accu[0] - accu[6] * accu[6];
            covariance_matrix.coeffRef (1) = accu[1] - accu[6] * accu[7];
            covariance_matrix.coeffRef (2) = accu[2] - accu[6] * accu[8];
            covariance_matrix.coeffRef (4) = accu[3] - accu[7] * accu[7];
            covariance_matrix.coeffRef (5) = accu[4] - accu[7] * accu[8];
            covariance_matrix.coeffRef (8) = accu[5] - accu[8] * accu[8];
            covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
            covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
            covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);
          }

          return point_count;
        }

        /** \brief Computes the information gain obtained by the specified threshold.
         * \param[in] data_set The data set corresponding to the supplied result data.
         * \param[in] examples The examples used for extracting the supplied result data.
         * \param[in] label_data The label data corresponding to the specified examples.
         * \param[in] results The results computed using the specified examples.
         * \param[in] flags The flags corresponding to the results.
         * \param[in] threshold The threshold for which the information gain is computed.
         */
        float computeInformationGain(DataSet & data_set, std::vector<ExampleIndex> & examples, std::vector<LabelDataType> & label_data,
            std::vector<float> & results, std::vector<unsigned char> & flags, const float threshold) const override
        {
          const std::size_t num_of_examples = examples.size ();
          const std::size_t num_of_branches = getNumOfBranches ();

          // compute variance
          std::vector < LabelDataType > sums (num_of_branches + 1, 0.f);
          std::vector < LabelDataType > sqr_sums (num_of_branches + 1, 0.f);
          std::vector < std::size_t > branch_element_count (num_of_branches + 1, 0.f);

          for (std::size_t branch_index = 0; branch_index < num_of_branches; ++branch_index)
          {
            branch_element_count[branch_index] = 1;
            ++branch_element_count[num_of_branches];
          }

          for (std::size_t example_index = 0; example_index < num_of_examples; ++example_index)
          {
            unsigned char branch_index;
            computeBranchIndex (results[example_index], flags[example_index], threshold, branch_index);

            LabelDataType label = label_data[example_index];

            ++branch_element_count[branch_index];
            ++branch_element_count[num_of_branches];

            sums[branch_index] += label;
            sums[num_of_branches] += label;
          }

          std::vector<float> hp (num_of_branches + 1, 0.f);
          for (std::size_t branch_index = 0; branch_index < (num_of_branches + 1); ++branch_index)
          {
            float pf = sums[branch_index] / static_cast<float> (branch_element_count[branch_index]);
            float pnf = (static_cast<LabelDataType>(branch_element_count[branch_index]) - sums[branch_index] + 1.f)
                        / static_cast<LabelDataType> (branch_element_count[branch_index]);
            hp[branch_index] -= static_cast<float>(pf * std::log (pf) + pnf * std::log (pnf));
          }

          //use mean of the examples as purity
          float purity = sums[num_of_branches] / static_cast<LabelDataType>(branch_element_count[num_of_branches]);
          float tp = 0.8f;

          if (purity >= tp)
          {
            //compute covariance matrices from translation offsets and angles for the whole set and children
            //consider only positive examples...
            std::vector < std::size_t > branch_element_count (num_of_branches + 1, 0);
            std::vector < std::vector<ExampleIndex> > positive_examples;
            positive_examples.resize (num_of_branches + 1);

            std::size_t pos = 0;
            for (std::size_t example_index = 0; example_index < num_of_examples; ++example_index)
            {
              unsigned char branch_index;
              computeBranchIndex (results[example_index], flags[example_index], threshold, branch_index);

              LabelDataType label = label_data[example_index];

              if (label == 1 /*&& !flags[example_index]*/)
              {
                ++branch_element_count[branch_index];
                ++branch_element_count[num_of_branches];

                positive_examples[branch_index].push_back (examples[example_index]);
                positive_examples[num_of_branches].push_back (examples[example_index]);
                pos++;
              }
            }

            //compute covariance from offsets and angles for all branchs
            std::vector < Eigen::Matrix3d > offset_covariances;
            std::vector < Eigen::Matrix3d > angle_covariances;

            std::vector < Eigen::Vector3d > offset_centroids;
            std::vector < Eigen::Vector3d > angle_centroids;

            offset_covariances.resize (num_of_branches + 1);
            angle_covariances.resize (num_of_branches + 1);
            offset_centroids.resize (num_of_branches + 1);
            angle_centroids.resize (num_of_branches + 1);

            for (std::size_t branch_index = 0; branch_index < (num_of_branches + 1); ++branch_index)
            {
              computeMeanAndCovarianceOffset (data_set, positive_examples[branch_index], offset_covariances[branch_index],
                  offset_centroids[branch_index]);
              computeMeanAndCovarianceAngles (data_set, positive_examples[branch_index], angle_covariances[branch_index],
                  angle_centroids[branch_index]);
            }

            //update information_gain
            std::vector<float> hr (num_of_branches + 1, 0.f);
            for (std::size_t branch_index = 0; branch_index < (num_of_branches + 1); ++branch_index)
            {
              hr[branch_index] = static_cast<float>(0.5f * std::log (std::pow (2 * M_PI, 3)
                                                    * offset_covariances[branch_index].determinant ())
                                                    + 0.5f * std::log (std::pow (2 * M_PI, 3)
                                                    * angle_covariances[branch_index].determinant ()));
            }

            for (std::size_t branch_index = 0; branch_index < (num_of_branches + 1); ++branch_index)
            {
              hp[branch_index] += std::max (sums[branch_index] / static_cast<float> (branch_element_count[branch_index]) - tp, 0.f) * hr[branch_index];
            }
          }

          float information_gain = hp[num_of_branches + 1];
          for (std::size_t branch_index = 0; branch_index < (num_of_branches); ++branch_index)
          {
            information_gain -= static_cast<float> (branch_element_count[branch_index]) / static_cast<float> (branch_element_count[num_of_branches])
                * hp[branch_index];
          }

          return information_gain;
        }

        /** \brief Computes the branch indices for all supplied results.
         * \param[in] results The results the branch indices will be computed for.
         * \param[in] flags The flags corresponding to the specified results.
         * \param[in] threshold The threshold used to compute the branch indices.
         * \param[out] branch_indices The destination for the computed branch indices.
         */
        void computeBranchIndices(std::vector<float> & results, std::vector<unsigned char> & flags, const float threshold,
            std::vector<unsigned char> & branch_indices) const override
        {
          const std::size_t num_of_results = results.size ();

          branch_indices.resize (num_of_results);
          for (std::size_t result_index = 0; result_index < num_of_results; ++result_index)
          {
            unsigned char branch_index;
            computeBranchIndex (results[result_index], flags[result_index], threshold, branch_index);
            branch_indices[result_index] = branch_index;
          }
        }

        /** \brief Computes the branch index for the specified result.
         * \param[in] result The result the branch index will be computed for.
         * \param[in] flag The flag corresponding to the specified result.
         * \param[in] threshold The threshold used to compute the branch index.
         * \param[out] branch_index The destination for the computed branch index.
         */
        inline void computeBranchIndex(const float result, const unsigned char flag, const float threshold, unsigned char & branch_index) const override
        {
          branch_estimator_->computeBranchIndex (result, flag, threshold, branch_index);
        }

        /** \brief Computes and sets the statistics for a node.
         * \param[in] data_set The data set which is evaluated.
         * \param[in] examples The examples which define which parts of the data set are used for evaluation.
         * \param[in] label_data The label_data corresponding to the examples.
         * \param[out] node The destination node for the statistics.
         */
        void computeAndSetNodeStats(DataSet & data_set, std::vector<ExampleIndex> & examples, std::vector<LabelDataType> & label_data, NodeType & node) const override
        {
          const std::size_t num_of_examples = examples.size ();

          LabelDataType sum = 0.0f;
          LabelDataType sqr_sum = 0.0f;
          for (std::size_t example_index = 0; example_index < num_of_examples; ++example_index)
          {
            const LabelDataType label = label_data[example_index];

            sum += label;
            sqr_sum += label * label;
          }

          sum /= static_cast<float>(num_of_examples);
          sqr_sum /= static_cast<float>(num_of_examples);

          const float variance = sqr_sum - sum * sum;

          node.value = sum;
          node.variance = variance;

          //set node stats regarding pose regression
          std::vector < ExampleIndex > positive_examples;

          for (std::size_t example_index = 0; example_index < num_of_examples; ++example_index)
          {
            LabelDataType label = label_data[example_index];

            if (label == 1)
              positive_examples.push_back (examples[example_index]);

          }

          //compute covariance from offsets and angles
          computeMeanAndCovarianceOffset (data_set, positive_examples, node.covariance_trans_, node.trans_mean_);
          computeMeanAndCovarianceAngles (data_set, positive_examples, node.covariance_rot_, node.rot_mean_);
        }

        /** \brief Generates code for branch index computation.
         * \param[out] stream The destination for the generated code.
         */
        // param[in] node The node for which code is generated.
        void generateCodeForBranchIndexComputation(NodeType & /*node*/, std::ostream & stream) const override
        {
          stream << "ERROR: RegressionVarianceStatsEstimator does not implement generateCodeForBranchIndex(...)";
        }

        /** \brief Generates code for label output.
         * \param[out] stream The destination for the generated code.
         */
        // param[in] node The node for which code is generated.
        void generateCodeForOutput(NodeType & /*node*/, std::ostream & stream) const override
        {
          stream << "ERROR: RegressionVarianceStatsEstimator does not implement generateCodeForBranchIndex(...)";
        }

      private:
        /** \brief The branch estimator. */
        pcl::BranchEstimator * branch_estimator_;
    };
  }
}
