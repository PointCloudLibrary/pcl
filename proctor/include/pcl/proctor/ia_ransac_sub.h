#ifndef IA_RANSAC_SUB_H_
#define IA_RANSAC_SUB_H_

// PCL includes
#include <pcl/registration/ia_ransac.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b SubsetSAC_IA allows you to use SampleConsensusInitialAlignment's algorithm when you've only computed the
  * feature on a subset of the points in the point cloud.
  */
template <typename PointSource, typename PointTarget, typename FeatureT>
class SubsetSAC_IA : public SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>
{
public:
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::getClassName;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::getRandomIndex;

  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::corr_dist_threshold_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::feature_tree_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::final_transformation_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::input_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::input_features_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::k_correspondences_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::max_iterations_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::min_sample_distance_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::nr_samples_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::reg_name_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::target_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::target_features_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::transformation_;
  using SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::update_visualizer_;
  using Registration<PointSource, PointTarget>::transformation_estimation_;

  typedef typename SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::PointCloudSource PointCloudSource;
  typedef typename SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::FeatureCloud FeatureCloud;

  /** \brief Constructor. */
  SubsetSAC_IA ()
  {
    reg_name_ = "SubsetSAC_IA";
  }

  /** \brief Set the source point cloud's features indices
    * \param source_indices the indices of the computed source features
    */
  void
  setSourceIndices (IndicesConstPtr source_indices) { source_indices_ = source_indices; }

  /** \brief Get a pointer to the source point cloud's features indices */
  inline IndicesConstPtr const
  getSourceIndices () { return (source_indices_); }

  /** \brief Set the target point cloud's features indices
    * \param target_indices the indices of the computed target features
    */
  void
  setTargetIndices (IndicesConstPtr target_indices) { target_indices_ = target_indices; }

  /** \brief Get a pointer to the target point cloud's features indices */
  inline IndicesPtr const
  getTargetIndices () { return (target_indices_); }

protected:
  /** \brief Select \a nr_samples sample points from source while making sure
    * that their pairwise distances are greater than a user-defined minimum
    * distance, \a min_sample_distance.
    * \param nr_samples the number of samples to select
    * \param min_sample_distance the minimum distance between any two samples
    * \param sample_indices_features the resulting sample indices into the source features cloud
    * \param sample_indices_cloud the resulting sample indices into the source cloud
    */
  void
  selectSamplesSubset (int nr_samples, float min_sample_distance,std::vector<int> &sample_indices_features,
                       std::vector<int> &sample_indices_cloud);

  /** \brief For each of the sample points from the source features cloud, find
    * a list of points in the target cloud whose features are similar to the 
    * sample points' features. From these, select one randomly which will be
    * considered that sample point's correspondence.
    * \param sample_indices_features the indices into the source features cloud of each sample point
    * \param corresponding_indices_cloud the resulting indices of each sample's corresponding point in the target cloud
    */
  void
  findSimilarFeaturesSubset (const std::vector<int> &sample_indices_features, std::vector<int> &corresponding_indices_cloud);

  /** \brief Rigid transformation computation method.
    * \param output the transformed input point cloud dataset using the rigid transformation found
    */
  void
  computeTransformation (PointCloudSource &output);

  /** \brief The mapping of features indices to cloud indicies in the source. */
  IndicesConstPtr source_indices_;

  /** \brief The mapping of features indices to cloud indicies in the target. */
  IndicesConstPtr target_indices_;

};

#include "proctor/impl/ia_ransac_sub.hpp"

#endif  //#ifndef IA_RANSAC_SUB_H_
