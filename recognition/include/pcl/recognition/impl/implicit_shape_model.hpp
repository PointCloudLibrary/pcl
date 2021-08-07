/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Implementation of the ISM algorithm described in "Hough Transforms and 3D SURF for robust three dimensional classication"
 * by Jan Knopp, Mukta Prasad, Geert Willems, Radu Timofte, and Luc Van Gool
 *
 * Authors: Roman Shapovalov, Alexander Velizhev, Sergey Ushakov
 */

#ifndef PCL_IMPLICIT_SHAPE_MODEL_HPP_
#define PCL_IMPLICIT_SHAPE_MODEL_HPP_

#include "../implicit_shape_model.h"
#include <pcl/filters/voxel_grid.h> // for VoxelGrid
#include <pcl/filters/extract_indices.h> // for ExtractIndices

#include <pcl/memory.h>  // for dynamic_pointer_cast

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::features::ISMVoteList<PointT>::ISMVoteList () :
  votes_ (new pcl::PointCloud<pcl::InterestPoint> ()),
  tree_is_valid_ (false),
  votes_origins_ (new pcl::PointCloud<PointT> ()),
  votes_class_ (0),
  k_ind_ (0),
  k_sqr_dist_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::features::ISMVoteList<PointT>::~ISMVoteList ()
{
  votes_class_.clear ();
  votes_origins_.reset ();
  votes_.reset ();
  k_ind_.clear ();
  k_sqr_dist_.clear ();
  tree_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::features::ISMVoteList<PointT>::addVote (
    pcl::InterestPoint& vote, const PointT &vote_origin, int votes_class)
{
  tree_is_valid_ = false;
  votes_->points.insert (votes_->points.end (), vote);// TODO: adjust height and width

  votes_origins_->points.push_back (vote_origin);
  votes_class_.push_back (votes_class);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::features::ISMVoteList<PointT>::getColoredCloud (typename pcl::PointCloud<PointT>::Ptr cloud)
{
  pcl::PointXYZRGB point;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
  colored_cloud->height = 0;
  colored_cloud->width = 1;

  if (cloud != nullptr)
  {
    colored_cloud->height += cloud->size ();
    point.r = 255;
    point.g = 255;
    point.b = 255;
    for (const auto& i_point: *cloud)
    {
      point.x = i_point.x;
      point.y = i_point.y;
      point.z = i_point.z;
      colored_cloud->points.push_back (point);
    }
  }

  point.r = 0;
  point.g = 0;
  point.b = 255;
  for (const auto &i_vote : votes_->points)
  {
    point.x = i_vote.x;
    point.y = i_vote.y;
    point.z = i_vote.z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += votes_->size ();

  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::features::ISMVoteList<PointT>::findStrongestPeaks (
  std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > &out_peaks,
  int in_class_id,
  double in_non_maxima_radius,
  double in_sigma)
{
  validateTree ();

  const std::size_t n_vote_classes = votes_class_.size ();
  if (n_vote_classes == 0)
    return;
  for (std::size_t i = 0; i < n_vote_classes ; i++)
    assert ( votes_class_[i] == in_class_id );

  // heuristic: start from NUM_INIT_PTS different locations selected uniformly
  // on the votes. Intuitively, it is likely to get a good location in dense regions.
  const int NUM_INIT_PTS = 100;
  double SIGMA_DIST = in_sigma;// rule of thumb: 10% of the object radius
  const double FINAL_EPS = SIGMA_DIST / 100;// another heuristic

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > peaks (NUM_INIT_PTS);
  std::vector<double> peak_densities (NUM_INIT_PTS);
  double max_density = -1.0;
  for (int i = 0; i < NUM_INIT_PTS; i++)
  {
    Eigen::Vector3f old_center;
    const auto idx = votes_->size() * i / NUM_INIT_PTS;
    Eigen::Vector3f curr_center = (*votes_)[idx].getVector3fMap();

    do
    {
      old_center = curr_center;
      curr_center = shiftMean (old_center, SIGMA_DIST);
    } while ((old_center - curr_center).norm () > FINAL_EPS);

    pcl::PointXYZ point;
    point.x = curr_center (0);
    point.y = curr_center (1);
    point.z = curr_center (2);
    double curr_density = getDensityAtPoint (point, SIGMA_DIST);
    assert (curr_density >= 0.0);

    peaks[i] = curr_center;
    peak_densities[i] = curr_density;

    if ( max_density < curr_density )
      max_density = curr_density;
  }

  //extract peaks
  std::vector<bool> peak_flag (NUM_INIT_PTS, true);
  for (int i_peak = 0; i_peak < NUM_INIT_PTS; i_peak++)
  {
    // find best peak with taking into consideration peak flags
    double best_density = -1.0;
    Eigen::Vector3f strongest_peak;
    int best_peak_ind (-1);
    int peak_counter (0);
    for (int i = 0; i < NUM_INIT_PTS; i++)
    {
      if ( !peak_flag[i] )
        continue;

      if ( peak_densities[i] > best_density)
      {
        best_density = peak_densities[i];
        strongest_peak = peaks[i];
        best_peak_ind = i;
      }
      ++peak_counter;
    }

    if( peak_counter == 0 )
      break;// no peaks

    pcl::ISMPeak peak;
    peak.x = strongest_peak(0);
    peak.y = strongest_peak(1);
    peak.z = strongest_peak(2);
    peak.density = best_density;
    peak.class_id = in_class_id;
    out_peaks.push_back ( peak );

    // mark best peaks and all its neighbors
    peak_flag[best_peak_ind] = false;
    for (int i = 0; i < NUM_INIT_PTS; i++)
    {
      // compute distance between best peak and all unmarked peaks
      if ( !peak_flag[i] )
        continue;

      double dist = (strongest_peak - peaks[i]).norm ();
      if ( dist < in_non_maxima_radius )
        peak_flag[i] = false;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::features::ISMVoteList<PointT>::validateTree ()
{
  if (!tree_is_valid_)
  {
    if (tree_ == nullptr)
      tree_.reset (new pcl::KdTreeFLANN<pcl::InterestPoint>);
    tree_->setInputCloud (votes_);
    k_ind_.resize ( votes_->size (), -1 );
    k_sqr_dist_.resize ( votes_->size (), 0.0f );
    tree_is_valid_ = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::Vector3f
pcl::features::ISMVoteList<PointT>::shiftMean (const Eigen::Vector3f& snap_pt, const double in_sigma_dist)
{
  validateTree ();

  Eigen::Vector3f wgh_sum (0.0, 0.0, 0.0);
  double denom = 0.0;

  pcl::InterestPoint pt;
  pt.x = snap_pt[0];
  pt.y = snap_pt[1];
  pt.z = snap_pt[2];
  std::size_t n_pts = tree_->radiusSearch (pt, 3*in_sigma_dist, k_ind_, k_sqr_dist_);

  for (std::size_t j = 0; j < n_pts; j++)
  {
    double kernel = (*votes_)[k_ind_[j]].strength * std::exp (-k_sqr_dist_[j] / (in_sigma_dist * in_sigma_dist));
    Eigen::Vector3f vote_vec ((*votes_)[k_ind_[j]].x, (*votes_)[k_ind_[j]].y, (*votes_)[k_ind_[j]].z);
    wgh_sum += vote_vec * static_cast<float> (kernel);
    denom += kernel;
  }
  assert (denom > 0.0); // at least one point is close. In fact, this case should be handled too

  return (wgh_sum / static_cast<float> (denom));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::features::ISMVoteList<PointT>::getDensityAtPoint (
    const PointT &point, double sigma_dist)
{
  validateTree ();

  const std::size_t n_vote_classes = votes_class_.size ();
  if (n_vote_classes == 0)
    return (0.0);

  double sum_vote = 0.0;

  pcl::InterestPoint pt;
  pt.x = point.x;
  pt.y = point.y;
  pt.z = point.z;
  std::size_t num_of_pts = tree_->radiusSearch (pt, 3 * sigma_dist, k_ind_, k_sqr_dist_);

  for (std::size_t j = 0; j < num_of_pts; j++)
    sum_vote += (*votes_)[k_ind_[j]].strength * std::exp (-k_sqr_dist_[j] / (sigma_dist * sigma_dist));

  return (sum_vote);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::features::ISMVoteList<PointT>::getNumberOfVotes ()
{
  return (static_cast<unsigned int> (votes_->size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::features::ISMModel::ISMModel () :
  statistical_weights_ (0),
  learned_weights_ (0),
  classes_ (0),
  sigmas_ (0),
  clusters_ (0),
  number_of_classes_ (0),
  number_of_visual_words_ (0),
  number_of_clusters_ (0),
  descriptors_dimension_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::features::ISMModel::ISMModel (ISMModel const & copy)
{
  reset ();

  this->number_of_classes_ = copy.number_of_classes_;
  this->number_of_visual_words_ = copy.number_of_visual_words_;
  this->number_of_clusters_ = copy.number_of_clusters_;
  this->descriptors_dimension_ = copy.descriptors_dimension_;

  std::vector<float> vec;
  vec.resize (this->number_of_clusters_, 0.0f);
  this->statistical_weights_.resize (this->number_of_classes_, vec);
  for (unsigned int i_class = 0; i_class < this->number_of_classes_; i_class++)
    for (unsigned int i_cluster = 0; i_cluster < this->number_of_clusters_; i_cluster++)
      this->statistical_weights_[i_class][i_cluster] = copy.statistical_weights_[i_class][i_cluster];

  this->learned_weights_.resize (this->number_of_visual_words_, 0.0f);
  for (unsigned int i_visual_word = 0; i_visual_word < this->number_of_visual_words_; i_visual_word++)
    this->learned_weights_[i_visual_word] = copy.learned_weights_[i_visual_word];

  this->classes_.resize (this->number_of_visual_words_, 0);
  for (unsigned int i_visual_word = 0; i_visual_word < this->number_of_visual_words_; i_visual_word++)
    this->classes_[i_visual_word] = copy.classes_[i_visual_word];

  this->sigmas_.resize (this->number_of_classes_, 0.0f);
  for (unsigned int i_class = 0; i_class < this->number_of_classes_; i_class++)
    this->sigmas_[i_class] = copy.sigmas_[i_class];

  this->directions_to_center_.resize (this->number_of_visual_words_, 3);
  for (unsigned int i_visual_word = 0; i_visual_word < this->number_of_visual_words_; i_visual_word++)
    for (unsigned int i_dim = 0; i_dim < 3; i_dim++)
      this->directions_to_center_ (i_visual_word, i_dim) = copy.directions_to_center_ (i_visual_word, i_dim);

  this->clusters_centers_.resize (this->number_of_clusters_, 3);
  for (unsigned int i_cluster = 0; i_cluster < this->number_of_clusters_; i_cluster++)
    for (unsigned int i_dim = 0; i_dim < this->descriptors_dimension_; i_dim++)
      this->clusters_centers_ (i_cluster, i_dim) = copy.clusters_centers_ (i_cluster, i_dim);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::features::ISMModel::~ISMModel ()
{
  reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::features::ISMModel::saveModelToFile (std::string& file_name)
{
  std::ofstream output_file (file_name.c_str (), std::ios::trunc);
  if (!output_file)
  {
    output_file.close ();
    return (false);
  }

  output_file << number_of_classes_ << " ";
  output_file << number_of_visual_words_ << " ";
  output_file << number_of_clusters_ << " ";
  output_file << descriptors_dimension_ << " ";

  //write statistical weights
  for (unsigned int i_class = 0; i_class < number_of_classes_; i_class++)
    for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
      output_file << statistical_weights_[i_class][i_cluster] << " ";

  //write learned weights
  for (unsigned int i_visual_word = 0; i_visual_word < number_of_visual_words_; i_visual_word++)
    output_file << learned_weights_[i_visual_word] << " ";

  //write classes
  for (unsigned int i_visual_word = 0; i_visual_word < number_of_visual_words_; i_visual_word++)
    output_file << classes_[i_visual_word] << " ";

  //write sigmas
  for (unsigned int i_class = 0; i_class < number_of_classes_; i_class++)
    output_file << sigmas_[i_class] << " ";

  //write directions to centers
  for (unsigned int i_visual_word = 0; i_visual_word < number_of_visual_words_; i_visual_word++)
    for (unsigned int i_dim = 0; i_dim < 3; i_dim++)
      output_file << directions_to_center_ (i_visual_word, i_dim) << " ";

  //write clusters centers
  for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
    for (unsigned int i_dim = 0; i_dim < descriptors_dimension_; i_dim++)
      output_file << clusters_centers_ (i_cluster, i_dim) << " ";

  //write clusters
  for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
  {
    output_file << static_cast<unsigned int> (clusters_[i_cluster].size ()) << " ";
    for (const unsigned int &visual_word : clusters_[i_cluster])
      output_file << visual_word << " ";
  }

  output_file.close ();
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::features::ISMModel::loadModelFromfile (std::string& file_name)
{
  reset ();
  std::ifstream input_file (file_name.c_str ());
  if (!input_file)
  {
    input_file.close ();
    return (false);
  }

  char line[256];

  input_file.getline (line, 256, ' ');
  number_of_classes_ = static_cast<unsigned int> (strtol (line, nullptr, 10));
  input_file.getline (line, 256, ' '); number_of_visual_words_ = atoi (line);
  input_file.getline (line, 256, ' '); number_of_clusters_ = atoi (line);
  input_file.getline (line, 256, ' '); descriptors_dimension_ = atoi (line);

  //read statistical weights
  std::vector<float> vec;
  vec.resize (number_of_clusters_, 0.0f);
  statistical_weights_.resize (number_of_classes_, vec);
  for (unsigned int i_class = 0; i_class < number_of_classes_; i_class++)
    for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
      input_file >> statistical_weights_[i_class][i_cluster];

  //read learned weights
  learned_weights_.resize (number_of_visual_words_, 0.0f);
  for (unsigned int i_visual_word = 0; i_visual_word < number_of_visual_words_; i_visual_word++)
    input_file >> learned_weights_[i_visual_word];

  //read classes
  classes_.resize (number_of_visual_words_, 0);
  for (unsigned int i_visual_word = 0; i_visual_word < number_of_visual_words_; i_visual_word++)
    input_file >> classes_[i_visual_word];

  //read sigmas
  sigmas_.resize (number_of_classes_, 0.0f);
  for (unsigned int i_class = 0; i_class < number_of_classes_; i_class++)
    input_file >> sigmas_[i_class];

  //read directions to centers
  directions_to_center_.resize (number_of_visual_words_, 3);
  for (unsigned int i_visual_word = 0; i_visual_word < number_of_visual_words_; i_visual_word++)
    for (unsigned int i_dim = 0; i_dim < 3; i_dim++)
      input_file >> directions_to_center_ (i_visual_word, i_dim);

  //read clusters centers
  clusters_centers_.resize (number_of_clusters_, descriptors_dimension_);
  for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
    for (unsigned int i_dim = 0; i_dim < descriptors_dimension_; i_dim++)
      input_file >> clusters_centers_ (i_cluster, i_dim);

  //read clusters
  std::vector<unsigned int> vect;
  clusters_.resize (number_of_clusters_, vect);
  for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
  {
    unsigned int size_of_current_cluster = 0;
    input_file >> size_of_current_cluster;
    clusters_[i_cluster].resize (size_of_current_cluster, 0);
    for (unsigned int i_visual_word = 0; i_visual_word < size_of_current_cluster; i_visual_word++)
      input_file >> clusters_[i_cluster][i_visual_word];
  }

  input_file.close ();
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::features::ISMModel::reset ()
{
  statistical_weights_.clear ();
  learned_weights_.clear ();
  classes_.clear ();
  sigmas_.clear ();
  directions_to_center_.resize (0, 0);
  clusters_centers_.resize (0, 0);
  clusters_.clear ();
  number_of_classes_ = 0;
  number_of_visual_words_ = 0;
  number_of_clusters_ = 0;
  descriptors_dimension_ = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::features::ISMModel&
pcl::features::ISMModel::operator = (const pcl::features::ISMModel& other)
{
  if (this != &other)
  {
    this->reset ();

    this->number_of_classes_ = other.number_of_classes_;
    this->number_of_visual_words_ = other.number_of_visual_words_;
    this->number_of_clusters_ = other.number_of_clusters_;
    this->descriptors_dimension_ = other.descriptors_dimension_;

    std::vector<float> vec;
    vec.resize (number_of_clusters_, 0.0f);
    this->statistical_weights_.resize (this->number_of_classes_, vec);
    for (unsigned int i_class = 0; i_class < this->number_of_classes_; i_class++)
      for (unsigned int i_cluster = 0; i_cluster < this->number_of_clusters_; i_cluster++)
        this->statistical_weights_[i_class][i_cluster] = other.statistical_weights_[i_class][i_cluster];

    this->learned_weights_.resize (this->number_of_visual_words_, 0.0f);
    for (unsigned int i_visual_word = 0; i_visual_word < this->number_of_visual_words_; i_visual_word++)
      this->learned_weights_[i_visual_word] = other.learned_weights_[i_visual_word];

    this->classes_.resize (this->number_of_visual_words_, 0);
    for (unsigned int i_visual_word = 0; i_visual_word < this->number_of_visual_words_; i_visual_word++)
      this->classes_[i_visual_word] = other.classes_[i_visual_word];

    this->sigmas_.resize (this->number_of_classes_, 0.0f);
    for (unsigned int i_class = 0; i_class < this->number_of_classes_; i_class++)
      this->sigmas_[i_class] = other.sigmas_[i_class];

    this->directions_to_center_.resize (this->number_of_visual_words_, 3);
    for (unsigned int i_visual_word = 0; i_visual_word < this->number_of_visual_words_; i_visual_word++)
      for (unsigned int i_dim = 0; i_dim < 3; i_dim++)
        this->directions_to_center_ (i_visual_word, i_dim) = other.directions_to_center_ (i_visual_word, i_dim);

    this->clusters_centers_.resize (this->number_of_clusters_, 3);
    for (unsigned int i_cluster = 0; i_cluster < this->number_of_clusters_; i_cluster++)
      for (unsigned int i_dim = 0; i_dim < this->descriptors_dimension_; i_dim++)
        this->clusters_centers_ (i_cluster, i_dim) = other.clusters_centers_ (i_cluster, i_dim);
  }
  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT>
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::ImplicitShapeModelEstimation () :
  training_clouds_ (0),
  training_classes_ (0),
  training_normals_ (0),
  training_sigmas_ (0),
  sampling_size_ (0.1f),
  feature_estimator_ (),
  number_of_clusters_ (184),
  n_vot_ON_ (true)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT>
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::~ImplicitShapeModelEstimation ()
{
  training_clouds_.clear ();
  training_classes_.clear ();
  training_normals_.clear ();
  training_sigmas_.clear ();
  feature_estimator_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> std::vector<typename pcl::PointCloud<PointT>::Ptr>
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::getTrainingClouds ()
{
  return (training_clouds_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::setTrainingClouds (
  const std::vector< typename pcl::PointCloud<PointT>::Ptr >& training_clouds)
{
  training_clouds_.clear ();
  std::vector<typename pcl::PointCloud<PointT>::Ptr > clouds ( training_clouds.begin (), training_clouds.end () );
  training_clouds_.swap (clouds);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> std::vector<unsigned int>
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::getTrainingClasses ()
{
  return (training_classes_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::setTrainingClasses (const std::vector<unsigned int>& training_classes)
{
  training_classes_.clear ();
  std::vector<unsigned int> classes ( training_classes.begin (), training_classes.end () );
  training_classes_.swap (classes);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> std::vector<typename pcl::PointCloud<NormalT>::Ptr>
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::getTrainingNormals ()
{
  return (training_normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::setTrainingNormals (
  const std::vector< typename pcl::PointCloud<NormalT>::Ptr >& training_normals)
{
  training_normals_.clear ();
  std::vector<typename pcl::PointCloud<NormalT>::Ptr > normals ( training_normals.begin (), training_normals.end () );
  training_normals_.swap (normals);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> float
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::getSamplingSize ()
{
  return (sampling_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::setSamplingSize (float sampling_size)
{
  if (sampling_size >= std::numeric_limits<float>::epsilon ())
    sampling_size_ = sampling_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> typename pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::FeaturePtr
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::getFeatureEstimator ()
{
  return (feature_estimator_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::setFeatureEstimator (FeaturePtr feature)
{
  feature_estimator_ = feature;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> unsigned int
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::getNumberOfClusters ()
{
  return (number_of_clusters_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::setNumberOfClusters (unsigned int num_of_clusters)
{
  if (num_of_clusters > 0)
    number_of_clusters_ = num_of_clusters;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> std::vector<float>
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::getSigmaDists ()
{
  return (training_sigmas_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::setSigmaDists (const std::vector<float>& training_sigmas)
{
  training_sigmas_.clear ();
  std::vector<float> sigmas ( training_sigmas.begin (), training_sigmas.end () );
  training_sigmas_.swap (sigmas);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> bool
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::getNVotState ()
{
  return (n_vot_ON_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::setNVotState (bool state)
{
  n_vot_ON_ = state;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> bool
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::trainISM (ISMModelPtr& trained_model)
{
  bool success = true;

  if (trained_model == nullptr)
    return (false);
  trained_model->reset ();

  std::vector<pcl::Histogram<FeatureSize> > histograms;
  std::vector<LocationInfo, Eigen::aligned_allocator<LocationInfo> > locations;
  success = extractDescriptors (histograms, locations);
  if (!success)
    return (false);

  Eigen::MatrixXi labels;
  success = clusterDescriptors(histograms, labels, trained_model->clusters_centers_);
  if (!success)
    return (false);

  std::vector<unsigned int> vec;
  trained_model->clusters_.resize (number_of_clusters_, vec);
  for (std::size_t i_label = 0; i_label < locations.size (); i_label++)
    trained_model->clusters_[labels (i_label)].push_back (i_label);

  calculateSigmas (trained_model->sigmas_);

  calculateWeights(
    locations,
    labels,
    trained_model->sigmas_,
    trained_model->clusters_,
    trained_model->statistical_weights_,
    trained_model->learned_weights_);

  trained_model->number_of_classes_ = *std::max_element (training_classes_.begin (), training_classes_.end () ) + 1;
  trained_model->number_of_visual_words_ = static_cast<unsigned int> (histograms.size ());
  trained_model->number_of_clusters_ = number_of_clusters_;
  trained_model->descriptors_dimension_ = FeatureSize;

  trained_model->directions_to_center_.resize (locations.size (), 3);
  trained_model->classes_.resize (locations.size ());
  for (std::size_t i_dir = 0; i_dir < locations.size (); i_dir++)
  {
    trained_model->directions_to_center_(i_dir, 0) = locations[i_dir].dir_to_center_.x;
    trained_model->directions_to_center_(i_dir, 1) = locations[i_dir].dir_to_center_.y;
    trained_model->directions_to_center_(i_dir, 2) = locations[i_dir].dir_to_center_.z;
    trained_model->classes_[i_dir] = training_classes_[locations[i_dir].model_num_];
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> typename pcl::features::ISMVoteList<PointT>::Ptr
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::findObjects (
  ISMModelPtr model,
  typename pcl::PointCloud<PointT>::Ptr in_cloud,
  typename pcl::PointCloud<Normal>::Ptr in_normals,
  int in_class_of_interest)
{
  typename pcl::features::ISMVoteList<PointT>::Ptr out_votes (new pcl::features::ISMVoteList<PointT> ());

  if (in_cloud->points.empty ())
    return (out_votes);

  typename pcl::PointCloud<PointT>::Ptr sampled_point_cloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<NormalT>::Ptr sampled_normal_cloud (new pcl::PointCloud<NormalT> ());
  simplifyCloud (in_cloud, in_normals, sampled_point_cloud, sampled_normal_cloud);
  if (sampled_point_cloud->points.empty ())
    return (out_votes);

  typename pcl::PointCloud<pcl::Histogram<FeatureSize> >::Ptr feature_cloud (new pcl::PointCloud<pcl::Histogram<FeatureSize> > ());
  estimateFeatures (sampled_point_cloud, sampled_normal_cloud, feature_cloud);

  //find nearest cluster
  const unsigned int n_key_points = static_cast<unsigned int> (sampled_point_cloud->size ());
  std::vector<int> min_dist_inds (n_key_points, -1);
  for (unsigned int i_point = 0; i_point < n_key_points; i_point++)
  {
    Eigen::VectorXf curr_descriptor (FeatureSize);
    for (int i_dim = 0; i_dim < FeatureSize; i_dim++)
      curr_descriptor (i_dim) = (*feature_cloud)[i_point].histogram[i_dim];

    float descriptor_sum = curr_descriptor.sum ();
    if (descriptor_sum < std::numeric_limits<float>::epsilon ())
      continue;

    unsigned int min_dist_idx = 0;
    Eigen::VectorXf clusters_center (FeatureSize);
    for (int i_dim = 0; i_dim < FeatureSize; i_dim++)
      clusters_center (i_dim) = model->clusters_centers_ (min_dist_idx, i_dim);

    float best_dist = computeDistance (curr_descriptor, clusters_center);
    for (unsigned int i_clust_cent = 0; i_clust_cent < number_of_clusters_; i_clust_cent++)
    {
      for (int i_dim = 0; i_dim < FeatureSize; i_dim++)
        clusters_center (i_dim) = model->clusters_centers_ (i_clust_cent, i_dim);
      float curr_dist = computeDistance (clusters_center, curr_descriptor);
      if (curr_dist < best_dist)
      {
        min_dist_idx = i_clust_cent;
        best_dist = curr_dist;
      }
    }
    min_dist_inds[i_point] = min_dist_idx;
  }//next keypoint

  for (std::size_t i_point = 0; i_point < n_key_points; i_point++)
  {
    int min_dist_idx = min_dist_inds[i_point];
    if (min_dist_idx == -1)
      continue;

    const unsigned int n_words = static_cast<unsigned int> (model->clusters_[min_dist_idx].size ());
    //compute coord system transform
    Eigen::Matrix3f transform = alignYCoordWithNormal ((*sampled_normal_cloud)[i_point]);
    for (unsigned int i_word = 0; i_word < n_words; i_word++)
    {
      unsigned int index = model->clusters_[min_dist_idx][i_word];
      unsigned int i_class = model->classes_[index];
      if (static_cast<int> (i_class) != in_class_of_interest)
        continue;//skip this class

      //rotate dir to center as needed
      Eigen::Vector3f direction (
        model->directions_to_center_(index, 0),
        model->directions_to_center_(index, 1),
        model->directions_to_center_(index, 2));
      applyTransform (direction, transform.transpose ());

      pcl::InterestPoint vote;
      Eigen::Vector3f vote_pos = (*sampled_point_cloud)[i_point].getVector3fMap () + direction;
      vote.x = vote_pos[0];
      vote.y = vote_pos[1];
      vote.z = vote_pos[2];
      float statistical_weight = model->statistical_weights_[in_class_of_interest][min_dist_idx];
      float learned_weight = model->learned_weights_[index];
      float power = statistical_weight * learned_weight;
      vote.strength = power;
      if (vote.strength > std::numeric_limits<float>::epsilon ())
        out_votes->addVote (vote, (*sampled_point_cloud)[i_point], i_class);
    }
  }//next point

  return (out_votes);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> bool
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::extractDescriptors (
  std::vector< pcl::Histogram<FeatureSize> >& histograms,
  std::vector< LocationInfo, Eigen::aligned_allocator<LocationInfo> >& locations)
{
  histograms.clear ();
  locations.clear ();

  int n_key_points = 0;

  if (training_clouds_.empty () || training_classes_.empty () || feature_estimator_ == nullptr)
    return (false);

  for (std::size_t i_cloud = 0; i_cloud < training_clouds_.size (); i_cloud++)
  {
    //compute the center of the training object
    Eigen::Vector3f models_center (0.0f, 0.0f, 0.0f);
    const auto num_of_points =  training_clouds_[i_cloud]->size ();
    for (auto point_j = training_clouds_[i_cloud]->begin (); point_j != training_clouds_[i_cloud]->end (); point_j++)
      models_center += point_j->getVector3fMap ();
    models_center /= static_cast<float> (num_of_points);

    //downsample the cloud
    typename pcl::PointCloud<PointT>::Ptr sampled_point_cloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<NormalT>::Ptr sampled_normal_cloud (new pcl::PointCloud<NormalT> ());
    simplifyCloud (training_clouds_[i_cloud], training_normals_[i_cloud], sampled_point_cloud, sampled_normal_cloud);
    if (sampled_point_cloud->points.empty ())
      continue;

    shiftCloud (training_clouds_[i_cloud], models_center);
    shiftCloud (sampled_point_cloud, models_center);

    n_key_points += static_cast<int> (sampled_point_cloud->size ());

    typename pcl::PointCloud<pcl::Histogram<FeatureSize> >::Ptr feature_cloud (new pcl::PointCloud<pcl::Histogram<FeatureSize> > ());
    estimateFeatures (sampled_point_cloud, sampled_normal_cloud, feature_cloud);

    int point_index = 0;
    for (auto point_i = sampled_point_cloud->points.cbegin (); point_i != sampled_point_cloud->points.cend (); point_i++, point_index++)
    {
      float descriptor_sum = Eigen::VectorXf::Map ((*feature_cloud)[point_index].histogram, FeatureSize).sum ();
      if (descriptor_sum < std::numeric_limits<float>::epsilon ())
        continue;

      histograms.insert ( histograms.end (), feature_cloud->begin () + point_index, feature_cloud->begin () + point_index + 1 );

      int dist = static_cast<int> (std::distance (sampled_point_cloud->points.cbegin (), point_i));
      Eigen::Matrix3f new_basis = alignYCoordWithNormal ((*sampled_normal_cloud)[dist]);
      Eigen::Vector3f zero;
      zero (0) = 0.0;
      zero (1) = 0.0;
      zero (2) = 0.0;
      Eigen::Vector3f new_dir = zero - point_i->getVector3fMap ();
      applyTransform (new_dir, new_basis);

      PointT point (new_dir[0], new_dir[1], new_dir[2]);
      LocationInfo info (static_cast<unsigned int> (i_cloud), point, *point_i, (*sampled_normal_cloud)[dist]);
      locations.insert(locations.end (), info);
    }
  }//next training cloud

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> bool
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::clusterDescriptors (
  std::vector< pcl::Histogram<FeatureSize> >& histograms,
  Eigen::MatrixXi& labels,
  Eigen::MatrixXf& clusters_centers)
{
  Eigen::MatrixXf points_to_cluster (histograms.size (), FeatureSize);

  for (std::size_t i_feature = 0; i_feature < histograms.size (); i_feature++)
    for (int i_dim = 0; i_dim < FeatureSize; i_dim++)
      points_to_cluster (i_feature, i_dim) = histograms[i_feature].histogram[i_dim];

  labels.resize (histograms.size(), 1);
  computeKMeansClustering (
    points_to_cluster,
    number_of_clusters_,
    labels,
    TermCriteria(TermCriteria::EPS|TermCriteria::COUNT, 10, 0.01f),//1000
    5,
    PP_CENTERS,
    clusters_centers);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::calculateSigmas (std::vector<float>& sigmas)
{
  if (!training_sigmas_.empty ())
  {
    sigmas.resize (training_sigmas_.size (), 0.0f);
    for (std::size_t i_sigma = 0; i_sigma < training_sigmas_.size (); i_sigma++)
      sigmas[i_sigma] = training_sigmas_[i_sigma];
    return;
  }

  sigmas.clear ();
  unsigned int number_of_classes = *std::max_element (training_classes_.begin (), training_classes_.end () ) + 1;
  sigmas.resize (number_of_classes, 0.0f);

  std::vector<float> vec;
  std::vector<std::vector<float> > objects_sigmas;
  objects_sigmas.resize (number_of_classes, vec);

  unsigned int number_of_objects = static_cast<unsigned int> (training_clouds_.size ());
  for (unsigned int i_object = 0; i_object < number_of_objects; i_object++)
  {
    float max_distance = 0.0f;
    const auto number_of_points = training_clouds_[i_object]->size ();
    for (unsigned int i_point = 0; i_point < number_of_points - 1; i_point++)
      for (unsigned int j_point = i_point + 1; j_point < number_of_points; j_point++)
      {
         float curr_distance = 0.0f;
         curr_distance += (*training_clouds_[i_object])[i_point].x * (*training_clouds_[i_object])[j_point].x;
         curr_distance += (*training_clouds_[i_object])[i_point].y * (*training_clouds_[i_object])[j_point].y;
         curr_distance += (*training_clouds_[i_object])[i_point].z * (*training_clouds_[i_object])[j_point].z;
         if (curr_distance > max_distance)
           max_distance = curr_distance;
      }
    max_distance = static_cast<float> (sqrt (max_distance));
    unsigned int i_class = training_classes_[i_object];
    objects_sigmas[i_class].push_back (max_distance);
  }

  for (unsigned int i_class = 0; i_class < number_of_classes; i_class++)
  {
    float sig = 0.0f;
    unsigned int number_of_objects_in_class = static_cast<unsigned int> (objects_sigmas[i_class].size ());
    for (unsigned int i_object = 0; i_object < number_of_objects_in_class; i_object++)
      sig += objects_sigmas[i_class][i_object];
    sig /= (static_cast<float> (number_of_objects_in_class) * 10.0f);
    sigmas[i_class] = sig;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::calculateWeights (
  const std::vector< LocationInfo, Eigen::aligned_allocator<LocationInfo> >& locations,
  const Eigen::MatrixXi &labels,
  std::vector<float>& sigmas,
  std::vector<std::vector<unsigned int> >& clusters,
  std::vector<std::vector<float> >& statistical_weights,
  std::vector<float>& learned_weights)
{
  unsigned int number_of_classes = *std::max_element (training_classes_.begin (), training_classes_.end () ) + 1;
  //Temporary variable
  std::vector<float> vec;
  vec.resize (number_of_clusters_, 0.0f);
  statistical_weights.clear ();
  learned_weights.clear ();
  statistical_weights.resize (number_of_classes, vec);
  learned_weights.resize (locations.size (), 0.0f);

  //Temporary variable
  std::vector<int> vect;
  vect.resize (*std::max_element (training_classes_.begin (), training_classes_.end () ) + 1, 0);

  //Number of features from which c_i was learned
  std::vector<int> n_ftr;

  //Total number of votes from visual word v_j
  std::vector<int> n_vot;

  //Number of visual words that vote for class c_i
  std::vector<int> n_vw;

  //Number of votes for class c_i from v_j
  std::vector<std::vector<int> > n_vot_2;

  n_vot_2.resize (number_of_clusters_, vect);
  n_vot.resize (number_of_clusters_, 0);
  n_ftr.resize (number_of_classes, 0);
  for (std::size_t i_location = 0; i_location < locations.size (); i_location++)
  {
    int i_class = training_classes_[locations[i_location].model_num_];
    int i_cluster = labels (i_location);
    n_vot_2[i_cluster][i_class] += 1;
    n_vot[i_cluster] += 1;
    n_ftr[i_class] += 1;
  }

  n_vw.resize (number_of_classes, 0);
  for (unsigned int i_class = 0; i_class < number_of_classes; i_class++)
    for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
      if (n_vot_2[i_cluster][i_class] > 0)
        n_vw[i_class] += 1;

  //computing learned weights
  learned_weights.resize (locations.size (), 0.0);
  for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
  {
    unsigned int number_of_words_in_cluster = static_cast<unsigned int> (clusters[i_cluster].size ());
    for (unsigned int i_visual_word = 0; i_visual_word < number_of_words_in_cluster; i_visual_word++)
    {
      unsigned int i_index = clusters[i_cluster][i_visual_word];
      int i_class = training_classes_[locations[i_index].model_num_];
      float square_sigma_dist = sigmas[i_class] * sigmas[i_class];
      if (square_sigma_dist < std::numeric_limits<float>::epsilon ())
      {
        std::vector<float> calculated_sigmas;
        calculateSigmas (calculated_sigmas);
        square_sigma_dist = calculated_sigmas[i_class] * calculated_sigmas[i_class];
        if (square_sigma_dist < std::numeric_limits<float>::epsilon ())
          continue;
      }
      Eigen::Matrix3f transform = alignYCoordWithNormal (locations[i_index].normal_);
      Eigen::Vector3f direction = locations[i_index].dir_to_center_.getVector3fMap ();
      applyTransform (direction, transform);
      Eigen::Vector3f actual_center = locations[i_index].point_.getVector3fMap () + direction;

      //collect gaussian weighted distances
      std::vector<float> gauss_dists;
      for (unsigned int j_visual_word = 0; j_visual_word < number_of_words_in_cluster; j_visual_word++)
      {
        unsigned int j_index = clusters[i_cluster][j_visual_word];
        int j_class = training_classes_[locations[j_index].model_num_];
        if (i_class != j_class)
          continue;
        //predict center
        Eigen::Matrix3f transform_2 = alignYCoordWithNormal (locations[j_index].normal_);
        Eigen::Vector3f direction_2 = locations[i_index].dir_to_center_.getVector3fMap ();
        applyTransform (direction_2, transform_2);
        Eigen::Vector3f predicted_center = locations[j_index].point_.getVector3fMap () + direction_2;
        float residual = (predicted_center - actual_center).norm ();
        float value = -residual * residual / square_sigma_dist;
        gauss_dists.push_back (static_cast<float> (std::exp (value)));
      }//next word
      //find median gaussian weighted distance
      std::size_t mid_elem = (gauss_dists.size () - 1) / 2;
      std::nth_element (gauss_dists.begin (), gauss_dists.begin () + mid_elem, gauss_dists.end ());
      learned_weights[i_index] = *(gauss_dists.begin () + mid_elem);
    }//next word
  }//next cluster

  //computing statistical weights
  for (unsigned int i_cluster = 0; i_cluster < number_of_clusters_; i_cluster++)
  {
    for (unsigned int i_class = 0; i_class < number_of_classes; i_class++)
    {
      if (n_vot_2[i_cluster][i_class] == 0)
        continue;//no votes per class of interest in this cluster
      if (n_vw[i_class] == 0)
        continue;//there were no objects of this class in the training dataset
      if (n_vot[i_cluster] == 0)
        continue;//this cluster has never been used
      if (n_ftr[i_class] == 0)
        continue;//there were no objects of this class in the training dataset
      float part_1 = static_cast<float> (n_vw[i_class]);
	  float part_2 = static_cast<float> (n_vot[i_cluster]);
      float part_3 = static_cast<float> (n_vot_2[i_cluster][i_class]) / static_cast<float> (n_ftr[i_class]);
      float part_4 = 0.0f;

      if (!n_vot_ON_)
        part_2 = 1.0f;

      for (unsigned int j_class = 0; j_class < number_of_classes; j_class++)
        if (n_ftr[j_class] != 0)
          part_4 += static_cast<float> (n_vot_2[i_cluster][j_class]) / static_cast<float> (n_ftr[j_class]);

      statistical_weights[i_class][i_cluster] = (1.0f / part_1) * (1.0f / part_2) * part_3 / part_4;
    }
  }//next cluster
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::simplifyCloud (
  typename pcl::PointCloud<PointT>::ConstPtr in_point_cloud,
  typename pcl::PointCloud<NormalT>::ConstPtr in_normal_cloud,
  typename pcl::PointCloud<PointT>::Ptr out_sampled_point_cloud,
  typename pcl::PointCloud<NormalT>::Ptr out_sampled_normal_cloud)
{
  //create voxel grid
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (sampling_size_, sampling_size_, sampling_size_);
  grid.setSaveLeafLayout (true);
  grid.setInputCloud (in_point_cloud);

  pcl::PointCloud<PointT> temp_cloud;
  grid.filter (temp_cloud);

  //extract indices of points from source cloud which are closest to grid points
  const float max_value = std::numeric_limits<float>::max ();

  const auto num_source_points = in_point_cloud->size ();
  const auto num_sample_points = temp_cloud.size ();

  std::vector<float> dist_to_grid_center (num_sample_points, max_value);
  std::vector<int> sampling_indices (num_sample_points, -1);

  for (std::size_t i_point = 0; i_point < num_source_points; i_point++)
  {
    int index = grid.getCentroidIndex ((*in_point_cloud)[i_point]);
    if (index == -1)
      continue;

    PointT pt_1 = (*in_point_cloud)[i_point];
    PointT pt_2 = temp_cloud[index];

    float distance = (pt_1.x - pt_2.x) * (pt_1.x - pt_2.x) + (pt_1.y - pt_2.y) * (pt_1.y - pt_2.y) + (pt_1.z - pt_2.z) * (pt_1.z - pt_2.z);
    if (distance < dist_to_grid_center[index])
    {
      dist_to_grid_center[index] = distance;
      sampling_indices[index] = static_cast<int> (i_point);
    }
  }

  //extract source points
  pcl::PointIndices::Ptr final_inliers_indices (new pcl::PointIndices ());
  pcl::ExtractIndices<PointT> extract_points;
  pcl::ExtractIndices<NormalT> extract_normals;

  final_inliers_indices->indices.reserve (num_sample_points);
  for (std::size_t i_point = 0; i_point < num_sample_points; i_point++)
  {
    if (sampling_indices[i_point] != -1)
      final_inliers_indices->indices.push_back ( sampling_indices[i_point] );
  }

  extract_points.setInputCloud (in_point_cloud);
  extract_points.setIndices (final_inliers_indices);
  extract_points.filter (*out_sampled_point_cloud);

  extract_normals.setInputCloud (in_normal_cloud);
  extract_normals.setIndices (final_inliers_indices);
  extract_normals.filter (*out_sampled_normal_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::shiftCloud (
  typename pcl::PointCloud<PointT>::Ptr in_cloud,
  Eigen::Vector3f shift_point)
{
  for (auto point_it = in_cloud->points.begin (); point_it != in_cloud->points.end (); point_it++)
  {
    point_it->x -= shift_point.x ();
    point_it->y -= shift_point.y ();
    point_it->z -= shift_point.z ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> Eigen::Matrix3f
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::alignYCoordWithNormal (const NormalT& in_normal)
{
  Eigen::Matrix3f result;
  Eigen::Matrix3f rotation_matrix_X;
  Eigen::Matrix3f rotation_matrix_Z;

  float A = 0.0f;
  float B = 0.0f;
  float sign = -1.0f;

  float denom_X = static_cast<float> (sqrt (in_normal.normal_z * in_normal.normal_z + in_normal.normal_y * in_normal.normal_y));
  A = in_normal.normal_y / denom_X;
  B = sign * in_normal.normal_z / denom_X;
  rotation_matrix_X << 1.0f,   0.0f,   0.0f,
                       0.0f,      A,     -B,
                       0.0f,      B,      A;

  float denom_Z = static_cast<float> (sqrt (in_normal.normal_x * in_normal.normal_x + in_normal.normal_y * in_normal.normal_y));
  A = in_normal.normal_y / denom_Z;
  B = sign * in_normal.normal_x / denom_Z;
  rotation_matrix_Z <<    A,     -B,   0.0f,
                          B,      A,   0.0f,
                       0.0f,   0.0f,   1.0f;

  result = rotation_matrix_X * rotation_matrix_Z;

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::applyTransform (Eigen::Vector3f& io_vec, const Eigen::Matrix3f& in_transform)
{
  io_vec = in_transform * io_vec;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::estimateFeatures (
  typename pcl::PointCloud<PointT>::Ptr sampled_point_cloud,
  typename pcl::PointCloud<NormalT>::Ptr normal_cloud,
  typename pcl::PointCloud<pcl::Histogram<FeatureSize> >::Ptr feature_cloud)
{
  typename pcl::search::Search<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//  tree->setInputCloud (point_cloud);

  feature_estimator_->setInputCloud (sampled_point_cloud->makeShared ());
//  feature_estimator_->setSearchSurface (point_cloud->makeShared ());
  feature_estimator_->setSearchMethod (tree);

//  typename pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<FeatureSize> >::Ptr feat_est_norm =
//    dynamic_pointer_cast<pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<FeatureSize> > > (feature_estimator_);
//  feat_est_norm->setInputNormals (normal_cloud);

  typename pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Histogram<FeatureSize> >::Ptr feat_est_norm =
    dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Histogram<FeatureSize> > > (feature_estimator_);
  feat_est_norm->setInputNormals (normal_cloud);

  feature_estimator_->compute (*feature_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> double
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::computeKMeansClustering (
  const Eigen::MatrixXf& points_to_cluster,
  int number_of_clusters,
  Eigen::MatrixXi& io_labels,
  TermCriteria criteria,
  int attempts,
  int flags,
  Eigen::MatrixXf& cluster_centers)
{
  const int spp_trials = 3;
  std::size_t number_of_points = points_to_cluster.rows () > 1 ? points_to_cluster.rows () : points_to_cluster.cols ();
  int feature_dimension = points_to_cluster.rows () > 1 ? FeatureSize : 1;

  attempts = std::max (attempts, 1);
  srand (static_cast<unsigned int> (time (nullptr)));

  Eigen::MatrixXi labels (number_of_points, 1);

  if (flags & USE_INITIAL_LABELS)
    labels = io_labels;
  else
    labels.setZero ();

  Eigen::MatrixXf centers (number_of_clusters, feature_dimension);
  Eigen::MatrixXf old_centers (number_of_clusters, feature_dimension);
  std::vector<int> counters (number_of_clusters);
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > boxes (feature_dimension);
  Eigen::Vector2f* box = &boxes[0];

  double best_compactness = std::numeric_limits<double>::max ();
  double compactness = 0.0;

  if (criteria.type_ & TermCriteria::EPS)
    criteria.epsilon_ = std::max (criteria.epsilon_, 0.0f);
  else
    criteria.epsilon_ = std::numeric_limits<float>::epsilon ();

  criteria.epsilon_ *= criteria.epsilon_;

  if (criteria.type_ & TermCriteria::COUNT)
    criteria.max_count_ = std::min (std::max (criteria.max_count_, 2), 100);
  else
    criteria.max_count_ = 100;

  if (number_of_clusters == 1)
  {
    attempts = 1;
    criteria.max_count_ = 2;
  }

  for (int i_dim = 0; i_dim < feature_dimension; i_dim++)
    box[i_dim] = Eigen::Vector2f (points_to_cluster (0, i_dim), points_to_cluster (0, i_dim));

  for (std::size_t i_point = 0; i_point < number_of_points; i_point++)
    for (int i_dim = 0; i_dim < feature_dimension; i_dim++)
    {
      float v = points_to_cluster (i_point, i_dim);
      box[i_dim] (0) = std::min (box[i_dim] (0), v);
      box[i_dim] (1) = std::max (box[i_dim] (1), v);
    }

  for (int i_attempt = 0; i_attempt < attempts; i_attempt++)
  {
    float max_center_shift = std::numeric_limits<float>::max ();
    for (int iter = 0; iter < criteria.max_count_ && max_center_shift > criteria.epsilon_; iter++)
    {
      Eigen::MatrixXf temp (centers.rows (), centers.cols ());
      temp = centers;
      centers = old_centers;
      old_centers = temp;

      if ( iter == 0 && ( i_attempt > 0 || !(flags & USE_INITIAL_LABELS) ) )
      {
        if (flags & PP_CENTERS)
          generateCentersPP (points_to_cluster, centers, number_of_clusters, spp_trials);
        else
        {
          for (int i_cl_center = 0; i_cl_center < number_of_clusters; i_cl_center++)
          {
            Eigen::VectorXf center (feature_dimension);
            generateRandomCenter (boxes, center);
            for (int i_dim = 0; i_dim < feature_dimension; i_dim++)
              centers (i_cl_center, i_dim) = center (i_dim);
          }//generate center for next cluster
        }//end if-else random or PP centers
      }
      else
      {
        centers.setZero ();
        for (int i_cluster = 0; i_cluster < number_of_clusters; i_cluster++)
          counters[i_cluster] = 0;
        for (std::size_t i_point = 0; i_point < number_of_points; i_point++)
        {
          int i_label = labels (i_point, 0);
          for (int i_dim = 0; i_dim < feature_dimension; i_dim++)
            centers (i_label, i_dim) += points_to_cluster (i_point, i_dim);
          counters[i_label]++;
        }
        if (iter > 0)
          max_center_shift = 0.0f;
        for (int i_cl_center = 0; i_cl_center < number_of_clusters; i_cl_center++)
        {
          if (counters[i_cl_center] != 0)
          {
            float scale = 1.0f / static_cast<float> (counters[i_cl_center]);
            for (int i_dim = 0; i_dim < feature_dimension; i_dim++)
              centers (i_cl_center, i_dim) *= scale;
          }
          else
          {
            Eigen::VectorXf center (feature_dimension);
            generateRandomCenter (boxes, center);
            for(int i_dim = 0; i_dim < feature_dimension; i_dim++)
              centers (i_cl_center, i_dim) = center (i_dim);
          }

          if (iter > 0)
          {
            float dist = 0.0f;
            for (int i_dim = 0; i_dim < feature_dimension; i_dim++)
            {
              float diff = centers (i_cl_center, i_dim) - old_centers (i_cl_center, i_dim);
              dist += diff * diff;
            }
            max_center_shift = std::max (max_center_shift, dist);
          }
        }
      }
      compactness = 0.0f;
      for (std::size_t i_point = 0; i_point < number_of_points; i_point++)
      {
        Eigen::VectorXf sample (feature_dimension);
        sample = points_to_cluster.row (i_point);

        int k_best = 0;
        float min_dist = std::numeric_limits<float>::max ();

        for (int i_cluster = 0; i_cluster < number_of_clusters; i_cluster++)
        {
          Eigen::VectorXf center (feature_dimension);
          center = centers.row (i_cluster);
          float dist = computeDistance (sample, center);
          if (min_dist > dist)
          {
            min_dist = dist;
            k_best = i_cluster;
          }
        }
        compactness += min_dist;
        labels (i_point, 0) = k_best;
      }
    }//next iteration

    if (compactness < best_compactness)
    {
      best_compactness = compactness;
      cluster_centers = centers;
      io_labels = labels;
    }
  }//next attempt

  return (best_compactness);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::generateCentersPP (
  const Eigen::MatrixXf& data,
  Eigen::MatrixXf& out_centers,
  int number_of_clusters,
  int trials)
{
  std::size_t dimension = data.cols ();
  unsigned int number_of_points = static_cast<unsigned int> (data.rows ());
  std::vector<int> centers_vec (number_of_clusters);
  int* centers = &centers_vec[0];
  std::vector<double> dist (number_of_points);
  std::vector<double> tdist (number_of_points);
  std::vector<double> tdist2 (number_of_points);
  double sum0 = 0.0;

  unsigned int random_unsigned = rand ();
  centers[0] = random_unsigned % number_of_points;

  for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
  {
    Eigen::VectorXf first (dimension);
    Eigen::VectorXf second (dimension);
    first = data.row (i_point);
    second = data.row (centers[0]);
    dist[i_point] = computeDistance (first, second);
    sum0 += dist[i_point];
  }

  for (int i_cluster = 0; i_cluster < number_of_clusters; i_cluster++)
  {
    double best_sum = std::numeric_limits<double>::max ();
    int best_center = -1;
    for (int i_trials = 0; i_trials < trials; i_trials++)
    {
      unsigned int random_integer = rand () - 1;
      double random_double = static_cast<double> (random_integer) / static_cast<double> (std::numeric_limits<unsigned int>::max ());
      double p = random_double * sum0;

      unsigned int i_point;
      for (i_point = 0; i_point < number_of_points - 1; i_point++)
        if ( (p -= dist[i_point]) <= 0.0)
          break;

      int ci = i_point;

      double s = 0.0;
      for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
      {
        Eigen::VectorXf first (dimension);
        Eigen::VectorXf second (dimension);
        first = data.row (i_point);
        second = data.row (ci);
        tdist2[i_point] = std::min (static_cast<double> (computeDistance (first, second)), dist[i_point]);
        s += tdist2[i_point];
      }

      if (s <= best_sum)
      {
        best_sum = s;
        best_center = ci;
        std::swap (tdist, tdist2);
      }
    }

    centers[i_cluster] = best_center;
    sum0 = best_sum;
    std::swap (dist, tdist);
  }

  for (int i_cluster = 0; i_cluster < number_of_clusters; i_cluster++)
    for (std::size_t i_dim = 0; i_dim < dimension; i_dim++)
      out_centers (i_cluster, i_dim) = data (centers[i_cluster], i_dim);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> void
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::generateRandomCenter (const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >& boxes,
  Eigen::VectorXf& center)
{
  std::size_t dimension = boxes.size ();
  float margin = 1.0f / static_cast<float> (dimension);

  for (std::size_t i_dim = 0; i_dim < dimension; i_dim++)
  {
    unsigned int random_integer = rand () - 1;
    float random_float = static_cast<float> (random_integer) / static_cast<float> (std::numeric_limits<unsigned int>::max ());
    center (i_dim) = (random_float * (1.0f + margin * 2.0f)- margin) * (boxes[i_dim] (1) - boxes[i_dim] (0)) + boxes[i_dim] (0);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <int FeatureSize, typename PointT, typename NormalT> float
pcl::ism::ImplicitShapeModelEstimation<FeatureSize, PointT, NormalT>::computeDistance (Eigen::VectorXf& vec_1, Eigen::VectorXf& vec_2)
{
  std::size_t dimension = vec_1.rows () > 1 ? vec_1.rows () : vec_1.cols ();
  float distance = 0.0f;
  for(std::size_t i_dim = 0; i_dim < dimension; i_dim++)
  {
    float diff = vec_1 (i_dim) - vec_2 (i_dim);
    distance += diff * diff;
  }

  return (distance);
}

#endif //#ifndef PCL_IMPLICIT_SHAPE_MODEL_HPP_
