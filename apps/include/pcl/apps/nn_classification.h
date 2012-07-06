/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
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
 * $Id$
 *
 */

#ifndef NNCLASSIFICATION_H_
#define NNCLASSIFICATION_H_

#include <cstdlib>
#include <cfloat>
#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace pcl
{
  /**
    * \brief Nearest neighbor search based classification of PCL point type features.
    * FLANN is used to identify a neighborhood, based on which different scoring schemes
    * can be employed to obtain likelihood values for a specified list of classes.
    * \author Zoltan Csaba Marton
    */
  template <typename PointT>
  class NNClassification
  {
    private:

      typename pcl::KdTree<PointT>::Ptr tree_;

      /** \brief List of class labels */
      std::vector<std::string> classes_;
      /** \brief The index in the class labels list for all the training examples */
      std::vector<int> labels_idx_;

    public:

      NNClassification () : tree_ (), classes_ (), labels_idx_ () {}

      /** \brief Result is a list of class labels and scores */
      typedef std::pair<std::vector<std::string>, std::vector<float> > Result;
      typedef boost::shared_ptr<Result> ResultPtr;

      // TODO setIndices method, distance metrics and reset tree

      /** \brief Setting the training features.
        * \param[in] features the training features
        */
      void 
      setTrainingFeatures (const typename pcl::PointCloud<PointT>::ConstPtr &features)
      {
        // Do not limit the number of dimensions used in the tree
        typename pcl::CustomPointRepresentation<PointT>::Ptr cpr (new pcl::CustomPointRepresentation<PointT> (INT_MAX, 0));
        tree_.reset (new pcl::KdTreeFLANN<PointT>);
        tree_->setPointRepresentation (cpr);
        tree_->setInputCloud (features);
      }

      /** \brief Updating the labels for each training example.
        * \param classes the class labels
        * \param labels_idx the index in the class labels list for each training example
        */
      void 
      setTrainingLabelIndicesAndLUT (const std::vector<std::string> &classes, const std::vector<int> &labels_idx)
      {
        // TODO check if min/max index is inside classes?
        classes_ = classes;
        labels_idx_ = labels_idx;
      }

      /** \brief Setting the labels for each training example.
        * The unique labels from the list are stored as the class labels, and
        * for each training example an index pointing to these labels is stored.
        * \note See the setTrainingLabelIndicesAndLUT method for easily re-labeling.
        * \param labels the class label for each training example
        */
      void 
      setTrainingLabels (const std::vector<std::string> &labels)
      {
        // Create a list of unique labels
        classes_ = labels;
        std::sort (classes_.begin(), classes_.end());
        classes_.erase (std::unique (classes_.begin(), classes_.end()), classes_.end());

        // Save the mapping from labels to indices in the class list
        std::map<std::string, int> label2idx;
        for (std::vector<std::string>::const_iterator it = classes_.begin (); it != classes_.end (); it++)
          label2idx[*it] = int (it - classes_.begin ());

        // Create a list holding the class index of each label
        labels_idx_.reserve (labels.size ());
        BOOST_FOREACH (std::string s, labels)
          labels_idx_.push_back (label2idx[s]);
//        for (std::vector<std::string>::const_iterator it = labels.begin (); it != labels.end (); it++)
//          labels_idx_.push_back (label2idx[*it]);
      }

      /** \brief Load the list of training examples and corresponding labels.
        * \param file_name PCD file containing the training features
        * \param labels_file_name the class label for each training example
        * \return true on success, false on failure (read error or number of entries don't match)
        */
      bool 
      loadTrainingFeatures(std::string file_name, std::string labels_file_name)
      {
        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile (file_name.c_str (), *cloud) != 0)
          return (false);
        std::vector<std::string> labels;
        std::ifstream f (labels_file_name.c_str ());
        std::string label;
        while (getline (f, label))
          if (label.size () > 0)
            labels.push_back(label);
        if (labels.size () != cloud->points.size ())
          return (false);
        setTrainingFeatures (cloud);
        setTrainingLabels (labels);
        return (true);
      }

      /** \brief Save the list of training examples and corresponding labels.
        * \param file_name file name for writing the training features
        * \param labels_file_name file name for writing the class label for each training example
        * \return true on success, false on failure (write error or number of entries don't match)
        */
      bool 
      saveTrainingFeatures (std::string file_name, std::string labels_file_name)
      {
        typename pcl::PointCloud<PointT>::ConstPtr training_features = tree_->getInputCloud ();
        if (labels_idx_.size () == training_features->points.size ())
        {
          if (pcl::io::savePCDFile (file_name.c_str (), *training_features) != 0)
            return (false);
          std::ofstream f (labels_file_name.c_str ());
          BOOST_FOREACH (int i, labels_idx_)
            f << classes_[i] << "\n";
          return (true);
        }
        return (false);
      }

      /** \brief Utility function for the default classification process.
        * \param p_q the given query point
        * \param radius the radius of the sphere bounding all of p_q's neighbors
        * \param gaussian_param influences the width of the Gaussian by specifying where the 36.78 score should be: score = exp(-distance/gaussian_param)
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return pair of label and score for each training class from the neighborhood
        */
      ResultPtr 
      classify (const PointT &p_q, double radius, float gaussian_param, int max_nn = INT_MAX)
      {
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        getSimilarExemplars (p_q, radius, k_indices, k_sqr_distances, max_nn);
        return (getGaussianBestScores (gaussian_param, k_indices, k_sqr_distances));
      }

      /** \brief Search for k-nearest neighbors for the given query point.
        * \param p_q the given query point
        * \param k the number of neighbors to search for
        * \param k_indices the resultant indices of the neighboring points (does not have to be resized to \a k a priori!)
        * \param k_sqr_distances the resultant squared distances to the neighboring points (does not have be resized to \a k
        * a priori!)
        * \return number of neighbors found
        */
      int 
      getKNearestExemplars (const PointT &p_q, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
      {
        k_indices.resize (k);
        k_sqr_distances.resize (k);
        return (tree_->nearestKSearch (p_q, k, k_indices, k_sqr_distances));
      }

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
        * \param p_q the given query point
        * \param radius the radius of the sphere bounding all of p_q's neighbors
        * \param k_indices the resultant indices of the neighboring points
        * \param k_sqr_distances the resultant squared distances to the neighboring points
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
      int 
      getSimilarExemplars (const PointT &p_q, double radius, std::vector<int> &k_indices,
                           std::vector<float> &k_sqr_distances, int max_nn = INT_MAX)
      {
        return (tree_->radiusSearch (p_q, radius, k_indices, k_sqr_distances, max_nn));
      }

      /** \brief Gets the smallest square distance to each class given a neighborhood.
        * \param k_indices the resultant indices of the neighboring points
        * \param k_sqr_distances the resultant squared distances to the neighboring points
        * \return a square distance to each training class
        */
      boost::shared_ptr<std::vector<float> > 
      getSmallestSquaredDistances (std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
      {
        // Reserve space for distances
        boost::shared_ptr<std::vector<float> > sqr_distances (new std::vector<float> (classes_.size (), FLT_MAX));

        // Select square distance to each class
        for (std::vector<int>::const_iterator i = k_indices.begin (); i != k_indices.end (); ++i)
          if ((*sqr_distances)[labels_idx_[*i]] > k_sqr_distances[i - k_indices.begin ()])
            (*sqr_distances)[labels_idx_[*i]] = k_sqr_distances[i - k_indices.begin ()];
        return (sqr_distances);
      }

      /** \brief Computes a score that is inversely proportional to the distance to each class given a neighborhood.
        * \note Scores will sum up to one.
        * \param k_indices the resultant indices of the neighboring points
        * \param k_sqr_distances the resultant squared distances to the neighboring points
        * \return pair of label and score for each training class from the neighborhood
        */
      ResultPtr 
      getLinearBestScores (std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
      {
        // Get smallest squared distances and transform them to a score for each class
        boost::shared_ptr<std::vector<float> > sqr_distances = getSmallestSquaredDistances (k_indices, k_sqr_distances);

        // Transform distances to scores
        double sum_dist = 0;
        boost::shared_ptr<std::pair<std::vector<std::string>, std::vector<float> > > result (new std::pair<std::vector<std::string>, std::vector<float> > ());
        result->first.reserve (classes_.size ());
        result->second.reserve (classes_.size ());
        for (std::vector<float>::const_iterator it = sqr_distances->begin (); it != sqr_distances->end (); ++it)
          if (*it != FLT_MAX)
          {
            result->first.push_back (classes_[it - sqr_distances->begin ()]);
            result->second.push_back (sqrt (*it));
            sum_dist += result->second.back ();
          }
        for (std::vector<float>::iterator it = result->second.begin (); it != result->second.end (); ++it)
          *it = 1 - *it/sum_dist;

        // Return label/score list pair
        return (result);
      }

      /** \brief Computes a score exponentially decreasing with the distance for each class given a neighborhood.
        * \param[in] gaussian_param influences the width of the Gaussian: score = exp(-distance/gaussioan_param)
        * \param[out] k_indices the resultant indices of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
        * \return pair of label and score for each training class from the neighborhood
        */
      ResultPtr 
      getGaussianBestScores (float gaussian_param, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
      {
        // Get smallest squared distances and transform them to a score for each class
        boost::shared_ptr<std::vector<float> > sqr_distances = getSmallestSquaredDistances (k_indices, k_sqr_distances);

        // Transform distances to scores
        boost::shared_ptr<std::pair<std::vector<std::string>, std::vector<float> > > result (new std::pair<std::vector<std::string>, std::vector<float> > ());
        result->first.reserve (classes_.size ());
        result->second.reserve (classes_.size ());
        for (std::vector<float>::const_iterator it = sqr_distances->begin (); it != sqr_distances->end (); ++it)
          if (*it != FLT_MAX)
          {
            result->first.push_back (classes_[it - sqr_distances->begin ()]);
            // TODO leave it squared, and relate param to sigma...
            result->second.push_back (expf (-sqrtf (*it) / gaussian_param));
          }

        // Return label/score list pair
        return (result);
      }
  };
}

#endif /* NNCLASSIFICATION_H_ */
