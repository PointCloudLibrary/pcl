/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#pragma once

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/point_cloud.h>

namespace pcl {
namespace registration {
/** \brief CorrespondenceRejectorPoly implements a correspondence rejection method that
 * exploits low-level and pose-invariant geometric constraints between two point sets by
 * forming virtual polygons of a user-specifiable cardinality on each model using the
 * input correspondences. These polygons are then checked in a pose-invariant manner
 * (i.e. the side lengths must be approximately equal), and rejection is performed by
 * thresholding these edge lengths.
 *
 * If you use this in academic work, please cite:
 *
 * A. G. Buch, D. Kraft, J.-K. Kämäräinen, H. G. Petersen and N. Krüger.
 * Pose Estimation using Local Structure-Specific Shape and Appearance Context.
 * International Conference on Robotics and Automation (ICRA), 2013.
 *
 * \author Anders Glent Buch
 * \ingroup registration
 */
template <typename SourceT, typename TargetT>
class PCL_EXPORTS CorrespondenceRejectorPoly : public CorrespondenceRejector {
  using CorrespondenceRejector::getClassName;
  using CorrespondenceRejector::input_correspondences_;
  using CorrespondenceRejector::rejection_name_;

public:
  using Ptr = shared_ptr<CorrespondenceRejectorPoly<SourceT, TargetT>>;
  using ConstPtr = shared_ptr<const CorrespondenceRejectorPoly<SourceT, TargetT>>;

  using PointCloudSource = pcl::PointCloud<SourceT>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = pcl::PointCloud<TargetT>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  /** \brief Empty constructor */
  CorrespondenceRejectorPoly()
  : iterations_(10000)
  , cardinality_(3)
  , similarity_threshold_(0.75f)
  , similarity_threshold_squared_(0.75f * 0.75f)
  {
    rejection_name_ = "CorrespondenceRejectorPoly";
  }

  /** \brief Get a list of valid correspondences after rejection from the original set
   * of correspondences.
   * \param[in] original_correspondences the set of initial correspondences given
   * \param[out] remaining_correspondences the resultant filtered set of remaining
   * correspondences
   */
  void
  getRemainingCorrespondences(const pcl::Correspondences& original_correspondences,
                              pcl::Correspondences& remaining_correspondences) override;

  /** \brief Provide a source point cloud dataset (must contain XYZ data!), used to
   * compute the correspondence distance.
   * \param[in] cloud a cloud containing XYZ data
   */
  inline void
  setInputSource(const PointCloudSourceConstPtr& cloud)
  {
    input_ = cloud;
  }

  /** \brief Provide a target point cloud dataset (must contain XYZ data!), used to
   * compute the correspondence distance.
   * \param[in] target a cloud containing XYZ data
   */
  inline void
  setInputTarget(const PointCloudTargetConstPtr& target)
  {
    target_ = target;
  }

  /** \brief See if this rejector requires source points */
  bool
  requiresSourcePoints() const override
  {
    return (true);
  }

  /** \brief Blob method for setting the source cloud */
  void
  setSourcePoints(pcl::PCLPointCloud2::ConstPtr cloud2) override
  {
    PointCloudSourcePtr cloud(new PointCloudSource);
    fromPCLPointCloud2(*cloud2, *cloud);
    setInputSource(cloud);
  }

  /** \brief See if this rejector requires a target cloud */
  bool
  requiresTargetPoints() const override
  {
    return (true);
  }

  /** \brief Method for setting the target cloud */
  void
  setTargetPoints(pcl::PCLPointCloud2::ConstPtr cloud2) override
  {
    PointCloudTargetPtr cloud(new PointCloudTarget);
    fromPCLPointCloud2(*cloud2, *cloud);
    setInputTarget(cloud);
  }

  /** \brief Set the polygon cardinality
   * \param cardinality polygon cardinality
   */
  inline void
  setCardinality(int cardinality)
  {
    cardinality_ = cardinality;
  }

  /** \brief Get the polygon cardinality
   * \return polygon cardinality
   */
  inline int
  getCardinality()
  {
    return (cardinality_);
  }

  /** \brief Set the similarity threshold in [0,1[ between edge lengths,
   * where 1 is a perfect match
   * \param similarity_threshold similarity threshold
   */
  inline void
  setSimilarityThreshold(float similarity_threshold)
  {
    similarity_threshold_ = similarity_threshold;
    similarity_threshold_squared_ = similarity_threshold * similarity_threshold;
  }

  /** \brief Get the similarity threshold between edge lengths
   * \return similarity threshold
   */
  inline float
  getSimilarityThreshold()
  {
    return (similarity_threshold_);
  }

  /** \brief Set the number of iterations
   * \param iterations number of iterations
   */
  inline void
  setIterations(int iterations)
  {
    iterations_ = iterations;
  }

  /** \brief Get the number of iterations
   * \return number of iterations
   */
  inline int
  getIterations()
  {
    return (iterations_);
  }

  /** \brief Polygonal rejection of a single polygon, indexed by a subset of
   * correspondences \param corr all correspondences into \ref input_ and \ref target_
   * \param idx sampled indices into \b correspondences, must have a size equal to \ref
   * cardinality_ \return true if all edge length ratios are larger than or equal to
   * \ref similarity_threshold_
   */
  inline bool
  thresholdPolygon(const pcl::Correspondences& corr, const std::vector<int>& idx)
  {
    if (cardinality_ ==
        2) // Special case: when two points are considered, we only have one edge
    {
      return (thresholdEdgeLength(corr[idx[0]].index_query,
                                  corr[idx[1]].index_query,
                                  corr[idx[0]].index_match,
                                  corr[idx[1]].index_match,
                                  similarity_threshold_squared_));
    }
    // Otherwise check all edges
    for (int i = 0; i < cardinality_; ++i) {
      if (!thresholdEdgeLength(corr[idx[i]].index_query,
                               corr[idx[(i + 1) % cardinality_]].index_query,
                               corr[idx[i]].index_match,
                               corr[idx[(i + 1) % cardinality_]].index_match,
                               similarity_threshold_squared_)) {
        return (false);
      }
    }
    return (true);
  }

  /** \brief Polygonal rejection of a single polygon, indexed by two point index vectors
   * \param source_indices indices of polygon points in \ref input_, must have a size
   * equal to \ref cardinality_
   * \param target_indices corresponding indices of polygon points in \ref target_, must
   * have a size equal to \ref cardinality_
   * \return true if all edge length ratios are larger than or equal to
   * \ref similarity_threshold_
   */
  inline bool
  thresholdPolygon(const pcl::Indices& source_indices,
                   const pcl::Indices& target_indices)
  {
    // Convert indices to correspondences and an index vector pointing to each element
    pcl::Correspondences corr(cardinality_);
    std::vector<int> idx(cardinality_);
    for (int i = 0; i < cardinality_; ++i) {
      corr[i].index_query = source_indices[i];
      corr[i].index_match = target_indices[i];
      idx[i] = i;
    }

    return (thresholdPolygon(corr, idx));
  }

protected:
  /** \brief Apply the rejection algorithm.
   * \param[out] correspondences the set of resultant correspondences.
   */
  inline void
  applyRejection(pcl::Correspondences& correspondences) override
  {
    getRemainingCorrespondences(*input_correspondences_, correspondences);
  }

  /** \brief Get k unique random indices in range {0,...,n-1} (sampling without
   * replacement) \note No check is made to ensure that k <= n.
   * \param n upper index range, exclusive
   * \param k number of unique indices to sample
   * \return k unique random indices in range {0,...,n-1}
   */
  inline std::vector<int>
  getUniqueRandomIndices(int n, int k)
  {
    // Marked sampled indices and sample counter
    std::vector<bool> sampled(n, false);
    int samples = 0;
    // Resulting unique indices
    std::vector<int> result;
    result.reserve(k);
    do {
      // Pick a random index in the range
      const int idx = (std::rand() % n);
      // If unique
      if (!sampled[idx]) {
        // Mark as sampled and increment result counter
        sampled[idx] = true;
        ++samples;
        // Store
        result.push_back(idx);
      }
    } while (samples < k);

    return (result);
  }

  /** \brief Squared Euclidean distance between two points using the members x, y and z
   * \param p1 first point
   * \param p2 second point
   * \return squared Euclidean distance
   */
  inline float
  computeSquaredDistance(const SourceT& p1, const TargetT& p2)
  {
    const float dx = p2.x - p1.x;
    const float dy = p2.y - p1.y;
    const float dz = p2.z - p1.z;

    return (dx * dx + dy * dy + dz * dz);
  }

  /** \brief Edge length similarity thresholding
   * \param index_query_1 index of first source vertex
   * \param index_query_2 index of second source vertex
   * \param index_match_1 index of first target vertex
   * \param index_match_2 index of second target vertex
   * \param simsq squared similarity threshold in [0,1]
   * \return true if edge length ratio is larger than or equal to threshold
   */
  inline bool
  thresholdEdgeLength(int index_query_1,
                      int index_query_2,
                      int index_match_1,
                      int index_match_2,
                      float simsq)
  {
    // Distance between source points
    const float dist_src =
        computeSquaredDistance((*input_)[index_query_1], (*input_)[index_query_2]);
    // Distance between target points
    const float dist_tgt =
        computeSquaredDistance((*target_)[index_match_1], (*target_)[index_match_2]);
    // Edge length similarity [0,1] where 1 is a perfect match
    const float edge_sim =
        (dist_src < dist_tgt ? dist_src / dist_tgt : dist_tgt / dist_src);

    return (edge_sim >= simsq);
  }

  /** \brief Compute a linear histogram. This function is equivalent to the MATLAB
   * function \b histc, with the edges set as follows: <b>
   * lower:(upper-lower)/bins:upper </b>
   * \param data input samples
   * \param lower lower bound of input samples
   * \param upper upper bound of input samples
   * \param bins number of bins in output
   * \return linear histogram
   */
  std::vector<int>
  computeHistogram(const std::vector<float>& data, float lower, float upper, int bins);

  /** \brief Find the optimal value for binary histogram thresholding using Otsu's
   * method
   * \param histogram input histogram \return threshold value according to Otsu's
   * criterion
   */
  int
  findThresholdOtsu(const std::vector<int>& histogram);

  /** \brief The input point cloud dataset */
  PointCloudSourceConstPtr input_;

  /** \brief The input point cloud dataset target */
  PointCloudTargetConstPtr target_;

  /** \brief Number of iterations to run */
  int iterations_;

  /** \brief The polygon cardinality used during rejection */
  int cardinality_;

  /** \brief Lower edge length threshold in [0,1] used for verifying polygon
   * similarities, where 1 is a perfect match */
  float similarity_threshold_;

  /** \brief Squared value if \ref similarity_threshold_, only for internal use */
  float similarity_threshold_squared_;
};
} // namespace registration
} // namespace pcl

#include <pcl/registration/impl/correspondence_rejection_poly.hpp>
