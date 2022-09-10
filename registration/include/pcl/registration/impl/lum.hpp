/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: lum.hpp 5663 2012-05-02 13:49:39Z florentinus $
 *
 */

#ifndef PCL_REGISTRATION_IMPL_LUM_HPP_
#define PCL_REGISTRATION_IMPL_LUM_HPP_

#include <tuple>

namespace pcl {

namespace registration {

template <typename PointT>
inline void
LUM<PointT>::setLoopGraph(const SLAMGraphPtr& slam_graph)
{
  slam_graph_ = slam_graph;
}

template <typename PointT>
inline typename LUM<PointT>::SLAMGraphPtr
LUM<PointT>::getLoopGraph() const
{
  return (slam_graph_);
}

template <typename PointT>
typename LUM<PointT>::SLAMGraph::vertices_size_type
LUM<PointT>::getNumVertices() const
{
  return (num_vertices(*slam_graph_));
}

template <typename PointT>
void
LUM<PointT>::setMaxIterations(int max_iterations)
{
  max_iterations_ = max_iterations;
}

template <typename PointT>
inline int
LUM<PointT>::getMaxIterations() const
{
  return (max_iterations_);
}

template <typename PointT>
void
LUM<PointT>::setConvergenceThreshold(float convergence_threshold)
{
  convergence_threshold_ = convergence_threshold;
}

template <typename PointT>
inline float
LUM<PointT>::getConvergenceThreshold() const
{
  return (convergence_threshold_);
}

template <typename PointT>
typename LUM<PointT>::Vertex
LUM<PointT>::addPointCloud(const PointCloudPtr& cloud, const Eigen::Vector6f& pose)
{
  Vertex v = add_vertex(*slam_graph_);
  (*slam_graph_)[v].cloud_ = cloud;
  if (v == 0 && pose != Eigen::Vector6f::Zero()) {
    PCL_WARN(
        "[pcl::registration::LUM::addPointCloud] The pose estimate is ignored for the "
        "first cloud in the graph since that will become the reference pose.\n");
    (*slam_graph_)[v].pose_ = Eigen::Vector6f::Zero();
    return (v);
  }
  (*slam_graph_)[v].pose_ = pose;
  return (v);
}

template <typename PointT>
inline void
LUM<PointT>::setPointCloud(const Vertex& vertex, const PointCloudPtr& cloud)
{
  if (vertex >= getNumVertices()) {
    PCL_ERROR("[pcl::registration::LUM::setPointCloud] You are attempting to set a "
              "point cloud to a non-existing graph vertex.\n");
    return;
  }
  (*slam_graph_)[vertex].cloud_ = cloud;
}

template <typename PointT>
inline typename LUM<PointT>::PointCloudPtr
LUM<PointT>::getPointCloud(const Vertex& vertex) const
{
  if (vertex >= getNumVertices()) {
    PCL_ERROR("[pcl::registration::LUM::getPointCloud] You are attempting to get a "
              "point cloud from a non-existing graph vertex.\n");
    return (PointCloudPtr());
  }
  return ((*slam_graph_)[vertex].cloud_);
}

template <typename PointT>
inline void
LUM<PointT>::setPose(const Vertex& vertex, const Eigen::Vector6f& pose)
{
  if (vertex >= getNumVertices()) {
    PCL_ERROR("[pcl::registration::LUM::setPose] You are attempting to set a pose "
              "estimate to a non-existing graph vertex.\n");
    return;
  }
  if (vertex == 0) {
    PCL_ERROR("[pcl::registration::LUM::setPose] The pose estimate is ignored for the "
              "first cloud in the graph since that will become the reference pose.\n");
    return;
  }
  (*slam_graph_)[vertex].pose_ = pose;
}

template <typename PointT>
inline Eigen::Vector6f
LUM<PointT>::getPose(const Vertex& vertex) const
{
  if (vertex >= getNumVertices()) {
    PCL_ERROR("[pcl::registration::LUM::getPose] You are attempting to get a pose "
              "estimate from a non-existing graph vertex.\n");
    return (Eigen::Vector6f::Zero());
  }
  return ((*slam_graph_)[vertex].pose_);
}

template <typename PointT>
inline Eigen::Affine3f
LUM<PointT>::getTransformation(const Vertex& vertex) const
{
  Eigen::Vector6f pose = getPose(vertex);
  return (pcl::getTransformation(pose(0), pose(1), pose(2), pose(3), pose(4), pose(5)));
}

template <typename PointT>
void
LUM<PointT>::setCorrespondences(const Vertex& source_vertex,
                                const Vertex& target_vertex,
                                const pcl::CorrespondencesPtr& corrs)
{
  if (source_vertex >= getNumVertices() || target_vertex >= getNumVertices() ||
      source_vertex == target_vertex) {
    PCL_ERROR(
        "[pcl::registration::LUM::setCorrespondences] You are attempting to set a set "
        "of correspondences between non-existing or identical graph vertices.\n");
    return;
  }
  Edge e;
  bool present;
  std::tie(e, present) = edge(source_vertex, target_vertex, *slam_graph_);
  if (!present)
    std::tie(e, present) = add_edge(source_vertex, target_vertex, *slam_graph_);
  (*slam_graph_)[e].corrs_ = corrs;
}

template <typename PointT>
inline pcl::CorrespondencesPtr
LUM<PointT>::getCorrespondences(const Vertex& source_vertex,
                                const Vertex& target_vertex) const
{
  if (source_vertex >= getNumVertices() || target_vertex >= getNumVertices()) {
    PCL_ERROR("[pcl::registration::LUM::getCorrespondences] You are attempting to get "
              "a set of correspondences between non-existing graph vertices.\n");
    return (pcl::CorrespondencesPtr());
  }
  Edge e;
  bool present;
  std::tie(e, present) = edge(source_vertex, target_vertex, *slam_graph_);
  if (!present) {
    PCL_ERROR("[pcl::registration::LUM::getCorrespondences] You are attempting to get "
              "a set of correspondences from a non-existing graph edge.\n");
    return (pcl::CorrespondencesPtr());
  }
  return ((*slam_graph_)[e].corrs_);
}

template <typename PointT>
void
LUM<PointT>::compute()
{
  int n = static_cast<int>(getNumVertices());
  if (n < 2) {
    PCL_ERROR("[pcl::registration::LUM::compute] The slam graph needs at least 2 "
              "vertices.\n");
    return;
  }
  for (int i = 0; i < max_iterations_; ++i) {
    // Linearized computation of C^-1 and C^-1*D and convergence checking for all edges
    // in the graph (results stored in slam_graph_)
    typename SLAMGraph::edge_iterator e, e_end;
    for (std::tie(e, e_end) = edges(*slam_graph_); e != e_end; ++e)
      computeEdge(*e);

    // Declare matrices G and B
    Eigen::MatrixXf G = Eigen::MatrixXf::Zero(6 * (n - 1), 6 * (n - 1));
    Eigen::VectorXf B = Eigen::VectorXf::Zero(6 * (n - 1));

    // Start at 1 because 0 is the reference pose
    for (int vi = 1; vi != n; ++vi) {
      for (int vj = 0; vj != n; ++vj) {
        // Attempt to use the forward edge, otherwise use backward edge, otherwise there
        // was no edge
        Edge e;
        bool present1;
        std::tie(e, present1) = edge(vi, vj, *slam_graph_);
        if (!present1) {
          bool present2;
          std::tie(e, present2) = edge(vj, vi, *slam_graph_);
          if (!present2)
            continue;
        }

        // Fill in elements of G and B
        if (vj > 0)
          G.block(6 * (vi - 1), 6 * (vj - 1), 6, 6) = -(*slam_graph_)[e].cinv_;
        G.block(6 * (vi - 1), 6 * (vi - 1), 6, 6) += (*slam_graph_)[e].cinv_;
        B.segment(6 * (vi - 1), 6) += (present1 ? 1 : -1) * (*slam_graph_)[e].cinvd_;
      }
    }

    // Computation of the linear equation system: GX = B
    // TODO investigate accuracy vs. speed tradeoff and find the best solving method for
    // our type of linear equation (sparse)
    Eigen::VectorXf X = G.colPivHouseholderQr().solve(B);

    // Update the poses
    float sum = 0.0;
    for (int vi = 1; vi != n; ++vi) {
      Eigen::Vector6f difference_pose = static_cast<Eigen::Vector6f>(
          -incidenceCorrection(getPose(vi)).inverse() * X.segment(6 * (vi - 1), 6));
      sum += difference_pose.norm();
      setPose(vi, getPose(vi) + difference_pose);
    }

    // Convergence check
    if (sum <= convergence_threshold_ * static_cast<float>(n - 1))
      return;
  }
}

template <typename PointT>
typename LUM<PointT>::PointCloudPtr
LUM<PointT>::getTransformedCloud(const Vertex& vertex) const
{
  PointCloudPtr out(new PointCloud);
  pcl::transformPointCloud(*getPointCloud(vertex), *out, getTransformation(vertex));
  return (out);
}

template <typename PointT>
typename LUM<PointT>::PointCloudPtr
LUM<PointT>::getConcatenatedCloud() const
{
  PointCloudPtr out(new PointCloud);
  typename SLAMGraph::vertex_iterator v, v_end;
  for (std::tie(v, v_end) = vertices(*slam_graph_); v != v_end; ++v) {
    PointCloud temp;
    pcl::transformPointCloud(*getPointCloud(*v), temp, getTransformation(*v));
    *out += temp;
  }
  return (out);
}

template <typename PointT>
void
LUM<PointT>::computeEdge(const Edge& e)
{
  // Get necessary local data from graph
  PointCloudPtr source_cloud = (*slam_graph_)[source(e, *slam_graph_)].cloud_;
  PointCloudPtr target_cloud = (*slam_graph_)[target(e, *slam_graph_)].cloud_;
  Eigen::Vector6f source_pose = (*slam_graph_)[source(e, *slam_graph_)].pose_;
  Eigen::Vector6f target_pose = (*slam_graph_)[target(e, *slam_graph_)].pose_;
  pcl::CorrespondencesPtr corrs = (*slam_graph_)[e].corrs_;

  // Build the average and difference vectors for all correspondences
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> corrs_aver(
      corrs->size());
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> corrs_diff(
      corrs->size());
  int oci = 0;                       // oci = output correspondence iterator
  for (const auto& icorr : (*corrs)) // icorr = input correspondence
  {
    // Compound the point pair onto the current pose
    Eigen::Vector3f source_compounded =
        pcl::getTransformation(source_pose(0),
                               source_pose(1),
                               source_pose(2),
                               source_pose(3),
                               source_pose(4),
                               source_pose(5)) *
        (*source_cloud)[icorr.index_query].getVector3fMap();
    Eigen::Vector3f target_compounded =
        pcl::getTransformation(target_pose(0),
                               target_pose(1),
                               target_pose(2),
                               target_pose(3),
                               target_pose(4),
                               target_pose(5)) *
        (*target_cloud)[icorr.index_match].getVector3fMap();

    // NaN points can not be passed to the remaining computational pipeline
    if (!std::isfinite(source_compounded(0)) || !std::isfinite(source_compounded(1)) ||
        !std::isfinite(source_compounded(2)) || !std::isfinite(target_compounded(0)) ||
        !std::isfinite(target_compounded(1)) || !std::isfinite(target_compounded(2)))
      continue;

    // Compute the point pair average and difference and store for later use
    corrs_aver[oci] = 0.5 * (source_compounded + target_compounded);
    corrs_diff[oci] = source_compounded - target_compounded;
    oci++;
  }
  corrs_aver.resize(oci);
  corrs_diff.resize(oci);

  // Need enough valid correspondences to get a good triangulation
  if (oci < 3) {
    PCL_WARN("[pcl::registration::LUM::compute] The correspondences between vertex %d "
             "and %d do not contain enough valid correspondences to be considered for "
             "computation.\n",
             source(e, *slam_graph_),
             target(e, *slam_graph_));
    (*slam_graph_)[e].cinv_ = Eigen::Matrix6f::Zero();
    (*slam_graph_)[e].cinvd_ = Eigen::Vector6f::Zero();
    return;
  }

  // Build the matrices M'M and M'Z
  Eigen::Matrix6f MM = Eigen::Matrix6f::Zero();
  Eigen::Vector6f MZ = Eigen::Vector6f::Zero();
  for (int ci = 0; ci != oci; ++ci) // ci = correspondence iterator
  {
    // Fast computation of summation elements of M'M
    MM(0, 4) -= corrs_aver[ci](1);
    MM(0, 5) += corrs_aver[ci](2);
    MM(1, 3) -= corrs_aver[ci](2);
    MM(1, 4) += corrs_aver[ci](0);
    MM(2, 3) += corrs_aver[ci](1);
    MM(2, 5) -= corrs_aver[ci](0);
    MM(3, 4) -= corrs_aver[ci](0) * corrs_aver[ci](2);
    MM(3, 5) -= corrs_aver[ci](0) * corrs_aver[ci](1);
    MM(4, 5) -= corrs_aver[ci](1) * corrs_aver[ci](2);
    MM(3, 3) +=
        corrs_aver[ci](1) * corrs_aver[ci](1) + corrs_aver[ci](2) * corrs_aver[ci](2);
    MM(4, 4) +=
        corrs_aver[ci](0) * corrs_aver[ci](0) + corrs_aver[ci](1) * corrs_aver[ci](1);
    MM(5, 5) +=
        corrs_aver[ci](0) * corrs_aver[ci](0) + corrs_aver[ci](2) * corrs_aver[ci](2);

    // Fast computation of M'Z
    MZ(0) += corrs_diff[ci](0);
    MZ(1) += corrs_diff[ci](1);
    MZ(2) += corrs_diff[ci](2);
    MZ(3) +=
        corrs_aver[ci](1) * corrs_diff[ci](2) - corrs_aver[ci](2) * corrs_diff[ci](1);
    MZ(4) +=
        corrs_aver[ci](0) * corrs_diff[ci](1) - corrs_aver[ci](1) * corrs_diff[ci](0);
    MZ(5) +=
        corrs_aver[ci](2) * corrs_diff[ci](0) - corrs_aver[ci](0) * corrs_diff[ci](2);
  }
  // Remaining elements of M'M
  MM(0, 0) = MM(1, 1) = MM(2, 2) = static_cast<float>(oci);
  MM(4, 0) = MM(0, 4);
  MM(5, 0) = MM(0, 5);
  MM(3, 1) = MM(1, 3);
  MM(4, 1) = MM(1, 4);
  MM(3, 2) = MM(2, 3);
  MM(5, 2) = MM(2, 5);
  MM(4, 3) = MM(3, 4);
  MM(5, 3) = MM(3, 5);
  MM(5, 4) = MM(4, 5);

  // Compute pose difference estimation
  Eigen::Vector6f D = static_cast<Eigen::Vector6f>(MM.inverse() * MZ);

  // Compute s^2
  float ss = 0.0f;
  for (int ci = 0; ci != oci; ++ci) // ci = correspondence iterator
    ss += static_cast<float>(
        std::pow(corrs_diff[ci](0) -
                     (D(0) + corrs_aver[ci](2) * D(5) - corrs_aver[ci](1) * D(4)),
                 2.0f) +
        std::pow(corrs_diff[ci](1) -
                     (D(1) + corrs_aver[ci](0) * D(4) - corrs_aver[ci](2) * D(3)),
                 2.0f) +
        std::pow(corrs_diff[ci](2) -
                     (D(2) + corrs_aver[ci](1) * D(3) - corrs_aver[ci](0) * D(5)),
                 2.0f));

  // When reaching the limitations of computation due to linearization
  if (ss < 0.0000000000001 || !std::isfinite(ss)) {
    (*slam_graph_)[e].cinv_ = Eigen::Matrix6f::Zero();
    (*slam_graph_)[e].cinvd_ = Eigen::Vector6f::Zero();
    return;
  }

  // Store the results in the slam graph
  (*slam_graph_)[e].cinv_ = MM * (1.0f / ss);
  (*slam_graph_)[e].cinvd_ = MZ * (1.0f / ss);
}

template <typename PointT>
inline Eigen::Matrix6f
LUM<PointT>::incidenceCorrection(const Eigen::Vector6f& pose)
{
  Eigen::Matrix6f out = Eigen::Matrix6f::Identity();
  float cx = std::cos(pose(3)), sx = sinf(pose(3)), cy = std::cos(pose(4)),
        sy = sinf(pose(4));
  out(0, 4) = pose(1) * sx - pose(2) * cx;
  out(0, 5) = pose(1) * cx * cy + pose(2) * sx * cy;
  out(1, 3) = pose(2);
  out(1, 4) = -pose(0) * sx;
  out(1, 5) = -pose(0) * cx * cy + pose(2) * sy;
  out(2, 3) = -pose(1);
  out(2, 4) = pose(0) * cx;
  out(2, 5) = -pose(0) * sx * cy - pose(1) * sy;
  out(3, 5) = sy;
  out(4, 4) = sx;
  out(4, 5) = cx * cy;
  out(5, 4) = cx;
  out(5, 5) = -sx * cy;
  return (out);
}

} // namespace registration
} // namespace pcl

#define PCL_INSTANTIATE_LUM(T) template class PCL_EXPORTS pcl::registration::LUM<T>;

#endif // PCL_REGISTRATION_IMPL_LUM_HPP_
