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
 * $Id: lum.h 5663 2012-05-02 13:49:39Z florentinus $
 *
 */

#pragma once

#include <pcl/common/transforms.h>
#include <pcl/registration/boost_graph.h>
#include <pcl/correspondence.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>

namespace Eigen {
using Vector6f = Eigen::Matrix<float, 6, 1>;
using Matrix6f = Eigen::Matrix<float, 6, 6>;
} // namespace Eigen

namespace pcl {
namespace registration {
/** \brief Globally Consistent Scan Matching based on an algorithm by Lu and Milios.
 * \details A GraphSLAM algorithm where registration data is managed in a graph:
 * <ul>
 *  <li>Vertices represent poses and hold the point cloud data and relative
 * transformations.</li> <li>Edges represent pose constraints and hold the
 * correspondence data between two point clouds.</li>
 * </ul>
 * Computation uses the first point cloud in the SLAM graph as a reference pose and
 * attempts to align all other point clouds to it simultaneously. For more information:
 * <ul><li>
 * F. Lu, E. Milios,
 * Globally Consistent Range Scan Alignment for Environment Mapping,
 * Autonomous Robots 4, April 1997
 * </li><li>
 * Dorit Borrmann, Jan Elseberg, Kai Lingemann, Andreas Nüchter, and Joachim Hertzberg,
 * The Efficient Extension of Globally Consistent Scan Matching to 6 DoF,
 * In Proceedings of the 4th International Symposium on 3D Data Processing,
 * Visualization and Transmission (3DPVT '08), June 2008
 * </li></ul>
 * Usage example:
 * \code
 * pcl::registration::LUM<pcl::PointXYZ> lum;
 * // Add point clouds as vertices to the SLAM graph
 * lum.addPointCloud (cloud_0);
 * lum.addPointCloud (cloud_1);
 * lum.addPointCloud (cloud_2);
 * // Use your favorite pairwise correspondence estimation algorithm(s)
 * corrs_0_to_1 = someAlgo (cloud_0, cloud_1);
 * corrs_1_to_2 = someAlgo (cloud_1, cloud_2);
 * corrs_2_to_0 = someAlgo (lum.getPointCloud (2), lum.getPointCloud (0));
 * // Add the correspondence results as edges to the SLAM graph
 * lum.setCorrespondences (0, 1, corrs_0_to_1);
 * lum.setCorrespondences (1, 2, corrs_1_to_2);
 * lum.setCorrespondences (2, 0, corrs_2_to_0);
 * // Change the computation parameters
 * lum.setMaxIterations (5);
 * lum.setConvergenceThreshold (0.0);
 * // Perform the actual LUM computation
 * lum.compute ();
 * // Return the concatenated point cloud result
 * cloud_out = lum.getConcatenatedCloud ();
 * // Return the separate point cloud transformations
 * for(int i = 0; i < lum.getNumVertices (); i++)
 * {
 *   transforms_out[i] = lum.getTransformation (i);
 * }
 * \endcode
 * \author Frits Florentinus, Jochen Sprickerhof
 * \ingroup registration
 */
template <typename PointT>
class LUM {
public:
  using Ptr = shared_ptr<LUM<PointT>>;
  using ConstPtr = shared_ptr<const LUM<PointT>>;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  struct VertexProperties {
    PointCloudPtr cloud_;
    Eigen::Vector6f pose_;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
  struct EdgeProperties {
    pcl::CorrespondencesPtr corrs_;
    Eigen::Matrix6f cinv_;
    Eigen::Vector6f cinvd_;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  using SLAMGraph = boost::adjacency_list<boost::eigen_vecS,
                                          boost::eigen_vecS,
                                          boost::bidirectionalS,
                                          VertexProperties,
                                          EdgeProperties,
                                          boost::no_property,
                                          boost::eigen_listS>;
  using SLAMGraphPtr = shared_ptr<SLAMGraph>;
  using Vertex = typename SLAMGraph::vertex_descriptor;
  using Edge = typename SLAMGraph::edge_descriptor;

  /** \brief Empty constructor.
   */
  LUM() : slam_graph_(new SLAMGraph), max_iterations_(5), convergence_threshold_(0.0) {}

  /** \brief Set the internal SLAM graph structure.
   * \details All data used and produced by LUM is stored in this boost::adjacency_list.
   * It is recommended to use the LUM class itself to build the graph.
   * This method could otherwise be useful for managing several SLAM graphs in one
   * instance of LUM. \param[in] slam_graph The new SLAM graph.
   */
  inline void
  setLoopGraph(const SLAMGraphPtr& slam_graph);

  /** \brief Get the internal SLAM graph structure.
   * \details All data used and produced by LUM is stored in this boost::adjacency_list.
   * It is recommended to use the LUM class itself to build the graph.
   * This method could otherwise be useful for managing several SLAM graphs in one
   * instance of LUM. \return The current SLAM graph.
   */
  inline SLAMGraphPtr
  getLoopGraph() const;

  /** \brief Get the number of vertices in the SLAM graph.
   * \return The current number of vertices in the SLAM graph.
   */
  typename SLAMGraph::vertices_size_type
  getNumVertices() const;

  /** \brief Set the maximum number of iterations for the compute() method.
   * \details The compute() method finishes when max_iterations are met or when the
   * convergence criteria is met. \param[in] max_iterations The new maximum number of
   * iterations (default = 5).
   */
  void
  setMaxIterations(int max_iterations);

  /** \brief Get the maximum number of iterations for the compute() method.
   * \details The compute() method finishes when max_iterations are met or when the
   * convergence criteria is met. \return The current maximum number of iterations
   * (default = 5).
   */
  inline int
  getMaxIterations() const;

  /** \brief Set the convergence threshold for the compute() method.
   * \details When the compute() method computes the new poses relative to the old
   * poses, it will determine the length of the difference vector. When the average
   * length of all difference vectors becomes less than the convergence_threshold the
   * convergence is assumed to be met. \param[in] convergence_threshold The new
   * convergence threshold (default = 0.0).
   */
  void
  setConvergenceThreshold(float convergence_threshold);

  /** \brief Get the convergence threshold for the compute() method.
   * \details When the compute() method computes the new poses relative to the old
   * poses, it will determine the length of the difference vector. When the average
   * length of all difference vectors becomes less than the convergence_threshold the
   * convergence is assumed to be met. \return The current convergence threshold
   * (default = 0.0).
   */
  inline float
  getConvergenceThreshold() const;

  /** \brief Add a new point cloud to the SLAM graph.
   * \details This method will add a new vertex to the SLAM graph and attach a point
   * cloud to that vertex. Optionally you can specify a pose estimate for this point
   * cloud. A vertex' pose is always relative to the first vertex in the SLAM graph,
   * i.e. the first point cloud that was added. Because this first vertex is the
   * reference, you can not set a pose estimate for this vertex. Providing pose
   * estimates to the vertices in the SLAM graph will reduce overall computation time of
   * LUM. \note Vertex descriptors are typecastable to int. \param[in] cloud The new
   * point cloud. \param[in] pose (optional) The pose estimate relative to the reference
   * pose (first point cloud that was added). \return The vertex descriptor of the newly
   * created vertex.
   */
  Vertex
  addPointCloud(const PointCloudPtr& cloud,
                const Eigen::Vector6f& pose = Eigen::Vector6f::Zero());

  /** \brief Change a point cloud on one of the SLAM graph's vertices.
   * \details This method will change the point cloud attached to an existing vertex and
   * will not alter the SLAM graph structure. Note that the correspondences attached to
   * this vertex will not change and may need to be updated manually. \note Vertex
   * descriptors are typecastable to int. \param[in] vertex The vertex descriptor of
   * which to change the point cloud. \param[in] cloud The new point cloud for that
   * vertex.
   */
  inline void
  setPointCloud(const Vertex& vertex, const PointCloudPtr& cloud);

  /** \brief Return a point cloud from one of the SLAM graph's vertices.
   * \note Vertex descriptors are typecastable to int.
   * \param[in] vertex The vertex descriptor of which to return the point cloud.
   * \return The current point cloud for that vertex.
   */
  inline PointCloudPtr
  getPointCloud(const Vertex& vertex) const;

  /** \brief Change a pose estimate on one of the SLAM graph's vertices.
   * \details A vertex' pose is always relative to the first vertex in the SLAM graph,
   * i.e. the first point cloud that was added. Because this first vertex is the
   * reference, you can not set a pose estimate for this vertex. Providing pose
   * estimates to the vertices in the SLAM graph will reduce overall computation time of
   * LUM. \note Vertex descriptors are typecastable to int. \param[in] vertex The vertex
   * descriptor of which to set the pose estimate. \param[in] pose The new pose estimate
   * for that vertex.
   */
  inline void
  setPose(const Vertex& vertex, const Eigen::Vector6f& pose);

  /** \brief Return a pose estimate from one of the SLAM graph's vertices.
   * \note Vertex descriptors are typecastable to int.
   * \param[in] vertex The vertex descriptor of which to return the pose estimate.
   * \return The current pose estimate of that vertex.
   */
  inline Eigen::Vector6f
  getPose(const Vertex& vertex) const;

  /** \brief Return a pose estimate from one of the SLAM graph's vertices as an affine
   * transformation matrix. \note Vertex descriptors are typecastable to int. \param[in]
   * vertex The vertex descriptor of which to return the transformation matrix. \return
   * The current transformation matrix of that vertex.
   */
  inline Eigen::Affine3f
  getTransformation(const Vertex& vertex) const;

  /** \brief Add/change a set of correspondences for one of the SLAM graph's edges.
   * \details The edges in the SLAM graph are directional and point from source vertex
   * to target vertex. The query indices of the correspondences, index the points at the
   * source vertex' point cloud. The matching indices of the correspondences, index the
   * points at the target vertex' point cloud. If no edge was present at the specified
   * location, this method will add a new edge to the SLAM graph and attach the
   * correspondences to that edge. If the edge was already present, this method will
   * overwrite the correspondence information of that edge and will not alter the SLAM
   * graph structure. \note Vertex descriptors are typecastable to int. \param[in]
   * source_vertex The vertex descriptor of the correspondences' source point cloud.
   * \param[in] target_vertex The vertex descriptor of the correspondences' target point
   * cloud. \param[in] corrs The new set of correspondences for that edge.
   */
  void
  setCorrespondences(const Vertex& source_vertex,
                     const Vertex& target_vertex,
                     const pcl::CorrespondencesPtr& corrs);

  /** \brief Return a set of correspondences from one of the SLAM graph's edges.
   * \note Vertex descriptors are typecastable to int.
   * \param[in] source_vertex The vertex descriptor of the correspondences' source point
   * cloud. \param[in] target_vertex The vertex descriptor of the correspondences'
   * target point cloud. \return The current set of correspondences of that edge.
   */
  inline pcl::CorrespondencesPtr
  getCorrespondences(const Vertex& source_vertex, const Vertex& target_vertex) const;

  /** \brief Perform LUM's globally consistent scan matching.
   * \details Computation uses the first point cloud in the SLAM graph as a reference
   * pose and attempts to align all other point clouds to it simultaneously. <br> Things
   * to keep in mind: <ul> <li>Only those parts of the graph connected to the reference
   * pose will properly align to it.</li> <li>All sets of correspondences should span
   * the same space and need to be sufficient to determine a rigid transformation.</li>
   *  <li>The algorithm draws it strength from loops in the graph because it will
   * distribute errors evenly amongst those loops.</li>
   * </ul>
   * Computation ends when either of the following conditions hold:
   * <ul>
   *  <li>The number of iterations reaches max_iterations. Use setMaxIterations() to
   * change.</li> <li>The convergence criteria is met. Use setConvergenceThreshold() to
   * change.</li>
   * </ul>
   * Computation will change the pose estimates for the vertices of the SLAM graph, not
   * the point clouds attached to them. The results can be retrieved with getPose(),
   * getTransformation(), getTransformedCloud() or getConcatenatedCloud().
   */
  void
  compute();

  /** \brief Return a point cloud from one of the SLAM graph's vertices compounded onto
   * its current pose estimate. \note Vertex descriptors are typecastable to int.
   * \param[in] vertex The vertex descriptor of which to return the transformed point
   * cloud. \return The transformed point cloud of that vertex.
   */
  PointCloudPtr
  getTransformedCloud(const Vertex& vertex) const;

  /** \brief Return a concatenated point cloud of all the SLAM graph's point clouds
   * compounded onto their current pose estimates. \return The concatenated transformed
   * point clouds of the entire SLAM graph.
   */
  PointCloudPtr
  getConcatenatedCloud() const;

protected:
  /** \brief Linearized computation of C^-1 and C^-1*D (results stored in slam_graph_).
   */
  void
  computeEdge(const Edge& e);

  /** \brief Returns a pose corrected 6DoF incidence matrix. */
  inline Eigen::Matrix6f
  incidenceCorrection(const Eigen::Vector6f& pose);

private:
  /** \brief The internal SLAM graph structure. */
  SLAMGraphPtr slam_graph_;

  /** \brief The maximum number of iterations for the compute() method. */
  int max_iterations_;

  /** \brief The convergence threshold for the summed vector lengths of all poses. */
  float convergence_threshold_;
};
} // namespace registration
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/registration/impl/lum.hpp>
#endif
