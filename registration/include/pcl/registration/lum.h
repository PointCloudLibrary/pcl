/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_LUM_H_
#define PCL_LUM_H_

#include <pcl/pcl_base.h>
#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>
#include <boost/graph/adjacency_list.hpp>

namespace pcl
{
  namespace registration
  {
    /** \brief @b Globally Consistent Scan Matching based on an algorithm by Lu and Milios.
     * <br><br>
     * A GraphSLAM algorithm where registration data is managed in a graph:
     * <ul>
     *  <li>Vertices represent poses and hold the point cloud data and relative transformations.</li>
     *  <li>Edges represent pose constraints and hold the correspondence data between two point clouds.</li>
     * </ul>
     * This graph structure currently resides inside the LUM class but could be useful for other GraphSLAM algorithms or just for storing your registration data more elegantly.
     * <br><br>
     * Computation uses the first point cloud in the SLAM graph as a reference pose and attempts to align all other point clouds to it simultaneously.
     * This will only produce interesting results if the graph is fully connected and contains loops.
     * Currently you have to satisfy these two conditions yourself.
     * <br><br>
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
     * lum.setMaxIterations (10);
     * lum.setConvergenceDistance (0);
     * lum.setConvergenceAngle (0);
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
    template<typename PointT>
      class LUM : public PCLBase<PointT>
      {
      public:
        typedef boost::shared_ptr<LUM<PointT> > Ptr;
        typedef boost::shared_ptr<const LUM<PointT> > ConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef Eigen::Matrix<float, 6, 1> Vector6f;
        typedef Eigen::Matrix<float, 6, 6> Matrix6f;

        struct VertexProperties
        {
          PointCloudPtr cloud_;
          Vector6f pose_;
        };
        struct EdgeProperties
        {
          pcl::CorrespondencesPtr corrs_;
          Matrix6f cinv_;
          Vector6f cinvd_;
          bool converged_;
        };

        //space complexity: vecS < slistS < listS
        //time complexity: add/remove vertex: listS == slists < vecS
        //time complexity: add edge: listS == slists < vecS
        //time complexity: edge iteration: vecS < slistS < listS
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexProperties, EdgeProperties> SLAMGraph;
        typedef boost::shared_ptr<SLAMGraph> SLAMGraphPtr;
        typedef typename SLAMGraph::vertex_descriptor Vertex;
        typedef typename SLAMGraph::edge_descriptor Edge;

        /** \brief Empty constructor.
         */
        LUM () :
            slam_graph_ (new SLAMGraph), max_iterations_ (5), convergence_distance_ (0.001), convergence_angle_ (0.01)
        {
        }
        ;

        /** \brief Set the internal SLAM graph structure.
         * \param[in] slam_graph the new SLAM graph (based on boost::adjacency_list)
         */
        inline void
        setLoopGraph (SLAMGraphPtr slam_graph)
        {
          slam_graph_ = slam_graph;
        }

        /** \brief Return the internal SLAM graph structure.
         * \return the current SLAM graph (based on boost::adjacency_list)
         */
        inline SLAMGraphPtr
        getLoopGraph ()
        {
          return (slam_graph_);
        }

        /** \brief Return the number of vertices in the SLAM graph.
         * \return the current number of vertices in the SLAM graph
         */
        inline typename SLAMGraph::vertices_size_type
        getNumVertices ()
        {
          return num_vertices (*slam_graph_);
        }

        /** \brief Set the maximum number of iterations for the compute() method.
         * \note The compute() method finishes when max_iterations are met or when all edges in the SLAM graph have converged (min_iterations = 0).
         * \param[in] max_iterations the new maximum number of iterations (default = 5)
         */
        inline void
        setMaxIterations (size_t max_iterations)
        {
          max_iterations_ = max_iterations;
        }

        /** \brief Return the maximum number of iterations for the compute() method.
         * \return the current maximum number of iterations (default = 5)
         */
        inline size_t
        getMaxIterations ()
        {
          return (max_iterations_);
        }

        /** \brief Set the convergence threshold for the distance that two aligning point cloud pose estimates may be apart.
         * \note An edge in the SLAM graph converges when its relative pose distance is below convergence_distance and relative pose angle is below convergence_angle.
         *       The compute() method finishes when all edges in the SLAM graph have converged (or when max_iterations are met).
         * \param[in] convergence_distance the new convergence threshold for distances (default = 0.001)
         */
        inline void
        setConvergenceDistance (float convergence_distance)
        {
          convergence_distance_ = convergence_distance;
        }

        /** \brief Return the convergence threshold for the distance that two aligning point cloud pose estimates may be apart.
         * \return the current convergence threshold for distances (default = 0.001)
         */
        inline float
        getConvergenceDistance ()
        {
          return (convergence_distance_);
        }

        /** \brief Set the convergence threshold for the angle that two aligning point cloud pose estimates may be apart.
         * \note An edge in the SLAM graph converges when its relative pose distance is below convergence_distance and relative pose angle is below convergence_angle.
         *       The compute() method finishes when all edges in the SLAM graph have converged (or when max_iterations are met).
         * \param[in] convergence_angle the new convergence threshold for angles (default = 0.01)
         */
        inline void
        setConvergenceAngle (float convergence_angle)
        {
          convergence_angle_ = convergence_angle;
        }

        /** \brief Return the convergence threshold for the distance that two aligning point cloud pose estimates may be apart.
         * \return the current convergence threshold for angles (default = 0.01)
         */
        inline float
        getConvergenceAngle ()
        {
          return (convergence_angle_);
        }

        /** \brief Add a new point cloud to the SLAM graph.
         * \param[in] cloud the new point cloud
         * \return the vertex descriptor (typecastable to int) of the newly created vertex that references this point cloud
         */
        inline Vertex
        addPointCloud (PointCloudPtr cloud);

        /** \brief Add a new point cloud to the SLAM graph with a pose estimate.
         * \note The pose estimate is ignored for the first point cloud in the SLAM graph since that will become the reference pose.
         *       More accurate pose estimates result in faster computation times.
         * \param[in] cloud the new point cloud
         * \param[in] pose the pose estimate relative to the reference pose (first point cloud that was added)
         * \return the vertex descriptor (typecastable to int) of the newly created vertex that references this point cloud
         */
        inline Vertex
        addPointCloud (PointCloudPtr cloud, Vector6f pose);

        /** \brief Set a point cloud to one of the SLAM graph's vertices.
         * \note Is used on a previously added vertex, does not add a new vertex to the SLAM graph.
         * \param[in] vertex the vertex descriptor of which to set the point cloud
         * \param[in] cloud the new point cloud for that vertex
         */
        inline void
        setPointCloud (Vertex vertex, PointCloudPtr cloud);

        /** \brief Return a point cloud from one of the SLAM graph's vertices.
         * \param[in] vertex the vertex descriptor of which to return the point cloud
         * \return the current point cloud of that vertex
         */
        inline PointCloudPtr
        getPointCloud (Vertex vertex);

        /** \brief Set a pose estimate to one of the SLAM graph's vertices.
         * \note The pose estimate is ignored for the first point cloud in the SLAM graph since that is the reference pose.
         *       More accurate pose estimates result in faster computation times.
         * \param[in] vertex the vertex descriptor of which to set the pose estimate
         * \param[in] pose the new pose estimate for that vertex
         */
        inline void
        setPose (Vertex vertex, Vector6f pose);

        /** \brief Return a pose estimate from one of the SLAM graph's vertices.
         * \param[in] vertex the vertex descriptor of which to return the pose estimate
         * \return the current pose estimate of that vertex
         */
        inline Vector6f
        getPose (Vertex vertex);

        /** \brief Return a pose estimate from one of the SLAM graph's vertices as an affine transformation matrix.
         * \param[in] vertex the vertex descriptor of which to return the transformation
         * \return the current transformation of that vertex
         */
        inline Eigen::Affine3f
        getTransformation (Vertex vertex)
        {
          return poseToTransform (getPose (vertex));
        }

        /** \brief Set a set of correspondences to one of the SLAM graph's edges.
         * \note The set needs to contain at least 3 different correspondences.
         * \param[in] source_vertex the vertex descriptor of the correspondences' source point cloud
         * \param[in] target_vertex the vertex descriptor of the correspondences' target point cloud
         * \param[in] corr the new set of correspondences for that edge
         */
        inline void
        setCorrespondences (Vertex source_vertex, Vertex target_vertex, pcl::CorrespondencesPtr corrs);

        /** \brief Return a set of correspondences from one of the SLAM graph's edges.
         * \param[in] source_vertex the vertex descriptor of the correspondences' source point cloud
         * \param[in] target_vertex the vertex descriptor of the correspondences' target point cloud
         * \return the current set of correspondences of that edge
         */
        inline pcl::CorrespondencesPtr
        getCorrespondences (Vertex source_vertex, Vertex target_vertex);

        /** \brief Commence LUM computation.
         * <br><br>
         * Computation uses the first point cloud in the SLAM graph as a reference pose and attempts to align all other point clouds to it simultaneously.
         * This will only produce interesting results if the graph is fully connected and contains loops.
         * Currently you have to satisfy these two conditions yourself.
         * <br><br>
         * Computation will terminate for either of the following criteria:
         * <ul>
         *  <li>The number of iterations reaches max_iterations. Use setMaxIterations() to change.</li>
         *  <li>All edges in the SLAM graph have converged.</li>
         * </ul>
         * An edge in the SLAM graph is considered converged for either of the following criteria:
         * <ul>
         *  <li>Its relative pose distance is below convergence_distance and its relative pose angle is below convergence_angle. Use setConvergenceDistance() and setConvergenceAngle() to change.</li>
         *  <li>Results reached the accuracy limitations of LUM's modeling.</li>
         *  <li>Somewhere during computation NaN or Inf results were produced.</li>
         * </ul>
         * Computation will change the pose estimates for the vertices of the SLAM graph, not the point clouds attached to them.
         * The results can be retrieved with getPose(), getTransformation(), getTransformedCloud() or getConcatenatedCloud().
         */
        void
        compute ();

        /** \brief Return a point cloud from one of the SLAM graph's vertices compounded onto its current pose estimate.
         * \param[in] vertex the vertex descriptor of which to return the transformed point cloud
         * \return the transformed point cloud of that vertex
         */
        PointCloudPtr
        getTransformedCloud (Vertex vertex);

        /** \brief Return a concatenated point cloud of all the SLAM graph's point clouds compounded onto their current pose estimates.
         * \return the concatenated transformed point clouds of the entire SLAM graph
         */
        PointCloudPtr
        getConcatenatedCloud ();

      protected:
        // Linearized computation of C^-1 and C^-1*D and convergence checking for an edge in the graph (results stored in slam_graph_)
        bool
        computeEdge (Edge e);

        // Returns a point compounded onto a pose using a linearized 6DoF compound
        inline Eigen::Vector3f
        linearizedCompound (Vector6f pose, Eigen::Vector3f point);

        // Returns a pose corrected 6DoF incidence matrix
        inline Matrix6f
        incidenceCorrection (Vector6f pose);

        // Could not find this in current trunk, used to be in transform.h
        inline Eigen::Affine3f
        poseToTransform (Vector6f pose);

      private:
        SLAMGraphPtr slam_graph_;
        size_t max_iterations_;
        float convergence_distance_;
        float convergence_angle_;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
  }
}

#include "lum.hpp"

#endif // PCL_LUM_H_
