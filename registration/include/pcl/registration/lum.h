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

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Geometry>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b Globally Consistent Scan Matching based on an algorithm by Lu and Milios
     * A GraphSLAM algorithm where registration data is managed in a graph:
     *  Vertices represent poses and hold the point cloud data
     *  Edges represent pose constraints and hold the correspondence data between two point clouds
     * \author Frits Florentinus & Jochen Sprickerhof
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
          CorrespondencesPtr corrs_;
          Matrix6f cinv_;
          Vector6f cinvd_;
          bool computed_;
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
            slam_graph_ (new SLAMGraph), max_iterations_ (3)
        {
        }
        ;

        /** \brief Returns the internal SLAM graph structure (based on adjacency_list).
         */
        inline SLAMGraphPtr
        getLoopGraph ()
        {
          return (slam_graph_);
        }

        /** \brief Sets a new internal SLAM graph structure.
         * \param[in] slam_graph the new SLAM graph structure (based on adjacency_list)
         */
        inline void
        setLoopGraph (SLAMGraphPtr slam_graph)
        {
          slam_graph_ = slam_graph;
        }

        /** \brief Returns the maximum number of iterations for the compute() method (default = 3).
         */
        inline size_t
        getMaxIterations ()
        {
          return (max_iterations_);
        }

        /** \brief Sets the maximum number of iterations for the compute() method (default = 3).
         * \param[in] max_iterations the new maximum number of iterations
         */
        inline void
        setMaxIterations (size_t max_iterations)
        {
          max_iterations_ = max_iterations;
        }

        /** \brief Add a new point cloud to the SLAM graph and return its vertex descriptor.
         * \param[in] cloud the new point cloud
         */
        inline Vertex
        addPointCloud (PointCloudPtr cloud);

        /** \brief Add a new point cloud to the SLAM graph with a pose estimate and return its vertex descriptor.
         * \note The pose estimate is ignored for the first cloud in the graph since that will become the reference pose.
         *       More accurate pose estimates result in faster computation times.
         * \param[in] cloud the new point cloud
         * \param[in] pose the pose estimate relative to the reference pose (first point cloud that was added)
         */
        inline Vertex
        addPointCloud (PointCloudPtr cloud, Vector6f pose);

        /** \brief Sets a new pose estimate to one of the SLAM graph's vertices.
         * \note The pose estimate is ignored for the first cloud in the graph since that is the reference pose.
         *       More accurate pose estimates result in faster computation times.
         * \param[in] vertex the vertex descriptor of which to set the pose estimate
         * \param[in] pose the new pose estimate
         */
        inline void
        setPose (Vertex vertex, Vector6f pose);

        /** \brief Returns the pose estimate from one of the SLAM graph's vertices.
         * \param[in] vertex the vertex descriptor of which to return the pose estimate
         */
        inline Vector6f
        getPose (Vertex vertex);

        /** \brief Returns the pose estimate from one of the SLAM graph's vertices as an affine transformation matrix.
         * \param[in] vertex the vertex descriptor of which to return the transformation
         */
        inline Eigen::Affine3f
        getTransformation (Vertex vertex)
        {
          return poseToTransform (getPose (vertex));
        }

        /** \brief Sets a new set of correspondences to one of the SLAM graph's edges.
         * \note The set needs to contain at least 3 different correspondences.
         * \param[in] source_vertex the vertex descriptor of the correspondences' source point cloud
         * \param[in] target_vertex the vertex descriptor of the correspondences' target point cloud
         * \param[in] corr the new set of correspondences
         */
        inline void
        setCorrespondences (Vertex source_vertex, Vertex target_vertex, CorrespondencesPtr corrs);

        /** \brief Returns the set of correspondences from one of the SLAM graph's edges.
         * \param[in] source_vertex the vertex descriptor of the correspondences' source point cloud
         * \param[in] target_vertex the vertex descriptor of the correspondences' target point cloud
         */
        inline CorrespondencesPtr
        getCorrespondences (Vertex source_vertex, Vertex target_vertex);

        /** \brief Commence LUM computation.
         * \note The internal SLAM graph needs to be fully connected and contain at least 2 vertices.
         *       Computation stops when max_iterations are met or a certain convergence is met (NYI/TODO).
         *       The final results can be retrieved with getPose(), getTransformation() or getConcatenatedCloud().
         */
        void
        compute ();

        /** \brief Returns a concatenated point cloud of all input point clouds compounded onto their current pose estimates.
         */
        PointCloudPtr
        getConcatenatedCloud ();

      protected:
        // Linearized computation of C^-1 and C^-1*D for an edge in the graph (results stored in slam_graph_)
        void
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

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
  }
}

#include "lum.hpp"

#endif // PCL_LUM_H_
