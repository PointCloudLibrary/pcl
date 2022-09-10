/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#pragma once

#include <pcl/registration/boost_graph.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl {
namespace registration {
/** \brief @b ELCH (Explicit Loop Closing Heuristic) class
 * \author Jochen Sprickerhof
 * \ingroup registration
 */
template <typename PointT>
class ELCH : public PCLBase<PointT> {
public:
  using Ptr = shared_ptr<ELCH<PointT>>;
  using ConstPtr = shared_ptr<const ELCH<PointT>>;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  struct Vertex {
    Vertex() : cloud() {}
    PointCloudPtr cloud;
    Eigen::Affine3f transform;
  };

  /** \brief graph structure to hold the SLAM graph */
  using LoopGraph = boost::adjacency_list<boost::listS,
                                          boost::eigen_vecS,
                                          boost::undirectedS,
                                          Vertex,
                                          boost::no_property>;

  using LoopGraphPtr = shared_ptr<LoopGraph>;

  using Registration = pcl::Registration<PointT, PointT>;
  using RegistrationPtr = typename Registration::Ptr;
  using RegistrationConstPtr = typename Registration::ConstPtr;

  /** \brief Empty constructor. */
  ELCH()
  : loop_graph_(new LoopGraph)
  , loop_start_(0)
  , loop_end_(0)
  , reg_(new pcl::IterativeClosestPoint<PointT, PointT>)
  , compute_loop_(true)
  , vd_(){};

  /** \brief Empty destructor */
  ~ELCH() override = default;

  /** \brief Add a new point cloud to the internal graph.
   * \param[in] cloud the new point cloud
   */
  inline void
  addPointCloud(PointCloudPtr cloud)
  {
    typename boost::graph_traits<LoopGraph>::vertex_descriptor vd =
        add_vertex(*loop_graph_);
    (*loop_graph_)[vd].cloud = cloud;
    if (num_vertices(*loop_graph_) > 1)
      add_edge(vd_, vd, *loop_graph_);
    vd_ = vd;
  }

  /** \brief Getter for the internal graph. */
  inline LoopGraphPtr
  getLoopGraph()
  {
    return (loop_graph_);
  }

  /** \brief Setter for a new internal graph.
   * \param[in] loop_graph the new graph
   */
  inline void
  setLoopGraph(LoopGraphPtr loop_graph)
  {
    loop_graph_ = loop_graph;
  }

  /** \brief Getter for the first scan of a loop. */
  inline typename boost::graph_traits<LoopGraph>::vertex_descriptor
  getLoopStart()
  {
    return (loop_start_);
  }

  /** \brief Setter for the first scan of a loop.
   * \param[in] loop_start the scan that starts the loop
   */
  inline void
  setLoopStart(
      const typename boost::graph_traits<LoopGraph>::vertex_descriptor& loop_start)
  {
    loop_start_ = loop_start;
  }

  /** \brief Getter for the last scan of a loop. */
  inline typename boost::graph_traits<LoopGraph>::vertex_descriptor
  getLoopEnd()
  {
    return (loop_end_);
  }

  /** \brief Setter for the last scan of a loop.
   * \param[in] loop_end the scan that ends the loop
   */
  inline void
  setLoopEnd(const typename boost::graph_traits<LoopGraph>::vertex_descriptor& loop_end)
  {
    loop_end_ = loop_end;
  }

  /** \brief Getter for the registration algorithm. */
  inline RegistrationPtr
  getReg()
  {
    return (reg_);
  }

  /** \brief Setter for the registration algorithm.
   * \param[in] reg the registration algorithm used to compute the transformation
   * between the start and the end of the loop
   */
  inline void
  setReg(RegistrationPtr reg)
  {
    reg_ = reg;
  }

  /** \brief Getter for the transformation between the first and the last scan. */
  inline Eigen::Matrix4f
  getLoopTransform()
  {
    return (loop_transform_);
  }

  /** \brief Setter for the transformation between the first and the last scan.
   * \param[in] loop_transform the transformation between the first and the last scan
   */
  inline void
  setLoopTransform(const Eigen::Matrix4f& loop_transform)
  {
    loop_transform_ = loop_transform;
    compute_loop_ = false;
  }

  /** \brief Computes new poses for all point clouds by closing the loop
   * between start and end point cloud. This will transform all given point
   * clouds for now!
   */
  void
  compute();

protected:
  using PCLBase<PointT>::deinitCompute;

  /** \brief This method should get called before starting the actual computation. */
  virtual bool
  initCompute();

private:
  /** \brief graph structure for the internal optimization graph */
  using LOAGraph = boost::adjacency_list<boost::listS,
                                         boost::vecS,
                                         boost::undirectedS,
                                         boost::no_property,
                                         boost::property<boost::edge_weight_t, double>>;

  /**
   * graph balancer algorithm computes the weights
   * @param[in] g the graph
   * @param[in] f index of the first node
   * @param[in] l index of the last node
   * @param[out] weights array for the weights
   */
  void
  loopOptimizerAlgorithm(LOAGraph& g, double* weights);

  /** \brief The internal loop graph. */
  LoopGraphPtr loop_graph_;

  /** \brief The first scan of the loop. */
  typename boost::graph_traits<LoopGraph>::vertex_descriptor loop_start_;

  /** \brief The last scan of the loop. */
  typename boost::graph_traits<LoopGraph>::vertex_descriptor loop_end_;

  /** \brief The registration object used to close the loop. */
  RegistrationPtr reg_;

  /** \brief The transformation between that start and end of the loop. */
  Eigen::Matrix4f loop_transform_;
  bool compute_loop_;

  /** \brief previously added node in the loop_graph_. */
  typename boost::graph_traits<LoopGraph>::vertex_descriptor vd_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace registration
} // namespace pcl

#include <pcl/registration/impl/elch.hpp>
