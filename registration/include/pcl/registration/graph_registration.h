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

#include <pcl/registration/graph_handler.h>
#include <pcl/point_cloud.h>

namespace pcl {
/** \brief @b GraphRegistration class is the base class for graph-based registration
 * methods \author Nicola Fioraio \ingroup registration
 */
template <typename GraphT>
class GraphRegistration {
public:
  using GraphHandler = pcl::registration::GraphHandler<GraphT>;
  using GraphHandlerPtr = typename pcl::registration::GraphHandler<GraphT>::Ptr;
  using GraphHandlerConstPtr =
      typename pcl::registration::GraphHandler<GraphT>::ConstPtr;
  using GraphHandlerVertex = typename pcl::registration::GraphHandler<GraphT>::Vertex;

  /** \brief Empty constructor */
  GraphRegistration()
  : graph_handler_(new GraphHandler)
  , last_aligned_vertex_(boost::graph_traits<GraphT>::null_vertex())
  , last_vertices_()
  {}

  /** \brief Empty destructor */
  virtual ~GraphRegistration() = default;

  /** \brief Add a point cloud and the associated camera pose to the graph */
  template <typename PointT>
  inline void
  addPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                const Eigen::Matrix4f& pose)
  {
    last_vertices_.push_back(graph_handler_->addPointCloud(cloud, pose));
  }

  /** \brief Set the graph handler */
  inline void
  setGraphHandler(GraphHandlerPtr& gh)
  {
    graph_handler_ = gh;
  }

  /** \brief Get a pointer to the graph handler */
  inline GraphHandlerPtr
  getGraphHandler()
  {
    return graph_handler_;
  }

  /** \brief Get a pointer to the graph handler */
  inline GraphHandlerConstPtr
  getGraphHandler() const
  {
    return graph_handler_;
  }

  /** \brief Check if new poses have been added, then call the registration
   * method which is implemented by the subclasses
   */
  inline void
  compute()
  {
    if (last_vertices_.empty())
      return;
    computeRegistration();
    last_aligned_vertex_ = last_vertices_.back();
    last_vertices_.clear();
  }

protected:
  /** \brief The graph handler */
  GraphHandlerPtr graph_handler_;
  /** \brief The last estimated pose */
  GraphHandlerVertex last_aligned_vertex_;
  /** \brief The vertices added to the graph since the last call to compute */
  std::vector<GraphHandlerVertex> last_vertices_;

private:
  /** \brief The registration method */
  virtual void
  computeRegistration() = 0;
};
} // namespace pcl
