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
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_IMPL_PAIRWISE_GRAPH_REGISTRATION_HPP_
#define PCL_REGISTRATION_IMPL_PAIRWISE_GRAPH_REGISTRATION_HPP_

namespace pcl {

template <typename GraphT, typename PointT>
void
PairwiseGraphRegistration<GraphT, PointT>::computeRegistration()
{
  if (!registration_method_) {
    PCL_ERROR("[pcl::PairwiseGraphRegistration::computeRegistration] No registration "
              "method set!\n");
    return;
  }

  typename std::vector<GraphHandlerVertex>::iterator last_vx_it =
      last_vertices_.begin();
  if (last_aligned_vertex_ == boost::graph_traits<GraphT>::null_vertex()) {
    last_aligned_vertex_ = *last_vx_it;
    ++last_vx_it;
  }

  pcl::PointCloud<PointT> fake_cloud;
  registration_method_->setInputTarget(
      boost::get_cloud<PointT>(last_aligned_vertex_, *(graph_handler_->getGraph())));
  for (; last_vx_it < last_vertices_.end(); ++last_vx_it) {
    registration_method_->setInputCloud(
        boost::get_cloud<PointT>(*last_vx_it, *(graph_handler_->getGraph())));

    const Eigen::Matrix4f last_aligned_vertex_pose =
        boost::get_pose(last_aligned_vertex_, *(graph_handler_->getGraph()));
    if (!incremental_) {
      const Eigen::Matrix4f guess =
          last_aligned_vertex_pose.transpose() *
          boost::get_pose(*last_vx_it, *(graph_handler_->getGraph()));
      registration_method_->align(fake_cloud, guess);
    }
    else
      registration_method_->align(fake_cloud);

    const Eigen::Matrix4f global_ref_final_tr =
        last_aligned_vertex_pose * registration_method_->getFinalTransformation();
    boost::set_estimate<PointT>(
        *last_vx_it, global_ref_final_tr, *(graph_handler_->getGraph()));
    last_aligned_vertex_ = *last_vx_it;
    registration_method_->setInputTarget(
        boost::get_cloud<PointT>(last_aligned_vertex_, *(graph_handler_->getGraph())));
  }
}

} // namespace pcl

#endif // PCL_REGISTRATION_IMPL_PAIRWISE_GRAPH_REGISTRATION_HPP_
