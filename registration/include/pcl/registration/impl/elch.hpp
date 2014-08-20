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

#ifndef PCL_REGISTRATION_IMPL_ELCH_H_
#define PCL_REGISTRATION_IMPL_ELCH_H_

#include <list>
#include <algorithm>

#include <pcl/common/transforms.h>
#include <pcl/registration/eigen.h>
#include <pcl/registration/boost.h>
#include <pcl/registration/registration.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::registration::ELCH<PointT>::loopOptimizerAlgorithm (LOAGraph &g, double *weights)
{
  std::list<int> crossings, branches;
  crossings.push_back (static_cast<int> (loop_start_));
  crossings.push_back (static_cast<int> (loop_end_));
  weights[loop_start_] = 0;
  weights[loop_end_] = 1;

  int *p = new int[num_vertices (g)];
  int *p_min = new int[num_vertices (g)];
  double *d = new double[num_vertices (g)];
  double *d_min = new double[num_vertices (g)];
  double dist;
  bool do_swap = false;
  std::list<int>::iterator crossings_it, end_it, start_min, end_min;

  // process all junctions
  while (!crossings.empty ())
  {
    dist = -1;
    // find shortest crossing for all vertices on the loop
    for (crossings_it = crossings.begin (); crossings_it != crossings.end (); )
    {
      dijkstra_shortest_paths (g, *crossings_it,
          predecessor_map(boost::make_iterator_property_map(p, get(boost::vertex_index, g))).
          distance_map(boost::make_iterator_property_map(d, get(boost::vertex_index, g))));

      end_it = crossings_it;
      end_it++;
      // find shortest crossing for one vertex
      for (; end_it != crossings.end (); end_it++)
      {
        if (*end_it != p[*end_it] && (dist < 0 || d[*end_it] < dist))
        {
          dist = d[*end_it];
          start_min = crossings_it;
          end_min = end_it;
          do_swap = true;
        }
      }
      if (do_swap)
      {
        std::swap (p, p_min);
        std::swap (d, d_min);
        do_swap = false;
      }
      // vertex starts a branch
      if (dist < 0)
      {
        branches.push_back (static_cast<int> (*crossings_it));
        crossings_it = crossings.erase (crossings_it);
      }
      else
        crossings_it++;
    }

    if (dist > -1)
    {
      remove_edge (*end_min, p_min[*end_min], g);
      for (int i = p_min[*end_min]; i != *start_min; i = p_min[i])
      {
        //even right with weights[*start_min] > weights[*end_min]! (math works)
        weights[i] = weights[*start_min] + (weights[*end_min] - weights[*start_min]) * d_min[i] / d_min[*end_min];
        remove_edge (i, p_min[i], g);
        if (degree (i, g) > 0)
        {
          crossings.push_back (i);
        }
      }

      if (degree (*start_min, g) == 0)
        crossings.erase (start_min);

      if (degree (*end_min, g) == 0)
        crossings.erase (end_min);
    }
  }

  delete[] p;
  delete[] p_min;
  delete[] d;
  delete[] d_min;

  boost::graph_traits<LOAGraph>::adjacency_iterator adjacent_it, adjacent_it_end;
  int s;

  // error propagation
  while (!branches.empty ())
  {
    s = branches.front ();
    branches.pop_front ();

    for (boost::tuples::tie (adjacent_it, adjacent_it_end) = adjacent_vertices (s, g); adjacent_it != adjacent_it_end; ++adjacent_it)
    {
      weights[*adjacent_it] = weights[s];
      if (degree (*adjacent_it, g) > 1)
        branches.push_back (static_cast<int> (*adjacent_it));
    }
    clear_vertex (s, g);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::registration::ELCH<PointT>::initCompute ()
{
  /*if (!PCLBase<PointT>::initCompute ())
  {
    PCL_ERROR ("[pcl::registration:ELCH::initCompute] Init failed.\n");
    return (false);
  }*/ //TODO

  if (loop_end_ == 0)
  {
    PCL_ERROR ("[pcl::registration::ELCH::initCompute] no end of loop defined!\n");
    deinitCompute ();
    return (false);
  }

  //compute transformation if it's not given
  if (compute_loop_)
  {
    PointCloudPtr meta_start (new PointCloud);
    PointCloudPtr meta_end (new PointCloud);
    *meta_start = *(*loop_graph_)[loop_start_].cloud;
    *meta_end = *(*loop_graph_)[loop_end_].cloud;

    typename boost::graph_traits<LoopGraph>::adjacency_iterator si, si_end;
    for (boost::tuples::tie (si, si_end) = adjacent_vertices (loop_start_, *loop_graph_); si != si_end; si++)
      *meta_start += *(*loop_graph_)[*si].cloud;

    for (boost::tuples::tie (si, si_end) = adjacent_vertices (loop_end_, *loop_graph_); si != si_end; si++)
      *meta_end += *(*loop_graph_)[*si].cloud;

    //TODO use real pose instead of centroid
    //Eigen::Vector4f pose_start;
    //pcl::compute3DCentroid (*(*loop_graph_)[loop_start_].cloud, pose_start);

    //Eigen::Vector4f pose_end;
    //pcl::compute3DCentroid (*(*loop_graph_)[loop_end_].cloud, pose_end);

    PointCloudPtr tmp (new PointCloud);
    //Eigen::Vector4f diff = pose_start - pose_end;
    //Eigen::Translation3f translation (diff.head (3));
    //Eigen::Affine3f trans = translation * Eigen::Quaternionf::Identity ();
    //pcl::transformPointCloud (*(*loop_graph_)[loop_end_].cloud, *tmp, trans);

    reg_->setInputTarget (meta_start);

    reg_->setInputSource (meta_end);

    reg_->align (*tmp);

    loop_transform_ = reg_->getFinalTransformation ();
    //TODO hack
    //loop_transform_ *= trans.matrix ();

  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::registration::ELCH<PointT>::compute ()
{
  if (!initCompute ())
  {
    return;
  }

  LOAGraph grb[4];

  typename boost::graph_traits<LoopGraph>::edge_iterator edge_it, edge_it_end;
  for (boost::tuples::tie (edge_it, edge_it_end) = edges (*loop_graph_); edge_it != edge_it_end; edge_it++)
  {
    for (int j = 0; j < 4; j++)
      add_edge (source (*edge_it, *loop_graph_), target (*edge_it, *loop_graph_), 1, grb[j]);  //TODO add variance
  }

  double *weights[4];
  for (int i = 0; i < 4; i++)
  {
    weights[i] = new double[num_vertices (*loop_graph_)];
    loopOptimizerAlgorithm (grb[i], weights[i]);
  }

  //TODO use pose
  //Eigen::Vector4f cend;
  //pcl::compute3DCentroid (*((*loop_graph_)[loop_end_].cloud), cend);
  //Eigen::Translation3f tend (cend.head (3));
  //Eigen::Affine3f aend (tend);
  //Eigen::Affine3f aendI = aend.inverse ();

  //TODO iterate ovr loop_graph_
  //typename boost::graph_traits<LoopGraph>::vertex_iterator vertex_it, vertex_it_end;
  //for (boost::tuples::tie (vertex_it, vertex_it_end) = vertices (*loop_graph_); vertex_it != vertex_it_end; vertex_it++)
  for (size_t i = 0; i < num_vertices (*loop_graph_); i++)
  {
    Eigen::Vector3f t2;
    t2[0] = loop_transform_ (0, 3) * static_cast<float> (weights[0][i]);
    t2[1] = loop_transform_ (1, 3) * static_cast<float> (weights[1][i]);
    t2[2] = loop_transform_ (2, 3) * static_cast<float> (weights[2][i]);

    Eigen::Affine3f bl (loop_transform_);
    Eigen::Quaternionf q (bl.rotation ());
    Eigen::Quaternionf q2;
    q2 = Eigen::Quaternionf::Identity ().slerp (static_cast<float> (weights[3][i]), q);

    //TODO use rotation from branch start
    Eigen::Translation3f t3 (t2);
    Eigen::Affine3f a (t3 * q2);
    //a = aend * a * aendI;

    pcl::transformPointCloud (*(*loop_graph_)[i].cloud, *(*loop_graph_)[i].cloud, a);
    (*loop_graph_)[i].transform = a;
  }

  add_edge (loop_start_, loop_end_, *loop_graph_);

  deinitCompute ();
}

#endif // PCL_REGISTRATION_IMPL_ELCH_H_
