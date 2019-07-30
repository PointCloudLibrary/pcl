/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2017-, Open Perception, Inc.
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

#include <boost/make_shared.hpp>

#include <pcl/surface/ball_pivoting.h>
#include <pcl/surface/impl/ball_pivoting.hpp>

namespace pcl
{
  namespace ball_pivoting
  {
    BallPivotingFront::BallPivotingFront ()
    {
      clear ();
    }
    
    BallPivotingFront::~BallPivotingFront ()
    {
    }
    
    BallPivotingFront::Edge::Ptr
    BallPivotingFront::getActiveEdge ()
    {
      Edge::Ptr re;
      if (!active_edges_.empty ())
      {
        re = boost::make_shared<Edge> (active_edges_.begin ()->second);
        current_edge_ = re;
        active_edges_.erase (active_edges_.begin ());
      }
      return re;
    }
    
    void
    BallPivotingFront::addTriangle (const pcl::Vertices &seed, 
                                    const Eigen::Vector3f &center,
                                    const bool is_back_ball)
    {
      for (size_t idv = 0; idv < 3; ++idv)
      {
        addEdge (Edge (seed.vertices.at (idv), seed.vertices.at ((idv + 2) % 3), 
                       seed.vertices.at ((idv + 1) % 3), center, is_back_ball));
      }
    }
    
    void
    BallPivotingFront::addPoint (const Edge &last_edge, const uint32_t id_vertice_extended, 
                                 const Eigen::Vector3f &center, const bool is_back_ball)
    {
      addEdge (Edge (last_edge.id_point_start_, id_vertice_extended,
                     last_edge.id_point_end_, center, is_back_ball));
    
      addEdge (Edge (id_vertice_extended, last_edge.id_point_end_,
                     last_edge.id_point_start_, center, is_back_ball));
    }
    
    void
    BallPivotingFront::addEdge (const Edge &edge)
    {
      if (!isEdgeOnFront (edge) && !isEdgeFinished (edge))
      {
        active_edges_.insert (std::pair<Signature, const Edge> (
              edge.getSignature (), edge));
      }
    }
    
    bool
    BallPivotingFront::isEdgeOnFront (const Edge &edge) const
    {
      return active_edges_.find (edge.getSignature ()) != active_edges_.end () ||
             active_edges_.find (edge.getSignatureReverse ()) != active_edges_.end ();
    }
    
    void
    BallPivotingFront::removeEdge (const uint32_t id0, const uint32_t id1)
    {
      if (!active_edges_.empty ())
      {
        typename std::map<Signature, const Edge>::iterator loc = 
          active_edges_.find (Signature (id0, id1));
        if (loc != active_edges_.end ())
        {
          active_edges_.erase (loc);
        }
        else // comment to delete one-directionally
        {
          loc = active_edges_.find (Signature (id1, id0));
          if (loc != active_edges_.end ())
          {
            active_edges_.erase (loc);
          }
        }
      }
    }
    
    void
    BallPivotingFront::clear ()
    {
      active_edges_.clear ();
      finished_signatures_.clear ();
      current_edge_.reset ();
    }
    
    bool
    BallPivotingFront::isEdgeFinished (const Edge &edge) const
    {
      return finished_signatures_.find (edge.getSignature ()) != finished_signatures_.end () ||
             finished_signatures_.find (edge.getSignatureReverse ()) != finished_signatures_.end ();
    }
  } // namespace pcl::bpa
} // namespace pcl

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
// Instantiations of specific point types
PCL_INSTANTIATE(BallPivoting, (pcl::PointNormal)(pcl::PointXYZRGBNormal)(pcl::PointXYZINormal))
#endif    // PCL_NO_PRECOMPILE
