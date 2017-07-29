/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */

#include <boost/make_shared.hpp>

#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/surface/ball_pivoting.h>
#include <pcl/surface/impl/ball_pivoting.hpp>

namespace pcl
{
  namespace bpa
  {
    Front::Edge::Edge ()
    {
    }
    
    Front::Edge::Edge (const uint32_t id0, const uint32_t id1)
    {
      id_vertices_.resize (2);
      id_vertices_.at (0) = id0;
      id_vertices_.at (1) = id1;
    }
    
    Front::Edge::Edge (const std::vector<uint32_t> &edge, const uint32_t id_opposite, 
                       const Eigen::Vector3f &center, const bool is_back_ball):
      id_vertices_ (edge), 
      id_opposite_ (id_opposite), 
      center_ (center), 
      is_back_ball_ (is_back_ball)
    {
    }
    
    Front::Edge::~Edge ()
    {
    }
    
    Front::Front::Front ()
    {
      clear ();
    }
    
    Front::Front::~Front ()
    {
    }
    
    Front::Edge::Ptr
    Front::Front::getActiveEdge ()
    {
      Edge::Ptr re;
      if (!active_edges.empty ())
      {
        re = boost::make_shared<Edge> (active_edges.begin ()->second);
        current_edge_ = re;
        active_edges.erase (active_edges.begin ());

        finished_signatures_.insert (re->getSignature ());
      }
      return re;
    }
    
    void
    Front::Front::addTriangle (const pcl::Vertices::ConstPtr &seed, 
                               const Eigen::Vector3f &center,
                               const bool is_back_ball)
    {
      for (size_t idv = 0; idv < 3; ++idv)
      {
        std::vector<uint32_t> edge (2, 0);
        edge.at (0) = seed->vertices.at (idv);
        edge.at (1) = seed->vertices.at ((idv + 2) % 3);
    
        addEdge (Edge (edge, seed->vertices.at ((idv + 1) % 3), center, is_back_ball));
      }
    }
    
    void
    Front::Front::addPoint (const Edge &last_edge, const uint32_t id_vertice_extended, 
                            const Eigen::Vector3f &center, const bool is_back_ball)
    {
      std::vector<uint32_t> edge (2, 0);
    
      edge.at (0) = last_edge.getIdVertice (0);
      edge.at (1) = id_vertice_extended;
      addEdge (Edge (edge, last_edge.getIdVertice (1), center, is_back_ball));
    
      edge.at (0) = id_vertice_extended;
      edge.at (1) = last_edge.getIdVertice (1);
      addEdge (Edge (edge, last_edge.getIdVertice (0), center, is_back_ball));
    }
    
    void
    Front::Front::addEdge (const Edge &edge)
    {
      if (!isEdgeOnFront (edge))
      {
        active_edges[Signature (edge.getIdVertice (0), edge.getIdVertice (1))] = edge;
      }
    }
    
    bool
    Front::Front::isEdgeOnFront (const Edge &edge) const
    {
      return active_edges.find (edge.getSignature ()) != active_edges.end () ||
             active_edges.find (edge.getSignatureReverse ()) != active_edges.end ();
    }
    
    void
    Front::Front::removeEdge (const uint32_t id0, const uint32_t id1)
    {
      if (!active_edges.empty ())
      {
        typename std::map<Signature, Edge>::iterator loc = active_edges.find (Signature (id0, id1));
        if (loc != active_edges.end ())
        {
          active_edges.erase (loc);
        }
        else // comment to delete one-directionally
        {
          loc = active_edges.find (Signature (id1, id0));
          if (loc != active_edges.end ())
          {
            active_edges.erase (loc);
          }
        }
      }
    }
    
    void
    Front::Front::clear ()
    {
      active_edges.clear ();
      finished_signatures_.clear ();
      current_edge_.reset ();
    }
    
    bool
    Front::Front::isEdgeFinished (const Edge &edge) const
    {
      return finished_signatures_.find (edge.getSignature ()) != finished_signatures_.end () ||
             finished_signatures_.find (edge.getSignatureReverse ()) != finished_signatures_.end ();
    }
  } // namespace pcl::bpa
} // namespace pcl

// Instantiations of specific point types
PCL_INSTANTIATE(BallPivoting, (pcl::PointNormal)(pcl::PointXYZRGBNormal)(pcl::PointXYZINormal))
