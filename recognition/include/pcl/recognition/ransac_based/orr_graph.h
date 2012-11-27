/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */

/*
 * orr_graph.h
 *
 *  Created on: Nov 23, 2012
 *      Author: papazov
 */

#ifndef PCL_RECOGNITION_ORR_GRAPH_H_
#define PCL_RECOGNITION_ORR_GRAPH_H_

#include <vector>

namespace pcl
{
  namespace recognition
  {
    class ORRGraph
    {
      public:
    	class Node
    	{
    	  public:
            Node (){}
            virtual ~Node (){}

    	  public:
            std::set<Node*> neighbors_;
    	};

      public:
        ORRGraph (){}
        virtual ~ORRGraph (){}

        /** \brief Drops all existing graph nodes and creates 'n' new ones. */
        inline void
        resize (int n)
        {
          for ( std::vector<Node*>::iterator nit = nodes_.begin () ; nit != nodes_.end () ; ++nit )
            delete *nit;

          nodes_.resize (static_cast<size_t> (n));

          for ( int i = 0 ; i < n ; ++i )
            nodes_[i] = new Node ();
        }

        inline void
        insertEdge (int id1, int id2)
        {
          nodes_[id1]->neighbors_.insert (nodes_[id2]);
          nodes_[id2]->neighbors_.insert (nodes_[id1]);
        }

        inline void
        deleteEdge (int id1, int id2)
        {
          nodes_[id1]->neighbors_.erase (nodes_[id2]);
          nodes_[id2]->neighbors_.erase (nodes_[id1]);
        }

        inline std::vector<Node*>&
        getNodes (){ return nodes_;}

      public:
        std::vector<Node*> nodes_;
    };
  } // namespace recognition
} // namespace pcl

#endif /* PCL_RECOGNITION_ORR_GRAPH_H_ */
