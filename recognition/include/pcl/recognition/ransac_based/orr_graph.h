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

#pragma once

#include <algorithm>
#include <cstddef>
#include <list>
#include <set>
#include <vector>

namespace pcl
{
  namespace recognition
  {
    template<class NodeData>
    class ORRGraph
    {
      public:
        class Node
        {
          public:
            enum State {ON, OFF, UNDEF};

            Node (int id)
            : id_ (id),
              state_(UNDEF)
            {}

            virtual ~Node (){}

            inline const std::set<Node*>&
            getNeighbors () const
            {
              return (neighbors_);
            }

            inline const NodeData&
            getData () const
            {
              return (data_);
            }

            inline void
            setData (const NodeData& data)
            {
              data_ = data;
            }

            inline int
            getId () const
            {
              return (id_);
            }

            inline void
            setId (int id)
            {
              id_ = id;
            }

            inline void
            setFitness (int fitness)
            {
              fitness_ = fitness;
            }

            static inline bool
            compare (const Node* a, const Node* b)
            {
              return a->fitness_ > b->fitness_;
            }

            friend class ORRGraph;

          protected:
            std::set<Node*> neighbors_;
            NodeData data_;
            int id_;
            int fitness_;
            State state_;
        };

      public:
        ORRGraph (){}
        virtual ~ORRGraph (){ this->clear ();}

        inline void
        clear ()
        {
          for ( typename std::vector<Node*>::iterator nit = nodes_.begin () ; nit != nodes_.end () ; ++nit )
            delete *nit;

          nodes_.clear ();
        }

        /** \brief Drops all existing graph nodes and creates 'n' new ones. */
        inline void
        resize (int n)
        {
          if ( !n )
            return;

          for ( typename std::vector<Node*>::iterator nit = nodes_.begin () ; nit != nodes_.end () ; ++nit )
            delete *nit;

          nodes_.resize (static_cast<std::size_t> (n));

          for ( int i = 0 ; i < n ; ++i )
            nodes_[i] = new Node (i);
        }

        inline void
        computeMaximalOnOffPartition (std::list<Node*>& on_nodes, std::list<Node*>& off_nodes)
        {
          std::vector<Node*> sorted_nodes (nodes_.size ());
          int i = 0;

          // Set all nodes to undefined
          for ( typename std::vector<Node*>::iterator it = nodes_.begin () ; it != nodes_.end () ; ++it )
          {
            sorted_nodes[i++] = *it;
            (*it)->state_ = Node::UNDEF;
          }

          // Now sort the nodes according to the fitness
          std::sort (sorted_nodes.begin (), sorted_nodes.end (), Node::compare);

          // Now run through the array and start switching nodes on and off
          for ( typename std::vector<Node*>::iterator it = sorted_nodes.begin () ; it != sorted_nodes.end () ; ++it )
          {
            // Ignore graph nodes which are already OFF
            if ( (*it)->state_ == Node::OFF )
              continue;

            // Set the node to ON
            (*it)->state_ = Node::ON;

            // Set all its neighbors to OFF
            for ( typename std::set<Node*>::iterator neigh = (*it)->neighbors_.begin () ; neigh != (*it)->neighbors_.end () ; ++neigh )
            {
              (*neigh)->state_ = Node::OFF;
              off_nodes.push_back (*neigh);
            }

            // Output the node
            on_nodes.push_back (*it);
          }
        }

        inline void
        insertUndirectedEdge (int id1, int id2)
        {
          nodes_[id1]->neighbors_.insert (nodes_[id2]);
          nodes_[id2]->neighbors_.insert (nodes_[id1]);
        }

        inline void
        insertDirectedEdge (int id1, int id2)
        {
          nodes_[id1]->neighbors_.insert (nodes_[id2]);
        }

        inline void
        deleteUndirectedEdge (int id1, int id2)
        {
          nodes_[id1]->neighbors_.erase (nodes_[id2]);
          nodes_[id2]->neighbors_.erase (nodes_[id1]);
        }

        inline void
        deleteDirectedEdge (int id1, int id2)
        {
          nodes_[id1]->neighbors_.erase (nodes_[id2]);
        }

        inline typename std::vector<Node*>&
        getNodes (){ return nodes_;}

      public:
        typename std::vector<Node*> nodes_;
    };
  } // namespace recognition
} // namespace pcl
