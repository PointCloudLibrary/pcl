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

#ifndef PCL_GRAPH_HANDLER_H_
#define PCL_GRAPH_HANDLER_H_

#include <pcl/registration/vertex_estimates.h>
#include <pcl/registration/edge_measurements.h>
#include <pcl/exceptions.h>
#include "boost/graph/graph_traits.hpp"

namespace pcl
{
  namespace registration
  {
    /** \brief @b GraphHandler class is a wrapper for a general SLAM graph
      * The actual graph class must fulfil the following boost::graph concepts:
      * - BidirectionalGraph
      * - AdjacencyGraph
      * - VertexAndEdgeListGraph
      * - MutableGraph
      *
      * Other valid expressions:
      * - add_edge (m,g)          add a new edge according to the given measurement. Return type: std::pair<edge_descriptor, bool>
      * - add_vertex (e,g)        add a new vertex according to the given estimate. Return type: vertex_descriptor
      * - get_pose (v,g)          retrieve the pose estimate for v, if any. Return type: Eigen::Matrix4f
      * - get_cloud (v,g)         retrieve the cloud pointer associated to v, if any. Return type: pcl::PointCloud<PointT>::ConstPtr
      * - set_estimate (v,e,g)    set the estimate for an existing vertex. Return type: void.
      * - set_measurement (d,m,g) set the measurement for an existing edge. Return type: void.
      * Notation:
      * - m                       an edge measurement
      * - e                       a vertex estimate
      * - v                       a vertex
      * - d                       an edge
      * - g                       a graph
      * A valid graph implementation should accept at least the PoseEstimate estimate and the PoseMeasurement measurement
      *
      * If a specific graph implementation needs initialization and/or finalization,
      * specialize the protected methods init() and deinit() for your graph type
      *
      * \author Nicola Fioraio
      * \ingroup registration
      */
    template <typename GraphT>
    class GraphHandler
    {
      public:
        typedef boost::shared_ptr<GraphHandler<GraphT> > Ptr;
        typedef boost::shared_ptr<const GraphHandler<GraphT> > ConstPtr;
        typedef boost::shared_ptr<GraphT> GraphPtr;
        typedef boost::shared_ptr<const GraphT> GraphConstPtr;

        typedef typename boost::graph_traits<GraphT>::vertex_descriptor Vertex;
        typedef typename boost::graph_traits<GraphT>::edge_descriptor Edge;

        /** \brief Empty constructor. */
        GraphHandler () : graph_impl_ (new GraphT ())
        {
          if (!init ())
            throw InitFailedException ("Graph Initialization Failed", __FILE__, "pcl::registration::GraphHandler::GraphHandler ()", __LINE__);
        }

        /** \brief Destructor. */
        ~GraphHandler ()
        {
          deinit ();
        }

        /** \brief Clear the graph */
        inline void
        clear ()
        {
          deinit ();
          graph_impl_.reset (new GraphT ());
          if (!init ())
            throw InitFailedException ("Graph Initialization Failed", __FILE__, "pcl::registration::GraphHandler::clear ()", __LINE__);
        }

        /** \brief Get a pointer to the BGL graph */
        inline GraphConstPtr
        getGraph () const
        {
          return graph_impl_;
        }

        /** \brief Get a pointer to the BGL graph */
        inline GraphPtr
        getGraph ()
        {
          return graph_impl_;
        }

        /** \brief Add a new point cloud to the graph and return the new vertex
          * \param cloud the new point cloud
          * \param pose the pose estimate
          * \return a reference to the new vertex
          */
        template <class PointT> inline Vertex
        addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const Eigen::Matrix4f& pose)
        {
          return add_vertex (PoseEstimate<PointT> (cloud, pose), *graph_impl_);
        }

        /** \brief Add a new generic vertex created according to the given estimate
          * \param estimate the parameters' estimate
          * \return a reference to the new vertex
          */
        template <class EstimateT> inline Vertex
        addGenericVertex (const EstimateT& estimate)
        {
          return add_vertex (estimate, *graph_impl_);
        }

        /** \brief Add a new constraint between two poses
          * \param v_start the first pose
          * \param v_end the second pose
          * \param relative_transformation the transformation from v_start to v_end
          * \param information_matrix the uncertainty
          * \return a reference to the new edge
          */
        template <class InformationT> inline Edge
        addPoseConstraint ( const Vertex& v_start, const Vertex& v_end,
                            const Eigen::Matrix4f& relative_transformation,
                            const InformationT& information_matrix)
        {
          return add_edge ( PoseMeasurement<Vertex, InformationT> (v_start, v_end, relative_transformation, information_matrix),
                            *graph_impl_);
        }

        /** \brief Add a generic constraint created according to the given measurement
          * \param measurement the measurement
          * \return a reference to the new edge
          */
        template <class MeasurementT> inline Edge
        addGenericConstraint (const MeasurementT& measurement)
        {
          return add_edge (measurement, *graph_impl_);
        }

        /** \brief Remove a vertex from the graph
          * \param v the vertex
          */
        inline void
        removeVertex (const Vertex& v)
        {
          remove_vertex (v.v_, *graph_impl_);
        }

        /** \brief Remove a constraint from the graph
          * \param e the edge
          */
        inline void
        removeConstraint (const Edge& e)
        {
          remove_edge(e.e_, *graph_impl_);
        }

      protected:
        /** \brief This method is called right after the creation of graph_impl_ */
        inline bool
        init ()
        {
          return true;
        }

        /** \brief This method is called when graph_impl_ is going to be destroyed */
        inline bool
        deinit ()
        {
          return true;
        }

      private:
        GraphPtr graph_impl_;
    };
  }
}

#endif // PCL_GRAPH_HANDLER_H_
