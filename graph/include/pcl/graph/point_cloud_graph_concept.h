/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#ifndef PCL_GRAPH_POINT_CLOUD_GRAPH_CONCEPT_H
#define PCL_GRAPH_POINT_CLOUD_GRAPH_CONCEPT_H

#include <boost/graph/graph_concepts.hpp>
#include <boost/concept/detail/concept_def.hpp>

namespace pcl
{

  namespace graph
  {

    namespace concepts
    {

      /** \class pcl::graph::concepts::PointCloudGraphConcept
        *
        * A PointCloudGraph is a graph that has PCL points bundled in
        * vertices and can be viewed as a PCL point cloud (without data-copying).
        *
        * This concept is a refinement of [PropertyGraph]. Please refer to
        * [BGL concepts][GraphConcepts] for a complete list of graph-related
        * concepts in boost.
        *
        * [PropertyGraph]: http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/PropertyGraph.html
        * [GraphConcepts]: http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/graph_concepts.html
        *
        *
        * ## Notation ##
        *
        *
        * - `G` a type that is a model of PointCloudGraph
        * - `g` an object of type `G`
        *
        *
        * ## Associated types ##
        *
        *
        * - `pcl::graph::point_cloud_graph_traits<G>::point_type`
        *
        *   The type of PCL points bundled in vertices.
        *
        * - `pcl::graph::point_cloud_graph_traits<G>::point_cloud_type`
        *
        *   The type of PCL point cloud this graph can be viewed as.
        *
        * - `pcl::graph::point_cloud_graph_traits<G>::point_cloud_ptr`
        *
        *   The type of a shared pointer to PCL point cloud this graph can be
        *   viewed as.
        *
        * - `pcl::graph::point_cloud_graph_traits<G>::point_cloud_const_ptr`
        *
        *   The type of a shared pointer to const PCL point cloud this graph can
        *   be viewed as.
        *
        *
        * ## Valid expressions ##
        *
        * - `G (point_cloud_ptr)`
        * - \ref pcl::graph::point_cloud() "pcl::graph::point_cloud (g)"
        * - \ref pcl::graph::indices() "pcl::graph::indices (g)"
        *
        *
        * ## Models ##
        *
        *
        * - `pcl::graph::point_cloud_graph`
        * - `boost::subgraph<pcl::graph::point_cloud_graph>`
        *
        * \author Sergey Alexandrov
        * \ingroup graph
        */

      BOOST_concept (PointCloudGraph, (G))
      : boost::concepts::Graph<G>
      {

        typedef typename boost::vertex_bundle_type<G>::type vertex_bundled;
        typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
        typedef typename point_cloud_graph_traits<G>::point_type point_type;
        typedef typename point_cloud_graph_traits<G>::point_cloud_type point_cloud_type;
        typedef typename point_cloud_graph_traits<G>::point_cloud_ptr point_cloud_ptr;
        typedef typename point_cloud_graph_traits<G>::point_cloud_const_ptr point_cloud_const_ptr;

        BOOST_STATIC_ASSERT ((boost::mpl::not_<boost::is_same<vertex_bundled, boost::no_property> >::value));
        BOOST_STATIC_ASSERT ((boost::is_same<vertex_bundled, point_type>::value));

        BOOST_CONCEPT_USAGE (PointCloudGraph)
        {
          BOOST_CONCEPT_ASSERT ((boost::concepts::PropertyGraph<G, vertex_descriptor, boost::vertex_bundle_t>));
          p = point_cloud (g);
          i = indices (g);
          G a (p); // require that graph can be constructed from a point cloud pointer
          const_constraints (g);
        }

        void const_constraints (const G& cg)
        {
          pc = point_cloud (cg);
          i = indices (cg);
        }

        G g;
        point_cloud_ptr p;
        point_cloud_const_ptr pc;
        pcl::PointIndices::Ptr i;

      };

    } // namespace concepts

    using pcl::graph::concepts::PointCloudGraphConcept;

  } // namespace graph

} // namespace pcl

#include <boost/concept/detail/concept_undef.hpp>

#endif /* PCL_GRAPH_POINT_CLOUD_GRAPH_CONCEPT_H */

