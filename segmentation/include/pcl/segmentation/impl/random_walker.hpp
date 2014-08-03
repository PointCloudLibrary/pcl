/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#ifndef PCL_SEGMENTATION_IMPL_RANDOM_WALKER_HPP
#define PCL_SEGMENTATION_IMPL_RANDOM_WALKER_HPP

#include <boost/bimap.hpp>

#include <Eigen/Sparse>

namespace pcl
{

  namespace segmentation
  {

    namespace detail
    {

      /** \brief Multilabel graph segmentation using random walks.
        *
        * This is an implementation of the algorithm described in "Random Walks
        * for Image Segmentation" by Leo Grady.
        *
        * See the documentation of the randomWalker() function for details.
        *
        * \author Sergey Alexandrov
        * \ingroup segmentation
        */
      template <class Graph, class EdgeWeightMap, class VertexColorMap>
      class RandomWalker
      {

        public:

          typedef typename boost::property_traits<VertexColorMap>::value_type Color;
          typedef typename boost::property_traits<EdgeWeightMap>::value_type Weight;
          typedef boost::graph_traits<Graph> GraphTraits;
          typedef typename GraphTraits::edge_descriptor EdgeDescriptor;
          typedef typename GraphTraits::vertex_descriptor VertexDescriptor;
          typedef typename GraphTraits::edge_iterator EdgeIterator;
          typedef typename GraphTraits::out_edge_iterator OutEdgeIterator;
          typedef typename GraphTraits::vertex_iterator VertexIterator;
          typedef typename boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;
          typedef boost::iterator_property_map<typename std::vector<Weight>::iterator, VertexIndexMap> VertexDegreeMap;
          typedef Eigen::SparseMatrix<Weight> SparseMatrix;
          typedef Eigen::Matrix<Weight, Eigen::Dynamic, Eigen::Dynamic> Matrix;
          typedef Eigen::Matrix<Weight, Eigen::Dynamic, 1> Vector;

          RandomWalker (Graph& g, EdgeWeightMap weights, VertexColorMap colors)
          : g_ (g)
          , weight_map_ (weights)
          , color_map_ (colors)
          , index_map_ (boost::get (boost::vertex_index, g_))
          , degree_storage_ (boost::num_vertices (g_), 0)
          , degree_map_ (boost::make_iterator_property_map (degree_storage_.begin (), index_map_))
          {
          }

          bool
          segment ()
          {
            computeVertexDegrees ();
            buildLinearSystem ();
            return solveLinearSystem ();
          }

          void
          computeVertexDegrees ()
          {
            using namespace boost;
            EdgeIterator ei, e_end;
            for (tie (ei, e_end) = edges (g_); ei != e_end; ++ei)
            {
              Weight w = weight_map_[*ei];
              degree_map_[source (*ei, g_)] += w;
              degree_map_[target (*ei, g_)] += w;
            }
          }

          void
          buildLinearSystem ()
          {
            using namespace boost;

            typedef Eigen::Triplet<float> T;
            typedef std::vector<T> Triplets;
            Triplets L_triplets;
            Triplets B_triplets;

            VertexIterator vi, v_end;
            for (tie (vi, v_end) = vertices (g_); vi != v_end; ++vi)
            {
              // If this is a labeled vertex add it to the seeds list and register its color
              if (color_map_[*vi])
              {
                seeds_.push_back (*vi);
                colors_.insert (color_map_[*vi]);
              }
              // Skip seeds and vertices with zero connectivity
              if (color_map_[*vi] || std::fabs (degree_map_[*vi]) < std::numeric_limits<Weight>::epsilon ())
                continue;
              // Create a row in L matrix for the vertex
              size_t current_row = insertInBimap (L_vertex_bimap, *vi);
              // Add diagonal degree entry for the vertex
              L_triplets.push_back (T (current_row, current_row, degree_map_[*vi]));
              // Iterate over incident vertices and add entries on corresponding columns of L or B
              OutEdgeIterator ei, e_end;
              for (tie (ei, e_end) = out_edges (*vi, g_); ei != e_end; ++ei)
              {
                Weight w = weight_map_[*ei];
                VertexDescriptor tgt = target (*ei, g_);
                Color color = color_map_[tgt];
                if (color)
                {
                  // This is a seed and will go to B matrix
                  size_t column;
                  if (B_color_bimap.right.count (color) == 0)
                  {
                    // This is the first time we encountered this color, create a new column in B
                    column = insertInBimap (B_color_bimap, color);
                  }
                  else
                  {
                    column = B_color_bimap.right.at (color);
                  }
                  B_triplets.push_back (T (current_row, column, w));
                }
                else
                {
                  // This is a non-seed and will go to L matrix,
                  // but only if a row for this vertex already exists
                  if (L_vertex_bimap.right.count (tgt) && L_vertex_bimap.right.at (tgt) != current_row)
                  {
                    L_triplets.push_back (T (current_row, L_vertex_bimap.right.at (tgt), -w));
                  }
                }
              }
            }

            size_t num_equations = L_vertex_bimap.size ();
            size_t num_colors = B_color_bimap.size ();
            L.resize (num_equations, num_equations);
            B.resize (num_equations, num_colors);
            if (L_triplets.size ())
              L.setFromTriplets(L_triplets.begin(), L_triplets.end());
            if (B_triplets.size ())
              B.setFromTriplets(B_triplets.begin(), B_triplets.end());
          }

          bool solveLinearSystem()
          {
            X.resize (L.rows (), B.cols ());

            // Nothing to solve
            if (L.rows () == 0 || B.cols () == 0)
              return true;

            Eigen::SimplicialCholesky<SparseMatrix, Eigen::Lower> cg;
            cg.compute (L);
            bool succeeded = true;
            for (int i = 0; i < B.cols (); ++i)
            {
              Vector b = B.col (i);
              X.col (i) = cg.solve (b);
              if (cg.info () != Eigen::Success)
                succeeded = false;
            }

            assignColors ();
            return succeeded;
          }

          void
          assignColors ()
          {
            using namespace boost;
            if (X.cols ())
              for (int i = 0; i < X.rows (); ++i)
              {
                size_t max_column;
                X.row (i).maxCoeff (&max_column);
                VertexDescriptor vertex = L_vertex_bimap.left.at (i);
                Color color = B_color_bimap.left.at (max_column);
                color_map_[vertex] = color;
              }
          }

          void
          getPotentials (Matrix& potentials, std::map<Color, size_t>& color_to_column_map)
          {
            using namespace boost;
            potentials = Matrix::Zero (num_vertices (g_), colors_.size ());
            // Copy over rows from X
            for (int i = 0; i < X.rows (); ++i)
              potentials.row (L_vertex_bimap.left.at (i)).head (X.cols ()) = X.row (i);
            // In rows that correspond to seeds put ones in proper columns
            for (size_t i = 0; i < seeds_.size (); ++i)
            {
              VertexDescriptor v = seeds_[i];
              insertInBimap (B_color_bimap, color_map_[v]);
              potentials (seeds_[i], B_color_bimap.right.at (color_map_[seeds_[i]])) = 1;
            }
            // Fill in a map that associates colors with columns in potentials matrix
            color_to_column_map.clear ();
            for (int i = 0; i < potentials.cols (); ++i)
              color_to_column_map[B_color_bimap.left.at (i)] = i;
          }

          template <typename T> static inline size_t
          insertInBimap (boost::bimap<size_t, T>& bimap, T value)
          {
            if (bimap.right.count (value) != 0)
            {
              return bimap.right.at (value);
            }
            else
            {
              size_t s = bimap.size ();
              bimap.insert (typename boost::bimap<size_t, T>::value_type (s, value));
              return s;
            }
          }

          Graph& g_;
          EdgeWeightMap weight_map_;
          VertexColorMap color_map_;
          VertexIndexMap index_map_;

          std::vector<VertexDescriptor> seeds_;
          std::set<Color> colors_;

          std::vector<Weight> degree_storage_;
          VertexDegreeMap degree_map_;
          SparseMatrix L;
          SparseMatrix B;
          Matrix X;

          // Map vertex identifiers to the rows/columns of L and vice versa
          boost::bimap<size_t, VertexDescriptor> L_vertex_bimap;
          // Map colors to the columns of B and vice versa
          boost::bimap<size_t, Color> B_color_bimap;

      };

    }

    template <class Graph> bool
    randomWalker (Graph& graph)
    {
      return randomWalker (graph,
                           boost::get (boost::edge_weight, graph),
                           boost::get (boost::vertex_color, graph));
    }

    template <class Graph, class EdgeWeightMap, class VertexColorMap> bool
    randomWalker (Graph& graph,
                  EdgeWeightMap weights,
                  VertexColorMap colors)
    {
      using namespace boost;

      typedef typename graph_traits<Graph>::edge_descriptor EdgeDescriptor;
      typedef typename graph_traits<Graph>::vertex_descriptor VertexDescriptor;

      BOOST_CONCEPT_ASSERT ((VertexListGraphConcept<Graph>));                                 // to have vertices(), num_vertices()
      BOOST_CONCEPT_ASSERT ((EdgeListGraphConcept<Graph>));                                   // to have edges()
      BOOST_CONCEPT_ASSERT ((IncidenceGraphConcept<Graph>));                                  // to have source(), target() and out_edges()
      BOOST_CONCEPT_ASSERT ((ReadablePropertyMapConcept<EdgeWeightMap, EdgeDescriptor>));     // read weight-values from edges
      BOOST_CONCEPT_ASSERT ((ReadWritePropertyMapConcept<VertexColorMap, VertexDescriptor>)); // read and write color-values from vertices

      ::pcl::segmentation::detail::RandomWalker
      <
        Graph,
        EdgeWeightMap,
        VertexColorMap
      >
      rw (graph, weights, colors);
      return rw.segment ();
    }

    template <class Graph, class EdgeWeightMap, class VertexColorMap> bool
    randomWalker (Graph& graph,
                  EdgeWeightMap weights,
                  VertexColorMap colors,
                  Eigen::Matrix<typename boost::property_traits<EdgeWeightMap>::value_type, Eigen::Dynamic, Eigen::Dynamic>& potentials,
                  std::map<typename boost::property_traits<VertexColorMap>::value_type, size_t>& colors_to_columns_map)
    {
      using namespace boost;

      typedef typename graph_traits<Graph>::edge_descriptor EdgeDescriptor;
      typedef typename graph_traits<Graph>::vertex_descriptor VertexDescriptor;

      BOOST_CONCEPT_ASSERT ((VertexListGraphConcept<Graph>));                                 // to have vertices(), num_vertices()
      BOOST_CONCEPT_ASSERT ((EdgeListGraphConcept<Graph>));                                   // to have edges()
      BOOST_CONCEPT_ASSERT ((IncidenceGraphConcept<Graph>));                                  // to have source(), target() and out_edges()
      BOOST_CONCEPT_ASSERT ((ReadablePropertyMapConcept<EdgeWeightMap, EdgeDescriptor>));     // read weight-values from edges
      BOOST_CONCEPT_ASSERT ((ReadWritePropertyMapConcept<VertexColorMap, VertexDescriptor>)); // read and write color-values from vertices

      ::pcl::segmentation::detail::RandomWalker
      <
        Graph,
        EdgeWeightMap,
        VertexColorMap
      >
      rw (graph, weights, colors);
      bool result = rw.segment ();
      rw.getPotentials (potentials, colors_to_columns_map);
      return result;
    }

  }

}

#endif /* PCL_SEGMENTATION_IMPL_RANDOM_WALKER_HPP */

