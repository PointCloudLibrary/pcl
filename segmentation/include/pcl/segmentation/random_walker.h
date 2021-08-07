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

#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/concept/assert.hpp>

#include <Eigen/Core> // for Matrix

namespace pcl
{

  namespace segmentation
  {

    /** \brief Multilabel graph segmentation using random walks.
      *
      * This is an implementation of the algorithm described in "Random Walks
      * for Image Segmentation" by Leo Grady.
      *
      * Given a weighted undirected graph and a small number of user-defined
      * labels this algorithm analytically determines the probability that a
      * random walker starting at each unlabeled vertex will first reach one
      * of the prelabeled vertices. The unlabeled vertices are then assigned
      * to the label for which the greatest probability is calculated.
      *
      * The input is a BGL graph, a property map that associates a weight to
      * each edge of the graph, and a property map that contains initial
      * vertex colors (the term "color" is used interchangeably with "label").
      *
      * \note The colors of unlabeled vertices should be set to 0, the colors
      * of labeled vetices could be any positive numbers.
      *
      * \note This is the responsibility of the user to make sure that every
      * connected component of the graph has at least one colored vertex. If
      * the user failed to do so, then the behavior of the algorithm is
      * undefined, i.e. it may or may not succeed, and also may or may not
      * report failure.
      *
      * The output of the algorithm (i.e. label assignment) is written back
      * to the color map.
      *
      * \param[in] graph an undirected graph with internal edge weight and
      *            vertex color property maps
      *
      * Several overloads of randomWalker() function are provided for
      * convenience.
      *
      * \sa randomWalker(Graph&, EdgeWeightMap, VertexColorMap)
      * \sa randomWalker(Graph&, EdgeWeightMap, VertexColorMap, Eigen::Matrix <typename boost::property_traits<EdgeWeightMap>::value_type, Eigen::Dynamic, Eigen::Dynamic>&, std::map<typename boost::property_traits <VertexColorMap>::value_type, std::size_t>&)
      *
      * \author Sergey Alexandrov
      * \ingroup segmentation
      */

    template <class Graph> bool
    randomWalker (Graph& graph);

    /** \brief Multilabel graph segmentation using random walks.
      *
      * This is an overloaded function provided for convenience. See the
      * documentation for randomWalker().
      *
      * \param[in]      graph an undirected graph
      * \param[in]      weights an external edge weight property map
      * \param[in,out]  colors an external vertex color property map
      *
      * \author Sergey Alexandrov
      * \ingroup segmentation
      */
    template <class Graph, class EdgeWeightMap, class VertexColorMap> bool
    randomWalker (Graph& graph,
                  EdgeWeightMap weights,
                  VertexColorMap colors);

    /** \brief Multilabel graph segmentation using random walks.
      *
      * This is an overloaded function provided for convenience. See the
      * documentation for randomWalker().
      *
      * \param[in]      graph an undirected graph
      * \param[in]      weights an external edge weight property map
      * \param[in,out]  colors an external vertex color property map
      * \param[out]     potentials a matrix with calculated probabilities,
      *                 where rows correspond to vertices, and columns
      *                 correspond to colors
      * \param[out]     colors_to_columns_map a mapping between colors and
      *                 columns in \a potentials matrix
      *
      * \author Sergey Alexandrov
      * \ingroup segmentation
      */
    template <class Graph, class EdgeWeightMap, class VertexColorMap> bool
    randomWalker (Graph& graph,
                  EdgeWeightMap weights,
                  VertexColorMap colors,
                  Eigen::Matrix<typename boost::property_traits<EdgeWeightMap>::value_type, Eigen::Dynamic, Eigen::Dynamic>& potentials,
                  std::map<typename boost::property_traits<VertexColorMap>::value_type, std::size_t>& colors_to_columns_map);

  }

}

#include <pcl/segmentation/impl/random_walker.hpp>
