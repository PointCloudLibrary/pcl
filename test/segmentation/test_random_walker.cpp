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

#include <iostream>
#include <fstream>
#include <sstream>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/foreach.hpp>

#include <gtest/gtest.h>

#include <pcl/segmentation/random_walker.h>

std::string TEST_DATA_DIR;

typedef float Weight;
typedef uint32_t Color;
typedef boost::adjacency_list
        <boost::vecS,
         boost::vecS,
         boost::undirectedS,
         boost::property<boost::vertex_color_t, Color,
         boost::property<boost::vertex_degree_t, Weight> >,
         boost::property<boost::edge_weight_t, Weight> > Graph;
typedef boost::graph_traits<Graph> GraphTraits;
typedef GraphTraits::edge_descriptor EdgeDescriptor;
typedef GraphTraits::vertex_descriptor VertexDescriptor;
typedef GraphTraits::edge_iterator EdgeIterator;
typedef GraphTraits::vertex_iterator VertexIterator;
typedef boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap;
typedef boost::property_map<Graph, boost::vertex_color_t>::type VertexColorMap;
typedef Eigen::SparseMatrix<Weight> SparseMatrix;
typedef Eigen::Matrix<Weight, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Eigen::Matrix<Weight, Eigen::Dynamic, 1> Vector;
typedef boost::bimap<size_t, VertexDescriptor> VertexDescriptorBimap;
typedef boost::bimap<size_t, Color> ColorBimap;
typedef pcl::segmentation::detail::RandomWalker<Graph, EdgeWeightMap, VertexColorMap> RandomWalker;


struct GraphInfo
{

  GraphInfo (const std::string& filename)
  {
    using boost::property_tree::ptree;
    ptree pt;
    read_info (filename, pt);

    size = pt.get<size_t> ("Size");
    graph = Graph (size);
    segmentation.resize (size);
    color_map = boost::get (boost::vertex_color, graph);

    // Read graph topology
    BOOST_FOREACH (ptree::value_type& v, pt.get_child ("Topology"))
    {
      uint32_t source, target;
      float weight;
      std::stringstream (v.second.data ()) >> source >> target >> weight;
      boost::add_edge (source, target, weight, graph);
    }

    // Initialize color property map with zeros
    VertexIterator vi, v_end;
    for (boost::tie (vi, v_end) = boost::vertices (graph); vi != v_end; ++vi)
      color_map[*vi] = 0;

    // Read seeds
    BOOST_FOREACH (ptree::value_type& v, pt.get_child ("Seeds"))
    {
      uint32_t id, color;
      std::stringstream (v.second.data ()) >> id >> color;
      color_map[id] = color;
      colors.insert (color);
    }

    // Read expected cluster assignment
    std::stringstream ss (pt.get<std::string> ("Segmentation"));
    for (size_t i = 0; i < size; ++i)
      ss >> segmentation[i];

    // Read expected dimensions of matrices L and B
    std::stringstream (pt.get<std::string> ("Dimensions")) >> rows >> cols;

    // Read expected potentials
    BOOST_FOREACH (ptree::value_type& v, pt.get_child ("Potentials"))
    {
      Color color = boost::lexical_cast<uint32_t> (v.first);
      potentials[color] = Vector::Zero (size);
      std::stringstream ss (v.second.data ());
      for (size_t i = 0; i < size; ++i)
        ss >> potentials[color] (i);
    }
  }

  Graph graph;
  std::vector<Color> segmentation;
  VertexColorMap color_map;
  std::map<Color, Vector> potentials;
  std::set<Color> colors;
  size_t size; // number of vertices
  size_t rows; // expected number of rows in matrices L and B
  size_t cols; // expected number of cols in matrix B

};

class RandomWalkerTest : public ::testing::TestWithParam<const char*>
{

  public:

    RandomWalkerTest ()
    : g (TEST_DATA_DIR + "/" + GetParam ())
    {
    }

    GraphInfo g;

};

TEST_P (RandomWalkerTest, BuildLinearSystem)
{
  RandomWalker rw (g.graph,
                   boost::get (boost::edge_weight, g.graph),
                   boost::get (boost::vertex_color, g.graph));

  rw.computeVertexDegrees ();
  rw.buildLinearSystem ();

  ASSERT_EQ (g.rows, rw.L.rows ());
  ASSERT_EQ (g.rows, rw.L.cols ());
  ASSERT_EQ (g.rows, rw.B.rows ());
  ASSERT_EQ (g.cols, rw.B.cols ());

  std::vector<Weight> degrees (g.rows, 0.0);
  std::vector<Weight> L_sums (g.rows, 0.0);
  std::vector<Weight> B_sums (g.rows, 0.0);
  for (int k = 0; k < rw.L.outerSize (); ++k)
  {
    for (SparseMatrix::InnerIterator it (rw.L, k); it; ++it)
    {
      EXPECT_GE (it.row (), it.col ()); // the matrix should be lower triangular
      if (it.row () == it.col ())
      {
        degrees[it.row ()] = it.value ();
      }
      else
      {
        L_sums[it.row ()] -= it.value ();
        L_sums[it.col ()] -= it.value ();
      }
    }
  }
  for (int k = 0; k < rw.B.outerSize (); ++k)
  {
    for (SparseMatrix::InnerIterator it (rw.B, k); it; ++it)
    {
      B_sums[it.row ()] += it.value ();
    }
  }
  for (size_t i = 0; i < g.rows; ++i)
  {
    float sum = L_sums[i] + B_sums[i];
    EXPECT_FLOAT_EQ (degrees[i], sum);
  }
}

TEST_P (RandomWalkerTest, Segment)
{
  bool result = pcl::segmentation::randomWalker (g.graph);
  ASSERT_TRUE (result);
  VertexIterator vi, v_end;
  for (boost::tie (vi, v_end) = boost::vertices (g.graph); vi != v_end; ++vi)
    EXPECT_EQ (g.segmentation[*vi], g.color_map[*vi]);
}

TEST_P (RandomWalkerTest, GetPotentials)
{
  Matrix p;
  std::map<Color, size_t> map;

  pcl::segmentation::randomWalker (g.graph,
                                   boost::get (boost::edge_weight, g.graph),
                                   boost::get (boost::vertex_color, g.graph),
                                   p,
                                   map);

  ASSERT_EQ (g.size, p.rows ());
  ASSERT_EQ (g.colors.size (), p.cols ());
  ASSERT_EQ (g.colors.size (), map.size ());

  for (std::set<Color>::iterator it = g.colors.begin (); it != g.colors.end (); ++it)
    for (size_t i = 0; i < g.size; ++i)
      if (g.potentials.count (*it))
        EXPECT_NEAR (g.potentials[*it] (i), p (i, map[*it]), 0.01);
}

INSTANTIATE_TEST_CASE_P (VariousGraphs,
                         RandomWalkerTest,
                         ::testing::Values ("graph0.info",
                                            "graph1.info",
                                            "graph2.info",
                                            "graph3.info",
                                            "graph4.info",
                                            "graph5.info",
                                            "graph6.info",
                                            "graph7.info",
                                            "graph8.info"));

int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Please provide a path to the directory with test graph descriptions." << std::endl;
    return (-1);
  }

  TEST_DATA_DIR = std::string (argv[1]);

  try
  {
    ::testing::InitGoogleTest (&argc, argv);
    return RUN_ALL_TESTS ();
  }
  catch (std::exception& e)
  {
    std::cerr << "Unhandled exception: " << e.what () << "\n";
  }

  return 1;
}

