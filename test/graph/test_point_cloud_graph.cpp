#include <gtest/gtest.h>

#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>

#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/graph/point_cloud_graph.h>
#include <pcl/graph/point_cloud_graph_concept.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

typedef
  pcl::graph::point_cloud_graph<
    PointT
  , boost::vecS
  , boost::undirectedS
  , boost::property<boost::vertex_color_t, int>
  , boost::property<boost::edge_weight_t, float
  , boost::property<boost::edge_index_t, int> >
  , boost::listS
  >
Graph;

typedef boost::subgraph<Graph> SubGraph;

typedef boost::graph_traits<Graph>::edge_descriptor              EdgeId;
typedef boost::graph_traits<Graph>::vertex_descriptor            VertexId;
typedef boost::graph_traits<Graph>::edge_iterator                EdgeIterator;
typedef boost::graph_traits<Graph>::vertex_iterator              VertexIterator;
typedef boost::property_map<Graph, boost::edge_weight_t>::type   EdgeWeightPropertyMap;
typedef boost::property_map<Graph, boost::vertex_color_t>::type  VertexColorPropertyMap;
typedef boost::property_map<Graph, boost::vertex_bundle_t>::type VertexBundlePropertyMap;


/*********************************************************************
 *                          Concept checks                           *
 *                                                                   *
 * Both plain and wrapped by `boost::subgraph` variants of           *
 * `point_cloud_graph` are checked agains each concept. The failures *
 * (if any) will happend at compile time, so if the compilation is   *
 * successful the test runs are not supposed to fail.                *
 *********************************************************************/


TEST (PointCloudGraphConceptsCheck, GraphConcept)
{
  BOOST_CONCEPT_ASSERT ((boost::GraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT ((boost::GraphConcept<SubGraph>));
}

TEST (PointCloudGraphConceptsCheck, PointCloudGraphConcept)
{
  BOOST_CONCEPT_ASSERT ((pcl::graph::PointCloudGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT ((pcl::graph::PointCloudGraphConcept<SubGraph>));
}

TEST (PointCloudGraphConceptsCheck, CopyConstructibleConcept)
{
  BOOST_CONCEPT_ASSERT ((boost::CopyConstructibleConcept<Graph>));
  BOOST_CONCEPT_ASSERT ((boost::CopyConstructibleConcept<SubGraph>));
}

TEST (PointCloudGraphConceptsCheck, AssignableConcept)
{
  BOOST_CONCEPT_ASSERT ((boost::AssignableConcept<Graph>));
  BOOST_CONCEPT_ASSERT ((boost::AssignableConcept<SubGraph>));
}

TEST (PointCloudGraphConceptsCheck, VertexAndEdgeListGraphConcept)
{
  BOOST_CONCEPT_ASSERT ((boost::VertexAndEdgeListGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT ((boost::VertexAndEdgeListGraphConcept<SubGraph>));
}

TEST (PointCloudGraphConceptsCheck, VertexMutableGraphConcept)
{
  BOOST_CONCEPT_ASSERT ((boost::VertexMutableGraphConcept<Graph>));
  // Despite from what the documentation says, `subgraph` is not a model of
  // VertexMutableGraph because it does not implement `remove_vertex`.
}

TEST (PointCloudGraphConceptsCheck, EdgeMutableGraphConcept)
{
  BOOST_CONCEPT_ASSERT ((boost::EdgeMutableGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT ((boost::EdgeMutableGraphConcept<SubGraph>));
}

TEST (PointCloudGraphConceptsCheck, VertexMutablePropertyGraphConcept)
{
  BOOST_CONCEPT_ASSERT ((boost::VertexMutablePropertyGraphConcept<Graph>));
  // See the note for VertexMutableGraphConcept.
}

TEST (PointCloudGraphConceptsCheck, EdgeMutablePropertyGraphConcept)
{
  BOOST_CONCEPT_ASSERT ((boost::EdgeMutablePropertyGraphConcept<Graph>));
  BOOST_CONCEPT_ASSERT ((boost::EdgeMutablePropertyGraphConcept<SubGraph>));
}


/*********************************************************************
 *                      point_cloud_graph tests                      *
 *                                                                   *
 * Tests for member functions of `point_cloud_graph`, as well as     *
 * specializations of BGL non-member functions to modify graph,      *
 * access its properties, etc.                                       *
 *********************************************************************/


class PointCloudGraphTest : public ::testing::Test
{

  public:

    PointCloudGraphTest ()
    : pc0 (new PointCloud)
    , pc1 (new PointCloud (10, 1))
    {
      for (size_t i = 0; i < pc1->size (); ++i)
        pc1->at (i).getVector3fMap () << rand (), rand (), rand ();
    }

    PointCloudPtr pc0; // empty point cloud
    PointCloudPtr pc1; // point cloud with 10 random points

};

TEST_F (PointCloudGraphTest, ConstructorDefault)
{
  Graph g;
  ASSERT_EQ (0, boost::num_edges (g));
  ASSERT_EQ (0, boost::num_vertices (g));
  ASSERT_EQ (0, pcl::graph::point_cloud (g)->size ());
}

TEST_F (PointCloudGraphTest, ConstructorPointCloud)
{
  // Empty pointcloud
  {
    Graph g (pc0);
    ASSERT_EQ (0, boost::num_edges (g));
    ASSERT_EQ (0, boost::num_vertices (g));
    ASSERT_EQ (0, pcl::graph::point_cloud (g)->size ());
    ASSERT_EQ (pc0, pcl::graph::point_cloud (g));
  }
  // Non-empty pointcloud
  {
    Graph g (pc1);
    ASSERT_EQ (0, boost::num_edges (g));
    ASSERT_EQ (pc1->size (), boost::num_vertices (g));
    ASSERT_EQ (pc1->size (), pcl::graph::point_cloud (g)->size ());
    ASSERT_EQ (pc1, pcl::graph::point_cloud (g));
  }
}

TEST_F (PointCloudGraphTest, ConstructorNumVertices)
{
  // Zero vertices
  {
    Graph g (0);
    ASSERT_EQ (0, boost::num_edges (g));
    ASSERT_EQ (0, boost::num_vertices (g));
    ASSERT_EQ (0, pcl::graph::point_cloud (g)->size ());
  }
  // Several vertices
  {
    Graph g (5);
    ASSERT_EQ (0, boost::num_edges (g));
    ASSERT_EQ (5, boost::num_vertices (g));
    ASSERT_EQ (5, pcl::graph::point_cloud (g)->size ());
  }
}

TEST_F (PointCloudGraphTest, ConstructorCopy)
{
  Graph g1 (pc1);
  boost::add_edge (0, 1, g1);
  Graph g2 (g1);
  ASSERT_EQ (1, boost::num_edges (g2));
  ASSERT_EQ (boost::num_vertices (g1), boost::num_vertices (g2));
  ASSERT_EQ (boost::num_vertices (g1), pcl::graph::point_cloud (g2)->size ());
  ASSERT_NE (pcl::graph::point_cloud (g1), pcl::graph::point_cloud (g2));
}

TEST_F (PointCloudGraphTest, ConstructorAssignment)
{
  Graph g1 (pc1);
  boost::add_edge (0, 1, g1);
  Graph g2;
  g2 = g1;
  ASSERT_EQ (1, boost::num_edges (g2));
  ASSERT_EQ (boost::num_vertices (g1), boost::num_vertices (g2));
  ASSERT_EQ (boost::num_vertices (g1), pcl::graph::point_cloud (g2)->size ());
  ASSERT_NE (pcl::graph::point_cloud (g1), pcl::graph::point_cloud (g2));
}

TEST_F (PointCloudGraphTest, PointCloudGetter)
{
  // Non-const getter
  Graph g (pc1);
  ASSERT_EQ (pc1, pcl::graph::point_cloud (g));
  // Const getter
  const Graph& gc = g;
  ASSERT_EQ (pc1, pcl::graph::point_cloud (gc));
}

TEST_F (PointCloudGraphTest, IndicesGetter)
{
  // Non-const getter
  Graph g (pc1);
  ASSERT_EQ (pc1->size (), pcl::graph::indices (g)->indices.size ());
  for (size_t i = 0; i < pc1->size (); ++i)
    EXPECT_EQ (i, pcl::graph::indices (g)->indices[i]);
  // Const getter
  const Graph& gc = g;
  ASSERT_EQ (pc1->size (), pcl::graph::indices (gc)->indices.size ());
  for (size_t i = 0; i < pc1->size (); ++i)
    EXPECT_EQ (i, pcl::graph::indices (gc)->indices[i]);
}

TEST_F (PointCloudGraphTest, AddVertex)
{
  Graph g;

  size_t expected_size = 0;
  ASSERT_EQ (expected_size, boost::num_vertices (g));

  VertexId v1 = boost::add_vertex (g);
  ++expected_size;
  EXPECT_EQ (0, v1);
  ASSERT_EQ (expected_size, boost::num_vertices (g));
  ASSERT_EQ (expected_size, pcl::graph::point_cloud (g)->size ());

  VertexId v2 = boost::add_vertex (8, g);
  ++expected_size;
  EXPECT_EQ (1, v2);
  EXPECT_EQ (8, boost::get (boost::vertex_color, g, v2));
  ASSERT_EQ (expected_size, pcl::graph::point_cloud (g)->size ());
  ASSERT_EQ (expected_size, boost::num_vertices (g));
}

TEST_F (PointCloudGraphTest, RemoveVertex)
{
  Graph g (pc1);
  size_t expected_size = pc1->size ();
  ASSERT_EQ (expected_size, boost::num_vertices (g));
  VertexId v1 = 1;
  boost::remove_vertex (v1, g);
  --expected_size;
  ASSERT_EQ (expected_size, pc1->size ());
  ASSERT_EQ (expected_size, boost::num_vertices (g));
  ASSERT_EQ (expected_size, pcl::graph::point_cloud (g)->size ());
  VertexId v2 = 4;
  boost::remove_vertex (v2, g);
  --expected_size;
  ASSERT_EQ (expected_size, pc1->size ());
  ASSERT_EQ (expected_size, boost::num_vertices (g));
  ASSERT_EQ (expected_size, pcl::graph::point_cloud (g)->size ());
}

TEST_F (PointCloudGraphTest, Clear)
{
  Graph g (pc1);

  size_t expected_size = pc1->size ();
  ASSERT_EQ (expected_size, boost::num_vertices (g));
  ASSERT_EQ (expected_size, pcl::graph::point_cloud (g)->size ());

  g.clear ();
  expected_size = 0;
  ASSERT_EQ (expected_size, boost::num_vertices (g));
  ASSERT_EQ (expected_size, pcl::graph::point_cloud (g)->size ());
  ASSERT_EQ (expected_size, pc1->size ());
}

TEST_F (PointCloudGraphTest, VertexBundleAccess)
{
  Graph g (pc1);
  VertexId v1 = 1;
  VertexId v2 = 5;

  // Operator access
  {
    EXPECT_XYZ_EQ (pc1->at (v1), g[v1]);
    EXPECT_XYZ_EQ (pc1->at (v2), g[v2]);
    g[v1].x = 1;
    g[v2].y = 2;
    EXPECT_FLOAT_EQ (1, pc1->at (v1).x);
    EXPECT_FLOAT_EQ (2, pc1->at (v2).y);
  }

  // Get/Put access
  {
    EXPECT_XYZ_EQ (pc1->at (v1), boost::get (boost::vertex_bundle, g, v1));
    EXPECT_XYZ_EQ (pc1->at (v2), boost::get (boost::vertex_bundle, g, v2));
    boost::put (boost::vertex_bundle, g, v1, pc1->at (7));
    boost::put (boost::vertex_bundle, g, v2, pc1->at (8));
    EXPECT_XYZ_EQ (pc1->at (v1), pc1->at (7));
    EXPECT_XYZ_EQ (pc1->at (v2), pc1->at (8));
  }

  // Property map access
  {
    VertexBundlePropertyMap vb = boost::get (boost::vertex_bundle, g);
    vb[v1] = vb[v2];
    EXPECT_XYZ_EQ (pc1->at (v1), pc1->at (v2));
    vb[v2].y = 5;
    EXPECT_FLOAT_EQ (5, pc1->at (v2).y);
    for (VertexId i = 0; i < boost::num_vertices (g); ++i)
      EXPECT_XYZ_EQ (pc1->at (i), vb[i]);
  }
}

TEST_F (PointCloudGraphTest, VertexPropertyAccess)
{
  Graph g (pc1);
  VertexId v1 = 1;

  // Get/Put access
  {
    boost::put (boost::vertex_color, g, v1, 1);
    EXPECT_EQ (1, boost::get (boost::vertex_color, g, v1));
  }

  // Property map access
  {
    VertexColorPropertyMap vc = boost::get (boost::vertex_color, g);
    for (VertexId i = 0; i < boost::num_vertices (g); ++i)
      vc[i] = i;
    for (VertexId i = 0; i < boost::num_vertices (g); ++i)
      EXPECT_EQ (i, vc[i]);
  }
}

TEST_F (PointCloudGraphTest, EdgePropertyAccess)
{
  Graph g (pc1);
  boost::add_edge (0, 1, 1.0, g);
  boost::add_edge (0, 2, 2.0, g);
  boost::add_edge (0, 3, 3.0, g);

  EdgeIterator ei, ee;

  // Read access
  // via get()
  {
    {
      // With key
      float w = 1.0;
      for (boost::tie (ei, ee) = boost::edges (g); ei != ee; ++ei, w += 1.0)
        EXPECT_EQ (w, boost::get (boost::edge_weight, g, *ei));
    }
    {
      // With map
      float w = 1.0;
      EdgeWeightPropertyMap ew = boost::get (boost::edge_weight, g);
      for (boost::tie (ei, ee) = boost::edges (g); ei != ee; ++ei, w += 1.0)
        EXPECT_EQ (w, ew[*ei]);
    }
  }

  // Write access
  // via put()
  {
    float w = 4.0;
    for (boost::tie (ei, ee) = boost::edges (g); ei != ee; ++ei, w += 1.0)
    {
      boost::put (boost::edge_weight, g, *ei, w);
      EXPECT_EQ (w, boost::get (boost::edge_weight, g, *ei));
    }
  }
  // via get()
  {
    {
      // With key
      float w = 5.0;
      for (boost::tie (ei, ee) = boost::edges (g); ei != ee; ++ei, w += 1.0)
      {
        boost::get (boost::edge_weight, g, *ei) = w;
        EXPECT_EQ (w, boost::get (boost::edge_weight, g, *ei));
      }
    }
    {
      // With map
      float w = 6.0;
      EdgeWeightPropertyMap ew = boost::get (boost::edge_weight, g);
      for (boost::tie (ei, ee) = boost::edges (g); ei != ee; ++ei, w += 1.0)
      {
        ew[*ei] = w;
        EXPECT_EQ (w, ew[*ei]);
      }
    }
  }
}

TEST_F (PointCloudGraphTest, EdgeSourceTarget)
{
  Graph g (pc1);
  boost::add_edge (0, 1, 1.0, g);
  EXPECT_EQ (0, boost::source (*boost::edges (g).first, g));
  EXPECT_EQ (1, boost::target (*boost::edges (g).first, g));
}


/*********************************************************************
 *                 subgraph<point_cloud_graph> tests                 *
 *                                                                   *
 * Tests to make sure that `point_cloud_graph` wrapped in            *
 * `boost::subgraph` works as expected.                              *
 *********************************************************************/


class PointCloudSubGraphTest : public PointCloudGraphTest { };

TEST_F (PointCloudSubGraphTest, ConstructorDefault)
{
  SubGraph g;
  ASSERT_EQ (0, boost::num_edges (g));
  ASSERT_EQ (0, boost::num_vertices (g));
  ASSERT_EQ (0, pcl::graph::point_cloud (g)->size ());
}

TEST_F (PointCloudSubGraphTest, ConstructorPointCloud)
{
  // Empty pointcloud
  {
    SubGraph g (pc0);
    ASSERT_EQ (0, boost::num_edges (g));
    ASSERT_EQ (0, boost::num_vertices (g));
    ASSERT_EQ (0, pcl::graph::point_cloud (g)->size ());
    ASSERT_EQ (pc0, pcl::graph::point_cloud (g));
  }
  // Non-empty pointcloud
  {
    SubGraph g (pc1);
    ASSERT_EQ (0, boost::num_edges (g));
    ASSERT_EQ (pc1->size (), boost::num_vertices (g));
    ASSERT_EQ (pc1->size (), pcl::graph::point_cloud (g)->size ());
    ASSERT_EQ (pc1, pcl::graph::point_cloud (g));
  }
}

TEST_F (PointCloudSubGraphTest, ConstructorNumVertices)
{
  // Zero vertices
  {
    SubGraph g (0);
    ASSERT_EQ (0, boost::num_edges (g));
    ASSERT_EQ (0, boost::num_vertices (g));
    ASSERT_EQ (0, pcl::graph::point_cloud (g)->size ());
  }
  // Several vertices
  {
    SubGraph g (5);
    ASSERT_EQ (0, boost::num_edges (g));
    ASSERT_EQ (5, boost::num_vertices (g));
    ASSERT_EQ (5, pcl::graph::point_cloud (g)->size ());
  }
}

TEST_F (PointCloudSubGraphTest, ConstructorCopy)
{
  SubGraph g1 (pc1);
  boost::add_edge (0, 1, g1);
  SubGraph g2 (g1);
  ASSERT_EQ (1, boost::num_edges (g2));
  ASSERT_EQ (boost::num_vertices (g1), boost::num_vertices (g2));
  ASSERT_EQ (boost::num_vertices (g1), pcl::graph::point_cloud (g2)->size ());
  ASSERT_NE (pcl::graph::point_cloud (g1), pcl::graph::point_cloud (g2));
}

TEST_F (PointCloudSubGraphTest, ConstructorAssignment)
{
  SubGraph g1 (pc1);
  boost::add_edge (0, 1, g1);
  SubGraph g2;
  g2 = g1;
  ASSERT_EQ (1, boost::num_edges (g2));
  ASSERT_EQ (boost::num_vertices (g1), boost::num_vertices (g2));
  ASSERT_EQ (boost::num_vertices (g1), pcl::graph::point_cloud (g2)->size ());
  ASSERT_NE (pcl::graph::point_cloud (g1), pcl::graph::point_cloud (g2));
}

TEST_F (PointCloudSubGraphTest, PointCloudGetter)
{
  // Non-const getter
  SubGraph g (pc1);
  ASSERT_EQ (pc1, pcl::graph::point_cloud (g));
  // Const getter
  const SubGraph& gc = g;
  ASSERT_EQ (pc1, pcl::graph::point_cloud (gc));
  // Getter with child subgraph
  SubGraph& s = g.create_subgraph ();
  ASSERT_EQ (pc1, pcl::graph::point_cloud (s));
}

TEST_F (PointCloudSubGraphTest, IndicesGetter)
{
  // Non-const getter
  SubGraph g (pc1);
  ASSERT_EQ (pc1->size (), pcl::graph::indices (g)->indices.size ());
  for (size_t i = 0; i < pc1->size (); ++i)
    EXPECT_EQ (i, pcl::graph::indices (g)->indices[i]);
  // Const getter
  const SubGraph& gc = g;
  ASSERT_EQ (pc1->size (), pcl::graph::indices (gc)->indices.size ());
  for (size_t i = 0; i < pc1->size (); ++i)
    EXPECT_EQ (i, pcl::graph::indices (gc)->indices[i]);
  // Getter with child subgraph
  SubGraph& s = g.create_subgraph ();
  {
    // Empty subgraph
    ASSERT_EQ (0, pcl::graph::indices (s)->indices.size ());
  }
  {
    // Subgraph with several vertices
    VertexId global1 = 1;
    VertexId global2 = 3;
    boost::add_vertex (global1, s);
    boost::add_vertex (global2, s);
    ASSERT_EQ (2, pcl::graph::indices (s)->indices.size ());
    EXPECT_EQ (global1, pcl::graph::indices (s)->indices[0]);
    EXPECT_EQ (global2, pcl::graph::indices (s)->indices[1]);
  }
}

TEST_F (PointCloudSubGraphTest, AddVertex)
{
  // Add vertices to a root subgraph
  {
    SubGraph g;
    ASSERT_EQ (0, boost::num_vertices (g));
    VertexId v1 = boost::add_vertex (g);
    ASSERT_EQ (0, v1);
    ASSERT_EQ (1, boost::num_vertices (g));
    ASSERT_EQ (1, pcl::graph::point_cloud (g)->size ());
  }
  // Add vertices to a child subgraph
  {
    SubGraph g (pc1);
    SubGraph& s = g.create_subgraph ();
    VertexId global1 = 1;
    VertexId global2 = 3;
    VertexId global3 = 5;
    // Add vertices that already exist in root graph,
    // make sure the could be found with global descriptors
    {
      VertexId local1 = boost::add_vertex (global1, s);
      ASSERT_EQ (1, boost::num_vertices (s));
      ASSERT_EQ (10, boost::num_vertices (g));
      VertexId local2 = boost::add_vertex (global2, s);
      ASSERT_EQ (2, boost::num_vertices (s));
      ASSERT_EQ (10, boost::num_vertices (g));
      bool found;
      VertexId v;
      boost::tie (v, found) = s.find_vertex (global1);
      EXPECT_TRUE (found);
      EXPECT_EQ (v, local1);
      boost::tie (v, found) = s.find_vertex (global2);
      EXPECT_TRUE (found);
      EXPECT_EQ (v, local2);
      boost::tie (v, found) = s.find_vertex (global3);
      EXPECT_FALSE (found);
    }
    // Add new vertex, changes the size of the root graph as well
    {
      boost::add_vertex (s);
      ASSERT_EQ (3, boost::num_vertices (s));
      ASSERT_EQ (11, boost::num_vertices (g));
    }
  }
}

TEST_F (PointCloudSubGraphTest, VertexBundleAccess)
{
  SubGraph g (pc1);
  SubGraph& s = g.create_subgraph ();
  VertexId global1 = 1;
  VertexId global2 = 3;
  VertexId local1 = boost::add_vertex (global1, s);
  VertexId local2 = boost::add_vertex (global2, s);

  // Operator access
  {
    EXPECT_XYZ_EQ (g[global1], s[local1]);
    EXPECT_XYZ_EQ (g[global2], s[local2]);
    // By local descriptor
    {
      s[local1].x = 1;
      s[local2].y = 2;
      EXPECT_FLOAT_EQ (1, pc1->at (global1).x);
      EXPECT_FLOAT_EQ (2, pc1->at (global2).y);
    }
    // By global descriptor
    {
      g[global1].x = 2;
      g[global2].y = 1;
      EXPECT_FLOAT_EQ (2, pc1->at (global1).x);
      EXPECT_FLOAT_EQ (1, pc1->at (global2).y);
    }
  }

  // Get/Put access
  {
    EXPECT_XYZ_EQ (pc1->at (global1), boost::get (boost::vertex_bundle, s, local1));
    EXPECT_XYZ_EQ (pc1->at (global2), boost::get (boost::vertex_bundle, s, local2));
    boost::put (boost::vertex_bundle, s, local1, pc1->at (7));
    boost::put (boost::vertex_bundle, s, local2, pc1->at (8));
    EXPECT_XYZ_EQ (pc1->at (global1), pc1->at (7));
    EXPECT_XYZ_EQ (pc1->at (global2), pc1->at (8));
  }

  // Property map access
  {
    typedef
      boost::property_map<
        SubGraph
      , boost::vertex_bundle_t
      >::type
    VertexBundlePropertyMap;

    VertexBundlePropertyMap vb = boost::get (boost::vertex_bundle, s);
    for (VertexId i = 0; i < boost::num_vertices (s); ++i)
      EXPECT_XYZ_EQ (pc1->at (s.local_to_global (i)), vb[i]);
  }
}

TEST_F (PointCloudSubGraphTest, VertexPropertyAccess)
{
  SubGraph g (pc1);
  SubGraph& s = g.create_subgraph ();
  VertexId global1 = 1;
  VertexId global2 = 3;
  VertexId local1 = boost::add_vertex (global1, s);
  VertexId local2 = boost::add_vertex (global2, s);

  // Get/Put access
  {
    // By global descriptor
    {
      boost::put (boost::vertex_color, g, global1, 1);
      EXPECT_EQ (1, boost::get (boost::vertex_color, g, global1));
    }
    // By local descriptor
    {
      boost::put (boost::vertex_color, s, local2, 4);
      EXPECT_EQ (4, boost::get (boost::vertex_color, s, local2));
    }
    // Mixed
    {
      EXPECT_EQ (boost::get (boost::vertex_color, s, local1), boost::get (boost::vertex_color, g, global1));
      EXPECT_EQ (boost::get (boost::vertex_color, s, local2), boost::get (boost::vertex_color, g, global2));
    }
  }

  // Property map access
  {
    typedef
      boost::property_map<
        SubGraph
      , boost::vertex_color_t
      >::type
    VertexColorPropertyMap;

    VertexColorPropertyMap vcg = boost::get (boost::vertex_color, g);
    VertexColorPropertyMap vcs = boost::get (boost::vertex_color, s);

    // Write in parent, read in child subgraph
    {
      vcg[global1] = 8;
      EXPECT_EQ (8, vcs[local1]);
    }
    // Write in child, read in parent subgraph
    {
      vcs[local2] = 9;
      EXPECT_EQ (9, vcg[global2]);
    }
  }
}

TEST_F (PointCloudSubGraphTest, EdgeSourceTarget)
{
  SubGraph g (pc1);
  boost::add_edge (0, 1, 1.0, g);
  EXPECT_EQ (0, boost::source (*boost::edges (g).first, g));
  EXPECT_EQ (1, boost::target (*boost::edges (g).first, g));
}

int main (int argc, char **argv)
{
  try
  {
    ::testing::InitGoogleTest (&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS ();
  }
  catch (std::exception& e)
  {
    std::cerr << "Unhandled exception: " << e.what () << "\n";
  }
  return 1;
}

