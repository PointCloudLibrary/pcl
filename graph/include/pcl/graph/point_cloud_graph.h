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

#ifndef PCL_GRAPH_POINT_CLOUD_GRAPH_H
#define PCL_GRAPH_POINT_CLOUD_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

/** \class pcl::graph::point_cloud_graph
  *
  * A sibling of `boost::adjacency_list` with PCL points bundled in vertices and copy-free access to them as a PCL point
  * cloud.
  *
  *
  * # Motivation #
  *
  *
  * Boost Graph Library has a concept of [bundled properties][BundledProperties], i.e. custom data types that may be
  * stored in graph vertices and conveniently accessed with `operator[]`. In the context of PCL it makes a lot of sense
  * to have PCL points bundled in graph vertices. The following specialization of [boost::adjacency_list][AdjacencyList]
  * achieves that:
  *
  * ~~~cpp
  * typedef
  *   boost::adjacency_list<
  *     boost::vecS          // Represent edge-list with std::vector
  *   , boost::vecS          // Represent vertex-list with std::vector
  *   , boost::undirectedS   // Undirected graph
  *   , pcl::PointXYZ>       // Bundle XYZ points in vertices
  * Graph
  * ~~~
  *
  * A graph-based algorithm can easily access points associated with graph vertices:
  *
  * ~~~cpp
  * Graph graph;
  * Graph::vertex_descriptor v1 = boost::add_vertex (graph);
  * Graph::vertex_descriptor v2 = boost::add_vertex (graph);
  * graph[v1] = pcl::PointXYZ (1, 2, 3);
  * graph[v2].x = graph[v1].x;
  * ~~~
  *
  * The problem though is that there is no efficient way to put the point cloud data inside the graph and retrieve it
  * back. Suppose that in some application there is a point cloud that should first be filtered using some PCL tools
  * (e.g. pcl::PassThrough filter), then a graph-based algorithm should be applied to it, and finally some more
  * processing using PCL tools is required. The user will need to copy all the points of the original point cloud
  * one-by-one to the graph, run the algorithm, and then copy it back one-by-one again.
  *
  *
  * # Solution #
  *
  *
  * point_cloud_graph resolves this issue by storing bundled points internally as a PCL point cloud. In fact, it stores
  * a shared pointer to a point cloud, so input and output of points is really copy-free. One can create a new graph
  * based on an existing point cloud:
  *
  * ~~~cpp
  * // Create a point cloud with 10 points
  * pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (10, 1);
  * // Create a graph based on the cloud
  * pcl::graph::point_cloud_graph<pcl::PointXYZ> graph (cloud);
  * // The graph will have a vertex for each point of the original cloud
  * assert (10 == boost::num_vertices (graph));
  * // The points may be accessed using operator[]
  * graph[1].x = 14;
  * // The graph shares data with the original point cloud, so modifying a bundled point
  * // changes the corresponding point in the original cloud
  * assert (14 == cloud->points[1].x);
  * ~~~
  *
  * The internal point cloud as a whole may be accessed using `pcl::graph::point_cloud (Graph& g)`:
  *
  * ~~~cpp
  * // Perform some graph-based algorithm on point cloud graph
  * // ...
  * // Retrieve the bundled data as a point cloud
  * pcl::PointCloud<pcl::PointXYZ>::Ptr data = pcl::graph::point_cloud (graph);
  * // Continue to work with the data
  * pcl::io::savePCDFile ("output.pcd", *data);
  * ~~~
  *
  * Despite the fact that the point data are stored in a PCL point cloud, all the standard BGL features associated with
  * bundled properties are supported, including `operator[]` and `get`/`put` access.
  *
  * [BundledProperties]: http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/bundles.html
  *
  *
  * ## Comparison with [boost::adjacency_list][AdjacencyList] ##
  *
  *
  * point_cloud_graph uses the same back-bone implementation as [boost::adjacency_list][AdjacencyList] and therefore
  * models the same concepts and may be used everywhere `boost::adjacency_list` could. In particular, the very same
  * functions could be used to add and remove vertices, access properties, iterate over edges, etc.
  *
  * As `boost::adjacency_list`, point_cloud_graph allows to configure itself with a number of template parameters. They
  * are almost the same as in `boost::adjacency_list`, though there are several differences:
  *
  * 1. There is a required template parameter `PointT` which allows the user to select the type of PCL points to be
  *    bundled in graph vertices;
  *
  *    Note that the `VertexProperty` parameter is still available for the user, so it is possible to additionally
  *    attach as many [internal properties][InternalProperties] (such as `vertex_color_t`) as needed.
  *
  * 2. The default value for `Directed` parameter is changed to `undirectedS` since it is more relevant in the context
  *    of point cloud processing;
  *
  * 3. The vertex-list of the graph is constrained to be `std::vector`. The user has no control over it as there is no
  *    `VertexList` template parameter;
  *
  *    The choice of the container for the vertex-list determines the space complexity of the graph structure, as well
  *    as the time complexity of the various graph operations, see [using boost::adjacency_list][UsingAdjacencyList] for
  *    a more detailed discussion. In the case of point_cloud_graph the vertex bundled properties (PCL points) are
  *    stored in a PCL point cloud (which has `std::vector` behind the scenes), so it is only logical to have
  *    `std::vector` as the container type for the vertex-list itself.
  *
  * 4. There is no `GraphProperty` template parameter.
  *
  *    [boost::adjacency_list][AdjacencyList] provides a facility to store an arbitrary piece of data associated with
  *    the graph as a whole (graph property). While being useful in certain cases, we do not consider this as a vital
  *    feature and have sacrificed it to support [boost::subgraph][Subgraph]. For more details please refer to the
  *    section about `boost::subgraph` below.
  *
  * [AdjacencyList]: http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/adjacency_list.html "adjacency_list"
  * [InternalProperties]: http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/using_adjacency_list.html#sec:adjacency-list-properties
  * [UsingAdjacencyList]: http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/using_adjacency_list.html#sec:choosing-graph-type
  * [Subgraph]: http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/subgraph.html "subgraph"
  *
  *
  * ## Compatibility with [boost::subgraph][Subgraph] ##
  *
  *
  * `boost::subgraph` provides a convenient mechanism to keep track of a graph and its induced subgraphs. Two issues
  * have been accounted for in order to allow smooth inter-operation between point_cloud_graph and `boost::subgraph`.
  *
  *
  * ### Fixed interface ###
  *
  *
  * `boost::subgraph` is a template class that is *wrapped around* a graph class rather than a class that *inherits*
  * from a graph class. Therefore if we want it to be possible to use point_cloud_graph either way (plain or wrapped by
  * `boost::subgraph`), we have to limit ourselves to the interface exposed by `boost::subgraph`. In particular, we can
  * not have a PCL point cloud-based constructor for point_cloud_graph because (for apparent reasons) `boost::subgraph`
  * does not have it. However, the ability to construct a point_cloud_graph from an existing PCL point cloud is an
  * essential feature. We noted that `boost::subgraph` has a GraphProperty-based constructor that could be re-used as a
  * desired PCL point cloud-based constructor if we alias GraphProperty to PCL point cloud. Thus GraphProperty is used
  * to support internal mechanics of point_cloud_graph and is no longer available to the user.
  *
  *
  * ### Lack of internal storage for points ###
  *
  *
  * Recall that point_cloud_graph stores the points bundled in its vertices inside a PCL point cloud. It could be
  * conveniently retrieved using `pcl::graph::point_cloud (Graph& g)` and then used in PCL algorithms.
  * `boost::subgraph`, however, is designed in such a way that it stores the indices of the vertices and edges of the
  * parent (root) graph that belong to it, but not the data associated with these vertices and edges. In other words, a
  * hierarchy of subgraphs (each of which represents a particular subset of vertices and edges of the root graph) shares
  * common vertex/edge data, and all these data are contained inside the root graph. Consequently, there does not exist
  * a PCL point cloud for each subgraph, but rather a single PCL point cloud for the root graph. This raises a question
  * as of how the `pcl::graph::point_cloud (Subgraph& g)` function should be implemented.
  *
  * One option is to construct a new PCL point cloud and fill it with subgraph's points. This, however, would be
  * inconsistent with the behavior of `pcl::graph::point_cloud (Graph& g)`, which simply returns a pointer (meaning it
  * is *O(1)* operation) that could be used to read and modify the points of the graph. Luckily, working with subsets of
  * point clouds is such a common task in the PCL world that \ref pcl::PCLBase class (which lies at the top of the
  * hierarchy of PCL algorithms) has a means for the user to supply an input point cloud *and* a vector of indices to
  * which the processing should be limited to. Therefore, we decided that `pcl::graph::point_cloud (Subgraph& g)` should
  * return the PCL point cloud of its root graph (that is, the whole set of points), and there should be an auxiliary
  * function `pcl::graph::indices (Subgraph& g)` that returns indices of the points that belong to the subgraph.
  *
  *
  * - - -
  *
  *
  * # Specification #
  *
  *
  * As was noted, point_cloud_graph is virtually the same as [boost::adjacency_list][AdjacencyList]. Please refer to
  * its documentation. The specifications below only highlight the differences.
  *
  *
  * ## Template parameters ##
  *
  *
  * Parameter      | Description                                                                         | Default
  * -------------- | ------------------------------------------------------------------------------------|--------------
  * PointT         | Type of PCL points bundled in graph vertices                                        | --
  * OutEdgeList    | Selector for the container used to represent the edge-list for each of the vertices | `vecS`
  * Directed       | Selector to choose whether the graph is directed/undirected/bidirectional           | `undirectedS`
  * VertexProperty | For specifying internal vertex property storage (apart from bundled points)         | `no_property`
  * EdgeProperty   | For specifying internal edge property storage                                       | `no_property`
  * EdgeList       | Selector for the container used to represent the edge-list for the graph            | `listS`
  *
  *
  * ## Model of ##
  *
  *
  * [PointCloudGraph](\ref boost::concepts::PointCloudGraphConcept),
  * [VertexAndEdgeListGraph](http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/VertexAndEdgeListGraph.html),
  * [VertexMutablePropertyGraph](http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/MutablePropertyGraph.html),
  * [EdgeMutablePropertyGraph](http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/MutablePropertyGraph.html),
  * [CopyConstructible](http://www.boost.org/doc/libs/1_55_0/libs/utility/CopyConstructible.html),
  * [Assignable](http://www.boost.org/doc/libs/1_55_0/libs/utility/Assignable.html)
  *
  * Probably it could also support [Serializable][Serialization], however this option has not been explored.
  *
  * [Serialization]: http://www.boost.org/doc/libs/1_55_0/libs/serialization/doc/index.html
  *
  *
  * ## Associated types ##
  *
  *
  * point_cloud_graph_traits structure provides a means to access the type of points bundled in the point_cloud_graph
  * and, for convenience, the types of PCL point cloud and shared pointer to PCL point cloud of those points.
  *
  *
  * ## Non-member functions ##
  *
  *
  * ~~~cpp
  * typename pcl::PointCloud<PointT>::Ptr
  * pcl::graph::point_cloud (point_cloud_graph<PointT>& g);
  * ~~~
  *
  * Return a shared pointer to the PCL point cloud stored internally. There are both `const` and `non-const` versions.
  *
  * ~~~cpp
  * typename pcl::PointCloud<PointT>::Ptr
  * pcl::graph::point_cloud (boost::subgraph<point_cloud_graph<PointT>>& g);
  * ~~~
  *
  * Return a shared pointer to the PCL point cloud stored in the *root* graph. There are both `const` and `non-const`
  * versions.
  *
  * ~~~cpp
  * pcl::PointIndices::Ptr
  * pcl::graph::indices (point_cloud_graph<PointT>& g);
  * ~~~
  *
  * Return a shared pointer to a vector of indices of the points that belong to this graph. Since point_cloud_graph is
  * a complete graph (i.e. it is not a subgraph of some other graph), the returned indices are guaranteed to be
  * *[0..N-1]*, where *N* is the number of vertices. There are both `const`- and `non-const`- versions.
  *
  * ~~~cpp
  * pcl::PointIndices::Ptr
  * pcl::graph::indices (boost::subgraph<point_cloud_graph<PointT>>& g);
  * ~~~
  *
  * Return a shared pointer to a vector of indices of the points of the root graph that belong to this subgraph. There
  * are both `const`- and `non-const`- versions.
  *
  * \author Sergey Alexandrov
  * \ingroup graph */

namespace pcl
{

  namespace graph
  {

    template <typename PointT,
              typename OutEdgeListS = boost::vecS,
              typename DirectedS = boost::undirectedS,
              typename VertexProperty = boost::no_property,
              typename EdgeProperty = boost::no_property,
              typename EdgeListS = boost::listS>
    class point_cloud_graph
      : public boost::detail::adj_list_gen<
          point_cloud_graph<
            PointT
          , OutEdgeListS
          , DirectedS
          , VertexProperty
          , EdgeProperty
          , EdgeListS
          >
        , boost::vecS
        , OutEdgeListS
        , DirectedS
        , VertexProperty
        , EdgeProperty
        , typename pcl::PointCloud<PointT>::Ptr
        , EdgeListS
        >::type
        // Not even sure what `maybe_named_graph` is, but without it
        // CopyConstructible concept is not satisfied because of missing
        // `vertex_by_property` (and some others) member functions
      , public boost::graph::maybe_named_graph<
          point_cloud_graph<
            PointT
          , OutEdgeListS
          , DirectedS
          , VertexProperty
          , EdgeProperty
          , EdgeListS
          >
        , typename boost::adjacency_list_traits<
            OutEdgeListS
          , boost::vecS
          , DirectedS
          , EdgeListS
          >::vertex_descriptor
        , VertexProperty
        >
    {

      private:

        typedef point_cloud_graph self;
        typedef
          typename boost::detail::adj_list_gen<
            self
          , boost::vecS
          , OutEdgeListS
          , DirectedS
          , VertexProperty
          , EdgeProperty
          , typename pcl::PointCloud<PointT>::Ptr
          , EdgeListS
          >::type
        Base;

        // Ensure that the user did not provide his own Bundle for vertices
        BOOST_STATIC_ASSERT ((boost::is_same<typename Base::vertex_bundled, boost::no_property>::value));

      public:

        typedef typename pcl::PointCloud<PointT>::Ptr graph_property_type;
        typedef typename pcl::PointCloud<PointT>::Ptr graph_bundled;
        typedef typename Base::vertex_property_type vertex_property_type;
        typedef PointT vertex_bundled;
        typedef typename Base::edge_property_type edge_property_type;
        typedef typename Base::edge_bundled edge_bundled;

        typedef typename Base::stored_vertex stored_vertex;
        typedef typename Base::vertices_size_type vertices_size_type;
        typedef typename Base::edges_size_type edges_size_type;
        typedef typename Base::degree_size_type degree_size_type;
        typedef typename Base::vertex_descriptor vertex_descriptor;
        typedef typename Base::edge_descriptor edge_descriptor;
        typedef OutEdgeListS out_edge_list_selector;
        typedef boost::vecS vertex_list_selector;
        typedef DirectedS directed_selector;
        typedef EdgeListS edge_list_selector;

        /// Type of PCL points bundled in graph vertices
        typedef PointT point_type;
        typedef pcl::PointCloud<PointT> point_cloud_type;
        typedef typename point_cloud_type::Ptr point_cloud_ptr;
        typedef typename point_cloud_type::ConstPtr point_cloud_const_ptr;

        /** Construct a graph based on existing point cloud.
          *
          * The graph will have the same amount of vertices as the input point
          * cloud has. The shared pointer will be stored internally so that the
          * graph retains access to the point data.
          *
          * If the cloud is not given, then a new empty cloud will be created. */
        point_cloud_graph (const point_cloud_ptr& p = point_cloud_ptr (new point_cloud_type))
        : Base (p->size ())
        , m_point_cloud (p)
        {
        }

        /** Construct a graph with a given number of vertices.
          *
          * This constructor will create a new point cloud to store point data.
          * The second parameter is completely ignored and is provided only to
          * have the same interface as `adjacency_list`. */
        point_cloud_graph (vertices_size_type num_vertices,
                           const point_cloud_ptr& = point_cloud_ptr (new point_cloud_type))
        : Base (num_vertices)
        , m_point_cloud (new point_cloud_type (num_vertices, 1))
        {
        }

        /** Copy constructor.
          *
          * Acts just like the standard copy constructor of `adjacency_list`,
          * i.e. copies vertex and edge set along with associated properties.
          * Note that a **deep copy** of the underlying point cloud is made.
          *
          * This constructor reuses assignment operator. An alternative approach
          * would be to use the copy constructor of the base class and then copy
          * over the underlying point cloud, however the problem is that the
          * base constructor uses `boost::add_vertex` function, which will
          * attempt to push points to the point cloud, although it will not be
          * initialized yet at that point in time (hence segfault). */
        point_cloud_graph (const point_cloud_graph& x)
        {
          *this = x;
        }

        /** Assignment operator.
          *
          * Acts just like the standard assignment operator of `adjacency_list`,
          * i.e. copies vertex and edge set along with associated properties. Note
          * that a **deep copy** of the underlying point cloud is made. */
        point_cloud_graph&
        operator= (const point_cloud_graph& x)
        {
          if (&x != this)
          {
            /* The motivation behind this `reset` invocation is as follows. The
             * `operator=` function of the base class (`vec_adj_list_impl`) will
             * use the standard `add_vertex` to append each vertex of the source
             * graph one after another. Apparently, this is not the fastest way
             * to copy the underlying point cloud. Resetting the point cloud
             * pointer effectively makes `added_vertex` a no-op, so the time is
             * not wasted copying points one by one. Rather in the next line we
             * make a complete copy with one call. */
            m_point_cloud.reset ();
            Base::operator= (x);
            m_point_cloud.reset (new point_cloud_type (*x.m_point_cloud));
          }
          return (*this);
        }

        /** Remove all of the edges and vertices from the graph.
          *
          * Note that it wipes the underlying point cloud as well. */
        void clear ()
        {
          this->clearing_graph ();
          m_point_cloud->clear ();
          Base::clear ();
        }

        /* A note on the implementation of vertex addition/removal.
         *
         * In BGL vertices are added and removed to a graph via non-member
         * functions `add_vertex` and `remove_vertex`. The implementation that is
         * provided for `vec_adj_list_impl` (from which this class actually
         * derives) does everything that is needed, except to adding/removing
         * points from internal point cloud (for obvious reasons). In order to
         * augment the behavior we could have specialized these functions for
         * `point_cloud_graph` class, however that would involve copy-pasting of
         * quite some code. Luckily, these functions have some sort of hooks
         * which are member functions of graph object and got called in the end,
         * namely `added_vertex` and `removing_vertex`. (In fact, they are
         * designated to support some `named_graph` extension, but who cares.)
         * Therefore these two hooks are implemented in `point_cloud_graph` to
         * perform the desired point cloud maintenance. */

        void
        added_vertex (vertex_descriptor)
        {
          if (m_point_cloud)
            m_point_cloud->push_back (point_type ());
        }

        void
        removing_vertex (vertex_descriptor vertex)
        {
          if (m_point_cloud)
            m_point_cloud->erase (m_point_cloud->begin () + vertex);
        }

        /* This second version of `removing_vertex` is to account for the
         * change introduced in Boost 1.55, which added a second parameter to
         * this function. The type of the second parameter is unimportant and
         * likely is not present in earlier versions of Boost, hence generic
         * templated version. */

        template <typename T> void
        removing_vertex (vertex_descriptor vertex, T)
        {
          removing_vertex (vertex);
        }

        /** \name Access to bundled vertex/edge properties.
          *
          * The `operators[]`'s in this group may be used to access the data
          * bundled in vertices and edges of the graph. Implementation was
          * directly copied from `adjacency_list`.
          *
          * Note that there is no operator access to the GraphProperty as there
          * is no such thing in point_cloud_graph (see the corresponding
          * section in the class description for more information). */

        ///@{

        vertex_bundled&
        operator[] (vertex_descriptor v)
        {
          return (boost::get (boost::vertex_bundle, *this)[v]);
        }

        const vertex_bundled&
        operator[] (vertex_descriptor v) const
        {
          return (boost::get (boost::vertex_bundle, *this)[v]);
        }

        edge_bundled&
        operator[] (edge_descriptor e)
        {
          return (boost::get (boost::edge_bundle, *this)[e]);
        }

        const edge_bundled&
        operator[] (edge_descriptor e) const
        {
          return (boost::get (boost::edge_bundle, *this)[e]);
        }

        ///@}

        /// Storage for the internal cloud data.
        //  This is public for the same reasons as everything in `boost::graph`
        //  is public.
        point_cloud_ptr m_point_cloud;

    };

    /** Traits struct to access the types associated with point_cloud_graph. */
    template <typename Graph>
    struct point_cloud_graph_traits
    {
      /// The type of PCL points bundled in vertices.
      typedef typename Graph::point_type point_type;
      /// The type of PCL point cloud the graph can be viewed as.
      typedef typename Graph::point_cloud_type point_cloud_type;
      /// The type of a shared pointer to PCL point cloud the graph can be
      /// viewed as.
      typedef typename Graph::point_cloud_ptr point_cloud_ptr;
      /// The type of a shared pointer to const PCL point cloud the graph can be
      /// viewed as.
      typedef typename Graph::point_cloud_const_ptr point_cloud_const_ptr;
    };

    /** Specialization for point_cloud_graphs wrapped in `boost::subgraph`. */
    template <typename Graph>
    struct point_cloud_graph_traits<boost::subgraph<Graph> > : point_cloud_graph_traits<Graph>
    { };

    /** This class is to expose the point cloud stored in the point_cloud_graph
      * as a vertex bundle property map.
      *
      * The code is based on `boost::vector_property_map`. It might be the case
      * that some of the member functions are not needed. */
    template <typename Graph>
    class point_cloud_property_map
      : public boost::put_get_helper<
          typename std::iterator_traits<
            typename point_cloud_graph_traits<Graph>::point_cloud_type::iterator
          >::reference
        , point_cloud_property_map<Graph>
        >
    {

      public:

        typedef typename boost::property_traits<boost::identity_property_map>::key_type key_type;
        typedef typename point_cloud_graph_traits<Graph>::point_cloud_type::iterator iterator;
        typedef typename point_cloud_graph_traits<Graph>::point_cloud_type::const_iterator const_iterator;
        typedef typename std::iterator_traits<iterator>::reference reference;
        typedef typename point_cloud_graph_traits<Graph>::point_type value_type;
        typedef boost::lvalue_property_map_tag category;

        point_cloud_property_map (const Graph* g, boost::vertex_bundle_t)
        : data (g->m_point_cloud)
        , index (boost::identity_property_map ())
        {
        }

        iterator
        storage_begin ()
        {
          return (data->begin ());
        }

        iterator
        storage_end ()
        {
          return (data->end ());
        }

        const_iterator
        storage_begin () const
        {
          return (data->begin ());
        }

        const_iterator
        storage_end () const
        {
          return (data->end ());
        }

        boost::identity_property_map&
        get_index_map ()
        {
          return (index);
        }

        const boost::identity_property_map&
        get_index_map () const
        {
          return (index);
        }

        reference
        operator[] (const key_type& v) const
        {
          return ((*data)[get (index, v)]);
        }

      private:

        typename point_cloud_graph_traits<Graph>::point_cloud_ptr data;
        boost::identity_property_map index;

    };

  } // namespace graph

} // namespace pcl

#define PCG_PARAMS typename P, typename OEL, typename D, typename VP, typename EP, typename EL
#define PCG pcl::graph::point_cloud_graph<P, OEL, D, VP, EP, EL>

namespace boost
{

  /* In `point_cloud_graph` we use a special kind of property map to access the
   * vertex bundle. This specialization of the `property_map` traits struct
   * makes sure that everyone (especially `get` function defined in
   * 'boost/graph/detail/adjacency_list.hpp' which does the job of creating
   * these maps) is aware of this fact. */

  template <PCG_PARAMS>
  struct property_map<PCG, vertex_bundle_t>
  {
    typedef pcl::graph::point_cloud_property_map<PCG> type;
    typedef type const_type;
  };

  /* The following two functions (source, target) are required by the
   * EdgeListGraph concept. Implementation is the same as in `adjacency_list`. */

  template <typename Directed, typename Vertex, PCG_PARAMS>
  inline Vertex
  source (const detail::edge_base<Directed, Vertex>& e, const PCG&)
  {
    return e.m_source;
  }

  template <typename Directed, typename Vertex, PCG_PARAMS>
  inline Vertex
  target (const detail::edge_base<Directed, Vertex>& e, const PCG&)
  {
    return e.m_target;
  }

  template <PCG_PARAMS>
  struct graph_mutability_traits<PCG>
  {
    typedef mutable_property_graph_tag category;
  };

} // namespace boost

namespace pcl
{

  namespace graph
  {

    /** Retrieve the point cloud stored in a point cloud graph.
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <PCG_PARAMS>
    inline typename pcl::PointCloud<P>::Ptr
    point_cloud (PCG& g)
    {
      return (g.m_point_cloud);
    }

    /** Retrieve the point cloud stored in a point cloud graph (const version).
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <PCG_PARAMS>
    inline typename pcl::PointCloud<P>::ConstPtr
    point_cloud (const PCG& g)
    {
      return (g.m_point_cloud);
    }

    /** Retrieve the indices of the points of the point cloud stored in a point
      * cloud graph that actually belong to the graph.
      *
      * Since point_cloud_graph always contain all the points that it stores,
      * this function always returns a vector with indices from \c 0 to \c N-1,
      * where \c N is the number of vertices (points) in the graph.
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <PCG_PARAMS>
    inline pcl::PointIndices::Ptr
    indices (const PCG& g)
    {
      pcl::PointIndices::Ptr indices (new pcl::PointIndices);
      indices->indices.resize (g.m_point_cloud->size ());
      for (size_t i = 0; i < g.m_point_cloud->size (); ++i)
        indices->indices[i] = i;
      return (indices);
    }

    /** Retrieve the point cloud stored in a point cloud (sub)graph.
      *
      * The behavior of this function will be different for root and child
      * subgraphs. A root subgraph will return the point cloud stored in it,
      * whereas a child subgraph will return the point cloud stored in its
      * parent graph.
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <PCG_PARAMS>
    inline typename pcl::PointCloud<P>::Ptr
    point_cloud (boost::subgraph<PCG>& g)
    {
      return (g.root ().m_graph.m_point_cloud);
    }

    /** Retrieve the point cloud stored in a point cloud (sub)graph (const
      * version).
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <PCG_PARAMS>
    inline typename pcl::PointCloud<P>::ConstPtr
    point_cloud (const boost::subgraph<PCG>& g)
    {
      return (g.root ().m_graph.m_point_cloud);
    }

    /** Retrieve the indices of the points of the point cloud stored in a point
      * cloud (sub)graph that actually belong to the (sub)graph.
      *
      * A child subgraph contains only a subset of the vertices of the parent
      * graph. This function provides a vector of indices of the vertices of the
      * parent graph that belong to this subgraph as well. These vertex indices
      * are valid point indices for the point cloud stored in the parent graph.
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <PCG_PARAMS>
    inline pcl::PointIndices::Ptr
    indices (const boost::subgraph<PCG>& g)
    {
      if (g.is_root ())
        return indices (g.m_graph);
      pcl::PointIndices::Ptr indices (new pcl::PointIndices);
      indices->indices.resize (boost::num_vertices (g));
      for (size_t i = 0; i < boost::num_vertices (g); ++i)
        indices->indices[i] = g.m_global_vertex[i];
      return (indices);
    }

  } // namespace graph

} // namespace pcl

#endif /* PCL_GRAPH_POINT_CLOUD_GRAPH_H */

