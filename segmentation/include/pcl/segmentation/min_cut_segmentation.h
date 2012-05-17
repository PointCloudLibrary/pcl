/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * $Id:$
 *
 */

#ifndef PCL_MIN_CUT_SEGMENTATION_H_
#define PCL_MIN_CUT_SEGMENTATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <set>

namespace pcl
{
  /** \brief
    * This class implements the segmentation algorithm based on minimal cut of the graph.
    * Description can be found in the article
    * "Min-Cut Based Segmentation of Point Clouds"
    * \author: Aleksey Golovinskiy (mine_all_mine@bk.ru) and Thomas Funkhouser.
    */
  template <typename PointT>
  class PCL_EXPORTS MinCutSegmentation
  {
    public:

      typedef boost::adjacency_list_traits< boost::vecS, boost::vecS, boost::directedS > Traits;

      typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS,
                                     boost::property< boost::vertex_name_t, std::string,
                                       boost::property< boost::vertex_index_t, long,
                                         boost::property< boost::vertex_color_t, boost::default_color_type,
                                           boost::property< boost::vertex_distance_t, long,
                                             boost::property< boost::vertex_predecessor_t, Traits::edge_descriptor > > > > >,
                                     boost::property< boost::edge_capacity_t, double,
                                       boost::property< boost::edge_residual_capacity_t, double,
                                         boost::property< boost::edge_reverse_t, Traits::edge_descriptor > > > > mGraph;

      typedef boost::property_map< mGraph, boost::edge_capacity_t >::type CapacityMap;

      typedef boost::property_map< mGraph, boost::edge_residual_capacity_t >::type ResidualCapacityMap;

      typedef boost::property_map< mGraph, boost::edge_reverse_t>::type ReverseEdgeMap;

      typedef boost::property_map< mGraph, boost::vertex_index_t >::type IndexMap;

      typedef Traits::vertex_descriptor VertexDescriptor;

      typedef boost::graph_traits< mGraph >::edge_descriptor EdgeDescriptor;

      typedef boost::graph_traits< mGraph >::vertex_iterator VertexIterator;

      typedef boost::graph_traits< mGraph >::out_edge_iterator OutEdgeIterator;

      typedef boost::graph_traits< mGraph >::in_edge_iterator InEdgeIterator;

    public:

      /** \brief Constructor that sets default values for member variables. */
      MinCutSegmentation ();

      /** \brief Destructor that frees memory. */
      virtual
      ~MinCutSegmentation ();

      /** \brief Returns the cloud that was passed through setInputCloud(). */
      typename pcl::PointCloud<PointT>::Ptr
      getInputCloud () const;

      /** \brief Returns normalization value for binary potentials. For more information see the article. */
      double
      getSigma () const;

      /** \brief Returns radius to the background. */
      double
      getRadius () const;

      /** \brief Returns weight that every edge from the source point has. */
      double
      getSourceWeight () const;

      /** \brief Returns search method that is used for finding KNN.
        * The graph is build such way that it contains the edges that connect point and its KNN.
        */
      typename pcl::search::Search<PointT>::Ptr
      getSearchMethod () const;

      /** \brief Returns the number of neighbours to find. */
      unsigned int
      getNeighbourNumber () const;

      /** \brief Returns the points that must belong to foreground. */
      std::vector<PointT, Eigen::aligned_allocator<PointT> >
      getForegroundPoints () const;

      /** \brief Returns the points that must belong to background. */
      std::vector<PointT, Eigen::aligned_allocator<PointT> >
      getBackgroundPoints () const;

      /** \brief Returns that flow value that was calculated during the segmentation. */
      double
      getMaxFlow () const;

      /** \brief Returns the colored cloud. Points that belong to the object have the same color. */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      getColoredCloud ();

      /** \brief Returns the graph that was build for finding the minimum cut. */
      typename boost::shared_ptr<typename pcl::MinCutSegmentation<PointT>::mGraph>
      getGraph () const;

      /** \brief This method returns the vector of labels. It contains label for every point in the cloud.
        * If point belongs to object then it has a label equal 1. Otherwise label is 0.
        */
      std::vector<int>
      getSegments () const;

      /** \brief This method sets the cloud that must be segmented.
        * \param[in] input_cloud point cloud that must be segmented
        */
      void
      setInputCloud (typename pcl::PointCloud<PointT>::Ptr input_cloud);

      /** \brief Allows to set the normalization value for the binary potentials as described in the article.
        * \param[in] sigma new normalization value
        */
      void
      setSigma (double sigma);

      /** \brief Allows to set the radius to the background.
        * \param[in] radius new radius to the background
        */
      void
      setRadius (double radius);

      /** \brief Allows to set weight for source edges. Every edge that comes from the source point will have that weight.
        * \param[in] weight new weight
        */
      void
      setSourceWeight (double weight);

      /** \brief Allows to set search method for finding KNN.
        * The graph is build such way that it contains the edges that connect point and its KNN.
        * \param[in] search search method that will be used for finding KNN.
        */
      void
      setSearchMethod (typename pcl::search::Search<PointT>::Ptr search);

      /** \brief Allows to set the number of neighbours to find.
        * \param[in] number_of_neighbours new number of neighbours
        */
      void
      setNeighbourNumber (unsigned int number_of_neighbours);

      /** \brief Allows to specify points which are known to be the points of the object.
        * \param[in] foreground_points point cloud that contains foreground points. At least one point must be specified.
        */
      void
      setForegroundPoints (typename pcl::PointCloud<PointT>::Ptr foreground_points);

      /** \brief Allows to specify points which are known to be the points of the object.
        * \param[in] foreground_points_indices vector of indices of points that belong to the foreground.
        * At least one point must be specified.
        */
      void
      setForegroundPoints (std::vector<int>& foreground_points_indices);

      /** \brief Allows to specify points which are known to be the points of the background.
        * \param[in] background_points point cloud that contains background points.
        */
      void
      setBackgroundPoints (typename pcl::PointCloud<PointT>::Ptr background_points);

      /** \brief Allows to specify points which are known to be the points of the background.
        * \param[in] background_points_indices vector of indices of points that belong to the background.
        */
      void
      setBackgroundPoints (std::vector<int>& background_points_indices);

      /** \brief This method simply launches the segmentation algorithm. It returns number of points
        * that belong to the object.
        */
      bool
      segmentPoints ();

    protected:

      /** \brief This method simply builds the graph that will be used during the segmentation. */
      bool
      buildGraph ();

      /** \brief Returns the binary potential(smooth cost) for the given indices of points.
        * In other words it returns weight that must be assigned to the edge from source to target point.
        * \param[in] source index of the source point of the edge
        * \param[in] target index of the target point of the edge
        */
      double
      calculateBinaryPotential (int source, int target) const;

      /** \brief Returns unary potential(data cost) for the given point index.
        * In other words it calculates weights for (source, point) and (point, sink) edges.
        * \param[in] point index of the point for which weights will be calculated
        * \param[out] source_weight calculated weight for the (source, point) edge
        * \param[out] sink_weight calculated weight for the (point, sink) edge
        */
      void
      calculateUnaryPotential (int point, double& source_weight, double& sink_weight) const;

      /** \brief This method simply adds the edge from the source point to the target point with a given weight.
        * \param[in] source index of the source point of the edge
        * \param[in] target index of the target point of the edge
        * \param[in] weight weight that will be assigned to the (source, target) edge
        */
      bool
      addEdge (int source, int target, double weight);

      /** \brief This method recalculates unary potentials(data cost) if some changes were made, instead of creating new graph. */
      bool
      recalculateUnaryPotentials ();

      /** \brief This method recalculates binary potentials(smooth cost) if some changes were made, instead of creating new graph. */
      bool
      recalculateBinaryPotentials ();

      /** \brief This method analyzes the residual network and assigns a label to every point in the cloud.
        * \param[in] residual_capacity residual network that was obtained during the segmentation
        */
      void
      assembleLabels (ResidualCapacityMap& residual_capacity);

    protected:

      /** \brief Stores the cloud that need to be segmented. */
      typename pcl::PointCloud<PointT>::Ptr input_cloud_;

      /** \brief Stores the search method that will be used for finding K nearest neighbors. Neighbours are used for building the graph. */
      typename pcl::search::Search<PointT>::Ptr search_;

      /** \brief Stores the number of neighbors to find. */
      unsigned int number_of_neighbours_;

      /** \brief Stores the number of points in the cloud. */
      int number_of_points_;

      /** \brief Signalizes if the graph is valid. */
      bool graph_is_valid_;

      /** \brief Signalizes if the unary potentials are valid. */
      bool unary_potentials_are_valid_;

      /** \brief Signalizes if the binary potentials are valid. */
      bool binary_potentials_are_valid_;

      /** \brief Stores the graph for finding the maximum flow. */
      boost::shared_ptr<mGraph> graph_;

      /** \brief Stores the vertex that serves as source. */
      VertexDescriptor source_;

      /** \brief Stores the vertex that serves as sink. */
      VertexDescriptor sink_;

      /** \brief Stores the vertices of the graph. */
      std::vector< VertexDescriptor > vertices_;

      /** \brief Stores the information about the edges that were added to the graph. It is used to avoid the duplicate edges. */
      std::vector< std::set<int> > edge_marker_;

      /** \brief Stores the capacity of every edge in the graph. */
      boost::shared_ptr<CapacityMap> capacity_;

      /** \brief Stores reverse edges for every edge in the graph. */
      boost::shared_ptr<ReverseEdgeMap> reverse_edges_;

      /** \brief Stores the sigma coefficient. It is used for finding smooth costs. More information can be found in the article. */
      double inverse_sigma_;

      /** \brief Stores the distance to the background. */
      double radius_;

      /** \brief Stores the weight for every edge that comes from source point. */
      double source_weight_;

      /** \brief Stores the maximum flow value that was calculated during the segmentation. */
      double max_flow_;

      /** \brief Used for comparison of the floating point numbers. */
      double epsilon_;

      /** \brief Stores the labels for each point that were obtained during the segmentation. */
      std::vector<int> labels_;

      /** \brief Stores the points that are known to be in the foreground. */
      std::vector<PointT, Eigen::aligned_allocator<PointT> > foreground_points_;

      /** \brief Stores the points that are known to be in the background. */
      std::vector<PointT, Eigen::aligned_allocator<PointT> > background_points_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif
