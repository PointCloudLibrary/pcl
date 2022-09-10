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
 * $Id$
 *
 */

#pragma once

#include <deque> // for deque
#include <map> // for map
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

namespace pcl
{
  namespace segmentation
  {
    namespace grabcut
    {
      /** boost implementation of Boykov and Kolmogorov's maxflow algorithm doesn't support
        * negative flows which makes it inappropriate for this context.
        * This implementation of Boykov and Kolmogorov's maxflow algorithm by Stephen Gould
        * <stephen.gould@anu.edu.au> in DARWIN under BSD does the trick however solwer than original
        * implementation.
        */
      class PCL_EXPORTS BoykovKolmogorov
      {
        public:
          using vertex_descriptor = int;
          using edge_capacity_type = double;

          /// construct a maxflow/mincut problem with estimated max_nodes
          BoykovKolmogorov (std::size_t max_nodes = 0);
          /// destructor
          virtual ~BoykovKolmogorov () = default;
          /// get number of nodes in the graph
          std::size_t
          numNodes () const { return nodes_.size (); }
          /// reset all edge capacities to zero (but don't free the graph)
          void
          reset ();
          /// clear the graph and internal datastructures
          void
          clear ();
          /// add nodes to the graph (returns the id of the first node added)
          int
          addNodes (std::size_t n = 1);
          /// add constant flow to graph
          void
          addConstant (double c) { flow_value_ += c; }
          /// add edge from s to nodeId
          void
          addSourceEdge (int u, double cap);
          /// add edge from nodeId to t
          void
          addTargetEdge (int u, double cap);
          /// add edge from u to v and edge from v to u
          /// (requires cap_uv + cap_vu >= 0)
          void
          addEdge (int u, int v, double cap_uv, double cap_vu = 0.0);
          /// solve the max-flow problem and return the flow
          double
          solve ();
          /// return true if \p u is in the s-set after calling \ref solve.
          bool
          inSourceTree (int u) const { return (cut_[u] == SOURCE); }
          /// return true if \p u is in the t-set after calling \ref solve
          bool
          inSinkTree (int u) const { return (cut_[u] == TARGET); }
          /// returns the residual capacity for an edge (use -1 for terminal (-1,-1) is the current flow
          double
          operator() (int u, int v) const;

          double
          getSourceEdgeCapacity (int u) const;

          double
          getTargetEdgeCapacity (int u) const;

        protected:
          /// tree states
          enum nodestate { FREE = 0x00, SOURCE = 0x01, TARGET = 0x02 };
          /// capacitated edge
          using capacitated_edge = std::map<int, double>;
          /// edge pair
          using edge_pair = std::pair<capacitated_edge::iterator, capacitated_edge::iterator>;
          /// pre-augment s-u-t and s-u-v-t paths
          void
          preAugmentPaths ();
          /// initialize trees from source and target
          void
          initializeTrees ();
          /// expand trees until a path is found (or no path (-1, -1))
          std::pair<int, int>
          expandTrees ();
          /// augment the path found by expandTrees; return orphaned subtrees
          void
          augmentPath (const std::pair<int, int>& path, std::deque<int>& orphans);
          /// adopt orphaned subtrees
          void
          adoptOrphans (std::deque<int>& orphans);
          /// clear active set
          void clearActive ();
          /// \return true if active set is empty
          inline bool
          isActiveSetEmpty () const { return (active_head_ == TERMINAL); }
          /// active if head or previous node is not the terminal
          inline bool
          isActive (int u) const { return ((u == active_head_) || (active_list_[u].first != TERMINAL)); }
          /// mark vertex as active
          void
          markActive (int u);
          /// mark vertex as inactive
          void
          markInactive (int u);
          /// edges leaving the source
          std::vector<double> source_edges_;
          /// edges entering the target
          std::vector<double> target_edges_;
          /// nodes and their outgoing internal edges
          std::vector<capacitated_edge> nodes_;
          /// current flow value (includes constant)
          double flow_value_;
          /// identifies which side of the cut a node falls
          std::vector<unsigned char> cut_;

        private:
          /// parents_ flag for terminal state
          static const int TERMINAL; // -1
          /// search tree (also uses cut_)
          std::vector<std::pair<int, edge_pair> > parents_;
          /// doubly-linked list (prev, next)
          std::vector<std::pair<int, int> > active_list_;
          int active_head_, active_tail_;
      };

      /**\brief Structure to save RGB colors into floats */
      struct Color
      {
        Color () : r (0), g (0), b (0) {}
        Color (float _r, float _g, float _b) : r(_r), g(_g), b(_b) {}
        Color (const pcl::RGB& color) : r (color.r), g (color.g), b (color.b) {}

        template<typename PointT>
        Color (const PointT& p);

        template<typename PointT>
        operator PointT () const;

        float r, g, b;
      };
      /// An Image is a point cloud of Color
      using Image = pcl::PointCloud<Color>;
      /** \brief Compute squared distance between two colors
       * \param[in] c1 first color
       * \param[in] c2 second color
       * \return the squared distance measure in RGB space
       */
      float
      colorDistance (const Color& c1, const Color& c2);
      /// User supplied Trimap values
      enum TrimapValue { TrimapUnknown = -1, TrimapForeground, TrimapBackground };
      /// Grabcut derived hard segmentation values
      enum SegmentationValue { SegmentationForeground = 0, SegmentationBackground };
      /// Gaussian structure
      struct Gaussian
      {
        Gaussian () = default;
        /// mean of the gaussian
        Color mu;
        /// covariance matrix of the gaussian
        Eigen::Matrix3f covariance;
        /// determinant of the covariance matrix
        float determinant;
        /// inverse of the covariance matrix
        Eigen::Matrix3f inverse;
        /// weighting of this gaussian in the GMM.
        float pi;
        /// highest eigenvalue of covariance matrix
        float eigenvalue;
        /// eigenvector corresponding to the highest eigenvector
        Eigen::Vector3f eigenvector;
      };

      class PCL_EXPORTS GMM
      {
        public:
          /// Initialize GMM with ddesired number of gaussians.
          GMM () : gaussians_ (0) {}
          /// Initialize GMM with ddesired number of gaussians.
          GMM (std::size_t K) : gaussians_ (K) {}
          /// Destructor
          ~GMM () = default;
          /// \return K
          std::size_t
          getK () const { return gaussians_.size (); }
          /// resize gaussians
          void
          resize (std::size_t K) { gaussians_.resize (K); }
          /// \return a reference to the gaussian at a given position
          Gaussian&
          operator[] (std::size_t pos) { return (gaussians_[pos]); }
          /// \return a const reference to the gaussian at a given position
          const Gaussian&
          operator[] (std::size_t pos) const { return (gaussians_[pos]); }
          /// \brief \return the computed probability density of a color in this GMM
          float
          probabilityDensity (const Color &c);
          /// \brief \return the computed probability density of a color in just one Gaussian
          float
          probabilityDensity(std::size_t i, const Color &c);

        private:
          /// array of gaussians
          std::vector<Gaussian> gaussians_;
      };

      /** Helper class that fits a single Gaussian to color samples */
      class GaussianFitter
      {
        public:
        GaussianFitter (float epsilon = 0.0001)
          : sum_ (Eigen::Vector3f::Zero ())
          , accumulator_ (Eigen::Matrix3f::Zero ())
          , count_ (0)
          , epsilon_ (epsilon)
        { }

        /// Add a color sample
        void
        add (const Color &c);
        /// Build the gaussian out of all the added color samples
        void
        fit (Gaussian& g, std::size_t total_count, bool compute_eigens = false) const;
        /// \return epsilon
        float
        getEpsilon () { return (epsilon_); }
        /** set epsilon which will be added to the covariance matrix diagonal which avoids singular
          * covariance matrix
          * \param[in] epsilon user defined epsilon
          */
        void
        setEpsilon (float epsilon) { epsilon_ = epsilon; }

        private:
        /// sum of r,g, and b
        Eigen::Vector3f sum_;
        /// matrix of products (i.e. r*r, r*g, r*b), some values are duplicated.
        Eigen::Matrix3f accumulator_;
        /// count of color samples added to the gaussian
        std::uint32_t count_;
        /// small value to add to covariance matrix diagonal to avoid singular values
        float epsilon_;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
      };

      /** Build the initial GMMs using the Orchard and Bouman color clustering algorithm */
      PCL_EXPORTS void
      buildGMMs (const Image &image,
                 const Indices& indices,
                 const std::vector<SegmentationValue> &hardSegmentation,
                 std::vector<std::size_t> &components,
                 GMM &background_GMM, GMM &foreground_GMM);
      /** Iteratively learn GMMs using GrabCut updating algorithm */
      PCL_EXPORTS void
      learnGMMs (const Image& image,
                 const Indices& indices,
                 const std::vector<SegmentationValue>& hard_segmentation,
                 std::vector<std::size_t>& components,
                 GMM& background_GMM, GMM& foreground_GMM);
    }
  };

  /** \brief Implementation of the GrabCut segmentation in
    * "GrabCut — Interactive Foreground Extraction using Iterated Graph Cuts" by
    * Carsten Rother, Vladimir Kolmogorov and Andrew Blake.
    *
    * \author Justin Talbot, jtalbot@stanford.edu placed in Public Domain, 2010
    * \author Nizar Sallem port to PCL and adaptation of original code.
    * \ingroup segmentation
    */
  template <typename PointT>
  class GrabCut : public pcl::PCLBase<PointT>
  {
    public:
      using KdTree = pcl::search::Search<PointT>;
      using KdTreePtr = typename KdTree::Ptr;
      using PointCloudConstPtr = typename PCLBase<PointT>::PointCloudConstPtr;
      using PointCloudPtr = typename PCLBase<PointT>::PointCloudPtr;
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using PCLBase<PointT>::fake_indices_;

      /// Constructor
      GrabCut (std::uint32_t K = 5, float lambda = 50.f)
        : K_ (K)
        , lambda_ (lambda)
        , nb_neighbours_ (9)
        , initialized_ (false)
      {}
      /// Destructor
      ~GrabCut () override = default;
      // /// Set input cloud
      void
      setInputCloud (const PointCloudConstPtr& cloud) override;
      /// Set background points, foreground points = points \ background points
      void
      setBackgroundPoints (const PointCloudConstPtr& background_points);
      /// Set background indices, foreground indices = indices \ background indices
      void
      setBackgroundPointsIndices (int x1, int y1, int x2, int y2);
      /// Set background indices, foreground indices = indices \ background indices
      void
      setBackgroundPointsIndices (const PointIndicesConstPtr& indices);
      /// Run Grabcut refinement on the hard segmentation
      virtual void
      refine ();
      /// \return the number of pixels that have changed from foreground to background or vice versa
      virtual int
      refineOnce ();
      /// \return lambda
      float
      getLambda () { return (lambda_); }
      /** Set lambda parameter to user given value. Suggested value by the authors is 50
        * \param[in] lambda
        */
      void
      setLambda (float lambda) { lambda_ = lambda; }
      /// \return the number of components in the GMM
      std::uint32_t
      getK () { return (K_); }
      /** Set K parameter to user given value. Suggested value by the authors is 5
        * \param[in] K the number of components used in GMM
        */
      void
      setK (std::uint32_t K) { K_ = K; }
      /** \brief Provide a pointer to the search object.
        * \param tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }
      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr
      getSearchMethod () { return (tree_); }
      /** \brief Allows to set the number of neighbours to find.
        * \param[in] nb_neighbours new number of neighbours
        */
      void
      setNumberOfNeighbours (int nb_neighbours) { nb_neighbours_ = nb_neighbours; }
      /** \brief Returns the number of neighbours to find. */
      int
      getNumberOfNeighbours () const { return (nb_neighbours_); }
      /** \brief This method launches the segmentation algorithm and returns the clusters that were
        * obtained during the segmentation. The indices of points belonging to the object will be stored
        * in the cluster with index 1, other indices will be stored in the cluster with index 0.
        * \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
        */
      void
      extract (std::vector<pcl::PointIndices>& clusters);

    protected:
      // Storage for N-link weights, each pixel stores links to nb_neighbours
      struct NLinks
      {
        NLinks () : nb_links (0), indices (0), dists (0), weights (0) {}

        int nb_links;
        Indices indices;
        std::vector<float> dists;
        std::vector<float> weights;
      };
      bool
      initCompute ();
      using vertex_descriptor = pcl::segmentation::grabcut::BoykovKolmogorov::vertex_descriptor;
      /// Compute beta from image
      void
      computeBetaOrganized ();
      /// Compute beta from cloud
      void
      computeBetaNonOrganized ();
      /// Compute L parameter from given lambda
      void
      computeL ();
      /// Compute NLinks from image
      void
      computeNLinksOrganized ();
      /// Compute NLinks from cloud
      void
      computeNLinksNonOrganized ();
      /// Edit Trimap
      void
      setTrimap (const PointIndicesConstPtr &indices, segmentation::grabcut::TrimapValue t);
      int
      updateHardSegmentation ();
      /// Fit Gaussian Multi Models
      virtual void
      fitGMMs ();
      /// Build the graph for GraphCut
      void
      initGraph ();
      /// Add an edge to the graph, graph must be oriented so we add the edge and its reverse
      void
      addEdge (vertex_descriptor v1, vertex_descriptor v2, float capacity, float rev_capacity);
      /// Set the weights of SOURCE --> v and v --> SINK
      void
      setTerminalWeights (vertex_descriptor v, float source_capacity, float sink_capacity);
      /// \return true if v is in source tree
      inline bool
      isSource (vertex_descriptor v) { return (graph_.inSourceTree (v)); }
      /// image width
      std::uint32_t width_;
      /// image height
      std::uint32_t height_;
      // Variables used in formulas from the paper.
      /// Number of GMM components
      std::uint32_t K_;
      /// lambda = 50. This value was suggested the GrabCut paper.
      float lambda_;
      /// beta = 1/2 * average of the squared color distances between all pairs of 8-neighboring pixels.
      float beta_;
      /// L = a large value to force a pixel to be foreground or background
      float L_;
      /// Pointer to the spatial search object.
      KdTreePtr tree_;
      /// Number of neighbours
      int nb_neighbours_;
      /// is segmentation initialized
      bool initialized_;
      /// Precomputed N-link weights
      std::vector<NLinks> n_links_;
      /// Converted input
      segmentation::grabcut::Image::Ptr image_;
      std::vector<segmentation::grabcut::TrimapValue> trimap_;
      std::vector<std::size_t> GMM_component_;
      std::vector<segmentation::grabcut::SegmentationValue> hard_segmentation_;
      // Not yet implemented (this would be interpreted as alpha)
      std::vector<float> soft_segmentation_;
      segmentation::grabcut::GMM background_GMM_, foreground_GMM_;
      // Graph part
      /// Graph for Graphcut
      pcl::segmentation::grabcut::BoykovKolmogorov graph_;
      /// Graph nodes
      std::vector<vertex_descriptor> graph_nodes_;
  };
}

#include <pcl/segmentation/impl/grabcut_segmentation.hpp>
