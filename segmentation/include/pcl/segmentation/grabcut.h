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

#ifndef PCL_SEGMENTATION_GRABCUT
#define PCL_SEGMENTATION_GRABCUT

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/boost.h>
#include <pcl/search/search.h>

namespace pcl
{
  namespace segmentation
  {
    namespace grabcut
    {
      /** Structure to save RGB colors into floats 
       */
      struct Color
      {
        Color () : r (0), g (0), b (0) {}
        Color (float _r, float _g, float _b) : r(_r), g(_g), b(_b) {}
        Color (const pcl::RGB& color) : r (color.r), g (color.g), b (color.b) {}
        
        template<typename PointT>
        Color (const PointT& p) : r (p.r), g (p.g), b (p.b) {}
        
        template<typename PointT> PointT
        operator() () const
        { 
          PointT p;
          p.r = static_cast<uint32_t> (r);
          p.g = static_cast<uint32_t> (g);
          p.b = static_cast<uint32_t> (b);
          return (p);
        }
        
        float r, g, b;
      };
      
      typedef pcl::PointCloud<Color> Image;
      
      /** \brief Compute squared distance between two colors
       * \param[in] c1 first color
       * \param[in] c2 second color
       * \return the squared distance measure in RGB space
       */
      float 
      colorDistance (const Color& c1, const Color& c2);
      
      /// User supplied Trimap values
      enum TrimapValue { TrimapUnknown, TrimapForeground, TrimapBackground };
      
      /// Grabcut derived hard segementation values
      enum SegmentationValue { SegmentationForeground, SegmentationBackground };
      
      /** \brief clamp x value according to image width
       * \param[in] image
       * \param[in/out] x clamped x value
       */
      template <typename PointT> void
      clampX(const pcl::PointCloud<PointT>& image, int& x)
      {
        if (x < 0) x = 0;
        if (x >= image.width)  x = image.width - 1;
      }
      
      /** \brief clamp y value according to image height
       * \param[in] image
       * \param[in/out] y clamped y value
       */
      template <typename PointT> void
      clampY(const pcl::PointCloud<PointT>& image, int& y)
      {
        if (y < 0) y = 0;
        if (y >= image.height) y = image.height - 1;
      }
      
      /** \brief Fill a rectangular region of an image delimited by (x1,y1) -- (x2,y2) with a given value
       * \param[in/out] image to fill
       * \param[in] x1 top left corner X coordinate 
       * \param[in] y1 top left corner Y coordinate 
       * \param[in] x2 bottom right corner X coordinate 
       * \param[in] y2 bottom right corner Y coordinate 
       * \param[in] t the user defined value
       */
      template<typename PointT> void
      fillRectangle(pcl::PointCloud<PointT>& image, int x1, int y1, int x2, int y2, const PointT& t)
      {
        assert (image.isOrganized ());
        clampX(image, x1); clampY(image, y1);
        clampX(image, x2); clampY(image, y2);
        
        if(y1>y2) { std::swap (y1,y2); }
        if(x1>x2) { std::swap (x1,x2); }
        
        for (int i = y1; i <= y2; ++i)
        {
          for (int j = x1; j <= x2; ++j)
          {
            image[i*image.width+j] = t;
          }
        }
      }

      /** \brief Fill a rectangular region of an image delimited by (x1,y1) -- (x2,y2) with a given value
       * \param[in/out] image to fill
       * \param[in] x1 top left corner X coordinate 
       * \param[in] y1 top left corner Y coordinate 
       * \param[in] x2 bottom right corner X coordinate 
       * \param[in] y2 bottom right corner Y coordinate 
       * \param[in] t the user defined value
       */
      template<typename PointT> void
      fillRectangle(std::vector<PointT>& image, int width, int height, int x1, int y1, int x2, int y2, const PointT& t)
      {
        std::cout << "width " << width << std::endl;
        std::cout << "height " << height << std::endl;
        std::cout << "start " << x1 << "," << y1 << std::endl;
        std::cout << "end " << x2 << "," << y2 << std::endl;
        if (y1 < 0) y1 = 0;
        if (y1 >= height) y1 = height - 1;
        if (y2 < 0) y2 = 0;
        if (y2 >= height) y2 = height - 1;
        if (x1 < 0) x1 = 0;
        if (x1 >= width) x1 = width - 1;
        if (x2 < 0) x2 = 0;
        if (x2 >= width) x2 = width - 1;

        if(y1>y2) { std::swap (y1,y2); }
        if(x1>x2) { std::swap (x1,x2); }
        
        for (int i = y1; i <= y2; ++i)
        {
          for (int j = x1; j <= x2; ++j)
          {
            image[i*width+j] = t;
          }
        }
      }
      
      /** \brief Fill an image with a given value 
        * \param[in/out] image to be filled
        * \param[in] t value to fill with
        */
      template<typename PointT> void
      fill(pcl::PointCloud<PointT>& image, const PointT& t)
      {
        for (uint32_t i = 0; i < image.size (); ++i)
          image[i] = t;
      }
      
      struct Gaussian
      {
        Gaussian () {}
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
        /// heighest eigenvalue of covariance matrix
        float eigenvalue;
        /// eigenvector corresponding to the heighest eigenvector
        Eigen::Vector3f eigenvector;
      };
      
      class GMM
      {
        public:
        /// Initialize GMM with ddesired number of gaussians.
        GMM ()
          : gaussians_ (0)
        {}
        
        /// Initialize GMM with ddesired number of gaussians.
        GMM (uint32_t K)
          : gaussians_ (K)
          , K_ (K)
        {}

        /// Destructor
        ~GMM () {}
      
        uint32_t 
        getK () const { return K_; }
      
        void 
        resize (uint32_t K) 
        { 
          K_ = K; 
          gaussians_.resize (K);
        }
      
        /// \return a reference to the gaussian at a given position
        Gaussian& 
        operator[] (std::size_t pos) { return (gaussians_[pos]); }

        /// \return a const reference to the gaussian at a given position
        const Gaussian& 
        operator[] (std::size_t pos) const { return (gaussians_[pos]); }

        /** \brief Compute the probability density of a color in this GMM 
         * \param[in] c color
         * \return the probability density
         */
        float 
        probabilityDensity (const Color &c);
      
        /** \brief Compute the probability density of a color in just one Gaussian 
         * \param[in] i index of the gaussian
         * \param[in] c color
         * \return the probability density
         */
        float 
        probabilityDensity(uint32_t i, const Color &c);
      
        private:
        /// number of gaussians
        uint32_t K_;
        /// array of gaussians
        std::vector<Gaussian> gaussians_;      
      };

      /** Helper class that fits a single Gaussian to color samples */
      class GaussianFitter
      {
        public:
        GaussianFitter (float epsilon = 0.0001)
          : sum_ (Color ())
          , accumulator_ (Eigen::Matrix3f::Zero ())
          , count_ (0)
          , epsilon_ (epsilon)
        {}
      
        /// Add a color sample
        void 
        add (const Color &c);
      
        /// Build the gaussian out of all the added color samples
        void 
        fit (Gaussian& g, uint32_t total_count, bool compute_eigens = false) const;

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
        Color sum_;
        /// matrix of products (i.e. r*r, r*g, r*b), some values are duplicated.
        Eigen::Matrix3f accumulator_;
        /// count of color samples added to the gaussian
        uint32_t count_;	
        /// small value to add to covariance matrix diagonal to avoid singular values
        float epsilon_;
      };

      /** Build the initial GMMs using the Orchard and Bouman color clustering algorithm */
      void 
      buildGMMs (const Image &image, 
                 const std::vector<int>& indices,
                 const std::vector<SegmentationValue> &hardSegmentation,
                 std::vector<uint32_t> &components,
                 GMM &background_GMM, GMM &foreground_GMM);
      /** Iteratively learn GMMs using GrabCut updating algorithm */
      void 
      learnGMMs (const Image& image, 
                 const std::vector<int>& indices,
                 const std::vector<SegmentationValue>& hard_segmentation,
                 std::vector<uint32_t>& components,
                 GMM& background_GMM, GMM& foreground_GMM);
    }
  }
  
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
      typedef typename pcl::search::Search<PointT> KdTree;
      typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;
      typedef typename PCLBase<PointT>::PointCloudConstPtr PointCloudConstPtr;
      typedef typename PCLBase<PointT>::PointCloudPtr PointCloudPtr;
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;

      /// Constructor
      GrabCut (uint32_t K = 5, float lambda = 50.f)
        : K_ (K)
        , lambda_ (lambda)
        , build_clouds_ (true)
        , initialized_ (false)
        , nb_neighbours_ (9)
      {}
      /// Desctructor
      ~GrabCut () {};
      // /// Set input cloud
      void
      setInputCloud (const PointCloudConstPtr& cloud);
      /// Set background points, foreground points = points \ background points
      void
      setBackgroundPoints (const PointCloudConstPtr& background_points);
      /// Set background indices, foreground indices = indices \ background indices
      void
      setBackgroundPointsIndices (int x1, int y1, int x2, int y2);
      /// Set background indices, foreground indices = indices \ background indices
      void
      setBackgroundPointsIndices (const PointIndicesConstPtr& indices);
      /// Edit Trimap
      void 
      setTrimap (int x1, int y1, int x2, int y2, const segmentation::grabcut::TrimapValue &t);
      /// Fit Gaussian Multi Models
      void 
      fitGMMs ();
      /// Run Grabcut refinement on the hard segmentation
      void 
      refine ();
      /// \return the number of pixels that have changed from foreground to background or vice versa
      int 
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
      uint32_t
      getK () { return (K_); }
      /** Set K parameter to user given value. Suggested value by the authors is 5
        * \param[in] K the number of components used in GMM
        */
      void
      setK (uint32_t K) { K_ = K; }
      /** \brief Provide a pointer to the search object.
        * \param tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }
      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr
      getSearchMethod () { return (tree_); }
      /** \brief Returns the number of neighbours to find. */
      int
      getNumberOfNeighbours () const { return (nb_neighbours_); }
      /** \brief Allows to set the number of neighbours to find.
        * \param[in] number_of_neighbours new number of neighbours
        */
      void
      setNumberOfNeighbours (int nb_neighbours) { nb_neighbours_ = nb_neighbours; }
      /** \brief This method launches the segmentation algorithm and returns the clusters that were
        * obtained during the segmentation. The indices of points belonging to the object will be stored
        * in the cluster with index 1, other indices will be stored in the cluster with index 0.
        * \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
        */
      void
      extract (std::vector<pcl::PointIndices>& clusters);
      /// \return NLinks weights for visualization purpose only
      pcl::PointCloud<pcl::PointXYZI>::ConstPtr 
      getNLinksCloud () const { return (n_links_cloud_); }
      /// \return TLinks weights for visualization purpose only
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr 
      getTLinksCloud () const { return (t_links_cloud_); }
      /// \return GMM fitting for visualization purpose only
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
      getGMMCloud () const { return (GMM_cloud_); }
      /// \return input with alpha mask
      pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr 
      getAphaMask () const { return (alpha_cloud_); }

    protected:
      bool
      initCompute ();    

    private:
      // Storage for N-link weights, each pixel stores links to nb_neighbours      
      struct NLinks
      {
        NLinks () : nb_links (0), indices (0), dists (0), weights (0) {}
        
        int nb_links;
        std::vector<int> indices;
        std::vector<float> dists;
        std::vector<float> weights;
      };
    
      typedef float EdgeCapacityType;

      typedef boost::adjacency_list_traits <boost::vecS, boost::vecS, boost::directedS> GraphTraits;
      typedef boost::adjacency_list <boost::vecS, boost::vecS, boost::directedS,
                                     boost::property <boost::vertex_name_t, std::string,
                                     boost::property <boost::vertex_index_t, long,
                                     boost::property <boost::vertex_color_t, boost::default_color_type,
                                     boost::property <boost::vertex_distance_t, long,
                                     boost::property <boost::vertex_predecessor_t, GraphTraits::edge_descriptor> > > > >,
                                     boost::property <boost::edge_capacity_t, EdgeCapacityType,
                                     boost::property <boost::edge_residual_capacity_t, EdgeCapacityType,
                                     boost::property <boost::edge_reverse_t, GraphTraits::edge_descriptor > > > > Graph;

      typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
      typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;

      /** Update hard segmentation after running GraphCut, \return the number of pixels that have
        * changed from foreground to background or vice versa. 
        */
      int 
      updateHardSegmentation ();
      /// Compute beta from image
      void 
      computeBeta ();
      /// Compute L parameter from given lambda
      void 
      computeL ();
      /// Compute NLinks 
      void 
      computeNLinks ();
      /// Compute NLinks at a specific rectangular location
      float 
      computeNLink (uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2);
      /// Build the graph for GraphCut
      void 
      initGraph ();
      /// Build images used in GraphCut
      void 
      buildClouds ();
      /// Add an edge to the graph, graph must be oriented so we add the edge and its reverse
      void
      addEdge (vertex_descriptor &v1, vertex_descriptor &v2, float capacity, float rev_capacity);      
      /// Set the weights of SOURCE --> v and v --> SINK
      void
      setTerminalWeights (vertex_descriptor& v, float source_capacity, float sink_capacity);
      /// image width
      uint32_t width_;
      /// image height
      uint32_t height_;
      // Variables used in formulas from the paper.
      /// Number of GMM components
      uint32_t K_;
      /// lambda = 50. This value was suggested the GrabCut paper.
      float lambda_;		
      /// beta = 1/2 * average of the squared color distances between all pairs of 8-neighboring pixels.
      float beta_;
      /// L = a large value to force a pixel to be foreground or background
      float L_;
      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;
      int nb_neighbours_;
      bool build_clouds_;
      bool initialized_;
      // Precomputed N-link weights
      std::vector<NLinks> n_links_;
      segmentation::grabcut::Image::Ptr image_;
      std::vector<segmentation::grabcut::TrimapValue> trimap_;
      std::vector<uint32_t> GMM_component_;
      std::vector<segmentation::grabcut::SegmentationValue> hard_segmentation_;
      // Not yet implemented (this would be interpreted as alpha)
      std::vector<float> soft_segmentation_;	
      segmentation::grabcut::GMM background_GMM_, foreground_GMM_;
      // Graph part
      /// reverse edge property map
      boost::property_map <Graph, boost::edge_reverse_t>::type rev_;
      /// edge capacity property map
      boost::property_map <Graph, boost::edge_capacity_t >::type capacity_;
      /// Graph for Graphcut
      Graph graph_;
      /// Graph source out of the image nodes
      vertex_descriptor graph_source_;
      /// Graph sink out of the image nodes
      vertex_descriptor graph_sink_;
      /// Graph nodes
      std::vector<vertex_descriptor> graph_nodes_;
      // Clouds of various variables that can be displayed for debugging.
      pcl::PointCloud<pcl::PointXYZI>::Ptr n_links_cloud_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr t_links_cloud_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GMM_cloud_;
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alpha_cloud_;
  };
}

#include <pcl/segmentation/impl/grabcut.hpp>

#endif
