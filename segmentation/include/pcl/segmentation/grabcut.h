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
        operator() (const PointT& p) 
        { 
          PointT pp = p;
          pp.r = static_cast<uint32_t> (r);
          pp.g = static_cast<uint32_t> (g);
          pp.b = static_cast<uint32_t> (b);
          return (pp);
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
      distance2 (const Color& c1, const Color& c2);
      
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
          : gaussians_ ()
        {}
        
        /// Initialize GMM with ddesired number of gaussians.
        GMM (uint32_t K)
          : gaussians_ (K)
        {}

        /// Destructor
        ~GMM () {}
      
        uint32_t 
        getK () const { return K_; }
      
        void 
        resize (uint32_t K) { K_ = K; }
      
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
                 const pcl::PointCloud<SegmentationValue> &hardSegmentation,
                 pcl::PointCloud<uint32_t> &components,
                 GMM &background_GMM, GMM &foreground_GMM);
      /** Iteratively learn GMMs using GrabCut updating algorithm */
      void 
      learnGMMs (const Image& image, 
                 const pcl::PointCloud<SegmentationValue>& hard_segmentation,
                 pcl::PointCloud<uint32_t>& components,
                 GMM& background_GMM, GMM& foreground_GMM);
    }
  }
  
  /** \brief Implementation of the GrabCut segmentation in
    * "GrabCut — Interactive Foreground Extraction using Iterated Graph Cuts" by
    * Carsten Rother, Vladimir Kolmogorov and Andrew Blake.
    * 
    * \author Justin Talbot, jtalbot@stanford.edu placed in Public Domain, 2010
    * \author Nizar Sallem port to PCL and enhancement of original code.
    * \ingroup segmentation
    */
  class GrabCut
  {
    public:
    GrabCut (uint32_t K = 5, float lambda = 50)
      : K_ (K)
      , lambda_ (lambda)
    {}

    ~GrabCut ();
    
    template<typename PoinT>
    void
    setInputCloud (const pcl::PointCloud<PointT>& cloud);
    
    /// Initialize Trimap, inside rectangle is TrimapUnknown, outside is TrimapBackground
    void 
    initialize (uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2);
    /// Edit Trimap
    void 
    setTrimap (int x1, int y1, int x2, int y2, const grabcut::TrimapValue& t);
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
    
    // OpenGL display routine
    // void display ( int t );
    // void overlayAlpha ();

    protected:
    bool
    initCompute ();
    
    private:
    // Storage for N-link weights, each pixel stores links to only four of its 8-neighborhood neighbors.
    // This avoids duplication of links, while still allowing for relatively easy lookup.
    struct NLinks
    {
      float upleft;
      float up;
      float upright;
      float right;
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

    void 
    computeBeta ();

    void 
    computeL ();

    void 
    computeNLinks ();

    float 
    computeNLink (uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2);
    /// Build the graph for GraphCut
    void 
    initGraph ();
    /// Build images used in GraphCut
    void 
    buildImages ();
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
    // Images of various variables that can be displayed for debugging.
    // Precomputed N-link weights
    pcl::PointCloud<NLinks>::Ptr n_links_;
    pcl::PointCloud<float>::Ptr n_links_image_;
    grabcut::Image::Ptr t_links_image_;
    grabcut::Image::Ptr GMM_image_;
    pcl::PointCloud<float>::Ptr alpha_image_;
    // Store them here so we don't have to keep asking for them.
    grabcut::Image::ConstPtr image_;
    pcl::PointCloud<grabcut::TrimapValue>::Ptr trimap_;
    pcl::PointCloud<uint32_t>::Ptr GMM_component_;
    pcl::PointCloud<grabcut::SegmentationValue>::Ptr hard_segmentation_;
    // Not yet implemented (this would be interpreted as alpha)
    pcl::PointCloud<float>::Ptr soft_segmentation_;	
    grabcut::GMM background_GMM_, foreground_GMM_;
    // Graph part
    /// reverse edge property map
    boost::property_map <Graph, boost::vertex_color_t>::type color_map_;
    /// Graph
    Graph graph_;
    /// Graph source out of the image nodes
    vertex_descriptor graph_source_;
    /// Graph sink out of the image nodes
    vertex_descriptor graph_sink_;
    /// Graph for Graphcut
    pcl::PointCloud<vertex_descriptor>::Ptr nodes_;
  };
}

#endif
