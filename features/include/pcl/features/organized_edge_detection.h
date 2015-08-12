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

#ifndef PCL_FEATURES_ORGANIZED_EDGE_DETECTION_H_
#define PCL_FEATURES_ORGANIZED_EDGE_DETECTION_H_

#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>

namespace pcl
{
  /** \brief OrganizedEdgeBase, OrganizedEdgeFromRGB, OrganizedEdgeFromNormals, 
    * and OrganizedEdgeFromRGBNormals find 3D edges from an organized point 
    * cloud data. Given an organized point cloud, they will output a PointCloud 
    * of edge labels and a vector of PointIndices.
    * OrganizedEdgeBase accepts PCL_XYZ_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, and EDGELABEL_OCCLUDED.
    * OrganizedEdgeFromRGB accepts PCL_RGB_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, and EDGELABEL_RGB_CANNY.
    * OrganizedEdgeFromNormals accepts PCL_XYZ_POINT_TYPES with PCL_NORMAL_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, and EDGELABEL_HIGH_CURVATURE.
    * OrganizedEdgeFromRGBNormals accepts PCL_RGB_POINT_TYPES with PCL_NORMAL_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, EDGELABEL_HIGH_CURVATURE, and EDGELABEL_RGB_CANNY.
    *
    * \author Changhyun Choi
    */
  template <typename PointT, typename PointLT>
  class OrganizedEdgeBase : public PCLBase<PointT>
  {
    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      
    typedef typename pcl::PointCloud<PointLT> PointCloudL;
    typedef typename PointCloudL::Ptr PointCloudLPtr;
    typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

    public:
      typedef boost::shared_ptr<OrganizedEdgeBase<PointT, PointLT> > Ptr;
      typedef boost::shared_ptr<const OrganizedEdgeBase<PointT, PointLT> > ConstPtr;
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using PCLBase<PointT>::initCompute;
      using PCLBase<PointT>::deinitCompute;

      /** \brief Constructor for OrganizedEdgeBase */
      OrganizedEdgeBase ()
        : th_depth_discon_ (0.02f)
        , max_search_neighbors_ (50)
        , detecting_edge_types_ (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED)
      {
      }

      /** \brief Destructor for OrganizedEdgeBase */
      virtual
      ~OrganizedEdgeBase ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities)
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;
      
      /** \brief Set the tolerance in meters for difference in depth values between neighboring points. */
      inline void
      setDepthDisconThreshold (const float th)
      {
        th_depth_discon_ = th;
      }

      /** \brief Get the tolerance in meters for difference in depth values between neighboring points. */
      inline float
      getDepthDisconThreshold () const
      {
        return (th_depth_discon_);
      }

      /** \brief Set the max search distance for deciding occluding and occluded edges. */
      inline void
      setMaxSearchNeighbors (const int max_dist)
      {
        max_search_neighbors_ = max_dist;
      }

      /** \brief Get the max search distance for deciding occluding and occluded edges. */
      inline int
      getMaxSearchNeighbors () const
      {
        return (max_search_neighbors_);
      }

      /** \brief Set the detecting edge types. */
      inline void
      setEdgeType (int edge_types)
      {
        detecting_edge_types_ = edge_types;
      }

      /** \brief Get the detecting edge types. */
      inline int
      getEdgeType () const
      {
        return detecting_edge_types_;
      }
      
      enum {EDGELABEL_NAN_BOUNDARY=1, EDGELABEL_OCCLUDING=2, EDGELABEL_OCCLUDED=4, EDGELABEL_HIGH_CURVATURE=8, EDGELABEL_RGB_CANNY=16};
      static const int num_of_edgetype_ = 5;

    protected:
      /** \brief Perform the 3D edge detection (edges from depth discontinuities) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        */
      void
      extractEdges (pcl::PointCloud<PointLT>& labels) const;
      
      /** \brief Assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      assignLabelIndices (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;
      
      struct Neighbor
      {
        Neighbor (int dx, int dy, int didx)
        : d_x (dx)
        , d_y (dy)
        , d_index (didx)
        {}
        
        int d_x;
        int d_y;
        int d_index; // = dy * width + dx: pre-calculated
      };

      /** \brief The tolerance in meters for difference in depth values between neighboring points 
        * (The value is set for 1 meter and is adapted with respect to depth value linearly. 
        * (e.g. 2.0*th_depth_discon_ in 2 meter depth)) 
        */
      float th_depth_discon_;

      /** \brief The max search distance for deciding occluding and occluded edges */
      int max_search_neighbors_;

      /** \brief The bit encoded value that represents edge types to detect */
      int detecting_edge_types_;
  };

  template <typename PointT, typename PointLT>
  class OrganizedEdgeFromRGB : virtual public OrganizedEdgeBase<PointT, PointLT>
  {
    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      
    typedef typename pcl::PointCloud<PointLT> PointCloudL;
    typedef typename PointCloudL::Ptr PointCloudLPtr;
    typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

    public:
      using OrganizedEdgeBase<PointT, PointLT>::input_;
      using OrganizedEdgeBase<PointT, PointLT>::indices_;
      using OrganizedEdgeBase<PointT, PointLT>::initCompute;
      using OrganizedEdgeBase<PointT, PointLT>::deinitCompute;
      using OrganizedEdgeBase<PointT, PointLT>::detecting_edge_types_;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_NAN_BOUNDARY;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDING;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDED;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_RGB_CANNY;

      /** \brief Constructor for OrganizedEdgeFromRGB */
      OrganizedEdgeFromRGB ()
        : OrganizedEdgeBase<PointT, PointLT> ()
        , th_rgb_canny_low_ (40.0)
        , th_rgb_canny_high_ (100.0)
      {
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_RGB_CANNY);
      }

      /** \brief Destructor for OrganizedEdgeFromRGB */
      virtual
      ~OrganizedEdgeFromRGB ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities and RGB Canny edge) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;
      
      /** \brief Set the low threshold value for RGB Canny edge detection */
      inline void
      setRGBCannyLowThreshold (const float th)
      {
        th_rgb_canny_low_ = th;
      }

      /** \brief Get the low threshold value for RGB Canny edge detection */
      inline float
      getRGBCannyLowThreshold () const
      {
        return (th_rgb_canny_low_);
      }

      /** \brief Set the high threshold value for RGB Canny edge detection */
      inline void
      setRGBCannyHighThreshold (const float th)
      {
        th_rgb_canny_high_ = th;
      }

      /** \brief Get the high threshold value for RGB Canny edge detection */
      inline float
      getRGBCannyHighThreshold () const
      {
        return (th_rgb_canny_high_);
      }

    protected:
      /** \brief Perform the 3D edge detection (edges from depth discontinuities and RGB Canny edge)
        * \param[out] labels a PointCloud of edge labels
        */
      void
      extractEdges (pcl::PointCloud<PointLT>& labels) const;

      /** \brief The low threshold value for RGB Canny edge detection (default: 40.0) */
      float th_rgb_canny_low_;

      /** \brief The high threshold value for RGB Canny edge detection (default: 100.0) */
      float th_rgb_canny_high_;
  };

  template <typename PointT, typename PointNT, typename PointLT>
  class OrganizedEdgeFromNormals : virtual public OrganizedEdgeBase<PointT, PointLT>
  {
    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      
    typedef typename pcl::PointCloud<PointNT> PointCloudN;
    typedef typename PointCloudN::Ptr PointCloudNPtr;
    typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

    typedef typename pcl::PointCloud<PointLT> PointCloudL;
    typedef typename PointCloudL::Ptr PointCloudLPtr;
    typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

    public:
      using OrganizedEdgeBase<PointT, PointLT>::input_;
      using OrganizedEdgeBase<PointT, PointLT>::indices_;
      using OrganizedEdgeBase<PointT, PointLT>::initCompute;
      using OrganizedEdgeBase<PointT, PointLT>::deinitCompute;
      using OrganizedEdgeBase<PointT, PointLT>::detecting_edge_types_;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_NAN_BOUNDARY;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDING;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDED;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_HIGH_CURVATURE;

      /** \brief Constructor for OrganizedEdgeFromNormals */
      OrganizedEdgeFromNormals () 
        : OrganizedEdgeBase<PointT, PointLT> ()
        , normals_ ()
        , th_hc_canny_low_ (0.4f)
        , th_hc_canny_high_ (1.1f)
      {
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE);
      }

      /** \brief Destructor for OrganizedEdgeFromNormals */
      virtual
      ~OrganizedEdgeFromNormals ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities and high curvature regions) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;

      /** \brief Provide a pointer to the input normals.
        * \param[in] normals the input normal cloud
        */
      inline void
      setInputNormals (const PointCloudNConstPtr &normals) 
      {
        normals_ = normals;
      }

      /** \brief Get the input normals. */
      inline PointCloudNConstPtr
      getInputNormals () const
      {
        return (normals_);
      }

      /** \brief Set the low threshold value for high curvature Canny edge detection */
      inline void
      setHCCannyLowThreshold (const float th)
      {
        th_hc_canny_low_ = th;
      }

      /** \brief Get the low threshold value for high curvature Canny edge detection */
      inline float
      getHCCannyLowThreshold () const
      {
        return (th_hc_canny_low_);
      }

      /** \brief Set the high threshold value for high curvature Canny edge detection */
      inline void
      setHCCannyHighThreshold (const float th)
      {
        th_hc_canny_high_ = th;
      }

      /** \brief Get the high threshold value for high curvature Canny edge detection */
      inline float
      getHCCannyHighThreshold () const
      {
        return (th_hc_canny_high_);
      }
      
    protected:
      /** \brief Perform the 3D edge detection (edges from depth discontinuities and high curvature regions)
        * \param[out] labels a PointCloud of edge labels
        */
      void
      extractEdges (pcl::PointCloud<PointLT>& labels) const;

      /** \brief A pointer to the input normals */
      PointCloudNConstPtr normals_;

      /** \brief The low threshold value for high curvature Canny edge detection (default: 0.4) */
      float th_hc_canny_low_;

      /** \brief The high threshold value for high curvature Canny edge detection (default: 1.1) */
      float th_hc_canny_high_;
  };

  template <typename PointT, typename PointNT, typename PointLT>
  class OrganizedEdgeFromRGBNormals : public OrganizedEdgeFromRGB<PointT, PointLT>, public OrganizedEdgeFromNormals<PointT, PointNT, PointLT>
  {
    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      
    typedef typename pcl::PointCloud<PointNT> PointCloudN;
    typedef typename PointCloudN::Ptr PointCloudNPtr;
    typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

    typedef typename pcl::PointCloud<PointLT> PointCloudL;
    typedef typename PointCloudL::Ptr PointCloudLPtr;
    typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

    public:
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::input_;
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::indices_;
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::initCompute;
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::deinitCompute;
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::detecting_edge_types_;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_NAN_BOUNDARY;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDING;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDED;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_HIGH_CURVATURE;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_RGB_CANNY;
      
      /** \brief Constructor for OrganizedEdgeFromRGBNormals */
      OrganizedEdgeFromRGBNormals () 
        : OrganizedEdgeFromRGB<PointT, PointLT> ()
        , OrganizedEdgeFromNormals<PointT, PointNT, PointLT> ()
      {
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_RGB_CANNY | EDGELABEL_HIGH_CURVATURE);
      }

      /** \brief Destructor for OrganizedEdgeFromRGBNormals */
      virtual
      ~OrganizedEdgeFromRGBNormals ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities, RGB Canny edge, and high curvature regions) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;
  };


  // new stuff
  template <typename PointT, typename PointNT, typename PointLT>
  class OrganizedEdgeFromPoints : virtual public OrganizedEdgeBase<PointT, PointLT>
  {
    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef typename pcl::PointCloud<PointNT> PointCloudN;
    typedef typename PointCloudN::Ptr PointCloudNPtr;
    typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

    typedef typename pcl::PointCloud<PointLT> PointCloudL;
    typedef typename PointCloudL::Ptr PointCloudLPtr;
    typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

    public:
      using OrganizedEdgeBase<PointT, PointLT>::input_;
      using OrganizedEdgeBase<PointT, PointLT>::indices_;
      using OrganizedEdgeBase<PointT, PointLT>::initCompute;
      using OrganizedEdgeBase<PointT, PointLT>::deinitCompute;
      using OrganizedEdgeBase<PointT, PointLT>::detecting_edge_types_;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_NAN_BOUNDARY;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDING;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDED;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_HIGH_CURVATURE;

      /** \brief Compilation of all relevant algorithm parameters */
      struct EdgeDetectionConfig
      {
        /** \brief Noise suppression mode on differentiated images */
        enum NoiseReductionMode { NONE, GAUSSIAN };   //, BILATERAL }; bilateral not yet implemented

        /** \brief Noise suppression mode on differentiated images */
        NoiseReductionMode noise_reduction_mode;

        /** \brief Kernel size of noise reduction filter, usually 3, 5 or 7 */
        int noise_reduction_kernel_size;

        /** \brief Threshold for depth dependent depth edge (step) verification, corresponds to minimum depth step in [m] at 1m distance (default ~ 0.01 - 0.02) */
        float depth_step_factor;

        /** \brief Minimum angle of surrounding surfaces around a surface discontinuity (edge), in [deg] */
        double min_detectable_edge_angle;

        /** \brief Use fixed width scan line at certain depth (false) or adapt scan line width to surrounding edges, i.e. not crossing edges (true, default). If set to false no surface edge will be computed when the scan line crosses depth edges. If true, the scan line is adapted close to depth edges which yields more accurate results (though at little lower speed). */
        bool use_adaptive_scan_line;

        /** \brief Minimum scan line width used for surface slope approximation, in [pixels] (typically around 5pix) */
        int min_scan_line_width;

        /** \brief Maximum scan line width used for surface slope approximation, in [pixels] (typically between 20 and 40pix) */
        int max_scan_line_width;

        /** \brief Scan_line_width is the length of the scan line left and right of the query pixel at 2m distance (using linear model with scan line width at 0.5m of 5 pixels) */
        int scan_line_width_at_2m;

        /** \brief Computed value from scan_line_width_at_2m (slope in linear model) */
        double scan_line_model_m;

        /** \brief Computed value from scan_line_width_at_2m (offset in linear model) */
        double scan_line_model_n;

        /** \brief Standard constructor with default parameters */
        EdgeDetectionConfig()
        {
          noise_reduction_mode = GAUSSIAN;
          noise_reduction_kernel_size = 3;
          depth_step_factor = 0.01f;
          min_detectable_edge_angle = 45;
          use_adaptive_scan_line = true;
          min_scan_line_width = 5;
          max_scan_line_width = 20;
          scan_line_width_at_2m = 15;
          scan_line_model_n = (20.-scan_line_width_at_2m)/(double)3.0;
          scan_line_model_m = 10 - 2*scan_line_model_n;
        }

        /** \brief Constructor with user defined parameters */
        EdgeDetectionConfig(const NoiseReductionMode noise_reduction_mode_, const int noise_reduction_kernel_size_, const float depth_step_factor_, const double min_detectable_edge_angle_, const bool use_adaptive_scan_line_, const int min_scan_line_width_, const int max_scan_line_width_, const int scan_line_width_at_2m_)
        {
          noise_reduction_mode = noise_reduction_mode_;
          noise_reduction_kernel_size = noise_reduction_kernel_size_;
          depth_step_factor = depth_step_factor_;
          min_detectable_edge_angle = min_detectable_edge_angle_;
          use_adaptive_scan_line = use_adaptive_scan_line_;
          min_scan_line_width = min_scan_line_width_;
          max_scan_line_width = max_scan_line_width_;
          scan_line_width_at_2m = scan_line_width_at_2m_;
          scan_line_model_n = (20.-scan_line_width_at_2m)/(double)3.0;
          scan_line_model_m = 10 - 2*scan_line_model_n;
        }
      };

      /** \brief Constructor for OrganizedEdgeFromNormals */
      OrganizedEdgeFromPoints ()
        : OrganizedEdgeBase<PointT, PointLT> (),
          return_label_indices_ (true),
          use_fast_depth_discontinuity_mode_ (false),
          edge_detection_config_ ()
      {
        updateKernels();
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE);
      }

      /** \brief Destructor for OrganizedEdgeFromNormals */
      virtual
      ~OrganizedEdgeFromPoints ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities and high curvature regions) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        * \param[out] point cloud of normals to each point (optional output, if a NULL pointer is provided, no normal computations and any overhead will occur)
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices, PointCloudNPtr& normals = 0);

      /** \brief Set the flag whether a label_indices vector shall be computed in compute() */
      inline void
      setReturnLabelIndices (const bool return_label_indices)
      {
        return_label_indices_ = return_label_indices;
      }

      /** \brief Get the flag whether a label_indices vector shall be computed in compute() */
      inline bool
      getReturnLabelIndices () const
      {
        return (return_label_indices_);
      }

      /** \brief Set the flag for computing depth discontinuities in a faster way which does not distinguish between EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, or EDGELABEL_OCCLUDED but just sets depth edges to EDGELABEL_OCCLUDING */
      inline void
      setUseFastDepthDiscontinuityMode (const bool use_fast_depth_discontinuity_mode)
      {
        use_fast_depth_discontinuity_mode_ = use_fast_depth_discontinuity_mode;
        if (use_fast_depth_discontinuity_mode == false)
          this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE);
        else
          this->setEdgeType (EDGELABEL_OCCLUDING | EDGELABEL_HIGH_CURVATURE);
      }

      /** \brief Get the flag for computing depth discontinuities in a faster way which does not distinguish between EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, or EDGELABEL_OCCLUDED but just sets depth edges to EDGELABEL_OCCLUDING */
      inline bool
      getUseFastDepthDiscontinuityMode () const
      {
        return (use_fast_depth_discontinuity_mode_);
      }

      /** \brief Set the structure comprising all edge detection parameters */
      inline void
      setEdgeDetectionConfig (const EdgeDetectionConfig edge_detection_config)
      {
        edge_detection_config_ = edge_detection_config;
        updateKernels();
      }

      /** \brief Get the structure comprising all edge detection parameters */
      inline EdgeDetectionConfig
      getEdgeDetectionConfig () const
      {
        return (edge_detection_config_);
      }

    protected:
      /** \brief Perform the 3D edge detection (edges from depth discontinuities and surface discontinuities (high curvature regions))
        * \param[out] labels a PointCloud of edge labels
        * \param[out] point cloud of normals to each point (optional output, if a NULL pointer is provided, no normal computations and any overhead will occur)
        */
      void
      extractEdges (pcl::PointCloud<PointLT>& labels, PointCloudNPtr& normals = 0);

      /** \brief Computes the filter kernels needed by the algorithm according to the chosen parameters */
      void
      updateKernels ();

      /** \brief Prepares image-like data structures on the depth data */
      void
      prepareImages(pcl::PointCloud<pcl::Intensity>::Ptr& x_image, pcl::PointCloud<pcl::Intensity>::Ptr& y_image, pcl::PointCloud<pcl::Intensity>::Ptr& z_image,
                    pcl::PointCloud<pcl::Intensity>::Ptr& x_dx, pcl::PointCloud<pcl::Intensity>::Ptr& y_dy, pcl::PointCloud<pcl::Intensity>::Ptr& z_dx,
                    pcl::PointCloud<pcl::Intensity>::Ptr& z_dy);

      /** \brief Computes depth discontinuities in a fast way or copies edge detections from labels into edge (depending on use_fast_depth_discontinuity_mode_ setting) */
      void
      computeDepthDiscontinuities(pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, const pcl::PointCloud<pcl::Intensity>::Ptr& z_image,
                                  const pcl::PointCloud<pcl::Intensity>::Ptr& z_dx, const pcl::PointCloud<pcl::Intensity>::Ptr& z_dy, const pcl::PointCloud<PointLT>& labels);

      /** \brief Computes surface discontinuities */
      void
      computeSurfaceDiscontinuities(pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, const pcl::PointCloud<pcl::Intensity>::Ptr& x_dx, const pcl::PointCloud<pcl::Intensity>::Ptr& y_dy,
                                    const pcl::PointCloud<pcl::Intensity>::Ptr& z_image, const pcl::PointCloud<pcl::Intensity>::Ptr& z_dx, const pcl::PointCloud<pcl::Intensity>::Ptr& z_dy,
                                    PointCloudNPtr& normals = 0);

      /** \brief Suppresses non-maximal depth step responses (on depth discontinuities) */
      void
      nonMaximumSuppression(pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, const pcl::PointCloud<pcl::Intensity>::Ptr& dx, const pcl::PointCloud<pcl::Intensity>::Ptr& dy);

      /** \brief Creates an integral image within x-direction (i.e. line-wise, horizontally) for two source images */
      void
      computeIntegralImageX(const pcl::PointCloud<pcl::Intensity>::Ptr& srcX, pcl::PointCloud<pcl::Intensity>::Ptr& dstX,
                            const pcl::PointCloud<pcl::Intensity>::Ptr& srcZ, pcl::PointCloud<pcl::Intensity>::Ptr& dstZ);

      /** \brief Computes the horizontal distance to the next edge pixel left (.x) and right (.y) of the query point */
      void
      computeEdgeDistanceMapHorizontal(const pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, pcl::PointCloud<pcl::PointXY>::Ptr& distance_map);

      /** \brief Computes the vertical distance to the next edge pixel above (.x) and below (.y) of the query point */
      void
      computeEdgeDistanceMapVertical(const pcl::PointCloud<pcl::Intensity8u>::Ptr& edge, pcl::PointCloud<pcl::PointXY>::Ptr& distance_map);

      /** \brief Adapts the scan line around the query pixel (u,v) to edges closer than scan_line_length1 (left or above) or scan_line_length2 (right or below).
       * \param[in/out] scan_line_length1 Scan line length left or above query pixel (u,v).
       * \param[in/out] scan_line_length2 Scan line length right or below query pixel (u,v).
       * \param[in] distance_map Distances to the closest edges around query pixel (u,v) are stored in distance_map.
       * \param[in] u Query pixel coordinate u (x).
       * \param[in] v Query pixel coordinate v (y).
       * \param[in] min_scan_line_length Minimum acceptable length for scan_line_length1 or scan_line_length2.
       * \return Returns whether the total scan line length exceeds min_line_width.
       */
      bool
      adaptScanLine(int& scan_line_length_1, int& scan_line_length_2, const pcl::PointCloud<pcl::PointXY>::Ptr& distance_map, const int u, const int v, const int min_scan_line_length);

      /** \brief Adapts the scan line around the query pixel (u,v) to edges closer than scan_line_length1 (left or above) or scan_line_length2 (right or below). This is a
       * special implementation for normal computation.
       * \param[in/out] scan_line_length1 Scan line length left or above query pixel (u,v).
       * \param[in/out] scan_line_length2 Scan line length right or below query pixel (u,v).
       * \param[in] distance_map Distances to the closest edges around query pixel (u,v) are stored in distance_map.
       * \param[in] u Query pixel coordinate u (x).
       * \param[in] v Query pixel coordinate v (y).
       * \param[in] min_scan_line_length Minimum acceptable total scan line width.
       * \return Returns whether the total scan line length exceeds min_line_width.
       */
      bool
      adaptScanLineNormal(int& scan_line_length_1, int& scan_line_length_2, const pcl::PointCloud<pcl::PointXY>::Ptr& distance_map, const int u, const int v, const int min_scan_line_length);

      /** \brief Transposes a point cloud image */
      void
      transposeImage(const pcl::PointCloud<pcl::Intensity>::Ptr& src, pcl::PointCloud<pcl::Intensity>::Ptr& dst);

      /** \brief Fast implementation of computing atan2 [in rad], |error| < 0.005 rad (from https://gist.github.com/volkansalma/2972237 or http://lists.apple.com/archives/perfoptimization-dev/2005/Jan/msg00051.html) */
      float fast_atan2f_1(float y, float x)
      {
        if (x == 0.0f)
        {
          if (y > 0.0f) return pi_by_2_float_;
          if (y == 0.0f) return 0.0f;
          return -pi_by_2_float_;
        }
        float atan;
        float z = y/x;
        if (fabsf(z) < 1.0f)
        {
          atan = z/(1.0f + 0.28f*z*z);
          if (x < 0.0f)
          {
            if (y < 0.0f) return atan - pi_float_;
            return atan + pi_float_;
          }
        }
        else
        {
          atan = pi_by_2_float_ - z/(z*z + 0.28f);
          if ( y < 0.0f ) return atan - pi_float_;
        }
        return atan;
      }

      static const float pi_float_ = 3.14159265f;
      static const float pi_by_2_float_ = 1.5707963f;
      static const double pi_double_ = 3.1415926535897932384626433832795;

      /** \brief Gaussian kernel for smoothing operations */
      pcl::PointCloud<pcl::Intensity>::Ptr gaussian_kernel_;

      /** \brief Sobel x kernel for derivatives on depth */
      pcl::PointCloud<pcl::Intensity>::Ptr sobel_kernel_x_3x3_;

      /** \brief Sobel y kernel for derivatives on depth */
      pcl::PointCloud<pcl::Intensity>::Ptr sobel_kernel_y_3x3_;

      /** \brief The computation of the label_indices vector may be turned off for speed reasons by setting this variable to false (default: true) */
      bool return_label_indices_;

      /** \brief Computes depth discontinuities in a faster way which does not distinguish between EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, or EDGELABEL_OCCLUDED but just sets depth edges to EDGELABEL_OCCLUDING (default: false) */
      bool use_fast_depth_discontinuity_mode_;

      /** \brief Structure comprising all edge detection parameters */
      EdgeDetectionConfig edge_detection_config_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/organized_edge_detection.hpp>
#endif

#endif //#ifndef PCL_FEATURES_ORGANIZED_EDGE_DETECTION_H_
