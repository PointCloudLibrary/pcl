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

#include <sys/time.h>
#include <stdlib.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
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

#define PI_FLOAT 3.14159265f
#define PI_BY_2_FLOAT 1.5707963f
#define PI_DOUBLE 3.1415926535897932384626433832795

  /** \brief Compilation of all relevant edge detection algorithm parameters applicable to OrganizedEdgeFromPoints and OrganizedEdgeFromRGBPoints */
  struct EdgeDetectionConfig
  {
      /** \brief Noise suppression mode on differentiated images */
      enum NoiseReductionMode
      {
        NONE, GAUSSIAN
      };   //, BILATERAL }; bilateral not yet implemented

      /** \brief Noise suppression mode on differentiated images */
      NoiseReductionMode noise_reduction_mode_;

      /** \brief Kernel size of noise reduction filter, usually 3, 5 or 7 */
      int noise_reduction_kernel_size_;

      /** \brief Threshold for depth dependent depth edge (step) verification, corresponds to minimum depth step in [m] at 1m distance (default ~ 0.01 - 0.02) */
      float depth_step_factor_;

      /** \brief Minimum angle of surrounding surfaces around a surface discontinuity (edge), in [deg] */
      double min_detectable_edge_angle_;

      /** \brief Use fixed width scan line at certain depth (false) or adapt scan line width to surrounding edges, i.e. not crossing edges (true, default). If set to false no surface edge will be computed when the scan line crosses depth edges. If true, the scan line is adapted close to depth edges which yields more accurate results (though at little lower speed). */
      bool use_adaptive_scan_line_;

      /** \brief Minimum scan line width used for surface slope approximation, in [pixels] (typically around 5pix) */
      int min_scan_line_width_;

      /** \brief Maximum scan line width used for surface slope approximation, in [pixels] (typically between 20 and 40pix) */
      int max_scan_line_width_;

      /** \brief scan_line_width_at_2m_ is the length of the scan line left and right of the query pixel at 2m distance (using linear model with scan line width at 0.5m of min_scan_line_width_ pixels) */
      int scan_line_width_at_2m_;

      /** \brief Computed value from scan_line_width_at_2m_ (slope in linear model) */
      double scan_line_model_m_;

      /** \brief Computed value from scan_line_width_at_2m_ (offset in linear model) */
      double scan_line_model_n_;

      /** \brief Standard constructor with default parameters */
      EdgeDetectionConfig ()
      {
        noise_reduction_mode_ = GAUSSIAN;
        noise_reduction_kernel_size_ = 3;
        depth_step_factor_ = 0.01f;
        min_detectable_edge_angle_ = 45;
        use_adaptive_scan_line_ = true;
        min_scan_line_width_ = 5;
        max_scan_line_width_ = 20;
        scan_line_width_at_2m_ = 15;
        updateScanLineModel ();
      }

      /** \brief Constructor with user defined parameters */
      EdgeDetectionConfig (const NoiseReductionMode noise_reduction_mode,
                           const int noise_reduction_kernel_size,
                           const float depth_step_factor,
                           const double min_detectable_edge_angle,
                           const bool use_adaptive_scan_line,
                           const int min_scan_line_width,
                           const int max_scan_line_width,
                           const int scan_line_width_at_2m)
      {
        noise_reduction_mode_ = noise_reduction_mode;
        noise_reduction_kernel_size_ = noise_reduction_kernel_size;
        depth_step_factor_ = depth_step_factor;
        min_detectable_edge_angle_ = min_detectable_edge_angle;
        use_adaptive_scan_line_ = use_adaptive_scan_line;
        min_scan_line_width_ = min_scan_line_width;
        max_scan_line_width_ = max_scan_line_width;
        scan_line_width_at_2m_ = scan_line_width_at_2m;
        updateScanLineModel ();
      }

      void
      updateScanLineModel ()
      {
        scan_line_model_n_ = (4. * min_scan_line_width_ - scan_line_width_at_2m_) / (double) 3.0;
        scan_line_model_m_ = 2 * (min_scan_line_width_ - scan_line_model_n_);
      }
  };

  /** \brief OrganizedEdgeFromPoints and OrganizedEdgeFromRGBPoints efficiently find 3D depth edges and surface discontinuities (EDGELABEL_HIGH_CURVATURE)
   * directly from organized point cloud data without needing any input on pre-computed normals. Given an organized point cloud, they will output a PointCloud
   * of edge labels, a vector of PointIndices, and optionally a point cloud of efficiently computed surface normals.
   * That algorithm uses specialized integral images on local slope for discovering surface edges which makes it very fast.
   * OrganizedEdgeFromPoints accepts PCL_XYZ_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, and EDGELABEL_HIGH_CURVATURE.
   * OrganizedEdgeFromRGBPoints accepts PCL_RGB_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, EDGELABEL_HIGH_CURVATURE, and EDGELABEL_RGB_CANNY.
   *
   * \note If you use this code in any academic work, please cite:
   *   - Richard Bormann, Joshua Hampp, Martin Haegele, Markus Vincze.
   *     Fast and Accurate Normal Estimation by Efficient 3d Edge Detection.
   *     In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, Sept 28 - Oct 03, 2015, Congress Center Hamburg, Hamburg, Germany.
   *
   * \author Richard Bormann
   * \ingroup features
   */
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

      /** \brief Constructor for OrganizedEdgeFromPoints */
      OrganizedEdgeFromPoints () :
          OrganizedEdgeBase<PointT, PointLT> (),
          return_label_indices_ (true),
          use_fast_depth_discontinuity_mode_ (false),
          edge_detection_config_ ()
      {
        updateKernels ();
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE);
      }

      /** \brief Destructor for OrganizedEdgeFromPoints */
      virtual
      ~OrganizedEdgeFromPoints ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities and high curvature regions) and assign point indices for each edge label
       * \param[out] labels a PointCloud of edge labels
       * \param[out] label_indices a vector of PointIndices corresponding to each edge label (only if return_label_indices_ is true)
       * \param[out] normals (Optional) point cloud of efficiently computed normals to each point (optional output, if a NULL pointer is provided, no normal computations and no overhead will occur). Normal computation is very efficient and edge-aware, i.e. the support regions for normal estimation do not extend over detected edges.
       */
      void
      compute (pcl::PointCloud<PointLT>& labels,
               std::vector<pcl::PointIndices>& label_indices,
               PointCloudNPtr& normals = 0);

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
        int edge_type = this->getEdgeType () | (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE);
        if (use_fast_depth_discontinuity_mode == true)
          edge_type -= (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDED);   // removes unused labels from the list
        this->setEdgeType (edge_type);
      }

      /** \brief Get the flag for computing depth discontinuities in a faster way which does not distinguish between EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, or EDGELABEL_OCCLUDED but just sets depth edges to EDGELABEL_OCCLUDING */
      inline bool
      getUseFastDepthDiscontinuityMode () const
      {
        return (use_fast_depth_discontinuity_mode_);
      }

      /** \brief Set the structure comprising all edge detection parameters */
      inline void
      setEdgeDetectionConfig (const EdgeDetectionConfig& edge_detection_config)
      {
        edge_detection_config_ = edge_detection_config;
        edge_detection_config_.updateScanLineModel ();
        updateKernels ();
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
      extractEdges (pcl::PointCloud<PointLT>& labels,
                    PointCloudNPtr& normals = 0);

      /** \brief Computes the filter kernels needed by the algorithm according to the chosen parameters */
      void
      updateKernels ();

      /** \brief Prepares image-like data structures on the depth data */
      void
      prepareImages (pcl::PointCloud<pcl::Intensity>::Ptr& x_image,
                     pcl::PointCloud<pcl::Intensity>::Ptr& y_image,
                     pcl::PointCloud<pcl::Intensity>::Ptr& z_image,
                     pcl::PointCloud<pcl::Intensity>::Ptr& x_dx,
                     pcl::PointCloud<pcl::Intensity>::Ptr& y_dy,
                     pcl::PointCloud<pcl::Intensity>::Ptr& z_dx,
                     pcl::PointCloud<pcl::Intensity>::Ptr& z_dy);

      /** \brief Filters a single channel point cloud with a separable image filter of kernel size 3x3. Pixels beyond the image border are assumed to be zero.
       * \param[in] v1 The parameters v1, v2, v3 define the vertical components of the separable filter, e.g. 0.125, 0.25, 0.125 for a Sobel filter in x-direction.
       * \param[in] h1 The parameters h1, h2, h3 define the horizontal components of the separable filter, e.g. -1, 0, 1 for a Sobel filter in x-direction.
       * */
      void
      filterSeparable33 (pcl::PointCloud<pcl::Intensity>::Ptr& image,
                         pcl::PointCloud<pcl::Intensity>::Ptr& result,
                         const float v1,
                         const float v2,
                         const float v3,
                         const float h1,
                         const float h2,
                         const float h3);

      /** \brief Filters a single channel point cloud with a separable image filter of arbitrary kernel size. Pixels beyond the image border are assumed to be zero.
       * \param[in] vertical_coefficients The parameters v1, v2, v3 define the vertical components of the separable filter, e.g. 0.125, 0.25, 0.125 for a Sobel filter in x-direction.
       * \param[in] horizontal_coefficients The parameters h1, h2, h3 define the horizontal components of the separable filter, e.g. -1, 0, 1 for a Sobel filter in x-direction.
       * */
      void
      filterSeparable (pcl::PointCloud<pcl::Intensity>::Ptr& image,
                       pcl::PointCloud<pcl::Intensity>::Ptr& result,
                       const std::vector<float> vertical_coefficients,
                       const std::vector<float> horizontal_coefficients);

      /** \brief Computes depth discontinuities in a fast way or copies edge detections from labels into edge (depending on use_fast_depth_discontinuity_mode_ setting) */
      void
      computeDepthDiscontinuities (pcl::PointCloud<pcl::Intensity8u>::Ptr& edge,
                                   const pcl::PointCloud<pcl::Intensity>::Ptr& z_image,
                                   const pcl::PointCloud<pcl::Intensity>::Ptr& z_dx,
                                   const pcl::PointCloud<pcl::Intensity>::Ptr& z_dy,
                                   const pcl::PointCloud<PointLT>& labels);

      /** \brief Computes surface discontinuities */
      void
      computeSurfaceDiscontinuities (pcl::PointCloud<pcl::Intensity8u>::Ptr& edge,
                                     const pcl::PointCloud<pcl::Intensity>::Ptr& x_dx,
                                     const pcl::PointCloud<pcl::Intensity>::Ptr& y_dy,
                                     const pcl::PointCloud<pcl::Intensity>::Ptr& z_image,
                                     const pcl::PointCloud<pcl::Intensity>::Ptr& z_dx,
                                     const pcl::PointCloud<pcl::Intensity>::Ptr& z_dy,
                                     PointCloudNPtr& normals = 0);

      /** \brief Suppresses non-maximal depth step responses (on depth discontinuities) */
      void
      nonMaximumSuppression (pcl::PointCloud<pcl::Intensity8u>::Ptr& edge,
                             const pcl::PointCloud<pcl::Intensity>::Ptr& dx,
                             const pcl::PointCloud<pcl::Intensity>::Ptr& dy);

      /** \brief Creates an integral image within x-direction (i.e. line-wise, horizontally) for two source images */
      void
      computeIntegralImageX (const pcl::PointCloud<pcl::Intensity>::Ptr& srcX,
                             pcl::PointCloud<pcl::Intensity>::Ptr& dstX,
                             const pcl::PointCloud<pcl::Intensity>::Ptr& srcZ,
                             pcl::PointCloud<pcl::Intensity>::Ptr& dstZ);

      /** \brief Computes the horizontal distance to the next edge pixel left (.x) and right (.y) of the query point */
      void
      computeEdgeDistanceMapHorizontal (const pcl::PointCloud<pcl::Intensity8u>::Ptr& edge,
                                        std::vector<std::pair<int, int> >& distance_map);

      /** \brief Computes the vertical distance to the next edge pixel above (.x) and below (.y) of the query point */
      void
      computeEdgeDistanceMapVertical (const pcl::PointCloud<pcl::Intensity8u>::Ptr& edge,
                                      std::vector<std::pair<int, int> >& distance_map);

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
      adaptScanLine (int& scan_line_length_1,
                     int& scan_line_length_2,
                     const std::pair<int, int>& distance_to_edge,
                     const int min_scan_line_length);

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
      adaptScanLineNormal (int& scan_line_length_1,
                           int& scan_line_length_2,
                           const std::pair<int, int>& distance_to_edge,
                           const int min_scan_line_length);

      /** \brief Transposes a single channel image point cloud */
      void
      transposeImage (const pcl::PointCloud<pcl::Intensity>::Ptr& src,
                      pcl::PointCloud<pcl::Intensity>::Ptr& dst);

      /** \brief Fast implementation of computing atan2 [in rad], |error| < 0.005 rad
       * (from https://gist.github.com/volkansalma/2972237 or http://lists.apple.com/archives/perfoptimization-dev/2005/Jan/msg00051.html) */
      float
      fast_atan2f_1 (float y,
                     float x);

      /** \brief Sets the labels point cloud accordingly */
      void
      setLabels (const pcl::PointCloud<pcl::Intensity8u>::Ptr& edge,
                 pcl::PointCloud<PointLT>& labels);

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

  template <typename PointT, typename PointNT, typename PointLT>
  class OrganizedEdgeFromRGBPoints : public OrganizedEdgeFromRGB<PointT, PointLT>, public OrganizedEdgeFromPoints<PointT, PointNT, PointLT>
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
      using OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::gaussian_kernel_;
      using OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::sobel_kernel_x_3x3_;
      using OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::sobel_kernel_y_3x3_;
      using OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::return_label_indices_;
      using OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::use_fast_depth_discontinuity_mode_;
      using OrganizedEdgeFromPoints<PointT, PointNT, PointLT>::edge_detection_config_;
      using OrganizedEdgeFromRGB<PointT, PointLT>::th_rgb_canny_low_;
      using OrganizedEdgeFromRGB<PointT, PointLT>::th_rgb_canny_high_;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_NAN_BOUNDARY;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDING;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDED;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_HIGH_CURVATURE;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_RGB_CANNY;

      /** \brief Constructor for OrganizedEdgeFromRGBPoints */
      OrganizedEdgeFromRGBPoints () :
          OrganizedEdgeBase<PointT, PointLT> (),
          OrganizedEdgeFromRGB<PointT, PointLT> (),
          OrganizedEdgeFromPoints<PointT, PointNT, PointLT> ()
      {
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_RGB_CANNY | EDGELABEL_HIGH_CURVATURE);
      }

      /** \brief Destructor for OrganizedEdgeFromRGBPoints */
      virtual
      ~OrganizedEdgeFromRGBPoints ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities, RGB Canny edge, and high curvature regions) and assign point indices for each edge label
       * \param[out] labels a PointCloud of edge labels
       * \param[out] label_indices a vector of PointIndices corresponding to each edge label
       * \param[out] normals point cloud of efficiently computed normals to each point (optional output, if a NULL pointer is provided, no normal computations and no overhead will occur)
       */
      void
      compute (pcl::PointCloud<PointLT>& labels,
               std::vector<pcl::PointIndices>& label_indices,
               PointCloudNPtr& normals = 0);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/organized_edge_detection.hpp>
#endif

#endif //#ifndef PCL_FEATURES_ORGANIZED_EDGE_DETECTION_H_
