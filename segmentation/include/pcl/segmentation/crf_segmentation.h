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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#ifndef PCL_CRF_SEGMENTATION_H_
#define PCL_CRF_SEGMENTATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ml/densecrf.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_label.h>

//#include <pcl/ml/densecrfORI.h>

namespace pcl
{
  /** \brief
    * 
    */
  template <typename PointT>
  class PCL_EXPORTS CrfSegmentation
  {
    public:

    //typedef boost::shared_ptr<std::vector<int> > pcl::IndicesPtr;
    

      /** \brief Constructor that sets default values for member variables. */
      CrfSegmentation ();

      /** \brief This destructor destroys the cloud...
        * 
        */
      ~CrfSegmentation ();

      /** \brief This method sets the input cloud.
        * \param[in] input_cloud input point cloud
        */
      void
      setInputCloud (typename pcl::PointCloud<PointT>::Ptr input_cloud);

      void
      setAnnotatedCloud (typename pcl::PointCloud<pcl::PointXYZRGBL>::Ptr anno_cloud);

      void
      setNormalCloud (typename pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud);


      /** \brief Set the leaf size for the voxel grid.
        * \param[in] x leaf size x-axis
        * \param[in] y leaf size y-axis
        * \param[in] z leaf size z-axis
        */
      void
      setVoxelGridLeafSize (const float x, const float y, const float z);

      void
      setNumberOfIterations (unsigned int n_iterations = 10) {n_iterations_ = n_iterations;};

      /** \brief This method simply launches the segmentation algorithm */
      void
      segmentPoints (pcl::PointCloud<pcl::PointXYZRGBL> &output);

      /** \brief Create a voxel grid to discretize the scene */
      void
      createVoxelGrid ();

      /** \brief Get the data from the voxel grid and convert it into a vector */
      void
      createDataVectorFromVoxelGrid ();


      void
      createUnaryPotentials (std::vector<float> &unary,
                             std::vector<int> &colors,
                             unsigned int n_labels);
      

      /** \brief Set the smoothness kernel parameters.
       * \param[in] sx standard deviation x
       * \param[in] sy standard deviation y
       * \param[in] sz standard deviation z
       * \param[in] w weight
        */
      void
      setSmoothnessKernelParameters (const float sx, const float sy, const float sz, const float w);

      /** \brief Set the appearanche kernel parameters.
       * \param[in] sx standard deviation x
       * \param[in] sy standard deviation y
       * \param[in] sz standard deviation z
       * \param[in] sr standard deviation red
       * \param[in] sg standard deviation green
       * \param[in] sb standard deviation blue
       * \param[in] w weight
        */
      void
      setAppearanceKernelParameters (float sx, float sy, float sz, 
                                     float sr, float sg, float sb,
                                     float w);


      void
      setSurfaceKernelParameters (float sx, float sy, float sz,
                                  float snx, float sny, float snz,
                                  float w);
      

    protected:
      /** \brief Voxel grid to discretize the scene */
      typename pcl::VoxelGrid<PointT> voxel_grid_;

      /** \brief input cloud that will be segmented. */
      typename pcl::PointCloud<PointT>::Ptr input_cloud_;
      typename pcl::PointCloud<pcl::PointXYZRGBL>::Ptr anno_cloud_;
      typename pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud_;

      /** \brief voxel grid filtered cloud. */
      typename pcl::PointCloud<PointT>::Ptr filtered_cloud_;
      typename pcl::PointCloud<pcl::PointXYZRGBL>::Ptr filtered_anno_;
      typename pcl::PointCloud<pcl::PointNormal>::Ptr filtered_normal_;

      /** \brief indices of the filtered cloud. */
      //typename pcl::VoxelGrid::IndicesPtr cloud_indices_;      

      /** \brief Voxel grid leaf size */
      Eigen::Vector4f voxel_grid_leaf_size_;

      /** \brief Voxel grid dimensions */
      Eigen::Vector3i dim_;

      /** \brief voxel grid data points
          packing order [x0y0z0, x1y0z0,x2y0z0,...,x0y1z0,x1y1z0,...,x0y0z1,x1y0z1,...]
      */
      std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > data_;

      std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > color_;

      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > normal_;

      /** \brief smoothness kernel parameters 
       * [0] = standard deviation x
       * [1] = standard deviation y
       * [2] = standard deviation z
       * [3] = weight
       */
      float smoothness_kernel_param_[4];

      /** \brief appearance kernel parameters 
       * [0] = standard deviation x
       * [1] = standard deviation y
       * [2] = standard deviation z
       * [3] = standard deviation red
       * [4] = standard deviation green
       * [5] = standard deviation blue
       * [6] = weight
       */
      float appearance_kernel_param_[7];

      float surface_kernel_param_[7];
      
      
      unsigned int n_iterations_;
      

      /** \brief Contains normals of the points that will be segmented. */
      //typename pcl::PointCloud<pcl::Normal>::Ptr normals_;

      /** \brief Stores the cloud that will be segmented. */
      //typename pcl::PointCloud<PointT>::Ptr cloud_for_segmentation_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/crf_segmentation.hpp>
#endif

#endif
