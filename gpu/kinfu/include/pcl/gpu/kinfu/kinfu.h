/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/kinfu/pixel_rgb.h>
#include <pcl/gpu/kinfu/tsdf_volume.h>
#include <pcl/gpu/kinfu/color_volume.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vector>

// Focal lengths of RGB camera
#define KINFU_DEFAULT_RGB_FOCAL_X 525.f
#define KINFU_DEFAULT_RGB_FOCAL_Y 525.f

// Focal lengths of depth (i.e. NIR) camera
#define KINFU_DEFAULT_DEPTH_FOCAL_X 585.f
#define KINFU_DEFAULT_DEPTH_FOCAL_Y 585.f

namespace pcl
{
  namespace gpu
  {
    /** \brief KinfuTracker class encapsulates implementation of Microsoft Kinect Fusion algorithm
      * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
      */
    class PCL_EXPORTS KinfuTracker
    {
      public:
        /** \brief Pixel type for rendered image. */
        using PixelRGB = pcl::gpu::PixelRGB;

        using View = DeviceArray2D<PixelRGB>;
        using DepthMap = DeviceArray2D<unsigned short>;

        using PointType = pcl::PointXYZ;
        using NormalType = pcl::Normal;

        /** \brief Constructor
          * \param[in] rows height of depth image
          * \param[in] cols width of depth image
          */
        KinfuTracker (int rows = 480, int cols = 640);

        /** \brief Sets Depth camera intrinsics
          * \param[in] fx focal length x 
          * \param[in] fy focal length y
          * \param[in] cx principal point x
          * \param[in] cy principal point y
          */
        void
        setDepthIntrinsics (float fx, float fy, float cx = -1, float cy = -1);
        
        /** \brief Get Depth camera intrinsics
          * \param[out] fx focal length x 
          * \param[out] fy focal length y
          * \param[out] cx principal point x
          * \param[out] cy principal point y
          */
        void
        getDepthIntrinsics (float& fx, float& fy, float& cx, float& cy) const;
        

        /** \brief Sets initial camera pose relative to volume coordinate space
          * \param[in] pose Initial camera pose
          */
        void
        setInitalCameraPose (const Eigen::Affine3f& pose);
                        
		/** \brief Sets truncation threshold for depth image for ICP step only! This helps 
		  *  to filter measurements that are outside tsdf volume. Pass zero to disable the truncation.
          * \param[in] max_icp_distance Maximal distance, higher values are reset to zero (means no measurement). 
          */
        void
        setDepthTruncationForICP (float max_icp_distance = 0.f);

        /** \brief Sets ICP filtering parameters.
          * \param[in] distThreshold distance.
          * \param[in] sineOfAngle sine of angle between normals.
          */
        void
        setIcpCorespFilteringParams (float distThreshold, float sineOfAngle);
        
        /** \brief Sets integration threshold. TSDF volume is integrated iff a camera movement metric exceedes the threshold value. 
          * The metric represents the following: M = (rodrigues(Rotation).norm() + alpha*translation.norm())/2, where alpha = 1.f (hardcoded constant)
          * \param[in] threshold a value to compare with the metric. Suitable values are ~0.001          
          */
        void
        setCameraMovementThreshold(float threshold = 0.001f);

        /** \brief Performs initialization for color integration. Must be called before calling color integration. 
          * \param[in] max_weight max weighe for color integration. -1 means default weight.
          */
        void
        initColorIntegration(int max_weight = -1);

        /** \brief Returns cols passed to ctor */
        int
        cols ();

        /** \brief Returns rows passed to ctor */
        int
        rows ();

        /** \brief Processes next frame.
          * \param[in] depth next frame with values in millimeters
          * \param hint
          * \return true if can render 3D view.
          */
        bool operator() (const DepthMap& depth, Eigen::Affine3f* hint=nullptr);

        /** \brief Processes next frame (both depth and color integration). Please call initColorIntegration before invpoking this.
          * \param[in] depth next depth frame with values in millimeters
          * \param[in] colors next RGB frame
          * \return true if can render 3D view.
          */
        bool operator() (const DepthMap& depth, const View& colors);

        /** \brief Returns camera pose at given time, default the last pose
          * \param[in] time Index of frame for which camera pose is returned.
          * \return camera pose
          */
        Eigen::Affine3f
        getCameraPose (int time = -1) const;

        /** \brief Returns number of poses including initial */
        std::size_t
        getNumberOfPoses () const;

        /** \brief Returns TSDF volume storage */
        const TsdfVolume& volume() const;

        /** \brief Returns TSDF volume storage */
        TsdfVolume& volume();

        /** \brief Returns color volume storage */
        const ColorVolume& colorVolume() const;

        /** \brief Returns color volume storage */
        ColorVolume& colorVolume();
        
        /** \brief Renders 3D scene to display to human
          * \param[out] view output array with image
          */
        void
        getImage (View& view) const;
        
        /** \brief Returns point cloud abserved from last camera pose
          * \param[out] cloud output array for points
          */
        void
        getLastFrameCloud (DeviceArray2D<PointType>& cloud) const;

        /** \brief Returns point cloud abserved from last camera pose
          * \param[out] normals output array for normals
          */
        void
        getLastFrameNormals (DeviceArray2D<NormalType>& normals) const;

        /** \brief Disables ICP forever */
        void disableIcp();

      private:
        
        /** \brief Number of pyramid levels */
        enum { LEVELS = 3 };

        /** \brief ICP Correspondences  map type */
        using CorespMap = DeviceArray2D<int>;

        /** \brief Vertex or Normal Map type */
        using MapArr = DeviceArray2D<float>;
        
        using Matrix3frm = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>;
        using Vector3f = Eigen::Vector3f;

        /** \brief Height of input depth image. */
        int rows_;
        /** \brief Width of input depth image. */
        int cols_;
        /** \brief Frame counter */
        int global_time_;

        /** \brief Truncation threshold for depth image for ICP step */
        float max_icp_distance_;

        /** \brief Intrinsic parameters of depth camera. */
        float fx_, fy_, cx_, cy_;

        /** \brief Tsdf volume container. */
        TsdfVolume::Ptr tsdf_volume_;
        ColorVolume::Ptr color_volume_;
                
        /** \brief Initial camera rotation in volume coo space. */
        Matrix3frm init_Rcam_;

        /** \brief Initial camera position in volume coo space. */
        Vector3f   init_tcam_;

        /** \brief array with IPC iteration numbers for each pyramid level */
        int icp_iterations_[LEVELS];
        /** \brief distance threshold in correspondences filtering */
        float  distThres_;
        /** \brief angle threshold in correspondences filtering. Represents max sine of angle between normals. */
        float angleThres_;
        
        /** \brief Depth pyramid. */
        std::vector<DepthMap> depths_curr_;
        /** \brief Vertex maps pyramid for current frame in global coordinate space. */
        std::vector<MapArr> vmaps_g_curr_;
        /** \brief Normal maps pyramid for current frame in global coordinate space. */
        std::vector<MapArr> nmaps_g_curr_;

        /** \brief Vertex maps pyramid for previous frame in global coordinate space. */
        std::vector<MapArr> vmaps_g_prev_;
        /** \brief Normal maps pyramid for previous frame in global coordinate space. */
        std::vector<MapArr> nmaps_g_prev_;
                
        /** \brief Vertex maps pyramid for current frame in current coordinate space. */
        std::vector<MapArr> vmaps_curr_;
        /** \brief Normal maps pyramid for current frame in current coordinate space. */
        std::vector<MapArr> nmaps_curr_;

        /** \brief Array of buffers with ICP correspondences for each pyramid level. */
        std::vector<CorespMap> coresps_;
        
        /** \brief Buffer for storing scaled depth image */
        DeviceArray2D<float> depthRawScaled_;
        
        /** \brief Temporary buffer for ICP */
        DeviceArray2D<double> gbuf_;
        /** \brief Buffer to store MLS matrix. */
        DeviceArray<double> sumbuf_;

        /** \brief Array of camera rotation matrices for each moment of time. */
        std::vector<Matrix3frm> rmats_;
        
        /** \brief Array of camera translations for each moment of time. */
        std::vector<Vector3f> tvecs_;

        /** \brief Camera movement threshold. TSDF is integrated iff a camera movement metric exceedes some value. */
        float integration_metric_threshold_;

        /** \brief ICP step is completely disabled. Only integration now. */
        bool disable_icp_;
        
        /** \brief Allocates all GPU internal buffers.
          * \param[in] rows_arg
          * \param[in] cols_arg          
          */
        void
        allocateBufffers (int rows_arg, int cols_arg);

        /** \brief Performs the tracker reset to initial  state. It's used if case of camera tracking fail.
          */
        void
        reset ();

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW

    };
  }
};
