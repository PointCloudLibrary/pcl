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

#ifndef PCL_KINFU_KINFUTRACKER_HPP_
#define PCL_KINFU_KINFUTRACKER_HPP_

#include <pcl/pcl_macros.h>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vector>

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
        struct RGB
        {
          unsigned char r, g, b;
        };
        typedef DeviceArray2D<RGB> View;
        typedef DeviceArray2D<unsigned short> DepthMap;

        typedef pcl::PointXYZ PointType;
        typedef pcl::Normal NormalType;

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

        /** \brief Sets initial camera pose relative to volume coordiante space
          * \param[in] pose Initial camera pose
          */
        void
        setInitalCameraPose (const Eigen::Affine3f& pose);

        /** \brief Sets Tsdf volume size for each dimention in mm
          * \param[in] volume_size size of tsdf volume
          */
        void
        setVolumeSize (const Eigen::Vector3f& volume_size);

        /** \brief Sets Tsdf trancation distance in mm. Must be greater than 2 * volume cell size
          * \param[in] distance TSDF trancation distance 
          */
        void
        setTrancationDistance (float distance);

		/** \brief Sets truncation threshold for depth image for ICP step only! This helps 
		  *  to filter measurements that are outside tsdf volume. Pass zero to disable the truncation.
          * \param[in] max_icp_distance_ Maximal distance in mm, higher values are reset to zero (means no measurement). 
          */
        void
        setDepthTruncationForICP (unsigned short max_icp_distance = 0);

        /** \brief Sets ICP filtering parameters.
          * \param[in] distThreshold distance in mm.
          * \param[in] sineOfAngle sine of angle between normals.
          */
        void
        setIcpCorespFilteringParams (float distThreshold, float sineOfAngle);

        /** \brief Returns volume size in mm */
        Eigen::Vector3f
        getVolumeSize () const;

        /** \brief Returns cols passed to ctor */
        int
        cols ();

        /** \brief Returns rows passed to ctor */
        int
        rows ();

        /** \brief Processes next frame.
          * \param[in] Depth next frame with values in mm
          * \return true if can render 3D view.
          */
        bool operator() (const DepthMap& depth);

        /** \brief Returns camera pose at given time, default the last pose
          * \param[in] time Index of frame for which camera pose is returned.
          * \return camera pose
          */
        Eigen::Affine3f
        getCameraPose (int time = -1);

        /** \brief Renders 3D scene to display to human
          * \param[out] view output array with image
          */
        void
        getImage (View& view) const;

        /** \brief Renders 3D scene to display to human
          * \param[out] view output array with image
          * \param[in] light_source_pose Pose of light source for computing illumination
          */
        void
        getImage (View& view, const Eigen::Vector3f& light_source_pose) const;

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

        /** \brief Generates cloud on CPU
          * \param[out] cloud output array for cloud
          * \param[in] connected26 If false point cloud is extracted using 6 neighbor, otherwise 26.
          */
        void
        getCloudFromVolumeHost (PointCloud<PointType>& cloud, bool connected26 = false);

        /** \brief Generates cloud on GPU in connected6 mode only
          * \param[out] cloud_buffer buffer to store point cloud
          * \return DeviceArray with disabled reference counting that points to filled part of cloud_buffer.
          */
        DeviceArray<PointType>
        getCloudFromVolume (DeviceArray<PointType>& cloud_buffer);

        /** \brief Computes normals as gradient of tsdf for given points
          * \param[in] cloud Points for which normals are to be computed.
          * \param[out] normals Buffer for normals
          */
        void
        getNormalsFromVolume (const DeviceArray<PointType>& cloud, DeviceArray<PointType>& normals) const;

        /** \brief Computes normals as gradient of tsdf for given points
          * \param[in] cloud Points for which normals are to be computed.
          * \param[out] normals Buffer for normals
          */
        void
        getNormalsFromVolume (const DeviceArray<PointType>& cloud, DeviceArray<NormalType>& normals) const;

        /** \brief Downloads TSDF volume from GPU memory.           
          * \param[out] volume Array with tsdf values. Volume size is 512x512x512, so for voxel (x,y,z) tsdf value can be retrieved as volume[512*512*z + 512*y + x];
          */
        void
        getTsdfVolume( std::vector<float>& volume) const;

      private:
        
        enum
        {
          /** \brief Number of pyramid levels */
          LEVELS = 3,            
          DEFAULT_VOLUME_CLOUD_BUFFER_SIZE = 10 * 1000 * 1000,
        };

        /** \brief ICP Correspondences  map type */
        typedef DeviceArray2D<int> CorespMap;

        /** \brief Vertex or Normal Map type */
        typedef DeviceArray2D<float> MapArr;
        
        typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
        typedef Eigen::Vector3f Vector3f;

        /** \brief Height of input depth image. */
        int rows_;
        /** \brief Width of input depth image. */
        int cols_;
        /** \brief Frame counter */
        int global_time_;

		/** \brief Truncation threshold for depth image for ICP step */
		unsigned short max_icp_distance_;

        /** \brief Intrinsic parameters of depth camera. */
        float fx_, fy_, cx_, cy_;

        /** \brief Size of volume in mm. */
        Vector3f volume_size_;

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
        /** \brief TSDF truncation  distance in mm. Must be greater than tsdf volume cell size */
        float tranc_dist_;

        /** \brief Array of dpeth pyramids. */
        std::vector<DepthMap> depths_curr_;
        /** \brief Array of pyramids of vertex maps for current frame in global coordinate space. */
        std::vector<MapArr> vmaps_g_curr_;
        /** \brief Array of pyramids of normal maps for current frame in global coordinate space. */
        std::vector<MapArr> nmaps_g_curr_;

        /** \brief Array of pyramids of vertex maps for previous frame in global coordinate space. */
        std::vector<MapArr> vmaps_g_prev_;
        /** \brief Array of pyramids of normal maps for previous frame in global coordinate space. */
        std::vector<MapArr> nmaps_g_prev_;

        /** \brief Array of pyramids of vertex maps for current frame in current coordinate space. */
        std::vector<MapArr> vmaps_curr_;
        /** \brief Array of pyramids of vertex maps for current frame in current coordinate space. */
        std::vector<MapArr> nmaps_curr_;

        /** \brief Array of buffers with ICP correspondences for each pyramid level. */
        std::vector<CorespMap> coresps_;

        /** \brief TSDF volume storage */
        DeviceArray2D<int> volume_;
        /** \brief Buffer for storing scaled depth image */
        DeviceArray2D<float> depthRawScaled_;

        /** \brief Temporary buffer for ICP */
        DeviceArray2D<float> gbuf_;
        /** \brief Buffer to store MLS matrix. */
        DeviceArray<float> sumbuf_;

        /** \brief Array of camera rotation matrices for each moment of time. */
        std::vector<Matrix3frm> rmats_;
        /** \brief Array of camera translations for each moment of time. */
        std::vector<Vector3f>   tvecs_;

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
    };
  }
};

#endif /* PCL_KINFU_KINFUTRACKER_HPP_ */
