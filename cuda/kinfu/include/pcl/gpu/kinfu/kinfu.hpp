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
    /** \brief
      * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
      */
    class PCL_EXPORTS KinfuTracker
    {
      public:
        /** \brief . */
        struct RGB
        {
          unsigned char r, g, b;
        };
        typedef DeviceArray2D<RGB> View;
        typedef DeviceArray2D<unsigned short> DepthMap;

        typedef pcl::PointXYZ PointType;
        typedef pcl::Normal NormalType;

        /** \brief Constructor
          * \param[in] rows
          * \param[in] cols
          */
        KinfuTracker (int rows = 480, int cols = 640);

        /** \brief Sets Tsdf volume size for each dimention in mm
          * \param[in] fx
          * \param[in] fy
          * \param[in] cx
          * \param[in] cy
          */
        void
        setDepthIntrinsics (float fx, float fy, float cx = -1, float cy = -1);

        /** \brief Sets initial camera pose relative to volume coordiante space
          * \param[in] pose
          */
        void
        setInitalCameraPose (const Eigen::Affine3f& pose);

        /** \brief Sets Tsdf volume size for each dimention in mm
          * \param[in] volume_size
          */
        void
        setVolumeSize (const Eigen::Vector3f& volume_size);

        /** \brief Sets Tsdf trancation distance in mm. Must be greater than 2 * volume cell size
          * \param[in] distance
          */
        void
        setTrancationDistance (float distance);

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
          * \return true if can generate image for human.
          */
        bool operator() (const DepthMap& depth);

        /** \brief Returns camera pose at given time, default the last pose
          * \param[in] time
          */
        Eigen::Affine3f
        getCameraPose (int time = -1);

        /** \brief Generates image for human (?)
          * \param[out] view
          */
        void
        getImage (View& view) const;

        /** \brief Generates image for human (?)
          * \param[out] view
          * \param[in] light_source_pose
          */
        void
        getImage (View& view, const Eigen::Vector3f& light_source_pose) const;

        /** \brief Returns point cloud abserved from last camera pose
          * \param[out] cloud
          */
        void
        getLastFrameCloud (DeviceArray2D<PointType>& cloud) const;

        /** \brief Returns point cloud abserved from last camera pose
          * \param[out] normals
          */
        void
        getLastFrameNormals (DeviceArray2D<NormalType>& normals) const;

        /** \brief Generates cloud on CPU
          * \param[out] cloud
          * \param[in] connected26
          */
        void
        getCloudFromVolumeHost (PointCloud<PointType>& cloud, bool connected26 = false);

        /** \brief Generates cloud on GPU in connected6 mode only
          * \param[out] cloud_buffer
          */
        DeviceArray<PointType>
        getCloudFromVolume (DeviceArray<PointType>& cloud_buffer);

        /** \brief Computes normals as gradient of tsdf for given points
          * \param[in] cloud
          * \param[out] normals
          */
        void
        getNormalsFromVolume (const DeviceArray<PointType>& cloud, DeviceArray<PointType>& normals) const;

        /** \brief Computes normals as gradient of tsdf for given points
          * \param[in] cloud
          * \param[out] normals
          */
        void
        getNormalsFromVolume (const DeviceArray<PointType>& cloud, DeviceArray<NormalType>& normals) const;

      private:
        enum
        {
            LEVELS = 3,
            DEFAULT_VOLUME_CLOUD_BUFFER_SIZE = 10 * 1000 * 1000,
        };

        typedef DeviceArray2D<int> CorespMap;
        typedef DeviceArray2D<float> MapArr;

        typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
        typedef Eigen::Vector3f Vector3f;

        /** \brief . */
        int rows_;
        /** \brief . */
        int cols_;
        /** \brief . */
        int global_time_;

        /** \brief . */
        float fx_, fy_, cx_, cy_;

        /** \brief Size of volume in mm. */
        Vector3f volume_size_;

        /** \brief Initial camera rotation in volume coo space. */
        Matrix3frm init_Rcam_;

        /** \brief Initial camera position in volume coo space. */
        Vector3f   init_tcam_;

        /** \brief . */
        int icp_iterations_[LEVELS];
        /** \brief . */
        float  distThres_;
        /** \brief . */
        float angleThres_;
        /** \brief . */
        float tranc_dist_;

        /** \brief . */
        std::vector<DepthMap> depths_curr_;
        /** \brief . */
        std::vector<MapArr> vmaps_g_curr_;
        /** \brief . */
        std::vector<MapArr> nmaps_g_curr_;

        /** \brief . */
        std::vector<MapArr> vmaps_g_prev_;
        /** \brief . */
        std::vector<MapArr> nmaps_g_prev_;

        /** \brief . */
        std::vector<MapArr> vmaps_curr_;
        /** \brief . */
        std::vector<MapArr> nmaps_curr_;

        /** \brief . */
        std::vector<CorespMap> coresps_;

        /** \brief . */
        DeviceArray2D<int> volume_;
        /** \brief . */
        DeviceArray2D<float> depthRawScaled_;

        /** \brief . */
        DeviceArray2D<float> gbuf_;
        /** \brief . */
        DeviceArray<float> sumbuf_;

        /** \brief . */
        std::vector<Matrix3frm> rmats_;
        /** \brief . */
        std::vector<Vector3f>   tvecs_;

        /** \brief
          * \param[in] rows_arg
          * \param[in] cols_arg
          */
        void
        allocateBufffers (int rows_arg, int cols_arg);

        /** \brief
          */
        void
        reset ();
    };
  }
};

#endif /* PCL_KINFU_KINFUTRACKER_HPP_ */
