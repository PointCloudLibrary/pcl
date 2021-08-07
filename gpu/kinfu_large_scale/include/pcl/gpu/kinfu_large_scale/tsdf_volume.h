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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vector>

#include <pcl/gpu/kinfu_large_scale/tsdf_buffer.h>

#include <pcl/gpu/kinfu_large_scale/point_intensity.h>


namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      /** \brief TsdfVolume class
        * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
        */
      class PCL_EXPORTS TsdfVolume
      {
      public:
        using Ptr = shared_ptr<TsdfVolume>;
        using ConstPtr = shared_ptr<const TsdfVolume>;

        /** \brief Supported Point Types */
        using PointType = PointXYZ;
        using NormalType = Normal;

        /** \brief Structure storing voxel grid resolution, volume size (in mm) and element_size of data stored on host*/
        struct Header
        {
          Eigen::Vector3i resolution;
          Eigen::Vector3f volume_size;
          int volume_element_size, weights_element_size;

          Header ()
            : resolution (0,0,0),
              volume_size (0,0,0),
              volume_element_size (sizeof(float)),
              weights_element_size (sizeof(short))
          {};

          Header (const Eigen::Vector3i &res, const Eigen::Vector3f &size)
            : resolution (res),
              volume_size (size),
              volume_element_size (sizeof(float)),
              weights_element_size (sizeof(short))
          {};

          /** \brief Get the size of data stored on host*/
          inline std::size_t
          getVolumeSize () const { return resolution[0] * resolution[1] * resolution[2]; };

          friend inline std::ostream&
          operator << (std::ostream& os, const Header& h)
          {
            os << "(resolution = " << h.resolution.transpose() << ", volume size = " << h.volume_size.transpose() << ")";
            return (os);
          }

          public:
            PCL_MAKE_ALIGNED_OPERATOR_NEW
        };

        /** \brief Default buffer size for fetching cloud. It limits max number of points that can be extracted */
        enum { DEFAULT_CLOUD_BUFFER_SIZE = 10 * 1000 * 1000 };

        /** \brief Constructor
          * \param[in] resolution volume resolution
          */
        TsdfVolume (const Eigen::Vector3i& resolution);

        /** \brief Sets Tsdf volume size for each dimension
          * \param[in] size size of tsdf volume in meters
          */
        void
        setSize (const Eigen::Vector3f& size);

        /** \brief Sets Tsdf truncation distance. Must be greater than 2 * volume_voxel_size
          * \param[in] distance TSDF truncation distance
          */
        void
        setTsdfTruncDist (float distance);

        /** \brief Returns tsdf volume container that point to data in GPU memory */
        DeviceArray2D<int>
        data () const;

        /** \brief Returns volume size in meters */
        const Eigen::Vector3f&
        getSize () const;

        /** \brief Returns volume resolution */
        const Eigen::Vector3i&
        getResolution() const;

        /** \brief Returns volume voxel size in meters */
        const Eigen::Vector3f
        getVoxelSize () const;

        /** \brief Returns tsdf truncation distance in meters */
        float
        getTsdfTruncDist () const;

        /** \brief Resets tsdf volume data to uninitialized state */
        void
        reset ();

        /** \brief Generates cloud using CPU (downloads volumetric representation to CPU memory)
          * \param[out] cloud output array for cloud
          * \param[in] connected26 If false point cloud is extracted using 6 neighbor, otherwise 26.
          */
        void
        fetchCloudHost (PointCloud<PointType>& cloud, bool connected26 = false) const;

        /** \brief Generates cloud using CPU (downloads volumetric representation to CPU memory)
          * \param[out] cloud output array for cloud
          * \param[in] connected26 If false point cloud is extracted using 6 neighbor, otherwise 26.
          */
        void
        fetchCloudHost (PointCloud<PointXYZI>& cloud, bool connected26 = false) const;

        /** \brief Generates cloud using GPU in connected6 mode only
          * \param[out] cloud_buffer buffer to store point cloud
          * \return DeviceArray with disabled reference counting that points to filled part of cloud_buffer.
          */
        DeviceArray<PointType>
        fetchCloud (DeviceArray<PointType>& cloud_buffer) const;

          /** \brief Push a point cloud of previously scanned tsdf slice to the TSDF volume
            * \param[in] existing_data_cloud point cloud pointer to the existing data. This data will be pushed to the TSDf volume. The points with indices outside the range [0 ... VOLUME_X - 1][0 ... VOLUME_Y - 1][0 ... VOLUME_Z - 1] will not be added.
            * \param buffer
            */
        void
        pushSlice (const PointCloud<PointXYZI>::Ptr existing_data_cloud, const tsdf_buffer* buffer) const;

        /** \brief Generates cloud using GPU in connected6 mode only
          * \param[out] cloud_buffer_xyz buffer to store point cloud
          * \param cloud_buffer_intensity
          * \param[in] buffer Pointer to the buffer struct that contains information about memory addresses of the tsdf volume memory block, which are used for the cyclic buffer.
          * \param[in] shiftX Offset in indices.
          * \param[in] shiftY Offset in indices.
          * \param[in] shiftZ Offset in indices.
          * \return DeviceArray with disabled reference counting that points to filled part of cloud_buffer.
          */
        std::size_t
        fetchSliceAsCloud (DeviceArray<PointType>& cloud_buffer_xyz, DeviceArray<float>& cloud_buffer_intensity, const tsdf_buffer* buffer, int shiftX, int shiftY, int shiftZ ) const;

        /** \brief Computes normals as gradient of tsdf for given points
          * \param[in] cloud Points where normals are computed.
          * \param[out] normals array for normals
          */

        void
        fetchNormals (const DeviceArray<PointType>& cloud, DeviceArray<PointType>& normals) const;

        /** \brief Computes normals as gradient of tsdf for given points
          * \param[in] cloud Points where normals are computed.
          * \param[out] normals array for normals
          */
        void
        fetchNormals(const DeviceArray<PointType>& cloud, DeviceArray<NormalType>& normals) const;

        /** \brief Downloads tsdf volume from GPU memory.
          * \param[out] tsdf Array with tsdf values. if volume resolution is 512x512x512, so for voxel (x,y,z) tsdf value can be retrieved as volume[512*512*z + 512*y + x];
          */
        void
        downloadTsdf (std::vector<float>& tsdf) const;

        /** \brief Downloads tsdf volume from GPU memory to local CPU buffer*/
        void
        downloadTsdfLocal () const;

        /** \brief Downloads TSDF volume and according voxel weights from GPU memory
          * \param[out] tsdf Array with tsdf values. if volume resolution is 512x512x512, so for voxel (x,y,z) tsdf value can be retrieved as volume[512*512*z + 512*y + x];
          * \param[out] weights Array with tsdf voxel weights. Same size and access index as for tsdf. A weight of 0 indicates the voxel was never used.
          */
        void
        downloadTsdfAndWeights (std::vector<float>& tsdf, std::vector<short>& weights) const;

        /** \brief Downloads TSDF volume and according voxel weights from GPU memory to local CPU buffers*/
        void
        downloadTsdfAndWeightsLocal () const;

        /** \brief Releases tsdf buffer on GPU */
        void releaseVolume () {volume_.release();}

        void print_warn(const char* arg1, std::size_t size);

        /** \brief Set the header for data stored on host directly. Useful if directly writing into volume and weights */
        inline void
        setHeader (const Eigen::Vector3i& resolution, const Eigen::Vector3f& volume_size) {
          header_ = Header (resolution, volume_size);
          if (volume_host_->size() != this->size())
            pcl::console::print_warn ("[TSDFVolume::setHeader] Header volume size (%d) doesn't fit underlying data size (%d)", volume_host_->size(), size());
        }

        /** \brief Returns overall number of voxels in grid stored on host */
        inline std::size_t
        size () const {
          return header_.getVolumeSize ();
        }

        /** \brief Converts volume stored on host to cloud of TSDF values
          * \param[out] cloud - the output point cloud
          * \param[in] step - the decimation step to use
          */
        void
        convertToTsdfCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                            const unsigned step = 2) const;

        /** \brief Returns the voxel grid resolution */
        inline const Eigen::Vector3i &
        gridResolution () const { return header_.resolution; };

        /** \brief Saves local volume buffer to file */
        bool
        save (const std::string &filename = "tsdf_volume.dat", bool binary = true) const;

        /** \brief Loads local volume from file */
        bool
        load (const std::string &filename, bool binary = true);

      private:
        /** \brief tsdf volume size in meters */
        Eigen::Vector3f size_;

        /** \brief tsdf volume resolution */
        Eigen::Vector3i resolution_;

        /** \brief tsdf volume data container */
        DeviceArray2D<int> volume_;

        /** \brief tsdf truncation distance */
        float tranc_dist_;

        // The following member are resulting from the merge of TSDFVolume with TsdfVolume class.
        using VolumePtr = shared_ptr<std::vector<float> >;
        using WeightsPtr = shared_ptr<std::vector<short> >;

        Header header_;
        VolumePtr volume_host_;
        WeightsPtr weights_host_;

      public:
        PCL_MAKE_ALIGNED_OPERATOR_NEW
      };
    }
  }
}
