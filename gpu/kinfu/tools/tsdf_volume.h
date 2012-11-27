/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  $Id: tsdf_volume.h 6459 2012-07-18 07:50:37Z dpb $
 */


#ifndef TSDF_VOLUME_H_
#define TSDF_VOLUME_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>


#define DEFAULT_GRID_RES_X 512  // pcl::device::VOLUME_X ( and _Y, _Z)
#define DEFAULT_GRID_RES_Y 512
#define DEFAULT_GRID_RES_Z 512

#define DEFAULT_VOLUME_SIZE_X 3000
#define DEFAULT_VOLUME_SIZE_Y 3000
#define DEFAULT_VOLUME_SIZE_Z 3000


namespace pcl
{
  template <typename VoxelT, typename WeightT>
  class TSDFVolume
  {
  public:

    typedef boost::shared_ptr<TSDFVolume<VoxelT, WeightT> > Ptr;
    typedef boost::shared_ptr<const TSDFVolume<VoxelT, WeightT> > ConstPtr;

    // typedef Eigen::Matrix<VoxelT, Eigen::Dynamic, 1> VoxelTVec;
    typedef Eigen::VectorXf VoxelTVec;

    /** \brief Structure storing voxel grid resolution, volume size (in mm) and element_size of stored data */
    struct Header
    {
      Eigen::Vector3i resolution;
      Eigen::Vector3f volume_size;
      int volume_element_size, weights_element_size;

      Header ()
        : resolution (0,0,0),
          volume_size (0,0,0),
          volume_element_size (sizeof(VoxelT)),
          weights_element_size (sizeof(WeightT))
      {};

      Header (const Eigen::Vector3i &res, const Eigen::Vector3f &size)
        : resolution (res),
          volume_size (size),
          volume_element_size (sizeof(VoxelT)),
          weights_element_size (sizeof(WeightT))
      {};

      inline size_t
      getVolumeSize () const { return resolution[0] * resolution[1] * resolution[2]; };

      friend inline std::ostream&
      operator << (std::ostream& os, const Header& h)
      {
        os << "(resolution = " << h.resolution.transpose() << ", volume size = " << h.volume_size.transpose() << ")";
        return (os);
      }

public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

  #define DEFAULT_TRANCATION_DISTANCE 30.0f

    /** \brief Camera intrinsics structure
      */
    struct Intr
    {
      float fx, fy, cx, cy;
      Intr () {};
      Intr (float fx_, float fy_, float cx_, float cy_)
        : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {};

      Intr operator()(int level_index) const
      {
        int div = 1 << level_index;
        return (Intr (fx / div, fy / div, cx / div, cy / div));
      }

      friend inline std::ostream&
      operator << (std::ostream& os, const Intr& intr)
      {
        os << "([f = " << intr.fx << ", " << intr.fy << "] [cp = " << intr.cx << ", " << intr.cy << "])";
        return (os);
      }

    };


    ////////////////////////////////////////////////////////////////////////////////////////
    // Constructors

    /** \brief Default constructor */
    TSDFVolume ()
      : volume_ (new std::vector<VoxelT>),
        weights_ (new std::vector<WeightT>)
    {};

    /** \brief Constructor loading data from file */
    TSDFVolume (const std::string &filename)
      : volume_ (new std::vector<VoxelT>),
        weights_ (new std::vector<WeightT>)
    {
      if (load (filename))
        std::cout << "done [" << size() << "]" << std::endl;
      else
        std::cout << "error!" << std::endl;
    };

    /** \brief Set the header directly. Useful if directly writing into volume and weights */
    inline void
    setHeader (const Eigen::Vector3i &resolution, const Eigen::Vector3f &volume_size) {
      header_ = Header (resolution, volume_size);
      if (volume_->size() != this->size())
        pcl::console::print_warn ("[TSDFVolume::setHeader] Header volume size (%d) doesn't fit underlying data size (%d)", volume_->size(), size());
    };

    /** \brief Resizes the internal storage and updates the header accordingly */
    inline void
    resize (Eigen::Vector3i &grid_resolution, const Eigen::Vector3f& volume_size = Eigen::Vector3f (DEFAULT_VOLUME_SIZE_X, DEFAULT_VOLUME_SIZE_Y, DEFAULT_VOLUME_SIZE_Z)) {
      int lin_size = grid_resolution[0] * grid_resolution[1] * grid_resolution[2];
      volume_->resize (lin_size);
      weights_->resize (lin_size);
      setHeader (grid_resolution, volume_size);
    };

    /** \brief Resize internal storage and header to default sizes defined in tsdf_volume.h */
    inline void
    resizeDefaultSize () {
      resize (Eigen::Vector3i (DEFAULT_GRID_RES_X, DEFAULT_GRID_RES_Y, DEFAULT_GRID_RES_Z),
              Eigen::Vector3f (DEFAULT_VOLUME_SIZE_X, DEFAULT_VOLUME_SIZE_Y, DEFAULT_VOLUME_SIZE_Z));
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    // Storage and element access

    /** \brief Loads volume from file */
    bool
    load (const std::string &filename, bool binary = true);

    /** \brief Saves volume to file */
    bool
    save (const std::string &filename = "tsdf_volume.dat", bool binary = true) const;

    /** \brief Returns overall number of voxels in grid */
    inline size_t
    size () const { return header_.getVolumeSize(); };

    /** \brief Returns the volume size in mm */
    inline const Eigen::Vector3f &
    volumeSize () const { return header_.volume_size; };

    /** \brief Returns the size of one voxel in mm */
    inline Eigen::Vector3f
    voxelSize () const {
      Eigen::Array3f res = header_.resolution.array().template cast<float>();
      return header_.volume_size.array() / res;
    };

    /** \brief Returns the voxel grid resolution */
    inline const Eigen::Vector3i &
    gridResolution() const { return header_.resolution; };

    /** \brief Returns constant reference to header */
    inline const Header &
    header () const { return header_; };

    /** \brief Returns constant reference to the volume std::vector */
    inline const std::vector<VoxelT> &
    volume () const { return *volume_; };

    /** \brief Returns writebale(!) reference to volume */
    inline std::vector<VoxelT> &
    volumeWriteable () const { return *volume_; };

    /** \brief Returns constant reference to the weights std::vector */
    inline const std::vector<WeightT> &
    weights () const { return *weights_; };

    /** \brief Returns writebale(!) reference to volume */
    inline std::vector<WeightT> &
    weightsWriteable () const { return *weights_; };

    ////////////////////////////////////////////////////////////////////////////////////////
    // Functionality

    /** \brief Converts volume to cloud of TSDF values*/
    void
    convertToTsdfCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;

    /** \brief Converts the volume to a surface representation via a point cloud */
  //  void
  //  convertToCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const;

    /** \brief Crate Volume from Point Cloud */
  //   template <typename PointT> void
  //   createFromCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const Intr &intr);

    /** \brief Retunrs the 3D voxel coordinate */
    template <typename PointT> void
    getVoxelCoord (const PointT &point, Eigen::Vector3i &voxel_coord)  const;

    /** \brief Retunrs the 3D voxel coordinate and point offset wrt. to the voxel center (in mm) */
    template <typename PointT> void
    getVoxelCoordAndOffset (const PointT &point, Eigen::Vector3i &voxel_coord, Eigen::Vector3f &offset) const;

    /** extracts voxels in neighborhood of given voxel */
    bool
    extractNeighborhood (const Eigen::Vector3i &voxel_coord, int neighborhood_size, VoxelTVec &neighborhood) const;

    /** adds voxel values in local neighborhood */
    bool
    addNeighborhood (const Eigen::Vector3i &voxel_coord, int neighborhood_size, const VoxelTVec &neighborhood, WeightT voxel_weight);

    /** averages voxel values by the weight value */
    void
    averageValues ();

    /** \brief Returns and index for linear access of the volume and weights */
    inline int
    getLinearVoxelIndex (const Eigen::Array3i &indices) const {
      return indices(0) + indices(1) * header_.resolution[0] + indices(2) * header_.resolution[0] * header_.resolution[1];
    }

    /** \brief Returns a vector of linear indices for voxel coordinates given in 3xn matrix */
    inline Eigen::VectorXi
    getLinearVoxelIndinces (const Eigen::Matrix<int, 3, Eigen::Dynamic> &indices_matrix) const  {
      return (Eigen::RowVector3i (1, header_.resolution[0], header_.resolution[0] * header_.resolution[1]) * indices_matrix).transpose();
    }

  private:

    ////////////////////////////////////////////////////////////////////////////////////////
    // Private functions and members

  //  void
  //  scaleDepth (const Eigen::MatrixXf &depth, Eigen::MatrixXf &depth_scaled, const Intr &intr) const;

  //  void
  //  integrateVolume (const Eigen::MatrixXf &depth_scaled, float tranc_dist, const Eigen::Matrix3f &R_inv, const Eigen::Vector3f &t, const Intr &intr);

    typedef boost::shared_ptr<std::vector<VoxelT> > VolumePtr;
    typedef boost::shared_ptr<std::vector<WeightT> > WeightsPtr;

    Header header_;
    VolumePtr volume_;
    WeightsPtr weights_;
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

}

#endif /* TSDF_VOLUME_H_ */
