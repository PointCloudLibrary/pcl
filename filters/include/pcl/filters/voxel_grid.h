/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_FILTERS_VOXEL_GRID_MAP_H_
#define PCL_FILTERS_VOXEL_GRID_MAP_H_

#include "pcl/filters/filter.h"
#include <map>
#include <boost/unordered_map.hpp>
#include <boost/fusion/sequence/intrinsic/at_key.hpp>

namespace pcl
{
  PCL_EXPORTS void 
  getMinMax3D (const sensor_msgs::PointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx, 
               Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt);

  PCL_EXPORTS void 
  getMinMax3D (const sensor_msgs::PointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx, 
               const std::string &distance_field_name, float min_distance, float max_distance, 
               Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative = false);

  /** \brief Get the relative cell indices of the "upper half" 13 neighbors.
    * @note Useful in combination with getNeighborCentroidIndices() from \ref VoxelGrid
    * \ingroup filters
    */
  inline Eigen::MatrixXi
  getHalfNeighborCellIndices ()
  {
    Eigen::MatrixXi relative_coordinates(3, 13);
    int idx = 0;

    // 0 - 8
    for (int i = -1; i < 2; i++) {
      for (int j = -1; j < 2; j++) {
        relative_coordinates(0, idx) = i;
        relative_coordinates(1, idx) = j;
        relative_coordinates(2, idx) = -1;
        idx++;
      }
    }
    // 9 - 11
    for (int i = -1; i < 2; i++) {
      relative_coordinates(0, idx) = i;
      relative_coordinates(1, idx) = -1;
      relative_coordinates(2, idx) = 0;
      idx++;
    }
    // 12
    relative_coordinates(0, idx) = -1;
    relative_coordinates(1, idx) = 0;
    relative_coordinates(2, idx) = 0;

    return relative_coordinates;
  }

  /** \brief Get the relative cell indices of all the 26 neighbors.
    * @note Useful in combination with getNeighborCentroidIndices() from \ref VoxelGrid
    * \ingroup filters
    */
  inline Eigen::MatrixXi
  getAllNeighborCellIndices ()
  {
    Eigen::MatrixXi relative_coordinates = getHalfNeighborCellIndices();
    Eigen::MatrixXi relative_coordinates_all(3, 26);
    relative_coordinates_all.block<3, 13> (0, 0) = relative_coordinates;
    relative_coordinates_all.block<3, 13> (0, 13) = -relative_coordinates;
    return relative_coordinates_all;
  }

  /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
    * in a given pointcloud, without considering points outside of a distance threshold from the laser origin
    * \param cloud the point cloud data message
    * \param distance_field_name the field name that contains the distance values
    * \param min_distance the minimum distance a point will be considered from
    * \param max_distance the maximum distance a point will be considered to
    * \param min_pt the resultant minimum bounds
    * \param max_pt the resultant maximum bounds
    * \param limit_negative if set to true, then all points outside of the interval (min_distance;max_distace) are considered
    * \ingroup filters
    */
  template <typename PointT> void 
  getMinMax3D (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, 
               const std::string &distance_field_name, float min_distance, float max_distance,
               Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative = false);

  /** \brief @b VoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
    *
    * The @b VoxelGrid class creates a *3D voxel grid* (think about a voxel
    * grid as a set of tiny 3D boxes in space) over the input point cloud data.
    * Then, in each *voxel* (i.e., 3D box), all the points present will be
    * approximated (i.e., *downsampled*) with their centroid. This approach is
    * a bit slower than approximating them with the center of the voxel, but it
    * represents the underlying surface more accurately.
    *
    * \author Radu Bogdan Rusu, Bastian Steder
    * \ingroup filters
    */
  template <typename PointT>
  class VoxelGrid: public Filter<PointT>
  {
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::filter_limit_negative_;
    using Filter<PointT>::filter_limit_min_;
    using Filter<PointT>::filter_limit_max_;
    using Filter<PointT>::filter_field_name_;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      VoxelGrid () : downsample_all_data_ (true), save_leaf_layout_ (false)
      {
        leaf_size_.setZero ();
        min_b_.setZero ();
        max_b_.setZero ();
        filter_name_ = "VoxelGrid";
      }

      /** \brief Destructor. */
      virtual ~VoxelGrid ()
      {
        leaves_.clear();
      }

      /** \brief Set the voxel grid leaf size.
        * \param leaf_size the voxel grid leaf size
        */
      inline void 
      setLeafSize (const Eigen::Vector4f &leaf_size) 
      { 
        leaf_size_ = leaf_size; 
        // Avoid division errors
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
      }

      /** \brief Set the voxel grid leaf size.
        * \param lx the leaf size for X
        * \param ly the leaf size for Y
        * \param lz the leaf size for Z
        */
      inline void
      setLeafSize (float lx, float ly, float lz)
      {
        leaf_size_[0] = lx; leaf_size_[1] = ly; leaf_size_[2] = lz;
        // Avoid division errors
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
      }

      /** \brief Get the voxel grid leaf size. */
      inline Eigen::Vector3f 
      getLeafSize () { return (leaf_size_.head<3> ()); }

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
        * \param downsample the new value (true/false)
        */
      inline void 
      setDownsampleAllData (bool downsample) { downsample_all_data_ = downsample; }

      /** \brief Get the state of the internal downsampling parameter (true if
        * all fields need to be downsampled, false if just XYZ). 
        */
      inline bool 
      getDownsampleAllData () { return (downsample_all_data_); }

      /** \brief Set to true if leaf layout information needs to be saved for later access.
        * \param save_leaf_layout the new value (true/false)
        */
      inline void 
      setSaveLeafLayout (bool save_leaf_layout) { save_leaf_layout_ = save_leaf_layout; }

      /** \brief Returns true if leaf layout information will to be saved for later access. */
      inline bool 
      getSaveLeafLayout () { return (save_leaf_layout_); }

      /** \brief Get the minimum coordinates of the bounding box (after
        * filtering is performed). 
        */
      inline Eigen::Vector3i 
      getMinBoxCoordinates () { return (min_b_.head<3> ()); }

      /** \brief Get the minimum coordinates of the bounding box (after
        * filtering is performed). 
        */
      inline Eigen::Vector3i 
      getMaxBoxCoordinates () { return (max_b_.head<3> ()); }

      /** \brief Get the number of divisions along all 3 axes (after filtering
        * is performed). 
        */
      inline Eigen::Vector3i 
      getNrDivisions () { return (div_b_.head<3> ()); }

      /** \brief Get the multipliers to be applied to the grid coordinates in
        * order to find the centroid index (after filtering is performed). 
        */
      inline Eigen::Vector3i 
      getDivisionMultiplier () { return (divb_mul_.head<3> ()); }

      /** \brief Returns the index in the resulting downsampled cloud of the specified point.
        *
        * \note for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering 
        * performed, and that the point is inside the grid, to avoid invalid access (or use
        * getGridCoordinates+getCentroidIndexAt)
        *
        * \param p the point to get the index at
        */
      inline int 
      getCentroidIndex (const PointT &p)
      {
        return leaf_layout_.at ((Eigen::Vector4i (floor (p.x / leaf_size_[0]), floor (p.y / leaf_size_[1]), floor (p.z / leaf_size_[2]), 0) - min_b_).dot (divb_mul_));
      }

      /** \brief Returns the indices in the resulting downsampled cloud of the points at the specified grid coordinates,
        * relative to the grid coordinates of the specified point (or -1 if the cell was empty/out of bounds).
        * \param reference_point the coordinates of the reference point (corresponding cell is allowed to be empty/out of bounds)
        * \param relative_coordinates matrix with the columns being the coordinates of the requested cells, relative to the reference point's cell
        * \note for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering performed
        */
      inline std::vector<int> 
      getNeighborCentroidIndices (const PointT &reference_point, const Eigen::MatrixXi &relative_coordinates)
      {
        Eigen::Vector4i ijk (floor (reference_point.x / leaf_size_[0]), floor (reference_point.y / leaf_size_[1]), floor (reference_point.z / leaf_size_[2]), 0);
        Eigen::Array4i diff2min = min_b_ - ijk;
        Eigen::Array4i diff2max = max_b_ - ijk;
        std::vector<int> neighbors (relative_coordinates.cols());
        for (int ni = 0; ni < relative_coordinates.cols (); ni++)
        {
          Eigen::Vector4i displacement = (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();
          // checking if the specified cell is in the grid
          if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all())
            neighbors[ni] = leaf_layout_[((ijk + displacement - min_b_).dot (divb_mul_))]; // .at() can be omitted
          else
            neighbors[ni] = -1; // cell is out of bounds, consider it empty
        }
        return neighbors;
      }

      /** \brief Returns the layout of the leafs for fast access to cells relative to current position.
        * \note position at (i-min_x) + (j-min_y)*div_x + (k-min_z)*div_x*div_y holds the index of the element at coordinates (i,j,k) in the grid (-1 if empty)
        */
      inline std::vector<int> 
      getLeafLayout () { return (leaf_layout_); }

      /** \brief Returns the corresponding (i,j,k) coordinates in the grid of point (x,y,z). */
      inline Eigen::Vector3i 
      getGridCoordinates (float x, float y, float z) 
      { 
        return Eigen::Vector3i (floor (x / leaf_size_[0]), floor (y / leaf_size_[1]), floor (z / leaf_size_[2])); 
      }

      /** \brief Returns the index in the downsampled cloud corresponding to coordinates (i,j,k) in the grid (-1 if empty) */
      inline int 
      getCentroidIndexAt (const Eigen::Vector3i &ijk, bool verbose = true)
      {
        int idx = ((Eigen::Vector4i() << ijk, 0).finished() - min_b_).dot (divb_mul_);
        if (idx < 0 || idx >= (int)leaf_layout_.size ()) // this checks also if leaf_layout_.size () == 0 i.e. everything was computed as needed
        {
          if (verbose)
            PCL_ERROR ("[pcl::%s::getCentroidIndexAt] Specified coordinate is outside grid bounds, or leaf layout is not saved, make sure to call setSaveLeafLayout(true) and filter(output) first!\n", getClassName ().c_str ());
          return -1;
        }
        return leaf_layout_[idx];
      }

    protected:
      /** \brief Simple structure to hold an nD centroid and the number of points in a leaf. 
        */
      struct Leaf
      {
        Leaf () : nr_points(0) {}
        Eigen::VectorXf centroid;    // @todo we do not support FLOAT64 just yet due to memory issues. Need to fix this.
        int nr_points;
      };

      /** \brief The 3D grid leaves. */
      boost::unordered_map<size_t, Leaf> leaves_;

      /** \brief The size of a leaf. */
      Eigen::Vector4f leaf_size_;

      /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */ 
      Eigen::Array4f inverse_leaf_size_;

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
      bool downsample_all_data_;

      /** \brief Set to true if leaf layout information needs to be saved in \a leaf_layout_. */
      bool save_leaf_layout_;

      /** \brief The leaf layout information for fast access to cells relative to current position **/
      std::vector<int> leaf_layout_;

      /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
      Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

      typedef typename pcl::traits::fieldList<PointT>::type FieldList;

      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param output the resultant point cloud message
        */
      void 
      applyFilter (PointCloud &output);
  };

  /** \brief @b VoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
    *
    * The @b VoxelGrid class creates a *3D voxel grid* (think about a voxel
    * grid as a set of tiny 3D boxes in space) over the input point cloud data.
    * Then, in each *voxel* (i.e., 3D box), all the points present will be
    * approximated (i.e., *downsampled*) with their centroid. This approach is
    * a bit slower than approximating them with the center of the voxel, but it
    * represents the underlying surface more accurately.
    *
     * \author Radu Bogdan Rusu, Bastian Steder
    * \ingroup filters
    */
  template <>
  class PCL_EXPORTS VoxelGrid<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    using Filter<sensor_msgs::PointCloud2>::filter_name_;
    using Filter<sensor_msgs::PointCloud2>::getClassName;

    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
      /** \brief Empty constructor. */
      VoxelGrid () : downsample_all_data_ (true), save_leaf_layout_ (false)
      {
        leaf_size_.setZero ();
        min_b_.setZero ();
        max_b_.setZero ();
        filter_name_ = "VoxelGrid";
      }

      /** \brief Destructor. */
      virtual ~VoxelGrid ()
      {
        leaves_.clear ();
      }

      /** \brief Set the voxel grid leaf size.
        * \param leaf_size the voxel grid leaf size
        */
      inline void 
      setLeafSize (const Eigen::Vector4f &leaf_size) 
      { 
        leaf_size_ = leaf_size; 
        // Avoid division errors
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
      }

      /** \brief Set the voxel grid leaf size.
        * \param lx the leaf size for X
        * \param ly the leaf size for Y
        * \param lz the leaf size for Z
        */
      inline void
      setLeafSize (float lx, float ly, float lz)
      {
        leaf_size_[0] = lx; leaf_size_[1] = ly; leaf_size_[2] = lz;
        // Avoid division errors
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
      }

      /** \brief Get the voxel grid leaf size. */
      inline Eigen::Vector3f 
      getLeafSize () { return (leaf_size_.head<3> ()); }

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
        * \param downsample the new value (true/false)
        */
      inline void 
      setDownsampleAllData (bool downsample) { downsample_all_data_ = downsample; }

      /** \brief Get the state of the internal downsampling parameter (true if
        * all fields need to be downsampled, false if just XYZ). 
        */
      inline bool 
      getDownsampleAllData () { return (downsample_all_data_); }

      /** \brief Set to true if leaf layout information needs to be saved for later access.
        * \param save_leaf_layout the new value (true/false)
        */
      inline void 
      setSaveLeafLayout (bool save_leaf_layout) { save_leaf_layout_ = save_leaf_layout; }

      /** \brief Returns true if leaf layout information will to be saved for later access. */
      inline bool 
      getSaveLeafLayout () { return (save_leaf_layout_); }

      /** \brief Get the minimum coordinates of the bounding box (after
        * filtering is performed). 
        */
      inline Eigen::Vector3i 
      getMinBoxCoordinates () { return (min_b_.head<3> ()); }

      /** \brief Get the minimum coordinates of the bounding box (after
        * filtering is performed). 
        */
      inline Eigen::Vector3i 
      getMaxBoxCoordinates () { return (max_b_.head<3> ()); }

      /** \brief Get the number of divisions along all 3 axes (after filtering
        * is performed). 
        */
      inline Eigen::Vector3i 
      getNrDivisions () { return (div_b_.head<3> ()); }

      /** \brief Get the multipliers to be applied to the grid coordinates in
        * order to find the centroid index (after filtering is performed). 
        */
      inline Eigen::Vector3i 
      getDivisionMultiplier () { return (divb_mul_.head<3> ()); }

      /** \brief Returns the index in the resulting downsampled cloud of the specified point.
        * \note for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering performed,
        * and that the point is inside the grid, to avoid invalid access (or use getGridCoordinates+getCentroidIndexAt)
        */
      inline int 
      getCentroidIndex (float x, float y, float z)
      {
        return leaf_layout_.at ((Eigen::Vector4i ((int) floor (x / leaf_size_[0]), (int) floor (y / leaf_size_[1]), (int) floor (z / leaf_size_[2]), 0) - min_b_).dot (divb_mul_));
      }

      /** \brief Returns the indices in the resulting downsampled cloud of the points at the specified grid coordinates,
        * relative to the grid coordinates of the specified point (or -1 if the cell was empty/out of bounds).
        * \param x the X coordinate of the reference point (corresponding cell is allowed to be empty/out of bounds)
        * \param y the Y coordinate of the reference point (corresponding cell is allowed to be empty/out of bounds)
        * \param z the Z coordinate of the reference point (corresponding cell is allowed to be empty/out of bounds)
        * \param relative_coordinates matrix with the columns being the coordinates of the requested cells, relative to the reference point's cell
        * \note for efficiency, user must make sure that the saving of the leaf layout is enabled and filtering performed
        */
      inline std::vector<int> 
      getNeighborCentroidIndices (float x, float y, float z, const Eigen::MatrixXi &relative_coordinates)
      {
        Eigen::Vector4i ijk ((int) floor (x / leaf_size_[0]), (int) floor (y / leaf_size_[1]), (int) floor (z / leaf_size_[2]), 0);
        Eigen::Array4i diff2min = min_b_ - ijk;
        Eigen::Array4i diff2max = max_b_ - ijk;
        std::vector<int> neighbors (relative_coordinates.cols());
        for (int ni = 0; ni < relative_coordinates.cols (); ni++)
        {
          Eigen::Vector4i displacement = (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();
          // checking if the specified cell is in the grid
          if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all())
            neighbors[ni] = leaf_layout_[((ijk + displacement - min_b_).dot (divb_mul_))]; // .at() can be omitted
          else
            neighbors[ni] = -1; // cell is out of bounds, consider it empty
        }
        return neighbors;
      }
      
      inline std::vector<int> 
      getNeighborCentroidIndices (float x, float y, float z, const std::vector<Eigen::Vector3i> &relative_coordinates)
      {
        Eigen::Vector4i ijk ((int) floor (x / leaf_size_[0]), (int) floor (y / leaf_size_[1]), (int) floor (z / leaf_size_[2]), 0);
        std::vector<int> neighbors;
        neighbors.reserve (relative_coordinates.size ());
        for (std::vector<Eigen::Vector3i>::const_iterator it = relative_coordinates.begin (); it != relative_coordinates.end (); it++)
          neighbors.push_back (leaf_layout_[(ijk + (Eigen::Vector4i() << *it, 0).finished() - min_b_).dot (divb_mul_)]);
        return neighbors;
      }

      /** \brief Returns the layout of the leafs for fast access to cells relative to current position.
        * \note position at (i-min_x) + (j-min_y)*div_x + (k-min_z)*div_x*div_y holds the index of the element at coordinates (i,j,k) in the grid (-1 if empty)
        */
      inline std::vector<int> 
      getLeafLayout () { return (leaf_layout_); }

      /** \brief Returns the corresponding (i,j,k) coordinates in the grid of point (x,y,z). */
      inline Eigen::Vector3i 
      getGridCoordinates (float x, float y, float z) 
      { 
        return Eigen::Vector3i ((int) floor (x / leaf_size_[0]), (int) floor (y / leaf_size_[1]), (int) floor (z / leaf_size_[2])); 
      }

      /** \brief Returns the index in the downsampled cloud corresponding to coordinates (i,j,k) in the grid (-1 if empty) */
      inline int 
      getCentroidIndexAt (const Eigen::Vector3i &ijk, bool verbose = true)
      {
        int idx = ((Eigen::Vector4i() << ijk, 0).finished() - min_b_).dot (divb_mul_);
        if (idx < 0 || idx >= (int)leaf_layout_.size ()) // this checks also if leaf_layout_.size () == 0 i.e. everything was computed as needed
        {
          if (verbose)
            PCL_ERROR ("[pcl::%s::getCentroidIndexAt] Specified coordinate is outside grid bounds, or leaf layout is not saved, make sure to call setSaveLeafLayout(true) and filter(output) first!\n", getClassName ().c_str ());
          return -1;
        }
        return leaf_layout_[idx];
      }

    protected:
      /** \brief Simple structure to hold an nD centroid and the number of points in a leaf. 
        */
      struct Leaf
      {
        Leaf () : nr_points(0) { }
        Eigen::VectorXf centroid;    // @todo we do not support FLOAT64 just yet due to memory issues. Need to fix this.
        int nr_points;
      };

      /** \brief The 3D grid leaves. */
      boost::unordered_map<size_t, Leaf> leaves_;

      /** \brief The size of a leaf. */
      Eigen::Vector4f leaf_size_;

      /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */ 
      Eigen::Array4f inverse_leaf_size_;

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
      bool downsample_all_data_;

      /** \brief Set to true if leaf layout information needs to be saved in \a
        * leaf_layout. 
        */
      bool save_leaf_layout_;

      /** \brief The leaf layout information for fast access to cells relative
        * to current position 
        */
      std::vector<int> leaf_layout_;

      /** \brief The minimum and maximum bin coordinates, the number of
        * divisions, and the division multiplier. 
        */
      Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param output the resultant point cloud
        */
      void 
      applyFilter (PointCloud2 &output);
  };
}

#endif  //#ifndef PCL_FILTERS_VOXEL_GRID_MAP_H_
