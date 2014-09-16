/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#ifndef PCL_RANGE_IMAGE_H_
#define PCL_RANGE_IMAGE_H_

#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/vector_average.h>
#include <typeinfo>

namespace pcl
{
  /** \brief RangeImage is derived from pcl/PointCloud and provides functionalities with focus on situations where
    *  a 3D scene was captured from a specific view point. 
    * \author Bastian Steder
    * \ingroup range_image
    */
  class RangeImage : public pcl::PointCloud<PointWithRange>
  {
    public:
      // =====TYPEDEFS=====
      typedef pcl::PointCloud<PointWithRange> BaseClass;
      typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > VectorOfEigenVector3f;
      typedef boost::shared_ptr<RangeImage> Ptr;
      typedef boost::shared_ptr<const RangeImage> ConstPtr;
      
      enum CoordinateFrame
      {
        CAMERA_FRAME = 0,
        LASER_FRAME  = 1
      };

      
      // =====CONSTRUCTOR & DESTRUCTOR=====
      /** Constructor */
      PCL_EXPORTS RangeImage ();
      /** Destructor */
      PCL_EXPORTS virtual ~RangeImage ();
      
      // =====STATIC VARIABLES=====
      /** The maximum number of openmp threads that can be used in this class */
      static int max_no_of_threads;
      
      // =====STATIC METHODS=====
      /** \brief Get the size of a certain area when seen from the given pose
        * \param viewer_pose an affine matrix defining the pose of the viewer
        * \param center the center of the area
        * \param radius the radius of the area
        * \return the size of the area as viewed according to \a viewer_pose
        */
      static inline float
      getMaxAngleSize (const Eigen::Affine3f& viewer_pose, const Eigen::Vector3f& center, 
                       float radius);
      
      /** \brief Get Eigen::Vector3f from PointWithRange
        * \param point the input point
        * \return an Eigen::Vector3f representation of the input point
        */
      static inline Eigen::Vector3f
      getEigenVector3f (const PointWithRange& point);
      
      /** \brief Get the transformation that transforms the given coordinate frame into CAMERA_FRAME
        * \param coordinate_frame the input coordinate frame
        * \param transformation the resulting transformation that warps \a coordinate_frame into CAMERA_FRAME
        */
      PCL_EXPORTS static void
      getCoordinateFrameTransformation (RangeImage::CoordinateFrame coordinate_frame,
                                        Eigen::Affine3f& transformation);
      
      /** \brief Get the average viewpoint of a point cloud where each point carries viewpoint information as 
        * vp_x, vp_y, vp_z
        * \param point_cloud the input point cloud
        * \return the average viewpoint (as an Eigen::Vector3f)
        */
      template <typename PointCloudTypeWithViewpoints> static Eigen::Vector3f
      getAverageViewPoint (const PointCloudTypeWithViewpoints& point_cloud);
      
      /** \brief Check if the provided data includes far ranges and add them to far_ranges
        * \param point_cloud_data a PCLPointCloud2 message containing the input cloud
        * \param far_ranges the resulting cloud containing those points with far ranges
        */
      PCL_EXPORTS static void
      extractFarRanges (const pcl::PCLPointCloud2& point_cloud_data, PointCloud<PointWithViewpoint>& far_ranges);
      
      // =====METHODS=====
      /** \brief Get a boost shared pointer of a copy of this */
      inline Ptr 
      makeShared () { return Ptr (new RangeImage (*this)); } 
      
      /** \brief Reset all values to an empty range image */
      PCL_EXPORTS void 
      reset ();
      
      /** \brief Create the depth image from a point cloud
        * \param point_cloud the input point cloud
        * \param angular_resolution the angular difference (in radians) between the individual pixels in the image
        * \param max_angle_width an angle (in radians) defining the horizontal bounds of the sensor
        * \param max_angle_height an angle (in radians) defining the vertical bounds of the sensor
        * \param sensor_pose an affine matrix defining the pose of the sensor (defaults to
        *                    Eigen::Affine3f::Identity () )
        * \param coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
        * \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
        *                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and
        *                      will always take the minimum per cell.
        * \param min_range the minimum visible range (defaults to 0)
        * \param border_size the border size (defaults to 0)
        */
      template <typename PointCloudType> void
      createFromPointCloud (const PointCloudType& point_cloud, float angular_resolution=pcl::deg2rad (0.5f),
          float max_angle_width=pcl::deg2rad (360.0f), float max_angle_height=pcl::deg2rad (180.0f),
          const Eigen::Affine3f& sensor_pose = Eigen::Affine3f::Identity (),
          CoordinateFrame coordinate_frame=CAMERA_FRAME, float noise_level=0.0f,
          float min_range=0.0f, int border_size=0);
      
      /** \brief Create the depth image from a point cloud
        * \param point_cloud the input point cloud
        * \param angular_resolution_x the angular difference (in radians) between the
        *                             individual pixels in the image in the x-direction
        * \param angular_resolution_y the angular difference (in radians) between the
        *                             individual pixels in the image in the y-direction
        * \param max_angle_width an angle (in radians) defining the horizontal bounds of the sensor
        * \param max_angle_height an angle (in radians) defining the vertical bounds of the sensor
        * \param sensor_pose an affine matrix defining the pose of the sensor (defaults to
        *                    Eigen::Affine3f::Identity () )
        * \param coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
        * \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
        *                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and
        *                      will always take the minimum per cell.
        * \param min_range the minimum visible range (defaults to 0)
        * \param border_size the border size (defaults to 0)
        */
      template <typename PointCloudType> void
      createFromPointCloud (const PointCloudType& point_cloud,
          float angular_resolution_x=pcl::deg2rad (0.5f), float angular_resolution_y=pcl::deg2rad (0.5f),
          float max_angle_width=pcl::deg2rad (360.0f), float max_angle_height=pcl::deg2rad (180.0f),
          const Eigen::Affine3f& sensor_pose = Eigen::Affine3f::Identity (),
          CoordinateFrame coordinate_frame=CAMERA_FRAME,
          float noise_level=0.0f, float min_range=0.0f, int border_size=0);
      
      /** \brief Create the depth image from a point cloud, getting a hint about the size of the scene for 
        * faster calculation.
        * \param point_cloud the input point cloud
        * \param angular_resolution the angle (in radians) between each sample in the depth image
        * \param point_cloud_center the center of bounding sphere
        * \param point_cloud_radius the radius of the bounding sphere
        * \param sensor_pose an affine matrix defining the pose of the sensor (defaults to
        *                     Eigen::Affine3f::Identity () )
        * \param coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
        * \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
        *                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and
        *                      will always take the minimum per cell.
        * \param min_range the minimum visible range (defaults to 0)
        * \param border_size the border size (defaults to 0)
        */
      template <typename PointCloudType> void
      createFromPointCloudWithKnownSize (const PointCloudType& point_cloud, float angular_resolution,
                                         const Eigen::Vector3f& point_cloud_center, float point_cloud_radius,
                                         const Eigen::Affine3f& sensor_pose = Eigen::Affine3f::Identity (),
                                         CoordinateFrame coordinate_frame=CAMERA_FRAME,
                                         float noise_level=0.0f, float min_range=0.0f, int border_size=0);
      
      /** \brief Create the depth image from a point cloud, getting a hint about the size of the scene for 
        * faster calculation.
        * \param point_cloud the input point cloud
        * \param angular_resolution_x the angular difference (in radians) between the
        *                             individual pixels in the image in the x-direction
        * \param angular_resolution_y the angular difference (in radians) between the
        *                             individual pixels in the image in the y-direction
        * \param point_cloud_center the center of bounding sphere
        * \param point_cloud_radius the radius of the bounding sphere
        * \param sensor_pose an affine matrix defining the pose of the sensor (defaults to
        *                     Eigen::Affine3f::Identity () )
        * \param coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
        * \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
        *                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and
        *                      will always take the minimum per cell.
        * \param min_range the minimum visible range (defaults to 0)
        * \param border_size the border size (defaults to 0)
        */
      template <typename PointCloudType> void
      createFromPointCloudWithKnownSize (const PointCloudType& point_cloud,
                                         float angular_resolution_x, float angular_resolution_y,
                                         const Eigen::Vector3f& point_cloud_center, float point_cloud_radius,
                                         const Eigen::Affine3f& sensor_pose = Eigen::Affine3f::Identity (),
                                         CoordinateFrame coordinate_frame=CAMERA_FRAME,
                                         float noise_level=0.0f, float min_range=0.0f, int border_size=0);
      
      /** \brief Create the depth image from a point cloud, using the average viewpoint of the points 
        * (vp_x,vp_y,vp_z in the point type) in the point cloud as sensor pose (assuming a rotation of (0,0,0)).
        * \param point_cloud the input point cloud
        * \param angular_resolution the angle (in radians) between each sample in the depth image
        * \param max_angle_width an angle (in radians) defining the horizontal bounds of the sensor
        * \param max_angle_height an angle (in radians) defining the vertical bounds of the sensor
        * \param coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
        * \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
        *                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and
        *                      will always take the minimum per cell.
        * \param min_range the minimum visible range (defaults to 0)
        * \param border_size the border size (defaults to 0)
        * \note If wrong_coordinate_system is true, the sensor pose will be rotated to change from a coordinate frame
        * with x to the front, y to the left and z to the top to the coordinate frame we use here (x to the right, y 
        * to the bottom and z to the front) */
      template <typename PointCloudTypeWithViewpoints> void
      createFromPointCloudWithViewpoints (const PointCloudTypeWithViewpoints& point_cloud, float angular_resolution,
                                          float max_angle_width, float max_angle_height,
                                          CoordinateFrame coordinate_frame=CAMERA_FRAME, float noise_level=0.0f,
                                          float min_range=0.0f, int border_size=0);
      
      /** \brief Create the depth image from a point cloud, using the average viewpoint of the points 
        * (vp_x,vp_y,vp_z in the point type) in the point cloud as sensor pose (assuming a rotation of (0,0,0)).
        * \param point_cloud the input point cloud
        * \param angular_resolution_x the angular difference (in radians) between the
        *                             individual pixels in the image in the x-direction
        * \param angular_resolution_y the angular difference (in radians) between the
        *                             individual pixels in the image in the y-direction
        * \param max_angle_width an angle (in radians) defining the horizontal bounds of the sensor
        * \param max_angle_height an angle (in radians) defining the vertical bounds of the sensor
        * \param coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
        * \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
        *                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and
        *                      will always take the minimum per cell.
        * \param min_range the minimum visible range (defaults to 0)
        * \param border_size the border size (defaults to 0)
        * \note If wrong_coordinate_system is true, the sensor pose will be rotated to change from a coordinate frame
        * with x to the front, y to the left and z to the top to the coordinate frame we use here (x to the right, y 
        * to the bottom and z to the front) */
      template <typename PointCloudTypeWithViewpoints> void
      createFromPointCloudWithViewpoints (const PointCloudTypeWithViewpoints& point_cloud,
                                          float angular_resolution_x, float angular_resolution_y,
                                          float max_angle_width, float max_angle_height,
                                          CoordinateFrame coordinate_frame=CAMERA_FRAME, float noise_level=0.0f,
                                          float min_range=0.0f, int border_size=0);
      
      /** \brief Create an empty depth image (filled with unobserved points)
        * \param[in] angular_resolution the angle (in radians) between each sample in the depth image
        * \param[in] sensor_pose an affine matrix defining the pose of the sensor (defaults to  Eigen::Affine3f::Identity () )
        * \param[in] coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
        * \param[in] angle_width an angle (in radians) defining the horizontal bounds of the sensor (defaults to 2*pi (360deg))
        * \param[in] angle_height an angle (in radians) defining the vertical bounds of the sensor (defaults to pi (180deg))
        */
      void
      createEmpty (float angular_resolution, const Eigen::Affine3f& sensor_pose=Eigen::Affine3f::Identity (),
                   RangeImage::CoordinateFrame coordinate_frame=CAMERA_FRAME, float angle_width=pcl::deg2rad (360.0f),
                   float angle_height=pcl::deg2rad (180.0f));     
      
      /** \brief Create an empty depth image (filled with unobserved points)
        * \param angular_resolution_x the angular difference (in radians) between the
        *                             individual pixels in the image in the x-direction
        * \param angular_resolution_y the angular difference (in radians) between the
        *                             individual pixels in the image in the y-direction
        * \param[in] sensor_pose an affine matrix defining the pose of the sensor (defaults to  Eigen::Affine3f::Identity () )
        * \param[in] coordinate_frame the coordinate frame (defaults to CAMERA_FRAME)
        * \param[in] angle_width an angle (in radians) defining the horizontal bounds of the sensor (defaults to 2*pi (360deg))
        * \param[in] angle_height an angle (in radians) defining the vertical bounds of the sensor (defaults to pi (180deg))
        */
      void
      createEmpty (float angular_resolution_x, float angular_resolution_y,
                   const Eigen::Affine3f& sensor_pose=Eigen::Affine3f::Identity (),
                   RangeImage::CoordinateFrame coordinate_frame=CAMERA_FRAME, float angle_width=pcl::deg2rad (360.0f),
                   float angle_height=pcl::deg2rad (180.0f));
      
      /** \brief Integrate the given point cloud into the current range image using a z-buffer
        * \param point_cloud the input point cloud
        * \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
        *                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and
        *                      will always take the minimum per cell.
        * \param min_range the minimum visible range
        * \param top    returns the minimum y pixel position in the image where a point was added
        * \param right  returns the maximum x pixel position in the image where a point was added
        * \param bottom returns the maximum y pixel position in the image where a point was added
        * \param top returns the minimum y position in the image where a point was added
        * \param left   returns the minimum x pixel position in the image where a point was added
        */
      template <typename PointCloudType> void
      doZBuffer (const PointCloudType& point_cloud, float noise_level,
                 float min_range, int& top, int& right, int& bottom, int& left);
      
      /** \brief Integrates the given far range measurements into the range image */
      template <typename PointCloudType> void
      integrateFarRanges (const PointCloudType& far_ranges);
      
      /** \brief Cut the range image to the minimal size so that it still contains all actual range readings.
        * \param border_size allows increase from the minimal size by the specified number of pixels (defaults to 0)
        * \param top if positive, this value overrides the position of the top edge (defaults to -1)
        * \param right if positive, this value overrides the position of the right edge (defaults to -1)
        * \param bottom if positive, this value overrides the position of the bottom edge (defaults to -1)
        * \param left if positive, this value overrides the position of the left edge (defaults to -1)
        */
      PCL_EXPORTS void
      cropImage (int border_size=0, int top=-1, int right=-1, int bottom=-1, int left=-1);
      
      /** \brief Get all the range values in one float array of size width*height  
        * \return a pointer to a new float array containing the range values
        * \note This method allocates a new float array; the caller is responsible for freeing this memory.
        */
      PCL_EXPORTS float*
      getRangesArray () const;
      
      /** Getter for the transformation from the world system into the range image system
       *  (the sensor coordinate frame) */
      inline const Eigen::Affine3f&
      getTransformationToRangeImageSystem () const { return (to_range_image_system_); }
      
      /** Setter for the transformation from the range image system
       *  (the sensor coordinate frame) into the world system */
      inline void 
      setTransformationToRangeImageSystem (const Eigen::Affine3f& to_range_image_system);
      
      /** Getter for the transformation from the range image system
       *  (the sensor coordinate frame) into the world system */
      inline const Eigen::Affine3f&
      getTransformationToWorldSystem () const { return to_world_system_;}
      
      /** Getter for the angular resolution of the range image in x direction in radians per pixel.
       *  Provided for downwards compatability */
      inline float
      getAngularResolution () const { return angular_resolution_x_;}
      
      /** Getter for the angular resolution of the range image in x direction in radians per pixel. */
      inline float
      getAngularResolutionX () const { return angular_resolution_x_;}
      
      /** Getter for the angular resolution of the range image in y direction in radians per pixel. */
      inline float
      getAngularResolutionY () const { return angular_resolution_y_;}
      
      /** Getter for the angular resolution of the range image in x and y direction (in radians). */
      inline void
      getAngularResolution (float& angular_resolution_x, float& angular_resolution_y) const;
      
      /** \brief Set the angular resolution of the range image
        * \param angular_resolution the new angular resolution in x and y direction (in radians per pixel)
        */
      inline void
      setAngularResolution (float angular_resolution);
      
      /** \brief Set the angular resolution of the range image
        * \param angular_resolution_x the new angular resolution in x direction (in radians per pixel)
        * \param angular_resolution_y the new angular resolution in y direction (in radians per pixel)
        */
      inline void
      setAngularResolution (float angular_resolution_x, float angular_resolution_y);

      
      /** \brief Return the 3D point with range at the given image position
        * \param image_x the x coordinate
        * \param image_y the y coordinate
        * \return the point at the specified location (returns unobserved_point if outside of the image bounds)
        */
      inline const PointWithRange&
      getPoint (int image_x, int image_y) const;

      /** \brief Non-const-version of getPoint */
      inline PointWithRange&
      getPoint (int image_x, int image_y);
      
      /** Return the 3d point with range at the given image position */
      inline const PointWithRange&
      getPoint (float image_x, float image_y) const;
      
      /** Non-const-version of the above */
      inline PointWithRange&
      getPoint (float image_x, float image_y);
      
      /** \brief Return the 3D point with range at the given image position.  This methd performs no error checking
        * to make sure the specified image position is inside of the image!
        * \param image_x the x coordinate
        * \param image_y the y coordinate
        * \return the point at the specified location (program may fail if the location is outside of the image bounds)
        */
      inline const PointWithRange&
      getPointNoCheck (int image_x, int image_y) const;

      /** Non-const-version of getPointNoCheck */
      inline PointWithRange&
      getPointNoCheck (int image_x, int image_y);

      /** Same as above */
      inline void
      getPoint (int image_x, int image_y, Eigen::Vector3f& point) const;
      
      /** Same as above */
      inline void
      getPoint (int index, Eigen::Vector3f& point) const;
      
      /** Same as above */
      inline const Eigen::Map<const Eigen::Vector3f>
      getEigenVector3f (int x, int y) const;

      /** Same as above */
      inline const Eigen::Map<const Eigen::Vector3f>
      getEigenVector3f (int index) const;
      
      /** Return the 3d point with range at the given index (whereas index=y*width+x) */
      inline const PointWithRange&
      getPoint (int index) const;

      /** Calculate the 3D point according to the given image point and range */
      inline void
      calculate3DPoint (float image_x, float image_y, float range, PointWithRange& point) const;
      /** Calculate the 3D point according to the given image point and the range value at the closest pixel */
      inline void
      calculate3DPoint (float image_x, float image_y, PointWithRange& point) const;

      /** Calculate the 3D point according to the given image point and range */
      virtual inline void
      calculate3DPoint (float image_x, float image_y, float range, Eigen::Vector3f& point) const;
      /** Calculate the 3D point according to the given image point and the range value at the closest pixel */
      inline void
      calculate3DPoint (float image_x, float image_y, Eigen::Vector3f& point) const;
      
      /** Recalculate all 3D point positions according to their pixel position and range */
      PCL_EXPORTS void
      recalculate3DPointPositions ();
      
      /** Get imagePoint from 3D point in world coordinates */
      inline virtual void
      getImagePoint (const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const;

      /** Same as above */
      inline void
      getImagePoint (const Eigen::Vector3f& point, int& image_x, int& image_y, float& range) const;
      
      /** Same as above */
      inline void
      getImagePoint (const Eigen::Vector3f& point, float& image_x, float& image_y) const;
      
      /** Same as above */
      inline void
      getImagePoint (const Eigen::Vector3f& point, int& image_x, int& image_y) const;
      
      /** Same as above */
      inline void
      getImagePoint (float x, float y, float z, float& image_x, float& image_y, float& range) const;
      
      /** Same as above */
      inline void
      getImagePoint (float x, float y, float z, float& image_x, float& image_y) const;
      
      /** Same as above */
      inline void
      getImagePoint (float x, float y, float z, int& image_x, int& image_y) const;
      
      /** point_in_image will be the point in the image at the position the given point would be. Returns
       * the range of the given point. */
      inline float
      checkPoint (const Eigen::Vector3f& point, PointWithRange& point_in_image) const;

      /** Returns the difference in range between the given point and the range of the point in the image
       * at the position the given point would be.
       *  (Return value is point_in_image.range-given_point.range) */
      inline float
      getRangeDifference (const Eigen::Vector3f& point) const;
      
      /** Get the image point corresponding to the given angles */
      inline void
      getImagePointFromAngles (float angle_x, float angle_y, float& image_x, float& image_y) const;
      
      /** Get the angles corresponding to the given image point */
      inline void
      getAnglesFromImagePoint (float image_x, float image_y, float& angle_x, float& angle_y) const;
      
      /** Transforms an image point in float values to an image point in int values */
      inline void
      real2DToInt2D (float x, float y, int& xInt, int& yInt) const;
      
      /** Check if a point is inside of the image */
      inline bool
      isInImage (int x, int y) const;
      
      /** Check if a point is inside of the image and has a finite range */
      inline bool
      isValid (int x, int y) const;
      
      /** Check if a point has a finite range */
      inline bool
      isValid (int index) const;
      
      /** Check if a point is inside of the image and has either a finite range or a max reading (range=INFINITY) */
      inline bool
      isObserved (int x, int y) const;

      /** Check if a point is a max range (range=INFINITY) - please check isInImage or isObserved first! */
      inline bool
      isMaxRange (int x, int y) const;
      
      /** Calculate the normal of an image point using the neighbors with a maximum pixel distance of radius.
       *  step_size determines how many pixels are used. 1 means all, 2 only every second, etc..
       *  Returns false if it was unable to calculate a normal.*/
      inline bool
      getNormal (int x, int y, int radius, Eigen::Vector3f& normal, int step_size=1) const;
      
      /** Same as above, but only the no_of_nearest_neighbors points closest to the given point are considered. */
      inline bool
      getNormalForClosestNeighbors (int x, int y, int radius, const PointWithRange& point,
                                    int no_of_nearest_neighbors, Eigen::Vector3f& normal, int step_size=1) const;
      
      /** Same as above */
      inline bool
      getNormalForClosestNeighbors (int x, int y, int radius, const Eigen::Vector3f& point,
                                    int no_of_nearest_neighbors, Eigen::Vector3f& normal,
                                    Eigen::Vector3f* point_on_plane=NULL, int step_size=1) const;
      
      /** Same as above, using default values */
      inline bool
      getNormalForClosestNeighbors (int x, int y, Eigen::Vector3f& normal, int radius=2) const;
      
      /** Same as above but extracts some more data and can also return the extracted
       * information for all neighbors in radius if normal_all_neighbors is not NULL */
      inline bool
      getSurfaceInformation (int x, int y, int radius, const Eigen::Vector3f& point,
                             int no_of_closest_neighbors, int step_size,
                             float& max_closest_neighbor_distance_squared,
                             Eigen::Vector3f& normal, Eigen::Vector3f& mean, Eigen::Vector3f& eigen_values,
                             Eigen::Vector3f* normal_all_neighbors=NULL,
                             Eigen::Vector3f* mean_all_neighbors=NULL,
                             Eigen::Vector3f* eigen_values_all_neighbors=NULL) const;
      
      // Return the squared distance to the n-th neighbors of the point at x,y
      inline float
      getSquaredDistanceOfNthNeighbor (int x, int y, int radius, int n, int step_size) const;
      
      /** Calculate the impact angle based on the sensor position and the two given points - will return
       * -INFINITY if one of the points is unobserved */
      inline float
      getImpactAngle (const PointWithRange& point1, const PointWithRange& point2) const;
      //! Same as above
      inline float
      getImpactAngle (int x1, int y1, int x2, int y2) const;
      
      /** Extract a local normal (with a heuristic not to include background points) and calculate the impact
       *  angle based on this */
      inline float
      getImpactAngleBasedOnLocalNormal (int x, int y, int radius) const;
      /** Uses the above function for every point in the image */
      PCL_EXPORTS float*
      getImpactAngleImageBasedOnLocalNormals (int radius) const;

      /** Calculate a score [0,1] that tells how acute the impact angle is (1.0f - getImpactAngle/90deg)
       *  This uses getImpactAngleBasedOnLocalNormal
       *  Will return -INFINITY if no normal could be calculated */
      inline float
      getNormalBasedAcutenessValue (int x, int y, int radius) const;
      
      /** Calculate a score [0,1] that tells how acute the impact angle is (1.0f - getImpactAngle/90deg)
       *  will return -INFINITY if one of the points is unobserved */
      inline float
      getAcutenessValue (const PointWithRange& point1, const PointWithRange& point2) const;
      //! Same as above
      inline float
      getAcutenessValue (int x1, int y1, int x2, int y2) const;
      
      /** Calculate getAcutenessValue for every point */
      PCL_EXPORTS void
      getAcutenessValueImages (int pixel_distance, float*& acuteness_value_image_x,
                               float*& acuteness_value_image_y) const;
      
      /** Calculates, how much the surface changes at a point. Pi meaning a flat suface and 0.0f
       *  would be a needle point */
      //inline float
      //  getSurfaceChange (const PointWithRange& point, const PointWithRange& neighbor1,
      //                   const PointWithRange& neighbor2) const;
      
      /** Calculates, how much the surface changes at a point. 1 meaning a 90deg angle and 0 a flat suface */
      PCL_EXPORTS float
      getSurfaceChange (int x, int y, int radius) const;
      
      /** Uses the above function for every point in the image */
      PCL_EXPORTS float*
      getSurfaceChangeImage (int radius) const;
      
      /** Calculates, how much the surface changes at a point. Returns an angle [0.0f, PI] for x and y direction.
       *  A return value of -INFINITY means that a point was unobserved. */
      inline void
      getSurfaceAngleChange (int x, int y, int radius, float& angle_change_x, float& angle_change_y) const;
      
      /** Uses the above function for every point in the image */
      PCL_EXPORTS void
      getSurfaceAngleChangeImages (int radius, float*& angle_change_image_x, float*& angle_change_image_y) const;
      
      /** Calculates the curvature in a point using pca */
      inline float
      getCurvature (int x, int y, int radius, int step_size) const;
      
      //! Get the sensor position
      inline const Eigen::Vector3f
      getSensorPos () const;
      
      /** Sets all -INFINITY values to INFINITY */
      PCL_EXPORTS void
      setUnseenToMaxRange ();
      
      //! Getter for image_offset_x_
      inline int
      getImageOffsetX () const { return image_offset_x_;}
      //! Getter for image_offset_y_
      inline int
      getImageOffsetY () const { return image_offset_y_;}
      
      //! Setter for image offsets
      inline void
      setImageOffsets (int offset_x, int offset_y) { image_offset_x_=offset_x; image_offset_y_=offset_y;}
 

      
      /** Get a sub part of the complete image as a new range image.
        * \param sub_image_image_offset_x - The x coordinate of the top left pixel of the sub image.
        *                         This is always according to absolute 0,0 meaning -180°,-90°
        *                         and it is already in the system of the new image, so the
        *                         actual pixel used in the original image is
        *                         combine_pixels* (image_offset_x-image_offset_x_)
        * \param sub_image_image_offset_y - Same as image_offset_x for the y coordinate
        * \param sub_image_width - width of the new image
        * \param sub_image_height - height of the new image
        * \param combine_pixels - shrinking factor, meaning the new angular resolution
        *                         is combine_pixels times the old one
        * \param sub_image - the output image
        */
      virtual void
      getSubImage (int sub_image_image_offset_x, int sub_image_image_offset_y, int sub_image_width,
                   int sub_image_height, int combine_pixels, RangeImage& sub_image) const;
      
      //! Get a range image with half the resolution
      virtual void 
      getHalfImage (RangeImage& half_image) const;
      
      //! Find the minimum and maximum range in the image
      PCL_EXPORTS void
      getMinMaxRanges (float& min_range, float& max_range) const;
      
      //! This function sets the sensor pose to 0 and transforms all point positions to this local coordinate frame
      PCL_EXPORTS void
      change3dPointsToLocalCoordinateFrame ();
      
      /** Calculate a range patch as the z values of the coordinate frame given by pose.
       *  The patch will have size pixel_size x pixel_size and each pixel
       *  covers world_size/pixel_size meters in the world
       *  You are responsible for deleting the structure afterwards! */
      PCL_EXPORTS float*
      getInterpolatedSurfaceProjection (const Eigen::Affine3f& pose, int pixel_size, float world_size) const;
      
      //! Same as above, but using the local coordinate frame defined by point and the viewing direction
      PCL_EXPORTS float*
      getInterpolatedSurfaceProjection (const Eigen::Vector3f& point, int pixel_size, float world_size) const;
      
      //! Get the local coordinate frame with 0,0,0 in point, upright and Z as the viewing direction
      inline Eigen::Affine3f
      getTransformationToViewerCoordinateFrame (const Eigen::Vector3f& point) const;
      //! Same as above, using a reference for the retrurn value
      inline void
      getTransformationToViewerCoordinateFrame (const Eigen::Vector3f& point,
                                                Eigen::Affine3f& transformation) const;
      //! Same as above, but only returning the rotation
      inline void
      getRotationToViewerCoordinateFrame (const Eigen::Vector3f& point, Eigen::Affine3f& transformation) const;

      /** Get a local coordinate frame at the given point based on the normal. */
      PCL_EXPORTS bool
      getNormalBasedUprightTransformation (const Eigen::Vector3f& point,
                                           float max_dist, Eigen::Affine3f& transformation) const;
      
      /** Get the integral image of the range values (used for fast blur operations).
       *  You are responsible for deleting it after usage! */
      PCL_EXPORTS void
      getIntegralImage (float*& integral_image, int*& valid_points_num_image) const;
      
      /** Get a blurred version of the range image using box filters on the provided integral image*/
      PCL_EXPORTS void     // Template necessary so that this function also works in derived classes
      getBlurredImageUsingIntegralImage (int blur_radius, float* integral_image, int* valid_points_num_image,
                                         RangeImage& range_image) const;
      
      /** Get a blurred version of the range image using box filters */
      PCL_EXPORTS virtual void     // Template necessary so that this function also works in derived classes
      getBlurredImage (int blur_radius, RangeImage& range_image) const;
      
      /** Get the squared euclidean distance between the two image points.
       *  Returns -INFINITY if one of the points was not observed */
      inline float
      getEuclideanDistanceSquared (int x1, int y1, int x2, int y2) const;
      //! Doing the above for some steps in the given direction and averaging
      inline float
      getAverageEuclideanDistance (int x, int y, int offset_x, int offset_y, int max_steps) const;
      
      //! Project all points on the local plane approximation, thereby smoothing the surface of the scan
      PCL_EXPORTS void
      getRangeImageWithSmoothedSurface (int radius, RangeImage& smoothed_range_image) const;
      //void getLocalNormals (int radius) const;
      
      /** Calculates the average 3D position of the no_of_points points described by the start
       *  point x,y in the direction delta.
       *  Returns a max range point (range=INFINITY) if the first point is max range and an
       *  unobserved point (range=-INFINITY) if non of the points is observed. */
      inline void
      get1dPointAverage (int x, int y, int delta_x, int delta_y, int no_of_points,
                         PointWithRange& average_point) const;
      
      /** Calculates the overlap of two range images given the relative transformation
       *  (from the given image to *this) */
      PCL_EXPORTS float
      getOverlap (const RangeImage& other_range_image, const Eigen::Affine3f& relative_transformation,
                  int search_radius, float max_distance, int pixel_step=1) const;
      
      /** Get the viewing direction for the given point */
      inline bool
      getViewingDirection (int x, int y, Eigen::Vector3f& viewing_direction) const;
      
      /** Get the viewing direction for the given point */
      inline void
      getViewingDirection (const Eigen::Vector3f& point, Eigen::Vector3f& viewing_direction) const;
      
      /** Return a newly created Range image.
       *  Can be reimplmented in derived classes like RangeImagePlanar to return an image of the same type. */
      PCL_EXPORTS virtual RangeImage* 
      getNew () const { return new RangeImage; }

      /** Copy other to *this. Necessary for use in virtual functions that need to copy derived RangeImage classes (like RangeImagePlanar) */
      PCL_EXPORTS virtual void
      copyTo (RangeImage& other) const;

      
      // =====MEMBER VARIABLES=====
      // BaseClass:
      //   roslib::Header header;
      //   std::vector<PointT> points;
      //   uint32_t width;
      //   uint32_t height;
      //   bool is_dense;

      static bool debug; /**< Just for... well... debugging purposes. :-) */
      
    protected:
      // =====PROTECTED MEMBER VARIABLES=====
      Eigen::Affine3f to_range_image_system_;  /**< Inverse of to_world_system_ */
      Eigen::Affine3f to_world_system_;        /**< Inverse of to_range_image_system_ */
      float angular_resolution_x_;             /**< Angular resolution of the range image in x direction in radians per pixel */
      float angular_resolution_y_;             /**< Angular resolution of the range image in y direction in radians per pixel */
      float angular_resolution_x_reciprocal_;  /**< 1.0/angular_resolution_x_ - provided for better performace of
                                                *   multiplication compared to division */
      float angular_resolution_y_reciprocal_;  /**< 1.0/angular_resolution_y_ - provided for better performace of
                                                *   multiplication compared to division */
      int image_offset_x_, image_offset_y_;    /**< Position of the top left corner of the range image compared to
                                                *   an image of full size (360x180 degrees) */
      PointWithRange unobserved_point;         /**< This point is used to be able to return
                                                *   a reference to a non-existing point */
      
      // =====PROTECTED METHODS=====


      // =====STATIC PROTECTED=====
      static const int lookup_table_size;
      static std::vector<float> asin_lookup_table;
      static std::vector<float> atan_lookup_table;
      static std::vector<float> cos_lookup_table;
      /** Create lookup tables for trigonometric functions */
      static void
      createLookupTables ();

      /** Query the asin lookup table */
      static inline float
      asinLookUp (float value);
      
      /** Query the atan2 lookup table */
      static inline float
      atan2LookUp (float y, float x);
     
      /** Query the cos lookup table */
      static inline float
      cosLookUp (float value);


    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /**
    * /ingroup range_image
    */
  inline std::ostream&
  operator<< (std::ostream& os, const RangeImage& r)
  {
    os << "header: " << std::endl;
    os << r.header;
    os << "points[]: " << r.points.size () << std::endl;
    os << "width: " << r.width << std::endl;
    os << "height: " << r.height << std::endl;
    os << "sensor_origin_: "
       << r.sensor_origin_[0] << ' '
       << r.sensor_origin_[1] << ' '
       << r.sensor_origin_[2] << std::endl;
    os << "sensor_orientation_: "
       << r.sensor_orientation_.x () << ' '
       << r.sensor_orientation_.y () << ' '
       << r.sensor_orientation_.z () << ' '
       << r.sensor_orientation_.w () << std::endl;
    os << "is_dense: " << r.is_dense << std::endl;
    os << "angular resolution: "
       << RAD2DEG (r.getAngularResolutionX ()) << "deg/pixel in x and "
       << RAD2DEG (r.getAngularResolutionY ()) << "deg/pixel in y.\n" << std::endl;
    return (os);
  }

}  // namespace end


#include <pcl/range_image/impl/range_image.hpp>  // Definitions of templated and inline functions

#endif  //#ifndef PCL_RANGE_IMAGE_H_
