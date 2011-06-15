/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
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

#ifndef PCL_SURFACE_ORGANIZED_FAST_MESH_H_
#define PCL_SURFACE_ORGANIZED_FAST_MESH_H_

#include <pcl/common/angles.h>
#include <pcl/io/io.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/kdtree/kdtree_flann.h> // just for faking a search tree (is not needed)

namespace pcl
{

  /** \brief Simple triangulation/surface reconstruction for organized point
   *  clouds. Neighboring points (pixels in image space) are connected to
   *  construct a triangular mesh.
    * \author Dirk Holz
    * \ingroup surface
    */
  template <typename PointInT>
  class OrganizedFastMesh : public SurfaceReconstruction<PointInT>
  {
    public:
      using SurfaceReconstruction<PointInT>::input_;
      using SurfaceReconstruction<PointInT>::check_tree_;

      typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudPtr;

      OrganizedFastMesh()
      : max_edge_length_squared_(0.025f)
      , triangle_pixel_size_(1)
      {
        check_tree_ = false; // tree_.reset(new pcl::KdTreeFLANN<PointInT>());
      };
      ~OrganizedFastMesh(){};

      /** \brief Create the surface.
        *
        * Simply uses image indices to create an initial polygonal mesh for organized point clouds.
        * \a indices_ are ignored!
        *
        * \param output the resultant polygonal mesh
        */
      void
      performReconstruction (pcl::PolygonMesh &output);

      /** \brief Set a maximum edge length. TODO: Implement! */
      inline void
      setMaxEdgeLength(float max_edge_length)
      {
        max_edge_length_squared_ = max_edge_length*max_edge_length;
      };

      /** Set the edge length (in pixels) used for constructing the fixed mesh. */
      inline void
      setTrianglePixelSize(int triangle_size)
      {
        triangle_pixel_size_ = std::max(1, (triangle_size - 1));
      }

    protected:

      /** \brief Temporary variable to store a triangle **/
      pcl::Vertices triangle_;

      /** \brief max (squared) length of edge */
      float max_edge_length_squared_;

      /** \brief size of triangle endges (in pixels) */
      int triangle_pixel_size_;

      /** \brief Add a new triangle to the current polygon mesh
        * \param a index of the first vertex
        * \param b index of the second vertex
        * \param c index of the third vertex
        * \param output the polygon mesh to be updated
        */
      inline void
      addTriangle (int a, int b, int c, pcl::PolygonMesh &output)
      {
        triangle_.vertices.clear ();
        triangle_.vertices.push_back (a);
        triangle_.vertices.push_back (b);
        triangle_.vertices.push_back (c);
        output.polygons.push_back (triangle_);
      }


      /** \brief Set (all) coordinates of a particular point to the specified value
        * \param point_index index of point
        * \param mesh to modify
        * \param value value to use when re-setting
        */
      inline void
      resetPointData (const int &point_index, pcl::PolygonMesh &mesh, const float &value = 0.0f,
                      int field_x_idx = 0, int field_y_idx = 1, int field_z_idx = 2)
      {
        float new_value = value;
        memcpy(&mesh.cloud.data[point_index * mesh.cloud.point_step + mesh.cloud.fields[field_x_idx].offset], &new_value, sizeof(float));
        memcpy(&mesh.cloud.data[point_index * mesh.cloud.point_step + mesh.cloud.fields[field_y_idx].offset], &new_value, sizeof(float));
        memcpy(&mesh.cloud.data[point_index * mesh.cloud.point_step + mesh.cloud.fields[field_z_idx].offset], &new_value, sizeof(float));
      }

      inline bool
      isShadowed(const PointInT& point_a, const PointInT& point_b)
      {
          Eigen::Vector3f viewpoint(0.0f, 0.0f, 0.0f); // TODO: allow for passing viewpoint information
          Eigen::Vector3f dir_a = viewpoint - point_a.getVector3fMap();
          Eigen::Vector3f dir_b = point_b.getVector3fMap() - point_a.getVector3fMap();
          float angle = acos( dir_a.dot(dir_b) / (dir_a.norm()*dir_b.norm()) );
          if ( angle != angle ) angle = 0.0f;
          float angle_tolerance = pcl::deg2rad(15.0f); // (10 * pi) / 180
          return ( (angle < angle_tolerance) || (angle > (M_PI - angle_tolerance)) );
      }

      inline bool
      isValidTriangle (const PointInT& point_a, const PointInT& point_b, const PointInT& point_c)
      {
        if ( !pcl::hasValidXYZ(point_a) ) return false;
        if ( !pcl::hasValidXYZ(point_b) ) return false;
        if ( !pcl::hasValidXYZ(point_c) ) return false;

//        if ( dist(point_a, point_b) > max_edge_length_squared_ ) return false;
//        if ( dist(point_b, point_c) > max_edge_length_squared_ ) return false;
//        if ( dist(point_c, point_a) > max_edge_length_squared_ ) return false;

        if ( isShadowed(point_a, point_b) ) return false;
        if ( isShadowed(point_b, point_c) ) return false;
        if ( isShadowed(point_c, point_a) ) return false;

        return true;
      }


      inline int
      getIndex(int x, int y)
      {
        return (int)(y * input_->width + x);
      }

      inline float
      dist(const PointInT& point_a, const PointInT& point_b)
      {
        Eigen::Vector3f v = point_a.getVector3fMap() - point_b.getVector3fMap();
        return v.squaredNorm();
      }



  };

}

#endif  // PCL_SURFACE_ORGANIZED_FAST_MESH_H_
