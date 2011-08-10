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

      typedef std::vector<pcl::Vertices> Polygons;

      enum TriangulationType
      {
        TRIANGLE_RIGHT_CUT,     // _always_ "cuts" a quad from top left to bottom right
        TRIANGLE_LEFT_CUT,      // _always_ "cuts" a quad from top right to bottom left
        TRIANGLE_ADAPTIVE_CUT,  // "cuts" where possible and prefers larger differences in 'z' direction
        QUAD_MESH               // create a simple quad mesh
      };

      OrganizedFastMesh()
      : max_edge_length_squared_ (0.025f)
      , triangle_pixel_size_ (1)
      , triangulation_type_ (TRIANGLE_RIGHT_CUT)
      , store_shadowed_faces_(false)
      {
        check_tree_ = false;
        cos_angle_tolerance_ = fabs(cos(pcl::deg2rad(12.5f)));
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

      void
      reconstructPolygons (std::vector<pcl::Vertices>& polygons);

      /** \brief Set a maximum edge length. TODO: Implement! */
      inline void
      setMaxEdgeLength(float max_edge_length)
      {
        max_edge_length_squared_ = max_edge_length*max_edge_length;
      };

      /**
       * \brief Set the edge length (in pixels) used for constructing the fixed mesh.
       * \param triangle_size edge length in pixels
       * (Default: 1 = neighboring pixels are connected)
       */
      inline void
      setTrianglePixelSize(int triangle_size)
      {
        triangle_pixel_size_ = std::max(1, (triangle_size - 1));
      }

      /**
       * \brief Set the triangulation type (see \a TriangulationType)
       * @param type quad mesh, triangle mesh with fixed left, right cut,
       * or adaptive cut (splits a quad wrt. the depth (z) of the points)
       */
      inline void
      setTriangulationType(TriangulationType type)
      {
        triangulation_type_ = type;
      }

      inline void
      storeShadowedFaces(bool enable)
      {
        store_shadowed_faces_ = enable;
      }

    protected:

      /** \brief Temporary variable to store a triangle **/
      pcl::Vertices triangle_;

      /** \brief Temporary variable to store a quad **/
      pcl::Vertices quad_;

      /** \brief max (squared) length of edge */
      float max_edge_length_squared_;

      /** \brief size of triangle endges (in pixels) */
      int triangle_pixel_size_;

      /** \brief Type of meshin scheme (quads vs. triangles, left cut vs. right cut ... */
      TriangulationType triangulation_type_;

      /** \brief Whether or not shadowed faces are stored, e.g., for exploration */
      bool store_shadowed_faces_;

      float cos_angle_tolerance_;

      /** \brief Add a new triangle to the current polygon mesh
        * \param a index of the first vertex
        * \param b index of the second vertex
        * \param c index of the third vertex
        * \param output the polygon mesh to be updated
        */
      inline void
      addTriangle (int a, int b, int c, std::vector<pcl::Vertices>& polygons)
      {
        if (isShadowedTriangle(a, b, c))
          return;

        triangle_.vertices.clear ();
        triangle_.vertices.push_back (a);
        triangle_.vertices.push_back (b);
        triangle_.vertices.push_back (c);
        polygons.push_back (triangle_);
      }

      inline void
      addQuad(int a, int b, int c, int d, std::vector<pcl::Vertices>& polygons)
      {
        quad_.vertices.clear ();
        quad_.vertices.push_back (a);
        quad_.vertices.push_back (b);
        quad_.vertices.push_back (c);
        quad_.vertices.push_back (d);
        polygons.push_back (quad_);
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
        float distance_to_points = dir_a.norm();
        float distance_between_points = dir_b.norm();
        float cos_angle = dir_a.dot(dir_b) / (distance_to_points*distance_between_points);
        if (cos_angle != cos_angle) cos_angle = 1.0f;
        return ( fabs(cos_angle) >= cos_angle_tolerance_ );
        // TODO: check for both: angle almost 0/180 _and_ distance between points larger than noise level
      }

      inline bool
      isValidTriangle(const int& a, const int& b, const int& c)
      {
        if ( !pcl::hasValidXYZ(input_->points[a]) ) return false;
        if ( !pcl::hasValidXYZ(input_->points[b]) ) return false;
        if ( !pcl::hasValidXYZ(input_->points[c]) ) return false;
        return true;
      }

      inline bool
      isShadowedTriangle(const int& a, const int& b, const int& c)
      {
        if ( isShadowed(input_->points[a], input_->points[b]) ) return true;
        if ( isShadowed(input_->points[b], input_->points[c]) ) return true;
        if ( isShadowed(input_->points[c], input_->points[a]) ) return true;
        return false;
      }

      inline bool
      isValidQuad(const int& a, const int& b, const int& c, const int& d)
      {
        if ( !pcl::hasValidXYZ(input_->points[a]) ) return false;
        if ( !pcl::hasValidXYZ(input_->points[b]) ) return false;
        if ( !pcl::hasValidXYZ(input_->points[c]) ) return false;
        if ( !pcl::hasValidXYZ(input_->points[d]) ) return false;
        return true;
      }

      inline bool
      isShadowedQuad(const int& a, const int& b, const int& c, const int& d)
      {
        if ( isShadowed(input_->points[a], input_->points[b]) ) return true;
        if ( isShadowed(input_->points[b], input_->points[c]) ) return true;
        if ( isShadowed(input_->points[c], input_->points[d]) ) return true;
        if ( isShadowed(input_->points[d], input_->points[a]) ) return true;
        return false;
      }

      inline int
      getIndex(int x, int y)
      {
        return (int)(y * input_->width + x);
      }

      void
      makeQuadMesh (std::vector<pcl::Vertices>& polygons);

      void
      makeRightCutMesh (std::vector<pcl::Vertices>& polygons);

      void
      makeLeftCutMesh (std::vector<pcl::Vertices>& polygons);

      void
      makeAdaptiveCutMesh (std::vector<pcl::Vertices>& polygons);

  };

}

#endif  // PCL_SURFACE_ORGANIZED_FAST_MESH_H_
