/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_SURFACE_ORGANIZED_FAST_MESH_H_
#define PCL_SURFACE_ORGANIZED_FAST_MESH_H_

#include <pcl/common/angles.h>
#include <pcl/surface/reconstruction.h>

namespace pcl
{

  /** \brief Simple triangulation/surface reconstruction for organized point
    * clouds. Neighboring points (pixels in image space) are connected to
    * construct a triangular (or quad) mesh.
    *
    * \note If you use this code in any academic work, please cite:
    *   D. Holz and S. Behnke.
    *   Fast Range Image Segmentation and Smoothing using Approximate Surface Reconstruction and Region Growing.
    *   In Proceedings of the 12th International Conference on Intelligent Autonomous Systems (IAS),
    *   Jeju Island, Korea, June 26-29 2012.
    *   <a href="http://purl.org/holz/papers/holz_2012_ias.pdf">http://purl.org/holz/papers/holz_2012_ias.pdf</a>
    * 
    * \author Dirk Holz, Radu B. Rusu
    * \ingroup surface
    */
  template <typename PointInT>
  class OrganizedFastMesh : public MeshConstruction<PointInT>
  {
    public:
      typedef boost::shared_ptr<OrganizedFastMesh<PointInT> > Ptr;
      typedef boost::shared_ptr<const OrganizedFastMesh<PointInT> > ConstPtr;

      using MeshConstruction<PointInT>::input_;
      using MeshConstruction<PointInT>::check_tree_;

      typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudPtr;

      typedef std::vector<pcl::Vertices> Polygons;

      enum TriangulationType
      {
        TRIANGLE_RIGHT_CUT,     // _always_ "cuts" a quad from top left to bottom right
        TRIANGLE_LEFT_CUT,      // _always_ "cuts" a quad from top right to bottom left
        TRIANGLE_ADAPTIVE_CUT,  // "cuts" where possible and prefers larger differences in 'z' direction
        QUAD_MESH               // create a simple quad mesh
      };

      /** \brief Constructor. Triangulation type defaults to \a QUAD_MESH. */
      OrganizedFastMesh ()
      : max_edge_length_squared_ (0.025f)
      , triangle_pixel_size_rows_ (1)
      , triangle_pixel_size_columns_ (1)
      , triangulation_type_ (QUAD_MESH)
      , store_shadowed_faces_ (false)
      , cos_angle_tolerance_ (fabsf (cosf (pcl::deg2rad (12.5f))))
      {
        check_tree_ = false;
      };

      /** \brief Destructor. */
      virtual ~OrganizedFastMesh () {};

      /** \brief Set a maximum edge length. TODO: Implement!
        * \param[in] max_edge_length the maximum edge length
        */
      inline void
      setMaxEdgeLength (float max_edge_length)
      {
        max_edge_length_squared_ = max_edge_length * max_edge_length;
      };

      /** \brief Set the edge length (in pixels) used for constructing the fixed mesh.
        * \param[in] triangle_size edge length in pixels
        * (Default: 1 = neighboring pixels are connected)
        */
      inline void
      setTrianglePixelSize (int triangle_size)
      {
        setTrianglePixelSizeRows (triangle_size);
        setTrianglePixelSizeColumns (triangle_size);
      }

      /** \brief Set the edge length (in pixels) used for iterating over rows when constructing the fixed mesh.
        * \param[in] triangle_size edge length in pixels
        * (Default: 1 = neighboring pixels are connected)
        */
      inline void
      setTrianglePixelSizeRows (int triangle_size)
      {
        triangle_pixel_size_rows_ = std::max (1, (triangle_size - 1));
      }

      /** \brief Set the edge length (in pixels) used for iterating over columns when constructing the fixed mesh.
        * \param[in] triangle_size edge length in pixels
        * (Default: 1 = neighboring pixels are connected)
        */
      inline void
      setTrianglePixelSizeColumns (int triangle_size)
      {
        triangle_pixel_size_columns_ = std::max (1, (triangle_size - 1));
      }

      /** \brief Set the triangulation type (see \a TriangulationType)
        * \param[in] type quad mesh, triangle mesh with fixed left, right cut,
        * or adaptive cut (splits a quad w.r.t. the depth (z) of the points)
        */
      inline void
      setTriangulationType (TriangulationType type)
      {
        triangulation_type_ = type;
      }

      /** \brief Store shadowed faces or not.
        * \param[in] enable set to true to store shadowed faces
        */
      inline void
      storeShadowedFaces (bool enable)
      {
        store_shadowed_faces_ = enable;
      }

    protected:
      /** \brief max (squared) length of edge */
      float max_edge_length_squared_;

      /** \brief size of triangle edges (in pixels) for iterating over rows. */
      int triangle_pixel_size_rows_;

      /** \brief size of triangle edges (in pixels) for iterating over columns*/
      int triangle_pixel_size_columns_;

      /** \brief Type of meshing scheme (quads vs. triangles, left cut vs. right cut ... */
      TriangulationType triangulation_type_;

      /** \brief Whether or not shadowed faces are stored, e.g., for exploration */
      bool store_shadowed_faces_;

      /** \brief (Cosine of the) angle tolerance used when checking whether or not an edge between two points is shadowed. */
      float cos_angle_tolerance_;

      /** \brief Perform the actual polygonal reconstruction.
        * \param[out] polygons the resultant polygons
        */
      void
      reconstructPolygons (std::vector<pcl::Vertices>& polygons);

      /** \brief Create the surface.
        * \param[out] polygons the resultant polygons, as a set of vertices. The Vertices structure contains an array of point indices.
        */
      virtual void
      performReconstruction (std::vector<pcl::Vertices> &polygons);

      /** \brief Create the surface.
        *
        * Simply uses image indices to create an initial polygonal mesh for organized point clouds.
        * \a indices_ are ignored!
        *
        * \param[out] output the resultant polygonal mesh
        */
      void
      performReconstruction (pcl::PolygonMesh &output);

      /** \brief Add a new triangle to the current polygon mesh
        * \param[in] a index of the first vertex
        * \param[in] b index of the second vertex
        * \param[in] c index of the third vertex
        * \param[in] idx the index in the set of polygon vertices (assumes \a idx is valid in \a polygons)
        * \param[out] polygons the polygon mesh to be updated
        */
      inline void
      addTriangle (int a, int b, int c, int idx, std::vector<pcl::Vertices>& polygons)
      {
        assert (idx < static_cast<int> (polygons.size ()));
        polygons[idx].vertices.resize (3);
        polygons[idx].vertices[0] = a;
        polygons[idx].vertices[1] = b;
        polygons[idx].vertices[2] = c;
      }

      /** \brief Add a new quad to the current polygon mesh
        * \param[in] a index of the first vertex
        * \param[in] b index of the second vertex
        * \param[in] c index of the third vertex
        * \param[in] d index of the fourth vertex
        * \param[in] idx the index in the set of polygon vertices (assumes \a idx is valid in \a polygons)
        * \param[out] polygons the polygon mesh to be updated
        */
      inline void
      addQuad (int a, int b, int c, int d, int idx, std::vector<pcl::Vertices>& polygons)
      {
        assert (idx < static_cast<int> (polygons.size ()));
        polygons[idx].vertices.resize (4);
        polygons[idx].vertices[0] = a;
        polygons[idx].vertices[1] = b;
        polygons[idx].vertices[2] = c;
        polygons[idx].vertices[3] = d;
      }

      /** \brief Set (all) coordinates of a particular point to the specified value
        * \param[in] point_index index of point
        * \param[out] mesh to modify
        * \param[in] value value to use when re-setting
        * \param[in] field_x_idx the X coordinate of the point
        * \param[in] field_y_idx the Y coordinate of the point
        * \param[in] field_z_idx the Z coordinate of the point
        */
      inline void
      resetPointData (const int &point_index, pcl::PolygonMesh &mesh, const float &value = 0.0f,
                      int field_x_idx = 0, int field_y_idx = 1, int field_z_idx = 2)
      {
        float new_value = value;
        memcpy (&mesh.cloud.data[point_index * mesh.cloud.point_step + mesh.cloud.fields[field_x_idx].offset], &new_value, sizeof (float));
        memcpy (&mesh.cloud.data[point_index * mesh.cloud.point_step + mesh.cloud.fields[field_y_idx].offset], &new_value, sizeof (float));
        memcpy (&mesh.cloud.data[point_index * mesh.cloud.point_step + mesh.cloud.fields[field_z_idx].offset], &new_value, sizeof (float));
      }

      /** \brief Check if a point is shadowed by another point
        * \param[in] point_a the first point
        * \param[in] point_b the second point
        */
      inline bool
      isShadowed (const PointInT& point_a, const PointInT& point_b)
      {
        Eigen::Vector3f viewpoint = Eigen::Vector3f::Zero (); // TODO: allow for passing viewpoint information
        Eigen::Vector3f dir_a = viewpoint - point_a.getVector3fMap ();
        Eigen::Vector3f dir_b = point_b.getVector3fMap () - point_a.getVector3fMap ();
        float distance_to_points = dir_a.norm ();
        float distance_between_points = dir_b.norm ();
        float cos_angle = dir_a.dot (dir_b) / (distance_to_points*distance_between_points);
        if (cos_angle != cos_angle)
          cos_angle = 1.0f;
        return (fabs (cos_angle) >= cos_angle_tolerance_);
        // TODO: check for both: angle almost 0/180 _and_ distance between points larger than noise level
      }

      /** \brief Check if a triangle is valid.
        * \param[in] a index of the first vertex
        * \param[in] b index of the second vertex
        * \param[in] c index of the third vertex
        */
      inline bool
      isValidTriangle (const int& a, const int& b, const int& c)
      {
        if (!pcl::isFinite (input_->points[a])) return (false);
        if (!pcl::isFinite (input_->points[b])) return (false);
        if (!pcl::isFinite (input_->points[c])) return (false);
        return (true);
      }

      /** \brief Check if a triangle is shadowed.
        * \param[in] a index of the first vertex
        * \param[in] b index of the second vertex
        * \param[in] c index of the third vertex
        */
      inline bool
      isShadowedTriangle (const int& a, const int& b, const int& c)
      {
        if (isShadowed (input_->points[a], input_->points[b])) return (true);
        if (isShadowed (input_->points[b], input_->points[c])) return (true);
        if (isShadowed (input_->points[c], input_->points[a])) return (true);
        return (false);
      }

      /** \brief Check if a quad is valid.
        * \param[in] a index of the first vertex
        * \param[in] b index of the second vertex
        * \param[in] c index of the third vertex
        * \param[in] d index of the fourth vertex
        */
      inline bool
      isValidQuad (const int& a, const int& b, const int& c, const int& d)
      {
        if (!pcl::isFinite (input_->points[a])) return (false);
        if (!pcl::isFinite (input_->points[b])) return (false);
        if (!pcl::isFinite (input_->points[c])) return (false);
        if (!pcl::isFinite (input_->points[d])) return (false);
        return (true);
      }

      /** \brief Check if a triangle is shadowed.
        * \param[in] a index of the first vertex
        * \param[in] b index of the second vertex
        * \param[in] c index of the third vertex
        * \param[in] d index of the fourth vertex
        */
      inline bool
      isShadowedQuad (const int& a, const int& b, const int& c, const int& d)
      {
        if (isShadowed (input_->points[a], input_->points[b])) return (true);
        if (isShadowed (input_->points[b], input_->points[c])) return (true);
        if (isShadowed (input_->points[c], input_->points[d])) return (true);
        if (isShadowed (input_->points[d], input_->points[a])) return (true);
        return (false);
      }

      /** \brief Create a quad mesh.
        * \param[out] polygons the resultant mesh
        */
      void
      makeQuadMesh (std::vector<pcl::Vertices>& polygons);

      /** \brief Create a right cut mesh.
        * \param[out] polygons the resultant mesh
        */
      void
      makeRightCutMesh (std::vector<pcl::Vertices>& polygons);

      /** \brief Create a left cut mesh.
        * \param[out] polygons the resultant mesh
        */
      void
      makeLeftCutMesh (std::vector<pcl::Vertices>& polygons);

      /** \brief Create an adaptive cut mesh.
        * \param[out] polygons the resultant mesh
        */
      void
      makeAdaptiveCutMesh (std::vector<pcl::Vertices>& polygons);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#endif

#endif  // PCL_SURFACE_ORGANIZED_FAST_MESH_H_
