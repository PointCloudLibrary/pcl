/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/common/transforms.h>
#include <pcl/TextureMesh.h>
#include <pcl/octree/octree_search.h> // for OctreePointCloudSearch


namespace pcl
{
  namespace texture_mapping
  {
        
    /** \brief Structure to store camera pose and focal length. 
      *
      * One can assign a value to focal_length, to be used along 
      * both camera axes or, optionally, axis-specific values 
      * (focal_length_w and focal_length_h). Optionally, one can 
      * also specify center-of-focus using parameters
      * center_w and center_h. If the center-of-focus is not 
      * specified, it will be set to the geometric center of 
      * the camera, as defined by the width and height parameters.
      */
    struct Camera
    {
      Camera () : focal_length (), focal_length_w (-1), focal_length_h (-1),
        center_w (-1), center_h (-1), height (), width () {}
      Eigen::Affine3f pose;
      double focal_length;
      double focal_length_w;  // optional
      double focal_length_h;  // optinoal
      double center_w;  // optional
      double center_h;  // optional
      double height;
      double width;
      std::string texture_file;

      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** \brief Structure that links a uv coordinate to its 3D point and face.
      */
    struct UvIndex
    {
      UvIndex () : idx_cloud (), idx_face () {}
      int idx_cloud; // Index of the PointXYZ in the camera's cloud
      int idx_face; // Face corresponding to that projection
    };
    
    using CameraVector = std::vector<Camera, Eigen::aligned_allocator<Camera> >;
    
  }
  
  /** \brief The texture mapping algorithm
    * \author Khai Tran, Raphael Favier
    * \ingroup surface
    */
  template<typename PointInT>
  class TextureMapping
  {
    public:
     
      using Ptr = shared_ptr<TextureMapping<PointInT> >;
      using ConstPtr = shared_ptr<const TextureMapping<PointInT> >;

      using PointCloud = pcl::PointCloud<PointInT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      using Octree = pcl::octree::OctreePointCloudSearch<PointInT>;
      using OctreePtr = typename Octree::Ptr;
      using OctreeConstPtr = typename Octree::ConstPtr;
      
      using Camera = pcl::texture_mapping::Camera;
      using UvIndex = pcl::texture_mapping::UvIndex;

      /** \brief Constructor. */
      TextureMapping () :
        f_ ()
      {
      }

      /** \brief Destructor. */
      ~TextureMapping ()
      {
      }

      /** \brief Set mesh scale control
        * \param[in] f
        */
      inline void
      setF (float f)
      {
        f_ = f;
      }

      /** \brief Set vector field
        * \param[in] x data point x
        * \param[in] y data point y
        * \param[in] z data point z
        */
      inline void
      setVectorField (float x, float y, float z)
      {
        vector_field_ = Eigen::Vector3f (x, y, z);
        // normalize vector field
        vector_field_ /= std::sqrt (vector_field_.dot (vector_field_));
      }

      /** \brief Set texture files
        * \param[in] tex_files list of texture files
        */
      inline void
      setTextureFiles (std::vector<std::string> tex_files)
      {
        tex_files_ = tex_files;
      }

      /** \brief Set texture materials
        * \param[in] tex_material texture material
        */
      inline void
      setTextureMaterials (TexMaterial tex_material)
      {
        tex_material_ = tex_material;
      }

      /** \brief Map texture to a mesh synthesis algorithm
        * \param[in] tex_mesh texture mesh
        */
      void
      mapTexture2Mesh (pcl::TextureMesh &tex_mesh);

      /** \brief Map texture to a mesh UV mapping
        * \param[in] tex_mesh texture mesh
        */
      void
      mapTexture2MeshUV (pcl::TextureMesh &tex_mesh);

      /** \brief Map textures acquired from a set of cameras onto a mesh.
        * \details With UV mapping, the mesh must be divided into NbCamera + 1 sub-meshes.
        * Each sub-mesh corresponding to the faces visible by one camera. The last submesh containing all non-visible faces
        * \param[in] tex_mesh texture mesh
        * \param[in] cams cameras used for UV mapping
        */
      void
      mapMultipleTexturesToMeshUV (pcl::TextureMesh &tex_mesh, 
                                   pcl::texture_mapping::CameraVector &cams);

      /** \brief computes UV coordinates of point, observed by one particular camera
        * \param[in] pt XYZ point to project on camera plane
        * \param[in] cam the camera used for projection
        * \param[out] UV_coordinates the resulting uv coordinates. Set to (-1.0,-1.0) if the point is not visible by the camera
        * \returns false if the point is not visible by the camera
        */
      inline bool
      getPointUVCoordinates (const PointInT &pt, const Camera &cam, Eigen::Vector2f &UV_coordinates)
      {
        // if the point is in front of the camera
        if (pt.z > 0)
        {
          // compute image center and dimension
          double sizeX = cam.width;
          double sizeY = cam.height;
          double cx, cy;
          if (cam.center_w > 0)
            cx = cam.center_w;
          else
            cx = (sizeX) / 2.0;
          if (cam.center_h > 0)
            cy = cam.center_h;
          else
            cy = (sizeY) / 2.0;

          double focal_x, focal_y;
          if (cam.focal_length_w > 0)
            focal_x = cam.focal_length_w;
          else
            focal_x = cam.focal_length;
          if (cam.focal_length_h>0)
            focal_y = cam.focal_length_h;
          else
            focal_y = cam.focal_length;

          // project point on image frame
          UV_coordinates[0] = static_cast<float> ((focal_x * (pt.x / pt.z) + cx) / sizeX); //horizontal
          UV_coordinates[1] = 1.0f - static_cast<float> (((focal_y * (pt.y / pt.z) + cy) / sizeY)); //vertical

          // point is visible!
          if (UV_coordinates[0] >= 0.0 && UV_coordinates[0] <= 1.0 && UV_coordinates[1] >= 0.0 && UV_coordinates[1]
                                                                                                                 <= 1.0)
            return (true);
        }

        // point is NOT visible by the camera
        UV_coordinates[0] = -1.0;
        UV_coordinates[1] = -1.0;
        return (false);
      }

      /** \brief Check if a point is occluded using raycasting on octree.
        * \param[in] pt XYZ from which the ray will start (toward the camera)
        * \param[in] octree the octree used for raycasting. It must be initialized with a cloud transformed into the camera's frame
        * \returns true if the point is occluded.
        */
      inline bool
      isPointOccluded (const PointInT &pt, const OctreePtr octree);

      /** \brief Remove occluded points from a point cloud
        * \param[in] input_cloud the cloud on which to perform occlusion detection
        * \param[out] filtered_cloud resulting cloud, containing only visible points
        * \param[in] octree_voxel_size octree resolution (in meters)
        * \param[out] visible_indices will contain indices of visible points
        * \param[out] occluded_indices will contain indices of occluded points
        */
      void
      removeOccludedPoints (const PointCloudPtr &input_cloud,
                            PointCloudPtr &filtered_cloud, const double octree_voxel_size,
                            pcl::Indices &visible_indices, pcl::Indices &occluded_indices);

      /** \brief Remove occluded points from a textureMesh
        * \param[in] tex_mesh input mesh, on witch to perform occlusion detection
        * \param[out] cleaned_mesh resulting mesh, containing only visible points
        * \param[in] octree_voxel_size octree resolution (in meters)
        */
      void
      removeOccludedPoints (const pcl::TextureMesh &tex_mesh, pcl::TextureMesh &cleaned_mesh, const double octree_voxel_size);


      /** \brief Remove occluded points from a textureMesh
        * \param[in] tex_mesh input mesh, on witch to perform occlusion detection
        * \param[out] filtered_cloud resulting cloud, containing only visible points
        * \param[in] octree_voxel_size octree resolution (in meters)
        */
      void
      removeOccludedPoints (const pcl::TextureMesh &tex_mesh, PointCloudPtr &filtered_cloud, const double octree_voxel_size);


      /** \brief Segment faces by camera visibility. Point-based segmentation.
        * \details With N camera, faces will be arranged into N+1 groups: 1 for each camera, plus 1 for faces not visible from any camera.
        * \param[in] tex_mesh input mesh that needs sorting. Must contain only 1 sub-mesh.
        * \param[in] sorted_mesh resulting mesh, will contain nbCamera + 1 sub-mesh.
        * \param[in] cameras vector containing the cameras used for texture mapping.
        * \param[in] octree_voxel_size octree resolution (in meters)
        * \param[out] visible_pts cloud containing only visible points
        */
      int
      sortFacesByCamera (pcl::TextureMesh &tex_mesh, 
                         pcl::TextureMesh &sorted_mesh, 
                         const pcl::texture_mapping::CameraVector &cameras,
                         const double octree_voxel_size, PointCloud &visible_pts);

      /** \brief Colors a point cloud, depending on its occlusions.
        * \details If showNbOcclusions is set to True, each point is colored depending on the number of points occluding it.
        * Else, each point is given a different a 0 value is not occluded, 1 if occluded.
        * By default, the number of occlusions is bounded to 4.
        * \param[in] input_cloud input cloud on which occlusions will be computed.
        * \param[out] colored_cloud resulting colored cloud showing the number of occlusions per point.
        * \param[in] octree_voxel_size octree resolution (in meters).
        * \param[in] show_nb_occlusions If false, color information will only represent.
        * \param[in] max_occlusions Limit the number of occlusions per point.
        */
      void
      showOcclusions (const PointCloudPtr &input_cloud, 
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &colored_cloud,
                      const double octree_voxel_size, 
                      const bool show_nb_occlusions = true,
                      const int max_occlusions = 4);

      /** \brief Colors the point cloud of a Mesh, depending on its occlusions.
        * \details If showNbOcclusions is set to True, each point is colored depending on the number of points occluding it.
        * Else, each point is given a different a 0 value is not occluded, 1 if occluded.
        * By default, the number of occlusions is bounded to 4.
        * \param[in] tex_mesh input mesh on which occlusions will be computed.
        * \param[out] colored_cloud resulting colored cloud showing the number of occlusions per point.
        * \param[in] octree_voxel_size octree resolution (in meters).
        * \param[in] show_nb_occlusions If false, color information will only represent.
        * \param[in] max_occlusions Limit the number of occlusions per point.
        */
      void
      showOcclusions (pcl::TextureMesh &tex_mesh, 
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &colored_cloud,
                      double octree_voxel_size, 
                      bool show_nb_occlusions = true, 
                      int max_occlusions = 4);

      /** \brief Segment and texture faces by camera visibility. Face-based segmentation.
        * \details With N camera, faces will be arranged into N+1 groups: 1 for each camera, plus 1 for faces not visible from any camera.
        * The mesh will also contain uv coordinates for each face
        * \param mesh input mesh that needs sorting. Should contain only 1 sub-mesh.
        * \param[in] cameras vector containing the cameras used for texture mapping.
        */
      void 
      textureMeshwithMultipleCameras (pcl::TextureMesh &mesh, 
                                      const pcl::texture_mapping::CameraVector &cameras);

    protected:
      /** \brief mesh scale control. */
      float f_;

      /** \brief vector field */
      Eigen::Vector3f vector_field_;

      /** \brief list of texture files */
      std::vector<std::string> tex_files_;

      /** \brief list of texture materials */
      TexMaterial tex_material_;

      /** \brief Map texture to a face
        * \param[in] p1 the first point
        * \param[in] p2 the second point
        * \param[in] p3 the third point
        */
      std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
      mapTexture2Face (const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3);

      /** \brief Returns the circumcenter of a triangle and the circle's radius.
        * \details see http://en.wikipedia.org/wiki/Circumcenter for formulas.
        * \param[in] p1 first point of the triangle.
        * \param[in] p2 second point of the triangle.
        * \param[in] p3 third point of the triangle.
        * \param[out] circumcenter resulting circumcenter
        * \param[out] radius the radius of the circumscribed circle.
        */
      inline void
      getTriangleCircumcenterAndSize (const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius);
 
      
      /** \brief Returns the centroid of a triangle and the corresponding circumscribed circle's radius.
        * \details yield a tighter circle than getTriangleCircumcenterAndSize.
        * \param[in] p1 first point of the triangle.
        * \param[in] p2 second point of the triangle.
        * \param[in] p3 third point of the triangle.
        * \param[out] circumcenter resulting circumcenter
        * \param[out] radius the radius of the circumscribed circle.
        */
      inline void 
      getTriangleCircumcscribedCircleCentroid ( const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius);
 

      /** \brief computes UV coordinates of point, observed by one particular camera
        * \param[in] pt XYZ point to project on camera plane
        * \param[in] cam the camera used for projection
        * \param[out] UV_coordinates the resulting UV coordinates. Set to (-1.0,-1.0) if the point is not visible by the camera
        * \returns false if the point is not visible by the camera
        */
      inline bool
      getPointUVCoordinates (const PointInT &pt, const Camera &cam, pcl::PointXY &UV_coordinates);

      /** \brief Returns true if all the vertices of one face are projected on the camera's image plane.
        * \param[in] camera camera on which to project the face.
        * \param[in] p1 first point of the face.
        * \param[in] p2 second point of the face.
        * \param[in] p3 third point of the face.
        * \param[out] proj1 UV coordinates corresponding to p1.
        * \param[out] proj2 UV coordinates corresponding to p2.
        * \param[out] proj3 UV coordinates corresponding to p3.
        */
      inline bool
      isFaceProjected (const Camera &camera, 
                       const PointInT &p1, const PointInT &p2, const PointInT &p3, 
                       pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3);

      /** \brief Returns True if a point lays within a triangle
        * \details see http://www.blackpawn.com/texts/pointinpoly/default.html
        * \param[in] p1 first point of the triangle.
        * \param[in] p2 second point of the triangle.
        * \param[in] p3 third point of the triangle.
        * \param[in] pt the querry point.
        */
      inline bool
      checkPointInsideTriangle (const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt);

      /** \brief Class get name method. */
      std::string
      getClassName () const
      {
        return ("TextureMapping");
      }

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}
