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

#ifndef PCL_SURFACE_TEXTURE_MAPPING_H_
#define PCL_SURFACE_TEXTURE_MAPPING_H_

#include <pcl/surface/reconstruction.h>
#include <pcl/common/transforms.h>
#include <pcl/TextureMesh.h>

namespace pcl
{
  /** \brief The texture mapping algorithm
    * \author Khai Tran
    * \ingroup surface
    */
  template<typename PointInT>
  class TextureMapping
  {
    public:
      /** \brief Structure to store camera pose and focal length. */
      struct Camera
      {
        Camera () : pose (), focal_length (), height (), width (), texture_file () {}
        Eigen::Affine3f pose;
        double focal_length;
        double height;
        double width;
        std::string texture_file;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };

      typedef boost::shared_ptr< PointInT > Ptr;
      typedef boost::shared_ptr< const PointInT > ConstPtr;

      typedef pcl::PointCloud<PointInT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef pcl::octree::OctreePointCloudSearch<PointInT> Octree;
      typedef typename Octree::Ptr OctreePtr;
      typedef typename Octree::ConstPtr OctreeConstPtr;

      /** \brief Constructor. */
      TextureMapping () :
        f_ (), vector_field_ (), tex_files_ (), tex_material_ ()
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
        vector_field_ = vector_field_ / std::sqrt (vector_field_.dot (vector_field_));
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

      /** \brief map texture to a mesh UV mapping
        * \param[in] tex_mesh texture mesh
        */
      void
      mapTexture2MeshUV (pcl::TextureMesh &tex_mesh);

      /** \brief map textures aquired from a set of cameras onto a mesh, with UV mapping
        *  the mesh must be divided into NbCamera + 1 sub-meshes. Each sub-mesh corresponding
        *  to the faces visible by one camera. The last submesh containing all non-visible faces
        * \param[in] tex_mesh texture mesh
        * \param[in] cams cameras used for UV mapping
        */
      void
      mapMultipleTexturesToMeshUV (pcl::TextureMesh &tex_mesh, std::vector<Camera> &cams);

      /** \brief computes UV coordinates of point, observed by one particular camera
        * \param ...
        * \param ...
        * \param ...
        * \returns false if the point is not visible by the camera.
        */
      inline bool
      getPointUVCoordinates (pcl::PointXYZ &pt, Camera &cam, Eigen::Vector2f &UV_coordinates)
      {
        //if the point is in front of the camera
        if (pt.z > 0)
        {
          //compute image center and dimension
          double sizeX = cam.width;
          double sizeY = cam.height;
          double cx = (sizeX) / 2.0;
          double cy = (sizeY) / 2.0;

          double focal_x = cam.focal_length;
          double focal_y = cam.focal_length;

          //project point on image frame
          UV_coordinates[0] = static_cast<float> ((focal_x * (pt.x / pt.z) + cx) / sizeX); //horizontal
          UV_coordinates[1] = 1.0f - static_cast<float> (((focal_y * (pt.y / pt.z) + cy) / sizeY)); //vertical

          //point is visible!
          if (UV_coordinates[0] >= 0.0 && UV_coordinates[0] <= 1.0 && UV_coordinates[1] >= 0.0 && UV_coordinates[1]
                                                                                                                 <= 1.0)
            return (true);
        }

        //point is NOT visible by the camera
        UV_coordinates[0] = -1.0;
        UV_coordinates[1] = -1.0;
        return (false);
      }

      /** \brief Return true if point is occluded
        * The octree must be initialized with a cloud transformed into the camera's frame
        */
      inline bool
      isPointOccluded (const pcl::PointXYZ &pt, OctreePtr octree) const;

      /** \brief Remove occluded points from a point cloud
       */
      void
      removeOccludedPoints (PointCloudPtr &input_cloud,
                            PointCloudPtr &filtered_cloud, double octree_voxel_size,
                            std::vector<int> &visible_indices, std::vector<int> &occluded_indices);

      /** \brief Remove occluded points from a textureMesh
       *
       */
      void
      removeOccludedPoints (pcl::TextureMesh &tex_mesh, pcl::TextureMesh &cleaned_mesh, double octree_voxel_size);


      /* \brief Remove occluded points from a textureMesh
       */
      void
      removeOccludedPoints (pcl::TextureMesh &tex_mesh, PointCloudPtr &filtered_cloud, double octree_voxel_size);


      /* \brief Segment faces by camera visibility.
       * With N camera, faces are gonna be segmented into N+1 groups:
       * 1 for each camera, plus one for faces not visible from any camera.
       */
      int
      sortFacesByCamera (pcl::TextureMesh &tex_mesh, pcl::TextureMesh &sorted_mesh, std::vector<Camera> &cameras,
                         double octree_voxel_size, PointCloud &visible_pts);

      /** \brief Colors a point cloud, depending on its occlusions
       * If showNbOcclusions is set to True, each point is colored depending on the number of points occluding it
       * Else, each point is given a different a 0 value is not occluded, 1 if occluded.
       * By default, the number of occlusions is bounded to 4
       */
      void
      showOcclusions (PointCloudPtr &input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &colored_cloud,
                      double octree_voxel_size, bool show_nb_occlusions = true,
                      int max_occlusions = 4);

      /** \brief Colors the point cloud of a Mesh, depending on its occlusions
        * If showNbOcclusions is set to True, each point is colored depending on the number of points occluding it
        * Else, each point is given a different a 0 value is not occluded, 1 if occluded.
        * By default, the number of occlusions is bounded to 4
        */
      void
      showOcclusions (pcl::TextureMesh &tex_mesh, pcl::PointCloud<pcl::PointXYZI>::Ptr &colored_cloud,
                      double octree_voxel_size, bool show_nb_occlusions = true, int max_occlusions = 4);

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
      std::vector<Eigen::Vector2f>
      mapTexture2Face (const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3);

      /** \brief Class get name method. */
      std::string
      getClassName () const
      {
        return ("TextureMapping");
      }

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif /* TEXTURE_MAPPING_H_ */

