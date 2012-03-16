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
 * $Id: texture_mapping.h 3527 2011-12-14 19:57:12Z rusu $
 *
 */

#ifndef PCL_SURFACE_TEXTURE_MAPPING_H_
#define PCL_SURFACE_TEXTURE_MAPPING_H_

#include <pcl/surface/reconstruction.h>
#include <pcl/TextureMesh.h>

namespace pcl
{
  /** \brief The texture mapping algorithm
    * \author Khai Tran
    * \ingroup surface
    */
  template <typename PointInT>
  class TextureMapping
  {
    public:
      /** \brief Constructor. */
      TextureMapping () : f_ (), vector_field_ (), tex_files_ (), tex_material_ () {};

      /** \brief Destructor. */
      ~TextureMapping () {};

      /** \brief Set mesh scale control
        * \param[in] f 
        */
      inline void
      setF (float f)
      {
        f_ = f;
      };

      /** \brief Set vector field
        * \param[in] x data point x
        * \param[in] y data point y
        * \param[in] z data point z
        */
      inline void
      setVectorField (float x, float y, float z)
      {
        vector_field_ =  Eigen::Vector3f(x, y, z);
        // normalize vector field
        vector_field_ = vector_field_/std::sqrt(vector_field_.dot(vector_field_));
      };

      /** \brief Set texture files
        * \param[in] tex_files list of texture files
        */
      inline void
      setTextureFiles (std::vector<std::string> tex_files)
      {
        tex_files_ = tex_files;
      };

      /** \brief Set texture materials
        * \param[in] tex_material texture material
        */
      inline void
      setTextureMaterials (TexMaterial tex_material)
      {
        tex_material_ = tex_material;
      };

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

    protected:

      /** \brief mesh scale control. */
      float f_;

      /** \brief vector field */
      Eigen::Vector3f vector_field_;

      /** \brief list of texture files */
      std::vector<std::string> tex_files_;

      /** \brief list of texture materials */
      TexMaterial tex_material_;


      /** \brief get the distance of 2 3D points.
        * \param[in] p1 the first point
        * \param[in] p2 the second point
        */
      float
      getDistance (Eigen::Vector3f &p1, Eigen::Vector3f &p2);

      /** \brief Map texture to a face
        * \param[in] p1 the first point
        * \param[in] p2 the second point
        * \param[in] p3 the third point
        */
      std::vector<Eigen::Vector2f>
      mapTexture2Face (Eigen::Vector3f  &p1, Eigen::Vector3f  &p2, Eigen::Vector3f &p3);

       /** \brief Class get name method. */
      std::string getClassName () const { return ("TextureMapping"); }
  };
}

#endif /* TEXTURE_MAPPING_H_ */

