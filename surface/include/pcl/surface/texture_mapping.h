/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: texture_mapping.h 1005 2011-07-13 13:07:00 ktran $
 *
 */
/** \author Khai Tran */

#ifndef TEXTURE_MAPPING_H_
#define TEXTURE_MAPPING_H_

#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/pcl_macros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/pcl_macros.h>

namespace pcl
{
  class TextureMapping{
  public:

    /** \brief Constructor. */
    TextureMapping();

    /** \brief Destructor. */
    ~TextureMapping();

    /** \brief set mesh scale control
    * \param
    */
    PCL_EXPORTS void
       setF(float f);

    /** \brief set vector field
    * \param
    */
    PCL_EXPORTS void
       setVectorField(float x, float y, float z);

    /** \brief set texture files
    * \param
    */
    PCL_EXPORTS void
       setTextureFiles( std::vector<char*> tex_files);

    /** \brief set texture materials
     * \param
     */
    PCL_EXPORTS void
      setTextureMaterials(TexMaterial tex_material);

     /** \brief set texture Mesh
      * \param
      */
    PCL_EXPORTS void
       setTextureMesh(TextureMesh tex_mesh);

    /** \brief map texture to a face
    * \param
    */
    PCL_EXPORTS pcl::TextureMesh
      mapTexture2Mesh();

  protected:
    /** \brief mesh scale control. */
    float f;

    /** \brief vector field */
    Eigen::Vector3f vector_field;

    /** \brief list of texture files */
    std::vector<char*> tex_files;

    /** \brief list of texture materials */
    TexMaterial tex_material;

    /** \brief texture mesh */
    TextureMesh tex_mesh;

    /** \brief list of functions */

    /** \brief get the distance of 2 3D points.
    * \param 2 3D points
    */
    float
      getDistance(pcl::PointXYZ p1, pcl::PointXYZ p2);

    /** \brief blend edge of 2 faces
    * \param
    */

    int
      blendTexture2Faces(std::vector<pcl::PointXY> pp1, std::vector<pcl::PointXY> pp2, IplImage* img_tex1, IplImage* img_tex2);

    /** \brief map texture to a face
    * \param
    */
     std::vector<pcl::PointXY>
       mapTexture2Face(pcl::PointXYZ pp1, pcl::PointXYZ pp2, pcl::PointXYZ pp3);

  };
}

#endif /* TEXTURE_MAPPING_H_ */
