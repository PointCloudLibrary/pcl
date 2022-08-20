/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */

#pragma once

#include <Eigen/Core>
#include <string>
#include <pcl/PCLPointCloud2.h>
#include <pcl/Vertices.h>

namespace pcl
{
  /** \author Khai Tran */
  struct TexMaterial
  {
    struct RGB
    {
      float r = 0;
      float g = 0;
      float b = 0;
    }; //RGB

    /** \brief Texture name. */
    std::string tex_name;

    /** \brief Texture file. */
    std::string tex_file;

    /** \brief Defines the ambient color of the material to be (r,g,b). */
    RGB         tex_Ka;

    /** \brief Defines the diffuse color of the material to be (r,g,b). */
    RGB         tex_Kd;

    /** \brief Defines the specular color of the material to be (r,g,b). This color shows up in highlights. */
    RGB         tex_Ks;

    /** \brief Defines the transparency of the material to be alpha. */
    float       tex_d;

    /** \brief Defines the shininess of the material to be s. */
    float       tex_Ns;

    /** \brief Denotes the illumination model used by the material.
      *
      * illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
      * illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
      */
    int         tex_illum;
  }; // TexMaterial

  /** \author Khai Tran */
  struct TextureMesh
  {
    pcl::PCLPointCloud2  cloud;
    pcl::PCLHeader  header;


    /** \brief polygon which is mapped with specific texture defined in TexMaterial */
    std::vector<std::vector<pcl::Vertices> >    tex_polygons;
    /** \brief UV coordinates */
    std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > tex_coordinates;
    /** \brief define texture material */
    std::vector<pcl::TexMaterial>               tex_materials;
    /** \brief Specifies which texture coordinates from tex_coordinates each polygon/face uses.
      * The vectors must have the same sizes as in tex_polygons, but the pcl::Vertices
      * may be empty for those polygons/faces that do not use coordinates.
      */
    std::vector<std::vector<pcl::Vertices> >    tex_coord_indices;

    public:
      using Ptr = shared_ptr<pcl::TextureMesh>;
      using ConstPtr = shared_ptr<const pcl::TextureMesh>;
   }; // struct TextureMesh

   using TextureMeshPtr = TextureMesh::Ptr;
   using TextureMeshConstPtr = TextureMesh::ConstPtr;
} // namespace pcl
