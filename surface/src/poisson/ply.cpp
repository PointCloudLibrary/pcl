/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
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


#include "pcl/surface/poisson/ply.h"

namespace pcl
{
  namespace surface
  {

    //
    // PLY data structures
    //
    char *elem_names[] = {"vertex", "face"};

    typedef struct PlyVertex
    {
      float x, y, z;
    } PlyVertex;

    typedef struct PlyOrientedVertex
    {
      float x, y, z, nx, ny, nz;
    } PlyOrientedVertex;

    typedef struct PlyFace
    {
      unsigned char nr_vertices;
      int *vertices;
      int segment;
    } PlyFace;

    static PlyProperty vert_props[] = { {"x", PLY_FLOAT, PLY_FLOAT, offsetof (PlyVertex, x), 0, 0, 0, 0},
                                        {"y", PLY_FLOAT, PLY_FLOAT, offsetof (PlyVertex, y), 0, 0, 0, 0},
                                        {"z", PLY_FLOAT, PLY_FLOAT, offsetof (PlyVertex, z), 0, 0, 0, 0}};
    static PlyProperty oriented_vert_props[] = { {"x", PLY_FLOAT, PLY_FLOAT, offsetof (PlyOrientedVertex, x), 0, 0, 0,
                                                  0}, {"y", PLY_FLOAT, PLY_FLOAT, offsetof (PlyOrientedVertex, y), 0,
                                                       0, 0, 0}, {"z", PLY_FLOAT, PLY_FLOAT,
                                                                  offsetof (PlyOrientedVertex, z), 0, 0, 0, 0},
                                                 {"nx", PLY_FLOAT, PLY_FLOAT, offsetof (PlyOrientedVertex, nx), 0, 0,
                                                  0, 0}, {"ny", PLY_FLOAT, PLY_FLOAT, offsetof (PlyOrientedVertex, ny),
                                                          0, 0, 0, 0}, {"nz", PLY_FLOAT, PLY_FLOAT,
                                                                        offsetof (PlyOrientedVertex, nz), 0, 0, 0, 0}};

    // List of property information for a vertex
    static PlyProperty face_props[] = { {"vertex_indices", PLY_INT, PLY_INT, offsetof (PlyFace, vertices), 1,
                                         PLY_UCHAR, PLY_UCHAR, offsetof (PlyFace, nr_vertices)}, };

    int
    PlyDefaultFileType (void)
    {
      return PLY_ASCII;
    }

    int
    PlyWriteTriangles (char* fileName, CoredMeshData* mesh, int file_type, const Point3D<float>& translate,
                       const float& scale, char** comments, const int& commentNum)
    {
      int i;
      int nr_vertices = int (mesh->outOfCorePointCount () + mesh->inCorePoints.size ());
      int nr_faces = mesh->triangleCount ();
      float version;
      PlyFile *ply = ply_open_for_writing (fileName, 2, elem_names, file_type, &version);
      if (!ply)
      {
        return 0;
      }

      mesh->resetIterator ();

      //
      // describe vertex and face properties
      //
      ply_element_count (ply, "vertex", nr_vertices);
      ply_describe_property (ply, "vertex", &vert_props[0]);
      ply_describe_property (ply, "vertex", &vert_props[1]);
      ply_describe_property (ply, "vertex", &vert_props[2]);

      ply_element_count (ply, "face", nr_faces);
      ply_describe_property (ply, "face", &face_props[0]);

      // Write in the comments
      for (i = 0; i < commentNum; i++)
      {
        ply_put_comment (ply, comments[i]);
      }

      ply_header_complete (ply);

      // write vertices
      ply_put_element_setup (ply, "vertex");
      Point3D<float> p;
      for (i = 0; i < int (mesh->inCorePoints.size ()); i++)
      {
        PlyVertex ply_vertex;
        p = mesh->inCorePoints[i];
        ply_vertex.x = p.coords[0] * scale + translate.coords[0];
        ply_vertex.y = p.coords[1] * scale + translate.coords[1];
        ply_vertex.z = p.coords[2] * scale + translate.coords[2];
        ply_put_element (ply, (void *)&ply_vertex);
      }
      for (i = 0; i < mesh->outOfCorePointCount (); i++)
      {
        PlyVertex ply_vertex;
        mesh->nextOutOfCorePoint (p);
        ply_vertex.x = p.coords[0] * scale + translate.coords[0];
        ply_vertex.y = p.coords[1] * scale + translate.coords[1];
        ply_vertex.z = p.coords[2] * scale + translate.coords[2];
        ply_put_element (ply, (void *)&ply_vertex);
      } // for, write vertices

      // write faces
      TriangleIndex tIndex;
      int inCoreFlag;
      ply_put_element_setup (ply, "face");
      for (i = 0; i < nr_faces; i++)
      {
        //
        // create and fill a struct that the ply code can handle
        //
        PlyFace ply_face;
        ply_face.nr_vertices = 3;
        ply_face.vertices = new int[3];
        mesh->nextTriangle (tIndex, inCoreFlag);
        if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0]))
        {
          tIndex.idx[0] += int (mesh->inCorePoints.size ());
        }
        if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1]))
        {
          tIndex.idx[1] += int (mesh->inCorePoints.size ());
        }
        if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2]))
        {
          tIndex.idx[2] += int (mesh->inCorePoints.size ());
        }
        for (int j = 0; j < 3; j++)
        {
          ply_face.vertices[j] = tIndex.idx[j];
        }
        ply_put_element (ply, (void *)&ply_face);
        delete[] ply_face.vertices;
      } // for, write faces

      ply_close (ply);
      return 1;
    }
  }
}
