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

#ifndef PCL_POISSON_MARCHING_CUBES_POISSON_H_
#define PCL_POISSON_MARCHING_CUBES_POISSON_H_

#include <vector>
#include "geometry.h"
#include <pcl/pcl_exports.h>

namespace pcl {
  namespace poisson {


    class Square
    {
    public:
      const static int CORNERS = 4, EDGES = 4, NEIGHBORS = 4;
      static int  PCL_EXPORTS CornerIndex (const int& x, const int& y);
      static void PCL_EXPORTS FactorCornerIndex (const int& idx, int& x, int& y);
      static int  PCL_EXPORTS EdgeIndex (const int& orientation, const int& i);
      static void PCL_EXPORTS FactorEdgeIndex (const int& idx, int& orientation, int& i);

      static int  PCL_EXPORTS ReflectCornerIndex (const int& idx, const int& edgeIndex);
      static int  PCL_EXPORTS ReflectEdgeIndex (const int& idx, const int& edgeIndex);

      static void PCL_EXPORTS EdgeCorners (const int& idx, int& c1, int &c2);
    };

    class Cube{
    public:
      const static int CORNERS = 8, EDGES = 12, NEIGHBORS = 6;

      static int  PCL_EXPORTS CornerIndex (const int& x, const int& y, const int& z);
      static void PCL_EXPORTS FactorCornerIndex (const int& idx, int& x, int& y, int& z);
      static int  PCL_EXPORTS EdgeIndex (const int& orientation, const int& i, const int& j);
      static void PCL_EXPORTS FactorEdgeIndex (const int& idx, int& orientation, int& i, int &j);
      static int  PCL_EXPORTS FaceIndex (const int& dir, const int& offSet);
      static int  PCL_EXPORTS FaceIndex (const int& x, const int& y, const int& z);
      static void PCL_EXPORTS FactorFaceIndex (const int& idx, int& x, int &y, int& z);
      static void PCL_EXPORTS FactorFaceIndex (const int& idx, int& dir, int& offSet);

      static int PCL_EXPORTS AntipodalCornerIndex (const int& idx);
      static int PCL_EXPORTS FaceReflectCornerIndex (const int& idx, const int& faceIndex);
      static int PCL_EXPORTS FaceReflectEdgeIndex (const int& idx, const int& faceIndex);
      static int PCL_EXPORTS FaceReflectFaceIndex (const int& idx, const int& faceIndex);
      static int PCL_EXPORTS EdgeReflectCornerIndex	(const int& idx, const int& edgeIndex);
      static int PCL_EXPORTS EdgeReflectEdgeIndex (const int& edgeIndex);

      static int  PCL_EXPORTS FaceAdjacentToEdges (const int& eIndex1, const int& eIndex2);
      static void PCL_EXPORTS FacesAdjacentToEdge (const int& eIndex, int& f1Index, int& f2Index);

      static void PCL_EXPORTS EdgeCorners (const int& idx, int& c1, int &c2);
      static void PCL_EXPORTS FaceCorners (const int& idx, int& c1, int &c2, int& c3, int& c4);
    };

    class PCL_EXPORTS MarchingSquares
    {
      static double Interpolate (const double& v1, const double& v2);
      static void SetVertex (const int& e, const double values[Square::CORNERS], const double& iso);
    public:
      const static int MAX_EDGES = 2;
      static const int edgeMask (int idx);
      static const int edges (int x, int y);
      static double vertexList[Square::EDGES][2];

      static int GetIndex (const double values[Square::CORNERS], const double& iso);
      static int IsAmbiguous (const double v[Square::CORNERS] ,const double& isoValue);
      static int AddEdges (const double v[Square::CORNERS], const double& isoValue, Edge* edges);
      static int AddEdgeIndices (const double v[Square::CORNERS], const double& isoValue, int* edges);
    };

    class PCL_EXPORTS MarchingCubes
    {
      static double Interpolate (const double& v1, const double& v2);
      static void SetVertex (const int& e, const double values[Cube::CORNERS], const double& iso);
      static int GetFaceIndex (const double values[Cube::CORNERS], const double& iso, const int& faceIndex);

      static float Interpolate (const float& v1, const float& v2);
      static void SetVertex (const int& e, const float values[Cube::CORNERS], const float& iso);
      static int GetFaceIndex (const float values[Cube::CORNERS], const float& iso, const int& faceIndex);

      static int GetFaceIndex (const int& mcIndex, const int& faceIndex);
    public:
      const static int MAX_TRIANGLES=5;
      static const int edgeMask (int idx);
      static const int triangles[1<<Cube::CORNERS][3*MAX_TRIANGLES+1];
      static const int cornerMap (int idx);
      static double vertexList[Cube::EDGES][3];

      static int AddTriangleIndices (const int& mcIndex, int* triangles);

      static int GetIndex (const double values[Cube::CORNERS],const double& iso);
      static int IsAmbiguous (const double v[Cube::CORNERS],const double& isoValue,const int& faceIndex);
      static int HasRoots (const double v[Cube::CORNERS],const double& isoValue);
      static int HasRoots (const double v[Cube::CORNERS],const double& isoValue,const int& faceIndex);
      static int AddTriangles (const double v[Cube::CORNERS],const double& isoValue,Triangle* triangles);
      static int AddTriangleIndices (const double v[Cube::CORNERS],const double& isoValue,int* triangles);

      static int GetIndex (const float values[Cube::CORNERS], const float& iso);
      static int IsAmbiguous (const float v[Cube::CORNERS], const float& isoValue, const int& faceIndex);
      static int HasRoots (const float v[Cube::CORNERS], const float& isoValue);
      static int HasRoots (const float v[Cube::CORNERS], const float& isoValue, const int& faceIndex);
      static int AddTriangles (const float v[Cube::CORNERS], const float& isoValue, Triangle* triangles);
      static int AddTriangleIndices (const float v[Cube::CORNERS], const float& isoValue, int* triangles);

      static int IsAmbiguous (const int& mcIndex, const int& faceIndex);
      static int HasRoots (const int& mcIndex);
      static int HasFaceRoots (const int& mcIndex, const int& faceIndex);
      static int HasEdgeRoots (const int& mcIndex, const int& edgeIndex);
    };


  }
}

#endif /* PCL_POISSON_MARCHING_CUBES_POISSON_H_ */
