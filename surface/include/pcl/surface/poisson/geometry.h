/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012,  Willow Garage,  Inc.
 *  Copyright (c) 2006,  Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms,  with or without
 *  modification,  are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice,  this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice,  this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage,  Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,  INCLUDING,  BUT NOT
 *  LIMITED TO,  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,  INDIRECT,
 *  INCIDENTAL,  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO,  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE,  DATA,  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY,  WHETHER IN CONTRACT,  STRICT
 *  LIABILITY,  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE,  EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: geometry.h 5531 2012-04-08 09:14:33Z aichim $
 *
 */

#ifndef PCL_POISSON_GEOMETRY_H_
#define PCL_POISSON_GEOMETRY_H_

#include <math.h>
#include <vector>
#include "hash.h"

namespace pcl 
{
  namespace poisson 
  {
    template<class Real>
    Real Random (void);

    template<class Real>
    struct Point3D{Real coords[3];};

    template<class Real>
    Point3D<Real> RandomBallPoint (void);

    template<class Real>
    Point3D<Real> RandomSpherePoint (void);

    template<class Real>
    double Length (const Point3D<Real>& p);

    template<class Real>
    double SquareLength (const Point3D<Real>& p);

    template<class Real>
    double Distance (const Point3D<Real>& p1, const Point3D<Real>& p2);

    template<class Real>
    double SquareDistance (const Point3D<Real>& p1, const Point3D<Real>& p2);

    template <class Real>
    void CrossProduct (const Point3D<Real>& p1, const Point3D<Real>& p2, Point3D<Real>& p);

    class Edge
    {
      public:
        double p[2][2];
        double Length (void) const
        {
          double d[2];
          d[0]=p[0][0]-p[1][0];
          d[1]=p[0][1]-p[1][1];

          return sqrt (d[0]*d[0]+d[1]*d[1]);
        }
    };
    
    class Triangle
    {
      public:
        double p[3][3];
        
        double 
        Area (void) const
        {
          double v1[3], v2[3], v[3];
          for (int d=0;d<3;d++)
          {
            v1[d] = p[1][d]-p[0][d];
            v2[d] = p[2][d]-p[0][d];
          }
          v[0] =  v1[1]*v2[2]-v1[2]*v2[1];
          v[1] = -v1[0]*v2[2]+v1[2]*v2[0];
          v[2] =  v1[0]*v2[1]-v1[1]*v2[0];

          return (sqrt (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / 2);
        }

        double 
        AspectRatio (void) const
        {
          double d=0;
          int i, j;
          for (i = 0; i < 3; i++)
          {
            for (i = 0; i < 3; i++)
              for (j = 0; j < 3; j++)
              {
                d += (p[(i+1)%3][j]-p[i][j])* (p[(i+1)%3][j]-p[i][j]);
              }
          }
          return (Area () / d);
        }
    };

    class CoredPointIndex
    {
      public:
        int index;
        char inCore;

        int operator == (const CoredPointIndex& cpi) const {return (index==cpi.index) && (inCore==cpi.inCore);};
        int operator != (const CoredPointIndex& cpi) const {return (index!=cpi.index) || (inCore!=cpi.inCore);};
    };

    class EdgeIndex
    {
      public:
        int idx[2];
    };

    class CoredEdgeIndex
    {
      public:
        CoredPointIndex idx[2];
    };

    class TriangleIndex
    {
      public:
        int idx[3];
    };

    class TriangulationEdge
    {
      public:
        TriangulationEdge (void);
        int pIndex[2];
        int tIndex[2];
    };

    class TriangulationTriangle
    {
      public:
        TriangulationTriangle (void);
        int eIndex[3];
    };

    template<class Real>
    class Triangulation
    {
      public:
        Triangulation () : points (),  edges (),  triangles (),  edgeMap () {}

        std::vector<Point3D<Real> >        points;
        std::vector<TriangulationEdge>     edges;
        std::vector<TriangulationTriangle> triangles;

        int 
        factor (const int& tIndex, int& p1, int& p2, int& p3);
        
        double 
        area (void);

        double 
        area (const int& tIndex);

        double 
        area (const int& p1, const int& p2, const int& p3);

        int 
        flipMinimize (const int& eIndex);

        int 
        addTriangle (const int& p1, const int& p2, const int& p3);

      protected:
        hash_map<long long, int> edgeMap;
        static long long EdgeIndex (const int& p1, const int& p2);
        double area (const Triangle& t);
    };


    template<class Real> void 
    EdgeCollapse (const Real& edgeRatio,
                  std::vector<TriangleIndex>& triangles,
                  std::vector< Point3D<Real> >& positions,
                  std::vector<Point3D<Real> >* normals);
    
    template<class Real> void 
    TriangleCollapse (const Real& edgeRatio,
                      std::vector<TriangleIndex>& triangles,
                      std::vector<Point3D<Real> >& positions,
                      std::vector<Point3D<Real> >* normals);

    struct CoredVertexIndex
    {
      int idx;
      bool inCore;
    };

    class CoredMeshData
    {
      public:
        CoredMeshData () : inCorePoints () {}

        virtual ~CoredMeshData () {}

        std::vector<Point3D<float> > inCorePoints;
        
        virtual void 
        resetIterator () = 0;

        virtual int 
        addOutOfCorePoint (const Point3D<float>& p) = 0;

        virtual int 
        addPolygon (const std::vector< CoredVertexIndex >& vertices) = 0;

        virtual int 
        nextOutOfCorePoint (Point3D<float>& p) = 0;
        
        virtual int 
        nextPolygon (std::vector<CoredVertexIndex >& vertices) = 0;

        virtual int 
        outOfCorePointCount () = 0;
        
        virtual int 
        polygonCount () = 0;
    };

    class CoredVectorMeshData : public CoredMeshData
    {
      std::vector<Point3D<float> > oocPoints;
      std::vector< std::vector< int > > polygons;
      int polygonIndex;
      int oocPointIndex;

      public:
        CoredVectorMeshData ();
        
        virtual ~CoredVectorMeshData () {}

        void resetIterator (void);

        int addOutOfCorePoint (const Point3D<float>& p);
        int addPolygon (const std::vector< CoredVertexIndex >& vertices);

        int nextOutOfCorePoint (Point3D<float>& p);
        int nextPolygon (std::vector< CoredVertexIndex >& vertices);

        int outOfCorePointCount (void);
        int polygonCount (void);
    };

    class CoredFileMeshData : public CoredMeshData
    {
      FILE *oocPointFile ,  *polygonFile;
      int oocPoints ,  polygons;
      public:
        CoredFileMeshData ();
        virtual ~CoredFileMeshData ();

        void resetIterator (void);

        int addOutOfCorePoint (const Point3D<float>& p);
        int addPolygon (const std::vector< CoredVertexIndex >& vertices);

        int nextOutOfCorePoint (Point3D<float>& p);
        int nextPolygon (std::vector< CoredVertexIndex >& vertices);

        int outOfCorePointCount (void);
        int polygonCount (void);
    };
  }
}

#include <pcl/surface/impl/poisson/geometry.hpp>

#endif /* PCL_POISSON_GEOMETRY_H_ */
