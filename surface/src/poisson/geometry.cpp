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


#include "pcl/surface/poisson/geometry.h"


namespace pcl
{
  namespace surface
  {
    namespace poisson
    {
      ///////////////////
      // CoredMeshData //
      ///////////////////
      const int CoredMeshData::IN_CORE_FLAG[] = {1, 2, 4};

      TriangulationEdge::TriangulationEdge (void)
      {
        pIndex[0] = pIndex[1] = tIndex[0] = tIndex[1] = -1;
      }


      TriangulationTriangle::TriangulationTriangle (void)
      {
        eIndex[0] = eIndex[1] = eIndex[2] = -1;
      }

      /////////////////////////
      // CoredVectorMeshData //
      /////////////////////////
      CoredVectorMeshData::CoredVectorMeshData (void)
      {
        oocPointIndex = triangleIndex = 0;
      }

      void
      CoredVectorMeshData::resetIterator (void)
      {
        oocPointIndex = triangleIndex = 0;
      }

      int
      CoredVectorMeshData::addOutOfCorePoint (const Point3D<float>& p)
      {
        oocPoints.push_back (p);
        return (int (oocPoints.size ()) - 1);
      }

      int
      CoredVectorMeshData::addTriangle (const TriangleIndex& t, const int& coreFlag)
      {
        TriangleIndex tt;
        if (coreFlag & CoredMeshData::IN_CORE_FLAG[0])
          tt.idx[0] = t.idx[0];
        else
          tt.idx[0] = -t.idx[0] - 1;
        if (coreFlag & CoredMeshData::IN_CORE_FLAG[1])
          tt.idx[1] = t.idx[1];
        else
          tt.idx[1] = -t.idx[1] - 1;
        if (coreFlag & CoredMeshData::IN_CORE_FLAG[2])
          tt.idx[2] = t.idx[2];
        else
          tt.idx[2] = -t.idx[2] - 1;
        triangles.push_back (tt);
        return (int (triangles.size ()) - 1);
      }

      int
      CoredVectorMeshData::nextOutOfCorePoint (Point3D<float>& p)
      {
        if (oocPointIndex < int (oocPoints.size ()))
        {
          p = oocPoints[oocPointIndex++];
          return (1);
        }
        else
          return (0);
      }

      int
      CoredVectorMeshData::nextTriangle (TriangleIndex& t, int& inCoreFlag)
      {
        inCoreFlag = 0;
        if (triangleIndex < int (triangles.size ()))
        {
          t = triangles[triangleIndex++];
          if (t.idx[0] < 0)
            t.idx[0] = -t.idx[0] - 1;
          else
            inCoreFlag |= CoredMeshData::IN_CORE_FLAG[0];
          if (t.idx[1] < 0)
            t.idx[1] = -t.idx[1] - 1;
          else
            inCoreFlag |= CoredMeshData::IN_CORE_FLAG[1];
          if (t.idx[2] < 0)
            t.idx[2] = -t.idx[2] - 1;
          else
            inCoreFlag |= CoredMeshData::IN_CORE_FLAG[2];
          return (1);
        }
        else
          return (0);
      }

      int
      CoredVectorMeshData::outOfCorePointCount (void)
      {
        return (int (oocPoints.size ()));
      }

      int
      CoredVectorMeshData::triangleCount (void)
      {
        return (int (triangles.size ()));
      }
    }
  }
}
