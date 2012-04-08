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
#ifndef PCL_POISSON_OCTREE_POISSON_H_
#define PCL_POISSON_OCTREE_POISSON_H_

#include "allocator.h"
#include "binary_node.h"
#include "marching_cubes_poisson.h"

#define DIMENSION 3

namespace pcl 
{
  namespace poisson 
  {

    template<class NodeData,class Real=float>
    class OctNode
    {
      private:
        static int UseAlloc;

        class AdjacencyCountFunction
        {
          public:
            int count;
            void Function(const OctNode<NodeData,Real>* node1,const OctNode<NodeData,Real>* node2);
        };

        template<class NodeAdjacencyFunction>
        void __processNodeFaces (OctNode* node,
                                 NodeAdjacencyFunction* F,
                                 const int& cIndex1, const int& cIndex2, const int& cIndex3, const int& cIndex4);
        template<class NodeAdjacencyFunction>
        void __processNodeEdges (OctNode* node,
                                 NodeAdjacencyFunction* F,
                                 const int& cIndex1, const int& cIndex2);
        template<class NodeAdjacencyFunction>
        void __processNodeNodes (OctNode* node, NodeAdjacencyFunction* F);
        template<class NodeAdjacencyFunction>
        static void __ProcessNodeAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                                OctNode* node1, const int& radius1,
                                                OctNode* node2, const int& radius2,
                                                const int& cWidth2,
                                                NodeAdjacencyFunction* F);
        template<class TerminatingNodeAdjacencyFunction>
        static void __ProcessTerminatingNodeAdjacentNodes(const int& dx, const int& dy, const int& dz,
                                                          OctNode* node1, const int& radius1,
                                                          OctNode* node2, const int& radius2,
                                                          const int& cWidth2,
                                                          TerminatingNodeAdjacencyFunction* F);
        template<class PointAdjacencyFunction>
        static void __ProcessPointAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                                 OctNode* node2, const int& radius2,
                                                 const int& cWidth2,
                                                 PointAdjacencyFunction* F);
        template<class NodeAdjacencyFunction>
        static void __ProcessFixedDepthNodeAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                                          OctNode* node1, const int& radius1,
                                                          OctNode* node2, const int& radius2,
                                                          const int& cWidth2,
                                                          const int& depth,
                                                          NodeAdjacencyFunction* F);
        template<class NodeAdjacencyFunction>
        static void __ProcessMaxDepthNodeAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                                        OctNode* node1, const int& radius1,
                                                        OctNode* node2, const int& radius2,
                                                        const int& cWidth2,
                                                        const int& depth,
                                                        NodeAdjacencyFunction* F);

        // This is made private because the division by two has been pulled out.
        static inline int Overlap (const int& c1, const int& c2, const int& c3, const int& dWidth);
        inline static int ChildOverlap (const int& dx, const int& dy, const int& dz, const int& d, const int& cRadius2);

        const OctNode* __faceNeighbor (const int& dir, const int& off) const;
        const OctNode* __edgeNeighbor (const int& o, const int i[2], const int idx[2]) const;
        OctNode* __faceNeighbor (const int& dir, const int& off, const int& forceChildren);
        OctNode* __edgeNeighbor (const int& o, const int i[2], const int idx[2], const int& forceChildren);
      public:
        static const int DepthShift,OffsetShift,OffsetShift1,OffsetShift2,OffsetShift3;
        static const int DepthMask,OffsetMask;

        static Allocator<OctNode> AllocatorOctNode;
        static int UseAllocator (void);
        static void SetAllocator (int blockSize);

        OctNode* parent;
        OctNode* children;
        short d,off[3];
        NodeData nodeData;


        OctNode (void);
        ~OctNode (void);
        int initChildren (void);

        void depthAndOffset (int& depth, int offset[3]) const;
        int depth (void) const;
        static inline void DepthAndOffset (const long long& index, int& depth, int offset[3]);
        static inline void CenterAndWidth (const long long& index, Point3D<Real>& center, Real& width);
        static inline int Depth (const long long& index);
        static inline void Index (const int& depth, const int offset[3], short& d, short off[3]);
        void centerAndWidth (Point3D<Real>& center, Real& width) const;

        int leaves (void) const;
        int maxDepthLeaves (const int& maxDepth) const;
        int nodes (void) const;
        int maxDepth (void) const;

        const OctNode* root (void) const;

        const OctNode* nextLeaf (const OctNode* currentLeaf = NULL) const;
        OctNode* nextLeaf (OctNode* currentLeaf = NULL);
        const OctNode* nextNode (const OctNode* currentNode = NULL) const;
        OctNode* nextNode (OctNode* currentNode = NULL);
        const OctNode* nextBranch (const OctNode* current) const;
        OctNode* nextBranch (OctNode* current);

        void setFullDepth (const int& maxDepth);

        void printLeaves (void) const;
        void printRange (void) const;

        template<class NodeAdjacencyFunction>
        void processNodeFaces (OctNode* node,NodeAdjacencyFunction* F, const int& fIndex, const int& processCurrent = 1);
        template<class NodeAdjacencyFunction>
        void processNodeEdges (OctNode* node, NodeAdjacencyFunction* F, const int& eIndex, const int& processCurrent = 1);
        template<class NodeAdjacencyFunction>
        void processNodeCorners (OctNode* node, NodeAdjacencyFunction* F, const int& cIndex, const int& processCurrent = 1);
        template<class NodeAdjacencyFunction>
        void processNodeNodes (OctNode* node, NodeAdjacencyFunction* F, const int& processCurrent=1);

        template<class NodeAdjacencyFunction>
        static void ProcessNodeAdjacentNodes (const int& maxDepth,
                                              OctNode* node1, const int& width1,
                                              OctNode* node2, const int& width2,
                                              NodeAdjacencyFunction* F,
                                              const int& processCurrent=1);
        template<class NodeAdjacencyFunction>
        static void ProcessNodeAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                              OctNode* node1, const int& radius1,
                                              OctNode* node2, const int& radius2,
                                              const int& width2,
                                              NodeAdjacencyFunction* F,
                                              const int& processCurrent = 1);
        template<class TerminatingNodeAdjacencyFunction>
        static void ProcessTerminatingNodeAdjacentNodes (const int& maxDepth,
                                                         OctNode* node1, const int& width1,
                                                         OctNode* node2, const int& width2,
                                                         TerminatingNodeAdjacencyFunction* F,
                                                         const int& processCurrent = 1);
        template<class TerminatingNodeAdjacencyFunction>
        static void ProcessTerminatingNodeAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                                         OctNode* node1, const int& radius1,
                                                         OctNode* node2, const int& radius2,
                                                         const int& width2,
                                                         TerminatingNodeAdjacencyFunction* F,
                                                         const int& processCurrent = 1);
        template<class PointAdjacencyFunction>
        static void ProcessPointAdjacentNodes (const int& maxDepth,
                                               const int center1[3],
                                               OctNode* node2, const int& width2,
                                               PointAdjacencyFunction* F,
                                               const int& processCurrent = 1);
        template<class PointAdjacencyFunction>
        static void ProcessPointAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                               OctNode* node2, const int& radius2, const int& width2,
                                               PointAdjacencyFunction* F,
                                               const int& processCurrent = 1);
        template<class NodeAdjacencyFunction>
        static void ProcessFixedDepthNodeAdjacentNodes (const int& maxDepth,
                                                        OctNode* node1, const int& width1,
                                                        OctNode* node2, const int& width2,
                                                        const int& depth,
                                                        NodeAdjacencyFunction* F,
                                                        const int& processCurrent = 1);
        template<class NodeAdjacencyFunction>
        static void ProcessFixedDepthNodeAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                                        OctNode* node1, const int& radius1,
                                                        OctNode* node2, const int& radius2,
                                                        const int& width2,
                                                        const int& depth,
                                                        NodeAdjacencyFunction* F,
                                                        const int& processCurrent = 1);
        template<class NodeAdjacencyFunction>
        static void ProcessMaxDepthNodeAdjacentNodes (const int& maxDepth,
                                                      OctNode* node1, const int& width1,
                                                      OctNode* node2, const int& width2,
                                                      const int& depth,
                                                      NodeAdjacencyFunction* F,
                                                      const int& processCurrent = 1);
        template<class NodeAdjacencyFunction>
        static void ProcessMaxDepthNodeAdjacentNodes (const int& dx, const int& dy, const int& dz,
                                                      OctNode* node1, const int& radius1,
                                                      OctNode* node2, const int& radius2,
                                                      const int& width2,
                                                      const int& depth,
                                                      NodeAdjacencyFunction* F,
                                                      const int& processCurrent = 1);

        static int CornerIndex (const Point3D<Real>& center, const Point3D<Real> &p);

        OctNode* faceNeighbor (const int& faceIndex, const int& forceChildren = 0);
        const OctNode* faceNeighbor (const int& faceIndex) const;
        OctNode* edgeNeighbor (const int& edgeIndex, const int& forceChildren = 0);
        const OctNode* edgeNeighbor (const int& edgeIndex) const;
        OctNode* cornerNeighbor (const int& cornerIndex, const int& forceChildren = 0);
        const OctNode* cornerNeighbor (const int& cornerIndex) const;

        OctNode* getNearestLeaf (const Point3D<Real>& p);
        const OctNode* getNearestLeaf (const Point3D<Real>& p) const;

        static int CommonEdge (const OctNode* node1, const int& eIndex1,
                               const OctNode* node2, const int& eIndex2);
        static int CompareForwardDepths (const void* v1, const void* v2);
        static int CompareForwardPointerDepths (const void* v1, const void* v2);
        static int CompareBackwardDepths (const void* v1, const void* v2);
        static int CompareBackwardPointerDepths (const void* v1, const void* v2);


        template<class NodeData2>
        OctNode& operator = (const OctNode<NodeData2, Real>& node);

        static inline int Overlap2 (const int &depth1,
                                    const int offSet1[DIMENSION],
                                    const Real& multiplier1,
                                    const int &depth2,
                                    const int offSet2[DIMENSION],
                                    const Real& multiplier2);


        int write (const char* fileName) const;
        int write (FILE* fp) const;
        int read (const char* fileName);
        int read (FILE* fp);

        class Neighbors{
        public:
          OctNode* neighbors[3][3][3];
          Neighbors (void);
          void clear (void);
        };
        class NeighborKey{
        public:
          Neighbors* neighbors;

          NeighborKey (void);
          ~NeighborKey (void);

          void set (const int& depth);
          Neighbors& setNeighbors (OctNode* node);
          Neighbors& getNeighbors (OctNode* node);
        };
        class Neighbors2{
        public:
          const OctNode* neighbors[3][3][3];
          Neighbors2 (void);
          void clear (void);
        };
        class NeighborKey2{
        public:
          Neighbors2* neighbors;

          NeighborKey2 (void);
          ~NeighborKey2 (void);

          void set (const int& depth);
          Neighbors2& getNeighbors (const OctNode* node);
        };

        void centerIndex (const int& maxDepth, int index[DIMENSION]) const;
        int width (const int& maxDepth) const;
    };

  }
}


#include <pcl/surface/impl/poisson/octree_poisson.hpp>

#endif /* PCL_POISSON_OCTREE_POISSON_H_ */
