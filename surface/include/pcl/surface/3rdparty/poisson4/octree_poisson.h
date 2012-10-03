/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#ifndef OCT_NODE_INCLUDED
#define OCT_NODE_INCLUDED

#if defined __GNUC__
#  pragma GCC system_header
#endif

#include "allocator.h"
#include "binary_node.h"
#include "marching_cubes_poisson.h"

#define DIMENSION 3

namespace pcl
{
  namespace poisson
  {

    template< class NodeData , class Real=float >
    class OctNode
    {
      private:
        static int UseAlloc;

        class AdjacencyCountFunction
        {
          public:
            int count;
            void Function( const OctNode<NodeData,Real>* node1 , const OctNode<NodeData,Real>* node2 );
        };
        template<class NodeAdjacencyFunction>
        void __processNodeFaces(OctNode* node,NodeAdjacencyFunction* F,int cIndex1,int cIndex2,int cIndex3,int cIndex4);
        template<class NodeAdjacencyFunction>
        void __processNodeEdges(OctNode* node,NodeAdjacencyFunction* F,int cIndex1,int cIndex2);
        template<class NodeAdjacencyFunction>
        void __processNodeNodes(OctNode* node,NodeAdjacencyFunction* F);
        template<class NodeAdjacencyFunction>
        static void __ProcessNodeAdjacentNodes(int dx,int dy,int dz,OctNode* node1,int radius1,OctNode* node2,int radius2,int cWidth2,NodeAdjacencyFunction* F);
        template<class TerminatingNodeAdjacencyFunction>
        static void __ProcessTerminatingNodeAdjacentNodes(int dx,int dy,int dz,OctNode* node1,int radius1,OctNode* node2,int radius2,int cWidth2,TerminatingNodeAdjacencyFunction* F);
        template<class PointAdjacencyFunction>
        static void __ProcessPointAdjacentNodes(int dx,int dy,int dz,OctNode* node2,int radius2,int cWidth2,PointAdjacencyFunction* F);
        template<class NodeAdjacencyFunction>
        static void __ProcessFixedDepthNodeAdjacentNodes(int dx,int dy,int dz,OctNode* node1,int radius1,OctNode* node2,int radius2,int cWidth2,int depth,NodeAdjacencyFunction* F);
        template<class NodeAdjacencyFunction>
        static void __ProcessMaxDepthNodeAdjacentNodes(int dx,int dy,int dz,OctNode* node1,int radius1,OctNode* node2,int radius2,int cWidth2,int depth,NodeAdjacencyFunction* F);

        // This is made private because the division by two has been pulled out.
        static inline int Overlap(int c1,int c2,int c3,int dWidth);
        inline static int ChildOverlap(int dx,int dy,int dz,int d,int cRadius2);

        const OctNode* __faceNeighbor(int dir,int off) const;
        const OctNode* __edgeNeighbor(int o,const int i[2],const int idx[2]) const;
        OctNode* __faceNeighbor(int dir,int off,int forceChildren);
        OctNode* __edgeNeighbor(int o,const int i[2],const int idx[2],int forceChildren);
      public:
        static const int DepthShift,OffsetShift,OffsetShift1,OffsetShift2,OffsetShift3;
        static const int DepthMask,OffsetMask;

        static Allocator<OctNode> internalAllocator;
        static int UseAllocator(void);
        static void SetAllocator(int blockSize);

        OctNode* parent;
        OctNode* children;
        short d , off[DIMENSION];
        NodeData nodeData;

        OctNode(void);
        ~OctNode(void);
        int initChildren(void);

        void depthAndOffset(int& depth,int offset[DIMENSION]) const;
        int depth(void) const;
        static inline void DepthAndOffset(const long long& index,int& depth,int offset[DIMENSION]);
        static inline void CenterAndWidth(const long long& index,Point3D<Real>& center,Real& width);
        static inline int Depth(const long long& index);
        static inline void Index(int depth,const int offset[3],short& d,short off[DIMENSION]);
        void centerAndWidth( Point3D<Real>& center , Real& width ) const;
        bool isInside( Point3D< Real > p ) const;

        int leaves(void) const;
        int maxDepthLeaves(int maxDepth) const;
        int nodes(void) const;
        int maxDepth(void) const;

        const OctNode* root(void) const;

        const OctNode* nextLeaf(const OctNode* currentLeaf=NULL) const;
        OctNode* nextLeaf(OctNode* currentLeaf=NULL);
        const OctNode* nextNode(const OctNode* currentNode=NULL) const;
        OctNode* nextNode(OctNode* currentNode=NULL);
        const OctNode* nextBranch(const OctNode* current) const;
        OctNode* nextBranch(OctNode* current);
        const OctNode* prevBranch(const OctNode* current) const;
        OctNode* prevBranch(OctNode* current);

        void setFullDepth(int maxDepth);

        void printLeaves(void) const;
        void printRange(void) const;

        template<class NodeAdjacencyFunction>
        void processNodeFaces(OctNode* node,NodeAdjacencyFunction* F,int fIndex,int processCurrent=1);
        template<class NodeAdjacencyFunction>
        void processNodeEdges(OctNode* node,NodeAdjacencyFunction* F,int eIndex,int processCurrent=1);
        template<class NodeAdjacencyFunction>
        void processNodeCorners(OctNode* node,NodeAdjacencyFunction* F,int cIndex,int processCurrent=1);
        template<class NodeAdjacencyFunction>
        void processNodeNodes(OctNode* node,NodeAdjacencyFunction* F,int processCurrent=1);

        template<class NodeAdjacencyFunction>
        static void ProcessNodeAdjacentNodes(int maxDepth,OctNode* node1,int width1,OctNode* node2,int width2,NodeAdjacencyFunction* F,int processCurrent=1);
        template<class NodeAdjacencyFunction>
        static void ProcessNodeAdjacentNodes(int dx,int dy,int dz,OctNode* node1,int radius1,OctNode* node2,int radius2,int width2,NodeAdjacencyFunction* F,int processCurrent=1);
        template<class TerminatingNodeAdjacencyFunction>
        static void ProcessTerminatingNodeAdjacentNodes(int maxDepth,OctNode* node1,int width1,OctNode* node2,int width2,TerminatingNodeAdjacencyFunction* F,int processCurrent=1);
        template<class TerminatingNodeAdjacencyFunction>
        static void ProcessTerminatingNodeAdjacentNodes(int dx,int dy,int dz,OctNode* node1,int radius1,OctNode* node2,int radius2,int width2,TerminatingNodeAdjacencyFunction* F,int processCurrent=1);
        template<class PointAdjacencyFunction>
        static void ProcessPointAdjacentNodes(int maxDepth,const int center1[3],OctNode* node2,int width2,PointAdjacencyFunction* F,int processCurrent=1);
        template<class PointAdjacencyFunction>
        static void ProcessPointAdjacentNodes(int dx,int dy,int dz,OctNode* node2,int radius2,int width2,PointAdjacencyFunction* F,int processCurrent=1);
        template<class NodeAdjacencyFunction>
        static void ProcessFixedDepthNodeAdjacentNodes(int maxDepth,OctNode* node1,int width1,OctNode* node2,int width2,int depth,NodeAdjacencyFunction* F,int processCurrent=1);
        template<class NodeAdjacencyFunction>
        static void ProcessFixedDepthNodeAdjacentNodes(int dx,int dy,int dz,OctNode* node1,int radius1,OctNode* node2,int radius2,int width2,int depth,NodeAdjacencyFunction* F,int processCurrent=1);
        template<class NodeAdjacencyFunction>
        static void ProcessMaxDepthNodeAdjacentNodes(int maxDepth,OctNode* node1,int width1,OctNode* node2,int width2,int depth,NodeAdjacencyFunction* F,int processCurrent=1);
        template<class NodeAdjacencyFunction>
        static void ProcessMaxDepthNodeAdjacentNodes(int dx,int dy,int dz,OctNode* node1,int radius1,OctNode* node2,int radius2,int width2,int depth,NodeAdjacencyFunction* F,int processCurrent=1);

        static int CornerIndex(const Point3D<Real>& center,const Point3D<Real> &p);

        OctNode* faceNeighbor(int faceIndex,int forceChildren=0);
        const OctNode* faceNeighbor(int faceIndex) const;
        OctNode* edgeNeighbor(int edgeIndex,int forceChildren=0);
        const OctNode* edgeNeighbor(int edgeIndex) const;
        OctNode* cornerNeighbor(int cornerIndex,int forceChildren=0);
        const OctNode* cornerNeighbor(int cornerIndex) const;

        OctNode* getNearestLeaf(const Point3D<Real>& p);
        const OctNode* getNearestLeaf(const Point3D<Real>& p) const;

        static int CommonEdge(const OctNode* node1,int eIndex1,const OctNode* node2,int eIndex2);
        static int CompareForwardDepths(const void* v1,const void* v2);
        static int CompareByDepthAndXYZ( const void* v1 , const void* v2 );
        static int CompareByDepthAndZIndex( const void* v1 , const void* v2 );
        static int CompareForwardPointerDepths(const void* v1,const void* v2);
        static int CompareBackwardDepths(const void* v1,const void* v2);
        static int CompareBackwardPointerDepths(const void* v1,const void* v2);


        template<class NodeData2>
        OctNode& operator = (const OctNode<NodeData2,Real>& node);

        static inline int Overlap2(const int &depth1,const int offSet1[DIMENSION],const Real& multiplier1,const int &depth2,const int offSet2[DIMENSION],const Real& multiplier2);


        int write(const char* fileName) const;
        int write(FILE* fp) const;
        int read(const char* fileName);
        int read(FILE* fp);

        class Neighbors3
        {
          public:
            OctNode* neighbors[3][3][3];
            Neighbors3( void );
            void clear( void );
        };
        class NeighborKey3
        {
          public:
            Neighbors3* neighbors;

            NeighborKey3( void );
            ~NeighborKey3( void );

            void set( int depth );
            Neighbors3& setNeighbors( OctNode* root , Point3D< Real > p , int d );
            Neighbors3& getNeighbors( OctNode* root , Point3D< Real > p , int d );
            Neighbors3& setNeighbors( OctNode* node , bool flags[3][3][3] );
            Neighbors3& setNeighbors( OctNode* node );
            Neighbors3& getNeighbors( OctNode* node );
        };
        class ConstNeighbors3
        {
          public:
            const OctNode* neighbors[3][3][3];
            ConstNeighbors3( void );
            void clear( void );
        };
        class ConstNeighborKey3
        {
          public:
            ConstNeighbors3* neighbors;

            ConstNeighborKey3(void);
            ~ConstNeighborKey3(void);

            void set(int depth);
            ConstNeighbors3& getNeighbors( const OctNode* node );
            ConstNeighbors3& getNeighbors( const OctNode* node , int minDepth );
        };
        class Neighbors5
        {
          public:
            OctNode*  neighbors[5][5][5];
            Neighbors5( void );
            void clear( void );
        };
        class ConstNeighbors5
        {
          public:
            const OctNode* neighbors[5][5][5];
            ConstNeighbors5( void );
            void clear( void );
        };

        class NeighborKey5
        {
            int _depth;
          public:
            Neighbors5* neighbors;

            NeighborKey5( void );
            ~NeighborKey5( void );

            void set( int depth );
            Neighbors5& getNeighbors( OctNode* node );
            Neighbors5& setNeighbors( OctNode* node ,  int xStart=0 , int xEnd=5 , int yStart=0 , int yEnd=5 , int zStart=0 , int zEnd=5 );
        };
        class ConstNeighborKey5
        {
            int _depth;
          public:
            ConstNeighbors5* neighbors;

            ConstNeighborKey5( void );
            ~ConstNeighborKey5( void );

            void set( int depth );
            ConstNeighbors5& getNeighbors( const OctNode* node );
        };

        void centerIndex(int maxDepth,int index[DIMENSION]) const;
        int width(int maxDepth) const;
    };


  }
}

#include "octree_poisson.hpp"



#endif // OCT_NODE
