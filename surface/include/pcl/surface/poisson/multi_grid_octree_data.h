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
 * $Id: multi_grid_octree_data.h 5531 2012-04-08 09:14:33Z aichim $
 *
 */

#ifndef PCL_POISSON_MULTI_GRID_OCTREE_DATA_H_
#define PCL_POISSON_MULTI_GRID_OCTREE_DATA_H_

#include "hash.h"

namespace pcl 
{
  namespace poisson 
  {
    typedef float Real;
    typedef float FunctionDataReal;
    typedef OctNode<class TreeNodeData,Real> TreeOctNode;

    class RootInfo
    {
      public:
        const TreeOctNode* node;
        int edgeIndex;
        long long key;
    };

    class VertexData
    {
      public:
        static long long 
        EdgeIndex (const TreeOctNode* node, const int& eIndex, const int& maxDepth, int index[DIMENSION]);

        static long long 
        EdgeIndex (const TreeOctNode* node, const int& eIndex, const int& maxDepth);

        static long long 
        FaceIndex (const TreeOctNode* node, const int& fIndex, const int& maxDepth, int index[DIMENSION]);

        static long long 
        FaceIndex (const TreeOctNode* node, const int& fIndex, const int& maxDepth);

        static long long 
        CornerIndex (const int& depth, const int offSet[DIMENSION] ,const int& cIndex, const int& maxDepth, int index[DIMENSION]);

        static long long 
        CornerIndex (const TreeOctNode* node, const int& cIndex, const int& maxDepth, int index[DIMENSION]);

        static long long 
        CornerIndex (const TreeOctNode* node, const int& cIndex, const int& maxDepth);

        static long long 
        CenterIndex (const int& depth, const int offSet[DIMENSION], const int& maxDepth, int index[DIMENSION]);

        static long long 
        CenterIndex (const TreeOctNode* node, const int& maxDepth, int index[DIMENSION]);
        
        static long long 
        CenterIndex (const TreeOctNode* node, const int& maxDepth);
    };

    class SortedTreeNodes
    {
      public:
        TreeOctNode** treeNodes;
        int *nodeCount;
        int maxDepth;
        SortedTreeNodes ();
        ~SortedTreeNodes ();
        void 
        set (TreeOctNode& root,const int& setIndex);
    };

    class TreeNodeData
    {
      public:
        static int UseIndex;
        union
        {
          int mcIndex;
          struct
          {
            int nodeIndex;
            Real centerWeightContribution;
          };
        };
        Real value;

        TreeNodeData (void);
        ~TreeNodeData (void);
    };

    template<int Degree>
    class Octree
    {
      TreeOctNode::NeighborKey neighborKey;
      TreeOctNode::NeighborKey2 neighborKey2;

      Real radius;
      int width;

      void 
      setNodeIndices (TreeOctNode& tree,int& idx);
      
      Real 
      GetDotProduct (const int index[DIMENSION]) const;

      Real 
      GetLaplacian (const int index[DIMENSION]) const;

      Real 
      GetDivergence (const int index[DIMENSION], const Point3D<Real>& normal) const;

      class DivergenceFunction
      {
        public:
          Point3D<Real> normal;
          Octree<Degree>* ot;
          int index[DIMENSION],scratch[DIMENSION];

          void 
          Function (TreeOctNode* node1, const TreeOctNode* node2);
      };

      class LaplacianProjectionFunction
      {
        public:
          double value;
          Octree<Degree>* ot;
          int index[DIMENSION],scratch[DIMENSION];

          void 
          Function (TreeOctNode* node1, const TreeOctNode* node2);
      };

      class LaplacianMatrixFunction
      {
        public:
          int x2,y2,z2,d2;
          Octree<Degree>* ot;
          int index[DIMENSION],scratch[DIMENSION];
          int elementCount,offset;
          MatrixEntry<float>* rowElements;

          int 
          Function (const TreeOctNode* node1, const TreeOctNode* node2);
      };

      class RestrictedLaplacianMatrixFunction
      {
        public:
          int depth,offset[3];
          Octree<Degree>* ot;
          Real radius;
          int index[DIMENSION], scratch[DIMENSION];
          int elementCount;
          MatrixEntry<float>* rowElements;

          int 
          Function (const TreeOctNode* node1, const TreeOctNode* node2);
      };

      ///////////////////////////
      // Evaluation Functions  //
      ///////////////////////////
      class PointIndexValueFunction
      {
        public:
          int res2;
          FunctionDataReal* valueTables;
          int index[DIMENSION];
          Real value;

          void 
          Function (const TreeOctNode* node);
      };

      class PointIndexValueAndNormalFunction
      {
        public:
          int res2;
          FunctionDataReal* valueTables;
          FunctionDataReal* dValueTables;
          Real value;
          Point3D<Real> normal;
          int index[DIMENSION];

          void 
          Function (const TreeOctNode* node);
      };

      class AdjacencyCountFunction
      {
        public:
          int adjacencyCount;

          void 
          Function (const TreeOctNode* node1, const TreeOctNode* node2);
      };

      class AdjacencySetFunction
      {
        public:
          int *adjacencies,adjacencyCount;
          void 
          Function (const TreeOctNode* node1, const TreeOctNode* node2);
      };

      class RefineFunction
      {
        public:
          int depth;
          void 
          Function (TreeOctNode* node1, const TreeOctNode* node2);
      };

      class FaceEdgesFunction 
      {
        public:
          int fIndex,maxDepth;
          std::vector<std::pair<long long,long long> >* edges;
          hash_map<long long, std::pair<RootInfo,int> >* vertexCount;

          void 
          Function (const TreeOctNode* node1, const TreeOctNode* node2);
      };

      int 
      SolveFixedDepthMatrix (const int& depth, const SortedTreeNodes& sNodes);
      
      int 
      SolveFixedDepthMatrix (const int& depth, const int& startingDepth, const SortedTreeNodes& sNodes);

      int 
      GetFixedDepthLaplacian (SparseSymmetricMatrix<float>& matrix, const int& depth, const SortedTreeNodes& sNodes);

      int 
      GetRestrictedFixedDepthLaplacian (SparseSymmetricMatrix<float>& matrix,
                                        const int& depth,
                                        const int* entries,
                                        const int& entryCount,
                                        const TreeOctNode* rNode,
                                        const Real& radius,
                                        const SortedTreeNodes& sNodes);

      void 
      SetIsoSurfaceCorners (const Real& isoValue, const int& subdivisionDepth, const int& fullDepthIso);

      static int 
      IsBoundaryFace (const TreeOctNode* node, const int& faceIndex, const int& subdivideDepth);
      
      static int 
      IsBoundaryEdge (const TreeOctNode* node, const int& edgeIndex, const int& subdivideDepth);
      
      static int 
      IsBoundaryEdge (const TreeOctNode* node, const int& dir, const int& x, const int& y, const int& subidivideDepth);
      
      void 
      PreValidate (const Real& isoValue, const int& maxDepth, const int& subdivideDepth);
      
      void 
      PreValidate (TreeOctNode* node, const Real& isoValue, const int& maxDepth, const int& subdivideDepth);
      
      void 
      Validate (TreeOctNode* node,
                const Real& isoValue,
                const int& maxDepth,
                const int& fullDepthIso,
                const int& subdivideDepth);

      void 
      Validate (TreeOctNode* node, const Real& isoValue, const int& maxDepth, const int& fullDepthIso);

      void 
      Subdivide (TreeOctNode* node, const Real& isoValue, const int& maxDepth);

      int 
      SetBoundaryMCRootPositions (const int& sDepth,const Real& isoValue,
                                  hash_map<long long,int>& boundaryRoots,
                                  hash_map<long long,
                                  std::pair<Real,Point3D<Real> > >& boundaryNormalHash,
                                  CoredMeshData* mesh,
                                  const int& nonLinearFit);

      int 
      SetMCRootPositions (TreeOctNode* node,
                          const int& sDepth,
                          const Real& isoValue,
                          hash_map<long long, int>& boundaryRoots,
                          hash_map<long long, int>* interiorRoots,
                          hash_map<long long, std::pair<Real,Point3D<Real> > >& boundaryNormalHash,
                          hash_map<long long, std::pair<Real,Point3D<Real> > >* interiorNormalHash,
                          std::vector<Point3D<float> >* interiorPositions,
                          CoredMeshData* mesh,
                          const int& nonLinearFit);

      int 
      GetMCIsoTriangles (TreeOctNode* node,
                         CoredMeshData* mesh,
                         hash_map<long long,int>& boundaryRoots,
                         hash_map<long long,int>* interiorRoots,
                         std::vector<Point3D<float> >* interiorPositions,
                         const int& offSet,
                         const int& sDepth, 
                         bool addBarycenter, 
                         bool polygonMesh);

      static int 
      AddTriangles (CoredMeshData* mesh,
                    std::vector<CoredPointIndex> edges[3],
                    std::vector<Point3D<float> >* interiorPositions, 
                    const int& offSet);
      
      static int 
      AddTriangles (CoredMeshData* mesh, 
                    std::vector<CoredPointIndex>& edges, std::vector<Point3D<float> >* interiorPositions, 
                    const int& offSet, 
                    bool addBarycenter, 
                    bool polygonMesh);

      void 
      GetMCIsoEdges (TreeOctNode* node,
                     hash_map<long long,int>& boundaryRoots,
                     hash_map<long long,int>* interiorRoots,
                     const int& sDepth,
                     std::vector<std::pair<long long,long long> >& edges);

      static int 
      GetEdgeLoops (std::vector<std::pair<long long,long long> >& edges,
                    std::vector<std::vector<std::pair<long long,long long> > >& loops);

      static int 
      InteriorFaceRootCount (const TreeOctNode* node,const int &faceIndex,const int& maxDepth);

      static int 
      EdgeRootCount (const TreeOctNode* node,const int& edgeIndex,const int& maxDepth);

      int 
      GetRoot (const RootInfo& ri,
               const Real& isoValue,
               const int& maxDepth,Point3D<Real> & position,
               hash_map<long long,std::pair<Real,Point3D<Real> > >& normalHash,
               Point3D<Real>* normal,
               const int& nonLinearFit);

      int 
      GetRoot (const RootInfo& ri,
               const Real& isoValue,
               Point3D<Real> & position,
               hash_map<long long,
               std::pair<Real,Point3D<Real> > >& normalHash,
               const int& nonLinearFit);

      static int 
      GetRootIndex (const TreeOctNode* node,const int& edgeIndex,const int& maxDepth,RootInfo& ri);

      static int 
      GetRootIndex (const TreeOctNode* node, 
                    const int& edgeIndex,
                    const int& maxDepth,
                    const int& sDepth,
                    RootInfo& ri);
      
      static int 
      GetRootIndex (const long long& key,
                    hash_map<long long,int>& boundaryRoots,
                    hash_map<long long,int>* interiorRoots,
                    CoredPointIndex& index);
      
      static int 
      GetRootPair (const RootInfo& root,const int& maxDepth,RootInfo& pair);

      int 
      NonLinearUpdateWeightContribution (TreeOctNode* node,
                                         const Point3D<Real>& position,
                                         const Real& weight = Real(1.0));

      Real 
      NonLinearGetSampleWeight (TreeOctNode* node,
                                const Point3D<Real>& position);
      
      void 
      NonLinearGetSampleDepthAndWeight (TreeOctNode* node,
                                        const Point3D<Real>& position,
                                        const Real& samplesPerNode,
                                        Real& depth,
                                        Real& weight);

      int 
      NonLinearSplatOrientedPoint (TreeOctNode* node,
                                   const Point3D<Real>& point,
                                   const Point3D<Real>& normal);
      
      void 
      NonLinearSplatOrientedPoint (const Point3D<Real>& point,
                                   const Point3D<Real>& normal,
                                   const int& kernelDepth,
                                   const Real& samplesPerNode,
                                   const int& minDepth,
                                   const int& maxDepth);

      int 
      HasNormals (TreeOctNode* node,const Real& epsilon);

      Real 
      getCenterValue (const TreeOctNode* node);

      Real 
      getCornerValue (const TreeOctNode* node,const int& corner);

      void 
      getCornerValueAndNormal (const TreeOctNode* node,const int& corner,Real& value,Point3D<Real>& normal);

      public:
        static double maxMemoryUsage;
        static double 
        MemoryUsage ();

        std::vector<Point3D<Real> >* normals;
        Real postNormalSmooth;
        TreeOctNode tree;
        FunctionData<Degree,FunctionDataReal> fData;
        Octree ();

        void 
        setFunctionData (const PPolynomial<Degree>& ReconstructionFunction,
                         const int& maxDepth,
                         const int& normalize,
                         const Real& normalSmooth = -1);
        
        void 
        finalize1 (const int& refineNeighbors=-1);
        
        void 
        finalize2 (const int& refineNeighbors=-1);

        //int setTree(char* fileName,const int& maxDepth,const int& binary,const int& kernelDepth,const Real& samplesPerNode,
        //	const Real& scaleFactor,Point3D<Real>& center,Real& scale,const int& resetSampleDepths,const int& useConfidence);

        template<typename PointNT> int
        setTree (boost::shared_ptr<const pcl::PointCloud<PointNT> > input_,
                 const int& maxDepth,
                 const int& kernelDepth,
                 const Real& samplesPerNode,
                 const Real& scaleFactor,
                 Point3D<Real>& center,
                 Real& scale,
                 const int& resetSamples,
                 const int& useConfidence);


        void 
        SetLaplacianWeights (void);
        
        void 
        ClipTree (void);

        int 
        LaplacianMatrixIteration (const int& subdivideDepth);

        Real 
        GetIsoValue (void);
        
        void 
        GetMCIsoTriangles (const Real& isoValue,
                           CoredMeshData* mesh,
                           const int& fullDepthIso = 0,
                           const int& nonLinearFit = 1, 
                           bool addBarycenter = false, 
                           bool polygonMesh = false);
        
        void 
        GetMCIsoTriangles (const Real& isoValue,
                           const int& subdivideDepth,
                           CoredMeshData* mesh,
                           const int& fullDepthIso = 0,
                           const int& nonLinearFit = 1, 
                           bool addBarycenter = false, 
                           bool polygonMesh = false );
    };
  }
}


#include <pcl/surface/impl/poisson/multi_grid_octree_data.hpp>
#endif /* PCL_POISSON_MULTI_GRID_OCTREE_DATA_H_ */
