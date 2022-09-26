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

#ifndef MULTI_GRID_OCTREE_DATA_INCLUDED
#define MULTI_GRID_OCTREE_DATA_INCLUDED

#if defined __GNUC__
#  pragma GCC system_header
#endif

#define MISHA_DEBUG 1

#define GRADIENT_DOMAIN_SOLUTION 1	// Given the constraint vector-field V(p), there are two ways to solve for the coefficients, x, of the indicator function
// with respect to the B-spline basis {B_i(p)}
// 1] Find x minimizing:
//			|| V(p) - \sum_i \nabla x_i B_i(p) ||^2
//		which is solved by the system A_1x = b_1 where:
//			A_1[i,j] = < \nabla B_i(p) , \nabla B_j(p) >
//			b_1[i]   = < \nabla B_i(p) , V(p) >
// 2] Formulate this as a Poisson equation:
//			\sum_i x_i \Delta B_i(p) = \nabla \cdot V(p)
//		which is solved by the system A_2x = b_2 where:
//			A_2[i,j] = - < \Delta B_i(p) , B_j(p) >
//			b_2[i]   = - < B_i(p) , \nabla \cdot V(p) >
// Although the two system matrices should be the same (assuming that the B_i satisfy dirichlet/neumann boundary conditions)
// the constraint vectors can differ when V does not satisfy the Neumann boundary conditions:
//		A_1[i,j] = \int_R < \nabla B_i(p) , \nabla B_j(p) >
//               = \int_R [ \nabla \cdot ( B_i(p) \nabla B_j(p) ) - B_i(p) \Delta B_j(p) ]
//               = \int_dR < N(p) , B_i(p) \nabla B_j(p) > + A_2[i,j]
// and the first integral is zero if either f_i is zero on the boundary dR or the derivative of B_i across the boundary is zero.
// However, for the constraints we have:
//		b_1(i)   = \int_R < \nabla B_i(p) , V(p) >
//               = \int_R [ \nabla \cdot ( B_i(p) V(p) ) - B_i(p) \nabla \cdot V(p) ]
//               = \int_dR < N(p) ,  B_i(p) V(p) > + b_2[i]
// In particular, this implies that if the B_i satisfy the Neumann boundary conditions (rather than Dirichlet),
// and V is not zero across the boundary, then the two constraints are different.
// Forcing the < V(p) , N(p) > = 0 on the boundary, by killing off the component of the vector-field in the normal direction
// (FORCE_NEUMANN_FIELD), makes the two systems equal, and the value of this flag should be immaterial.
// Note that under interpretation 1, we have:
//		\sum_i b_1(i) = < \nabla \sum_ i B_i(p) , V(p) > = 0
// because the B_i's sum to one. However, in general, we could have
//		\sum_i b_2(i) \neq 0.
// This could cause trouble because the constant functions are in the kernel of the matrix A, so CG will misbehave if the constraint
// has a non-zero DC term. (Again, forcing < V(p) , N(p) > = 0 along the boundary resolves this problem.)

#define FORCE_NEUMANN_FIELD 1		// This flag forces the normal component across the boundary of the integration domain to be zero.
// This should be enabled if GRADIENT_DOMAIN_SOLUTION is not, so that CG doesn't run into trouble.


#include <unordered_map>

#include "bspline_data.h"


namespace pcl
{
  namespace poisson
  {

    typedef float Real;
    typedef float BSplineDataReal;
    typedef pcl::poisson::OctNode< class TreeNodeData , Real > TreeOctNode;



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
        static long long EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth , int index[DIMENSION] );
        static long long EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth );
        static long long FaceIndex( const TreeOctNode* node , int fIndex , int maxDepth,int index[DIMENSION] );
        static long long FaceIndex( const TreeOctNode* node , int fIndex , int maxDepth );
        static long long CornerIndex( int depth , const int offSet[DIMENSION] , int cIndex , int maxDepth , int index[DIMENSION] );
        static long long CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth , int index[DIMENSION] );
        static long long CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth );
        static long long CenterIndex( int depth , const int offSet[DIMENSION] , int maxDepth , int index[DIMENSION] );
        static long long CenterIndex( const TreeOctNode* node , int maxDepth , int index[DIMENSION] );
        static long long CenterIndex( const TreeOctNode* node , int maxDepth );
        static long long CornerIndexKey( const int index[DIMENSION] );
    };
    class SortedTreeNodes
    {
      public:
        TreeOctNode** treeNodes;
        int *nodeCount;
        int maxDepth;
        SortedTreeNodes( void );
        ~SortedTreeNodes( void );
        void set( TreeOctNode& root );
        struct CornerIndices
        {
            int idx[pcl::poisson::Cube::CORNERS] = {-1, -1, -1, -1, -1, -1, -1, -1};
            CornerIndices( void ) = default;
            int& operator[] ( int i ) { return idx[i]; }
            const int& operator[] ( int i ) const { return idx[i]; }
        };
        struct CornerTableData
        {
            CornerTableData( void ) { cCount=0; }
            ~CornerTableData( void ) { clear(); }
            void clear( void ) { cTable.clear() ; cCount = 0; }
            CornerIndices& operator[] ( const TreeOctNode* node );
            const CornerIndices& operator[] ( const TreeOctNode* node ) const;
            CornerIndices& cornerIndices( const TreeOctNode* node );
            const CornerIndices& cornerIndices( const TreeOctNode* node ) const;
            int cCount;
            std::vector< CornerIndices > cTable;
            std::vector< int > offsets;
        };
        void setCornerTable( CornerTableData& cData , const TreeOctNode* rootNode , int depth , int threads ) const;
        void setCornerTable( CornerTableData& cData , const TreeOctNode* rootNode ,             int threads ) const { setCornerTable( cData , rootNode , maxDepth-1 , threads ); }
        void setCornerTable( CornerTableData& cData ,                                           int threads ) const { setCornerTable( cData , treeNodes[0] , maxDepth-1 , threads ); }
        int getMaxCornerCount( const TreeOctNode* rootNode , int depth , int maxDepth , int threads ) const ;
        struct EdgeIndices
        {
            int idx[pcl::poisson::Cube::EDGES] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
           EdgeIndices( void ) = default;
            int& operator[] ( int i ) { return idx[i]; }
            const int& operator[] ( int i ) const { return idx[i]; }
        };
        struct EdgeTableData
        {
            EdgeTableData( void ) { eCount=0; }
            ~EdgeTableData( void ) { clear(); }
            void clear( void ) { eTable.clear() , eCount=0; }
            EdgeIndices& operator[] ( const TreeOctNode* node );
            const EdgeIndices& operator[] ( const TreeOctNode* node ) const;
            EdgeIndices& edgeIndices( const TreeOctNode* node );
            const EdgeIndices& edgeIndices( const TreeOctNode* node ) const;
            int eCount;
            std::vector< EdgeIndices > eTable;
            std::vector< int > offsets;
        };
        void setEdgeTable( EdgeTableData& eData , const TreeOctNode* rootNode , int depth , int threads );
        void setEdgeTable( EdgeTableData& eData , const TreeOctNode* rootNode ,             int threads ) { setEdgeTable( eData , rootNode , maxDepth-1 , threads ); }
        void setEdgeTable( EdgeTableData& eData ,                                           int threads ) { setEdgeTable( eData , treeNodes[0] , maxDepth-1 , threads ); }
        int getMaxEdgeCount( const TreeOctNode* rootNode , int depth , int threads ) const ;
    };

    class TreeNodeData
    {
      public:
        static int UseIndex;
        int nodeIndex;
        union
        {
            int mcIndex;
            struct
            {
                Real centerWeightContribution;
                int normalIndex;
            };
        };
        Real constraint , solution;
        int pointIndex;

        TreeNodeData(void);
        ~TreeNodeData(void);
    };

    template< int Degree >
    class Octree
    {
        SortedTreeNodes _sNodes;
        int _minDepth;
        bool _constrainValues;
        std::vector< int > _pointCount;
        struct PointData
        {
            pcl::poisson::Point3D< Real > position;
            Real weight;
            Real value;
            PointData( pcl::poisson::Point3D< Real > p , Real w , Real v=0 ) { position = p , weight = w , value = v; }
        };
        std::vector< PointData > _points;
        TreeOctNode::NeighborKey3 neighborKey;
        TreeOctNode::ConstNeighborKey3 neighborKey2;

        Real radius;
        int width;
        Real GetLaplacian( const int index[DIMENSION] ) const;
        // Note that this is a slight misnomer. We're only taking the diveregence/Laplacian in the weak sense, so there is a change of sign.
        Real GetLaplacian( const TreeOctNode* node1 , const TreeOctNode* node2 ) const;
        Real GetDivergence( const TreeOctNode* node1 , const TreeOctNode* node2 , const pcl::poisson::Point3D<Real>& normal1 ) const;
        Real GetDivergenceMinusLaplacian( const TreeOctNode* node1 , const TreeOctNode* node2 , Real value1 , const pcl::poisson::Point3D<Real>& normal1 ) const;
        struct PointInfo
        {
            float splineValues[3][3];
            float weightedValue;
        };
        Real GetValue( const PointInfo points[3][3][3] , const bool hasPoints[3][3] , const int d[3] ) const;

        class AdjacencyCountFunction
        {
          public:
            int adjacencyCount;
            void Function(const TreeOctNode* node1,const TreeOctNode* node2);
        };
        class AdjacencySetFunction{
          public:
            int *adjacencies,adjacencyCount;
            void Function(const TreeOctNode* node1,const TreeOctNode* node2);
        };

        class RefineFunction{
          public:
            int depth;
            void Function(TreeOctNode* node1,const TreeOctNode* node2);
        };
        class FaceEdgesFunction
        {
          public:
            int fIndex , maxDepth;
            std::vector< std::pair< RootInfo , RootInfo > >* edges;
            std::unordered_map< long long , std::pair< RootInfo , int > >* vertexCount;
            void Function( const TreeOctNode* node1 , const TreeOctNode* node2 );
        };

        int SolveFixedDepthMatrix( int depth , const SortedTreeNodes& sNodes , Real* subConstraints ,                     bool showResidual , int minIters , double accuracy );
        int SolveFixedDepthMatrix( int depth , const SortedTreeNodes& sNodes , Real* subConstraints , int startingDepth , bool showResidual , int minIters , double accuracy );

        void SetMatrixRowBounds( const TreeOctNode* node , int rDepth , const int rOff[3] , int& xStart , int& xEnd , int& yStart , int& yEnd , int& zStart , int& zEnd ) const;
        int GetMatrixRowSize( const TreeOctNode::Neighbors5& neighbors5 ) const;
        int GetMatrixRowSize( const TreeOctNode::Neighbors5& neighbors5 , int xStart , int xEnd , int yStart , int yEnd , int zStart , int zEnd ) const;
        int SetMatrixRow( const TreeOctNode::Neighbors5& neighbors5 , pcl::poisson::MatrixEntry< float >* row , int offset , const double stencil[5][5][5] ) const;
        int SetMatrixRow( const TreeOctNode::Neighbors5& neighbors5 , pcl::poisson::MatrixEntry< float >* row , int offset , const double stencil[5][5][5] , int xStart , int xEnd , int yStart , int yEnd , int zStart , int zEnd ) const;
        void SetDivergenceStencil( int depth , pcl::poisson::Point3D< double > *stencil , bool scatter ) const;
        void SetLaplacianStencil( int depth , double stencil[5][5][5] ) const;
        template< class C , int N > struct Stencil{ C values[N][N][N]; };
        void SetLaplacianStencils( int depth , Stencil< double , 5 > stencil[2][2][2] ) const;
        void SetDivergenceStencils( int depth , Stencil< pcl::poisson::Point3D< double > , 5 > stencil[2][2][2] , bool scatter ) const;
        void SetEvaluationStencils( int depth , Stencil< double , 3 > stencil1[8] , Stencil< double , 3 > stencil2[8][8] ) const;

        static void UpdateCoarserSupportBounds( const TreeOctNode* node , int& startX , int& endX , int& startY , int& endY , int& startZ , int& endZ );
        void UpdateConstraintsFromCoarser( const TreeOctNode::NeighborKey5& neighborKey5 , TreeOctNode* node , Real* metSolution , const Stencil< double , 5 >& stencil ) const;
        void SetCoarserPointValues( int depth , const SortedTreeNodes& sNodes , Real* metSolution );
        Real WeightedCoarserFunctionValue( const TreeOctNode::NeighborKey3& neighborKey3 , const TreeOctNode* node , Real* metSolution ) const;
        void UpSampleCoarserSolution( int depth , const SortedTreeNodes& sNodes , pcl::poisson::Vector< Real >& solution ) const;
        void DownSampleFinerConstraints( int depth , SortedTreeNodes& sNodes ) const;
        template< class C > void DownSample( int depth , const SortedTreeNodes& sNodes , C* constraints ) const;
        template< class C > void   UpSample( int depth , const SortedTreeNodes& sNodes , C* coefficients ) const;
        int GetFixedDepthLaplacian( pcl::poisson::SparseSymmetricMatrix<float>& matrix , int depth , const SortedTreeNodes& sNodes , Real* subConstraints );
        int GetRestrictedFixedDepthLaplacian( pcl::poisson::SparseSymmetricMatrix<float>& matrix , int depth , const int* entries , int entryCount , const TreeOctNode* rNode, Real radius , const SortedTreeNodes& sNodes , Real* subConstraints );

        void SetIsoCorners( Real isoValue , TreeOctNode* leaf , SortedTreeNodes::CornerTableData& cData , char* valuesSet , Real* values , TreeOctNode::ConstNeighborKey3& nKey , const Real* metSolution , const Stencil< double , 3 > stencil1[8] , const Stencil< double , 3 > stencil2[8][8] );
        static int IsBoundaryFace( const TreeOctNode* node , int faceIndex , int subdivideDepth );
        static int IsBoundaryEdge( const TreeOctNode* node , int edgeIndex , int subdivideDepth );
        static int IsBoundaryEdge( const TreeOctNode* node , int dir , int x , int y , int subidivideDepth );

        // For computing the iso-surface there is a lot of re-computation of information across shared geometry.
        // For function values we don't care so much.
        // For edges we need to be careful so that the mesh remains water-tight
        struct RootData : public SortedTreeNodes::CornerTableData , public SortedTreeNodes::EdgeTableData
        {
            // Edge to iso-vertex map
            std::unordered_map< long long , int > boundaryRoots;
            // Vertex to ( value , normal ) map
            std::unordered_map< long long , std::pair< Real , pcl::poisson::Point3D< Real > > > *boundaryValues;
            int* interiorRoots;
            Real *cornerValues;
            pcl::poisson::Point3D< Real >* cornerNormals;
            char *cornerValuesSet , *cornerNormalsSet , *edgesSet;
        };

        int SetBoundaryMCRootPositions( int sDepth , Real isoValue , RootData& rootData , pcl::poisson::CoredMeshData* mesh , int nonLinearFit );
        int SetMCRootPositions( TreeOctNode* node , int sDepth , Real isoValue , TreeOctNode::ConstNeighborKey5& neighborKey5 , RootData& rootData ,
                                std::vector< pcl::poisson::Point3D< float > >* interiorPositions , pcl::poisson::CoredMeshData* mesh , const Real* metSolution , int nonLinearFit );
#if MISHA_DEBUG
        int GetMCIsoTriangles( TreeOctNode* node , pcl::poisson::CoredMeshData* mesh , RootData& rootData ,
                               std::vector< pcl::poisson::Point3D< float > >* interiorPositions , int offSet , int sDepth , bool polygonMesh , std::vector< pcl::poisson::Point3D< float > >* barycenters );
        static int AddTriangles(  pcl::poisson::CoredMeshData* mesh , std::vector<pcl::poisson::CoredPointIndex>& edges , std::vector<pcl::poisson::Point3D<float> >* interiorPositions , int offSet , bool polygonMesh , std::vector< pcl::poisson::Point3D< float > >* barycenters );
#else // !MISHA_DEBUG
        int GetMCIsoTriangles( TreeOctNode* node ,  pcl::poisson::CoredMeshData* mesh , RootData& rootData ,
                               std::vector< pcl::poisson::Point3D< float > >* interiorPositions , int offSet , int sDepth , bool addBarycenter , bool polygonMesh );
        static int AddTriangles(  pcl::poisson::CoredMeshData* mesh , std::vector<CoredPointIndex>& edges , std::vector<Point3D<float> >* interiorPositions , int offSet , bool addBarycenter , bool polygonMesh );
#endif // MISHA_DEBUG


        void GetMCIsoEdges( TreeOctNode* node , int sDepth , std::vector< std::pair< RootInfo , RootInfo > >& edges );
        static int GetEdgeLoops( std::vector< std::pair< RootInfo , RootInfo > >& edges , std::vector< std::vector< std::pair< RootInfo , RootInfo > > >& loops);
        static int InteriorFaceRootCount( const TreeOctNode* node , const int &faceIndex , int maxDepth );
        static int EdgeRootCount( const TreeOctNode* node , int edgeIndex , int maxDepth );
        static void GetRootSpan( const RootInfo& ri , pcl::poisson::Point3D< float >& start , pcl::poisson::Point3D< float >& end );
        int GetRoot( const RootInfo& ri , Real isoValue , TreeOctNode::ConstNeighborKey5& neighborKey5 , pcl::poisson::Point3D<Real> & position , RootData& rootData , int sDepth , const Real* metSolution , int nonLinearFit );
        static int GetRootIndex( const TreeOctNode* node , int edgeIndex , int maxDepth , RootInfo& ri );
        static int GetRootIndex( const TreeOctNode* node , int edgeIndex , int maxDepth , int sDepth , RootInfo& ri );
        static int GetRootIndex( const RootInfo& ri , RootData& rootData , pcl::poisson::CoredPointIndex& index );
        static int GetRootPair( const RootInfo& root , int maxDepth , RootInfo& pair );

        int NonLinearUpdateWeightContribution(TreeOctNode* node,const pcl::poisson::Point3D<Real>& position,Real weight=Real(1.0));
        Real NonLinearGetSampleWeight(TreeOctNode* node,const pcl::poisson::Point3D<Real>& position);
        void NonLinearGetSampleDepthAndWeight(TreeOctNode* node,const pcl::poisson::Point3D<Real>& position,Real samplesPerNode,Real& depth,Real& weight);
        int NonLinearSplatOrientedPoint(TreeOctNode* node,const pcl::poisson::Point3D<Real>& point,const pcl::poisson::Point3D<Real>& normal);
        Real NonLinearSplatOrientedPoint(const pcl::poisson::Point3D<Real>& point,const pcl::poisson::Point3D<Real>& normal , int kernelDepth , Real samplesPerNode , int minDepth , int maxDepth);

        int HasNormals(TreeOctNode* node,Real epsilon);
        Real getCornerValue( const TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int corner , const Real* metSolution );
        pcl::poisson::Point3D< Real > getCornerNormal( const TreeOctNode::ConstNeighborKey5& neighborKey5 , const TreeOctNode* node , int corner , const Real* metSolution );
        Real getCornerValue( const TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int corner , const Real* metSolution , const double stencil1[3][3][3] , const double stencil2[3][3][3] );
        Real getCenterValue( const TreeOctNode::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node );
      public:
        int threads;
        static double maxMemoryUsage;
        static double MemoryUsage( void );
        std::vector< pcl::poisson::Point3D<Real> >* normals;
        Real postNormalSmooth;
        TreeOctNode tree;
        pcl::poisson::BSplineData<Degree,BSplineDataReal> fData;
        Octree( void );

        void setBSplineData( int maxDepth , Real normalSmooth=-1 , bool reflectBoundary=false );
        void finalize( void );
        void RefineBoundary( int subdivisionDepth );
        Real* GetWeightGrid( int& res , int depth=-1 );
        Real* GetSolutionGrid( int& res , float isoValue=0.f , int depth=-1 );

        template<typename PointNT> int
        setTree(typename pcl::PointCloud<PointNT>::ConstPtr input_ , int maxDepth , int minDepth ,
                 int kernelDepth , Real samplesPerNode , Real scaleFactor , Point3D<Real>& center , Real& scale ,
                 int useConfidence , Real constraintWeight , bool adaptiveWeights );

        void SetLaplacianConstraints(void);
        void ClipTree(void);
        int LaplacianMatrixIteration( int subdivideDepth , bool showResidual , int minIters , double accuracy );

        Real GetIsoValue(void);
        void GetMCIsoTriangles( Real isoValue , int subdivideDepth ,  pcl::poisson::CoredMeshData* mesh , int fullDepthIso=0 , int nonLinearFit=1 , bool addBarycenter=false , bool polygonMesh=false );
    };


  }
}




#include "multi_grid_octree_data.hpp"
#endif // MULTI_GRID_OCTREE_DATA_INCLUDED
