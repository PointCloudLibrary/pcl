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

#include "octree_poisson.h"
#include "mat.h"

#if defined WIN32 || defined _WIN32
  #if !defined __MINGW32__
    #include <intrin.h>
    #include <hash_map>
  #endif
#endif


#define ITERATION_POWER 1.0/3
#define MEMORY_ALLOCATOR_BLOCK_SIZE 1<<12
#define SPLAT_ORDER 2

#ifndef _MSC_VER
namespace std
{
  using namespace __gnu_cxx;
}
#endif

namespace pcl
{
  namespace poisson
  {

    const Real MATRIX_ENTRY_EPSILON = Real(0);
    const Real EPSILON=Real(1e-6);
    const Real ROUND_EPS=Real(1e-5);

#if defined _WIN32 && !defined __MINGW32__
    using stdext::hash_map;
#else
    using std::hash_map;
#endif


    void atomicOr(volatile int& dest, int value)
    {
#if defined _WIN32 && !defined __MINGW32__
    #if defined (_M_IX86)
      _InterlockedOr( (long volatile*)&dest, value );
    #else
      InterlockedOr( (long volatile*)&dest , value );
    #endif
#else // !_WIN32 || __MINGW32__
    #pragma omp atomic
      dest |= value;
#endif // _WIN32 && !__MINGW32__
    }


    /////////////////////
    // SortedTreeNodes //
    /////////////////////
    SortedTreeNodes::SortedTreeNodes(void)
    {
      nodeCount=NULL;
      treeNodes=NULL;
      maxDepth=0;
    }
    SortedTreeNodes::~SortedTreeNodes(void){
      if( nodeCount ) delete[] nodeCount;
      if( treeNodes ) delete[] treeNodes;
      nodeCount = NULL;
      treeNodes = NULL;
    }

    void SortedTreeNodes::set( TreeOctNode& root )
    {
      if( nodeCount ) delete[] nodeCount;
      if( treeNodes ) delete[] treeNodes;
      maxDepth = root.maxDepth()+1;
      nodeCount = new int[ maxDepth+1 ];
      treeNodes = new TreeOctNode*[ root.nodes() ];

      nodeCount[0] = 0 , nodeCount[1] = 1;
      treeNodes[0] = &root;
      for( int d=1 ; d<maxDepth ; d++ )
      {
        nodeCount[d+1] = nodeCount[d];
        for( int i=nodeCount[d-1] ; i<nodeCount[d] ; i++ )
        {
          TreeOctNode* temp = treeNodes[i];
          if( temp->children ) for( int c=0 ; c<8 ; c++ ) treeNodes[ nodeCount[d+1]++ ] = temp->children + c;
        }
      }
      for( int i=0 ; i<nodeCount[maxDepth] ; i++ ) treeNodes[i]->nodeData.nodeIndex = i;
    }
    SortedTreeNodes::CornerIndices& SortedTreeNodes::CornerTableData::operator[] ( const TreeOctNode* node ) { return cTable[ node->nodeData.nodeIndex + offsets[node->d] ]; }
    const SortedTreeNodes::CornerIndices& SortedTreeNodes::CornerTableData::operator[] ( const TreeOctNode* node ) const { return cTable[ node->nodeData.nodeIndex + offsets[node->d] ]; }
    SortedTreeNodes::CornerIndices& SortedTreeNodes::CornerTableData::cornerIndices( const TreeOctNode* node ) { return cTable[ node->nodeData.nodeIndex + offsets[node->d] ]; }
    const SortedTreeNodes::CornerIndices& SortedTreeNodes::CornerTableData::cornerIndices( const TreeOctNode* node ) const { return cTable[ node->nodeData.nodeIndex + offsets[node->d] ]; }
    void SortedTreeNodes::setCornerTable( CornerTableData& cData , const TreeOctNode* rootNode , int maxDepth , int threads ) const
    {
      if( threads<=0 ) threads = 1;
      // The vector of per-depth node spans
      std::vector< std::pair< int , int > > spans( this->maxDepth , std::pair< int , int >( -1 , -1 ) );
      int minDepth , off[3];
      rootNode->depthAndOffset( minDepth , off );
      cData.offsets.resize( this->maxDepth , -1 );
      int nodeCount = 0;
      {
        int start=rootNode->nodeData.nodeIndex , end=start;
        for( int d=minDepth ; d<=maxDepth ; d++ )
        {
          spans[d] = std::pair< int , int >( start , end+1 );
          cData.offsets[d] = nodeCount - spans[d].first;
          nodeCount += spans[d].second - spans[d].first;
          if( d<maxDepth )
          {
            while( start< end && !treeNodes[start]->children ) start++;
            while( end> start && !treeNodes[end  ]->children ) end--;
            if(    start==end && !treeNodes[start]->children ) break;
            start = treeNodes[start]->children[0].nodeData.nodeIndex;
            end   = treeNodes[end  ]->children[7].nodeData.nodeIndex;
          }
        }
      }
      cData.cTable.resize( nodeCount );
      std::vector< int > count( threads );
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        TreeOctNode::ConstNeighborKey3 neighborKey;
        neighborKey.set( maxDepth );
        int offset = nodeCount * t * Cube::CORNERS;
        count[t] = 0;
        for( int d=minDepth ; d<=maxDepth ; d++ )
        {
          int start = spans[d].first , end = spans[d].second , width = end-start;
          for( int i=start + (width*t)/threads ; i<start + (width*(t+1))/threads ; i++ )
          {
            TreeOctNode* node = treeNodes[i];
            if( d<maxDepth && node->children ) continue;
            const TreeOctNode::ConstNeighbors3& neighbors = neighborKey.getNeighbors( node , minDepth );
            for( int c=0 ; c<Cube::CORNERS ; c++ )	// Iterate over the cell's corners
            {
              bool cornerOwner = true;
              int x , y , z;
              int ac = Cube::AntipodalCornerIndex( c ); // The index of the node relative to the corner
              Cube::FactorCornerIndex( c , x , y , z );
              for( int cc=0 ; cc<Cube::CORNERS ; cc++ ) // Iterate over the corner's cells
              {
                int xx , yy , zz;
                Cube::FactorCornerIndex( cc , xx , yy , zz );
                xx += x , yy += y , zz += z;
                if( neighbors.neighbors[xx][yy][zz] )
                  if( cc<ac || ( d<maxDepth && neighbors.neighbors[xx][yy][zz]->children ) )
                  {
                    int _d , _off[3];
                    neighbors.neighbors[xx][yy][zz]->depthAndOffset( _d , _off );
                    _off[0] >>= (d-minDepth) , _off[1] >>= (d-minDepth) , _off[2] >>= (d-minDepth);
                    if( _off[0]==off[0] && _off[1]==off[1] && _off[2]==off[2] )
                    {
                      cornerOwner = false;
                      break;
                    }
                    else fprintf( stderr , "[WARNING] How did we leave the subtree?\n" );
                  }
              }
              if( cornerOwner )
              {
                const TreeOctNode* n = node;
                int d = n->depth();
                do
                {
                  const TreeOctNode::ConstNeighbors3& neighbors = neighborKey.neighbors[d];
                  // Set all the corner indices at the current depth
                  for( int cc=0 ; cc<Cube::CORNERS ; cc++ )
                  {
                    int xx , yy , zz;
                    Cube::FactorCornerIndex( cc , xx , yy , zz );
                    xx += x , yy += y , zz += z;
                    if( neighborKey.neighbors[d].neighbors[xx][yy][zz] )
                      cData[ neighbors.neighbors[xx][yy][zz] ][ Cube::AntipodalCornerIndex(cc) ] = count[t] + offset;
                  }
                  // If we are not at the root and the parent also has the corner
                  if( d==minDepth || n!=(n->parent->children+c) ) break;
                  n = n->parent;
                  d--;
                }
                while( 1 );
                count[t]++;
              }
            }
          }
        }
      }
      cData.cCount = 0;
      std::vector< int > offsets( threads+1 );
      offsets[0] = 0;
      for( int t=0 ; t<threads ; t++ ) cData.cCount += count[t] , offsets[t+1] = offsets[t] + count[t];
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
        for( int d=minDepth ; d<=maxDepth ; d++ )
        {
          int start = spans[d].first , end = spans[d].second , width = end - start;
          for( int i=start + (width*t)/threads ; i<start+(width*(t+1))/threads ; i++ )
            for( int c=0 ; c<Cube::CORNERS ; c++ )
            {
              int& idx = cData[ treeNodes[i] ][c];
              if( idx<0 )
              {
                fprintf( stderr , "[ERROR] Found unindexed corner nodes[%d][%d] = %d (%d,%d)\n" , treeNodes[i]->nodeData.nodeIndex , c , idx , minDepth , maxDepth );
                int _d , _off[3];
                treeNodes[i]->depthAndOffset( _d , _off );
                printf( "(%d [%d %d %d) <-> (%d [%d %d %d])\n" , minDepth , off[0] , off[1] , off[2] , _d , _off[0] , _off[1] , _off[2] );
                printf( "[%d %d]\n" , spans[d].first , spans[d].second );
                exit( 0 );
              }
              else
              {
                int div = idx / ( nodeCount*Cube::CORNERS );
                int rem = idx % ( nodeCount*Cube::CORNERS );
                idx = rem + offsets[div];
              }
            }
        }
    }
    int SortedTreeNodes::getMaxCornerCount( const TreeOctNode* rootNode , int depth , int maxDepth , int threads ) const
    {
      if( threads<=0 ) threads = 1;
      int res = 1<<depth;
      std::vector< std::vector< int > > cornerCount( threads );
      for( int t=0 ; t<threads ; t++ ) cornerCount[t].resize( res*res*res , 0 );

#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        std::vector< int >& _cornerCount = cornerCount[t];
        TreeOctNode::ConstNeighborKey3 neighborKey;
        neighborKey.set( maxDepth );
        int start = nodeCount[depth] , end = nodeCount[maxDepth+1] , range = end-start;
        for( int i=(range*t)/threads ; i<(range*(t+1))/threads ; i++ )
        {
          TreeOctNode* node = treeNodes[start+i];
          int d , off[3];
          node->depthAndOffset( d , off );
          if( d<maxDepth && node->children ) continue;

          const TreeOctNode::ConstNeighbors3& neighbors = neighborKey.getNeighbors( node , depth );
          for( int c=0 ; c<Cube::CORNERS ; c++ )	// Iterate over the cell's corners
          {
            bool cornerOwner = true;
            int x , y , z;
            int ac = Cube::AntipodalCornerIndex( c ); // The index of the node relative to the corner
            Cube::FactorCornerIndex( c , x , y , z );
            for( int cc=0 ; cc<Cube::CORNERS ; cc++ ) // Iterate over the corner's cells
            {
              int xx , yy , zz;
              Cube::FactorCornerIndex( cc , xx , yy , zz );
              xx += x , yy += y , zz += z;
              if( neighbors.neighbors[xx][yy][zz] )
                if( cc<ac || ( d<maxDepth && neighbors.neighbors[xx][yy][zz]->children ) )
                {
                  cornerOwner = false;
                  break;
                }
            }
            if( cornerOwner ) _cornerCount[ ( ( off[0]>>(d-depth) ) * res * res) + ( ( off[1]>>(d-depth) ) * res) + ( off[2]>>(d-depth) ) ]++;
          }
        }
      }
      int maxCount = 0;
      for( int i=0 ; i<res*res*res ; i++ )
      {
        int c = 0;
        for( int t=0 ; t<threads ; t++ ) c += cornerCount[t][i];
        maxCount = std::max< int >( maxCount , c );
      }
      return maxCount;
    }
    SortedTreeNodes::EdgeIndices& SortedTreeNodes::EdgeTableData::operator[] ( const TreeOctNode* node ) { return eTable[ node->nodeData.nodeIndex + offsets[node->d] ]; }
    const SortedTreeNodes::EdgeIndices& SortedTreeNodes::EdgeTableData::operator[] ( const TreeOctNode* node ) const { return eTable[ node->nodeData.nodeIndex + offsets[node->d] ]; }
    SortedTreeNodes::EdgeIndices& SortedTreeNodes::EdgeTableData::edgeIndices( const TreeOctNode* node ) { return eTable[ node->nodeData.nodeIndex + offsets[node->d] ]; }
    const SortedTreeNodes::EdgeIndices& SortedTreeNodes::EdgeTableData::edgeIndices( const TreeOctNode* node ) const { return eTable[ node->nodeData.nodeIndex + offsets[node->d] ]; }
    void SortedTreeNodes::setEdgeTable( EdgeTableData& eData , const TreeOctNode* rootNode , int maxDepth , int threads )
    {
      if( threads<=0 ) threads = 1;
      std::vector< std::pair< int , int > > spans( this->maxDepth , std::pair< int , int >( -1 , -1 ) );

      int minDepth , off[3];
      rootNode->depthAndOffset( minDepth , off );
      eData.offsets.resize( this->maxDepth , -1 );
      int nodeCount = 0;
      {
        int start=rootNode->nodeData.nodeIndex , end=start;
        for( int d=minDepth ; d<=maxDepth ; d++ )
        {
          spans[d] = std::pair< int , int >( start , end+1 );
          eData.offsets[d] = nodeCount - spans[d].first;
          nodeCount += spans[d].second - spans[d].first;
          if( d<maxDepth )
          {
            while( start< end && !treeNodes[start]->children ) start++;
            while( end> start && !treeNodes[end  ]->children ) end--;
            if(    start==end && !treeNodes[start]->children ) break;
            start = treeNodes[start]->children[0].nodeData.nodeIndex;
            end   = treeNodes[end  ]->children[7].nodeData.nodeIndex;
          }
        }
      }
      eData.eTable.resize( nodeCount );
      std::vector< int > count( threads );
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        TreeOctNode::ConstNeighborKey3 neighborKey;
        neighborKey.set( maxDepth );
        int offset = nodeCount * t * Cube::EDGES;
        count[t] = 0;
        for( int d=minDepth ; d<=maxDepth ; d++ )
        {
          int start = spans[d].first , end = spans[d].second , width = end-start;
          for( int i=start + (width*t)/threads ; i<start + (width*(t+1))/threads ; i++ )
          {
            TreeOctNode* node = treeNodes[i];
            const TreeOctNode::ConstNeighbors3& neighbors = neighborKey.getNeighbors( node , minDepth );

            for( int e=0 ; e<Cube::EDGES ; e++ )
            {
              bool edgeOwner = true;
              int o , i , j;
              Cube::FactorEdgeIndex( e , o , i , j );
              int ac = Square::AntipodalCornerIndex( Square::CornerIndex( i , j ) );
              for( int cc=0 ; cc<Square::CORNERS ; cc++ )
              {
                int ii , jj , x , y , z;
                Square::FactorCornerIndex( cc , ii , jj );
                ii += i , jj += j;
                switch( o )
                {
                case 0: y = ii , z = jj , x = 1 ; break;
                case 1: x = ii , z = jj , y = 1 ; break;
                case 2: x = ii , y = jj , z = 1 ; break;
                }
                if( neighbors.neighbors[x][y][z] && cc<ac ) { edgeOwner = false ; break; }
              }
              if( edgeOwner )
              {
                // Set all edge indices
                for( int cc=0 ; cc<Square::CORNERS ; cc++ )
                {
                  int ii , jj , aii , ajj , x , y , z;
                  Square::FactorCornerIndex( cc , ii , jj );
                  Square::FactorCornerIndex( Square::AntipodalCornerIndex( cc ) , aii , ajj );
                  ii += i , jj += j;
                  switch( o )
                  {
                  case 0: y = ii , z = jj , x = 1 ; break;
                  case 1: x = ii , z = jj , y = 1 ; break;
                  case 2: x = ii , y = jj , z = 1 ; break;
                  }
                  if( neighbors.neighbors[x][y][z] )
                    eData[ neighbors.neighbors[x][y][z] ][ Cube::EdgeIndex( o , aii , ajj ) ] = count[t]+offset;
                }
                count[t]++;
              }
            }
          }
        }
      }
      eData.eCount = 0;
      std::vector< int > offsets( threads+1 );
      offsets[0] = 0;
      for( int t=0 ; t<threads ; t++ ) eData.eCount += count[t] , offsets[t+1] = offsets[t] + count[t];
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
        for( int d=minDepth ; d<=maxDepth ; d++ )
        {
          int start = spans[d].first , end = spans[d].second , width = end - start;
          for( int i=start + (width*t)/threads ; i<start+(width*(t+1))/threads ; i++ )
            for( int e=0 ; e<Cube::EDGES ; e++ )
            {
              int& idx = eData[ treeNodes[i] ][e];
              if( idx<0 ) fprintf( stderr , "[ERROR] Found unindexed edge %d (%d,%d)\n" , idx , minDepth , maxDepth ) , exit( 0 );
              else
              {
                int div = idx / ( nodeCount*Cube::EDGES );
                int rem = idx % ( nodeCount*Cube::EDGES );
                idx = rem + offsets[div];
              }
            }
        }
    }
    int SortedTreeNodes::getMaxEdgeCount( const TreeOctNode* rootNode , int depth , int threads ) const
    {
      if( threads<=0 ) threads = 1;
      int res = 1<<depth;
      std::vector< std::vector< int > > edgeCount( threads );
      for( int t=0 ; t<threads ; t++ ) edgeCount[t].resize( res*res*res , 0 );

#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        std::vector< int >& _edgeCount = edgeCount[t];
        TreeOctNode::ConstNeighborKey3 neighborKey;
        neighborKey.set( maxDepth-1 );
        int start = nodeCount[depth] , end = nodeCount[maxDepth] , range = end-start;
        for( int i=(range*t)/threads ; i<(range*(t+1))/threads ; i++ )
        {
          TreeOctNode* node = treeNodes[start+i];
          const TreeOctNode::ConstNeighbors3& neighbors = neighborKey.getNeighbors( node , depth );
          int d , off[3];
          node->depthAndOffset( d , off );

          for( int e=0 ; e<Cube::EDGES ; e++ )
          {
            bool edgeOwner = true;
            int o , i , j;
            Cube::FactorEdgeIndex( e , o , i , j );
            int ac = Square::AntipodalCornerIndex( Square::CornerIndex( i , j ) );
            for( int cc=0 ; cc<Square::CORNERS ; cc++ )
            {
              int ii , jj , x , y , z;
              Square::FactorCornerIndex( cc , ii , jj );
              ii += i , jj += j;
              switch( o )
              {
              case 0: y = ii , z = jj , x = 1 ; break;
              case 1: x = ii , z = jj , y = 1 ; break;
              case 2: x = ii , y = jj , z = 1 ; break;
              }
              if( neighbors.neighbors[x][y][z] && cc<ac ) { edgeOwner = false ; break; }
            }
            if( edgeOwner ) _edgeCount[ ( ( off[0]>>(d-depth) ) * res * res) + ( ( off[1]>>(d-depth) ) * res) + ( off[2]>>(d-depth) ) ]++;
          }
        }
      }
      int maxCount = 0;
      for( int i=0 ; i<res*res*res ; i++ )
      {
        int c = 0;
        for( int t=0 ; t<threads ; t++ ) c += edgeCount[t][i];
        maxCount = std::max< int >( maxCount , c );
      }
      return maxCount;
    }



    //////////////////
    // TreeNodeData //
    //////////////////
    int TreeNodeData::UseIndex=1;
    TreeNodeData::TreeNodeData( void )
    {
      if( UseIndex )
      {
        nodeIndex = -1;
        centerWeightContribution=0;
      }
      else mcIndex=0;
      normalIndex = -1;
      constraint = solution = 0;
      pointIndex = -1;
    }
    TreeNodeData::~TreeNodeData( void ) { }


    ////////////
    // Octree //
    ////////////
    template<int Degree>
    double Octree<Degree>::maxMemoryUsage=0;



    template<int Degree>
    double Octree<Degree>::MemoryUsage(void){
      double mem = 0;//double( MemoryInfo::Usage() ) / (1<<20);
      if(mem>maxMemoryUsage){maxMemoryUsage=mem;}
      return mem;
    }

    template<int Degree>
    Octree<Degree>::Octree(void)
    {
      threads = 1;
      radius = 0;
      width = 0;
      postNormalSmooth = 0;
      _constrainValues = false;
    }

    template<int Degree>
    int Octree<Degree>::NonLinearSplatOrientedPoint( TreeOctNode* node , const Point3D<Real>& position , const Point3D<Real>& normal )
    {
      double x , dxdy , dxdydz , dx[DIMENSION][SPLAT_ORDER+1];
      int off[3];
      TreeOctNode::Neighbors3& neighbors = neighborKey.setNeighbors( node );
      double width;
      Point3D<Real> center;
      Real w;
      node->centerAndWidth( center , w );
      width=w;
      for( int i=0 ; i<3 ; i++ )
      {
#if SPLAT_ORDER==2
        off[i] = 0;
        x = ( center[i] - position[i] - width ) / width;
        dx[i][0] = 1.125+1.500*x+0.500*x*x;
        x = ( center[i] - position[i] ) / width;
        dx[i][1] = 0.750        -      x*x;

        dx[i][2] = 1. - dx[i][1] - dx[i][0];
#elif SPLAT_ORDER==1
        x = ( position[i] - center[i] ) / width;
        if( x<0 )
        {
          off[i] = 0;
          dx[i][0] = -x;
        }
        else
        {
          off[i] = 1;
          dx[i][0] = 1. - x;
        }
        dx[i][1] = 1. - dx[i][0];
#elif SPLAT_ORDER==0
        off[i] = 1;
        dx[i][0] = 1.;
#else
#     error Splat order not supported
#endif // SPLAT_ORDER
      }
      for( int i=off[0] ; i<=off[0]+SPLAT_ORDER ; i++ ) for( int j=off[1] ; j<=off[1]+SPLAT_ORDER ; j++ )
      {
        dxdy = dx[0][i] * dx[1][j];
        for( int k=off[2] ; k<=off[2]+SPLAT_ORDER ; k++ )
          if( neighbors.neighbors[i][j][k] )
          {
            dxdydz = dxdy * dx[2][k];
            TreeOctNode* _node = neighbors.neighbors[i][j][k];
            int idx =_node->nodeData.normalIndex;
            if( idx<0 )
            {
              Point3D<Real> n;
              n[0] = n[1] = n[2] = 0;
              _node->nodeData.nodeIndex = 0;
              idx = _node->nodeData.normalIndex = int(normals->size());
              normals->push_back(n);
            }
            (*normals)[idx] += normal * Real( dxdydz );
          }
      }
      return 0;
    }
    template<int Degree>
    Real Octree<Degree>::NonLinearSplatOrientedPoint( const Point3D<Real>& position , const Point3D<Real>& normal , int splatDepth , Real samplesPerNode ,
                                                      int minDepth , int maxDepth )
    {
      double dx;
      Point3D<Real> n;
      TreeOctNode* temp;
      int cnt=0;
      double width;
      Point3D< Real > myCenter;
      Real myWidth;
      myCenter[0] = myCenter[1] = myCenter[2] = Real(0.5);
      myWidth = Real(1.0);

      temp = &tree;
      while( temp->depth()<splatDepth )
      {
        if( !temp->children )
        {
          fprintf( stderr , "Octree<Degree>::NonLinearSplatOrientedPoint error\n" );
          return -1;
        }
        int cIndex=TreeOctNode::CornerIndex(myCenter,position);
        temp=&temp->children[cIndex];
        myWidth/=2;
        if(cIndex&1) myCenter[0] += myWidth/2;
        else		 myCenter[0] -= myWidth/2;
        if(cIndex&2) myCenter[1] += myWidth/2;
        else		 myCenter[1] -= myWidth/2;
        if(cIndex&4) myCenter[2] += myWidth/2;
        else		 myCenter[2] -= myWidth/2;
      }
      Real alpha,newDepth;
      NonLinearGetSampleDepthAndWeight( temp , position , samplesPerNode , newDepth , alpha );

      if( newDepth<minDepth ) newDepth=Real(minDepth);
      if( newDepth>maxDepth ) newDepth=Real(maxDepth);
      int topDepth=int(ceil(newDepth));

      dx = 1.0-(topDepth-newDepth);
      if( topDepth<=minDepth )
      {
        topDepth=minDepth;
        dx=1;
      }
      else if( topDepth>maxDepth )
      {
        topDepth=maxDepth;
        dx=1;
      }
      while( temp->depth()>topDepth ) temp=temp->parent;
      while( temp->depth()<topDepth )
      {
        if(!temp->children) temp->initChildren();
        int cIndex=TreeOctNode::CornerIndex(myCenter,position);
        temp=&temp->children[cIndex];
        myWidth/=2;
        if(cIndex&1) myCenter[0] += myWidth/2;
        else		 myCenter[0] -= myWidth/2;
        if(cIndex&2) myCenter[1] += myWidth/2;
        else		 myCenter[1] -= myWidth/2;
        if(cIndex&4) myCenter[2] += myWidth/2;
        else		 myCenter[2] -= myWidth/2;
      }
      width = 1.0 / ( 1<<temp->depth() );
      n = normal * alpha / Real( pow( width , 3 ) ) * Real( dx );
      NonLinearSplatOrientedPoint( temp , position , n );
      if( fabs(1.0-dx) > EPSILON )
      {
        dx = Real(1.0-dx);
        temp = temp->parent;
        width = 1.0 / ( 1<<temp->depth() );

        n = normal * alpha / Real( pow( width , 3 ) ) * Real( dx );
        NonLinearSplatOrientedPoint( temp , position , n );
      }
      return alpha;
    }
    template<int Degree>
    void Octree<Degree>::NonLinearGetSampleDepthAndWeight(TreeOctNode* node,const Point3D<Real>& position,Real samplesPerNode,Real& depth,Real& weight){
      TreeOctNode* temp=node;
      weight = Real(1.0)/NonLinearGetSampleWeight(temp,position);
      if( weight>=samplesPerNode ) depth=Real( temp->depth() + log( weight / samplesPerNode ) / log(double(1<<(DIMENSION-1))) );
      else
      {
        Real oldAlpha,newAlpha;
        oldAlpha=newAlpha=weight;
        while( newAlpha<samplesPerNode && temp->parent )
        {
          temp=temp->parent;
          oldAlpha=newAlpha;
          newAlpha=Real(1.0)/NonLinearGetSampleWeight(temp,position);
        }
        depth = Real( temp->depth() + log( newAlpha / samplesPerNode ) / log( newAlpha / oldAlpha ) );
      }
      weight=Real(pow(double(1<<(DIMENSION-1)),-double(depth)));
    }

    template<int Degree>
    Real Octree<Degree>::NonLinearGetSampleWeight( TreeOctNode* node , const Point3D<Real>& position )
    {
      Real weight=0;
      double x,dxdy,dx[DIMENSION][3];
      TreeOctNode::Neighbors3& neighbors=neighborKey.setNeighbors( node );
      double width;
      Point3D<Real> center;
      Real w;
      node->centerAndWidth(center,w);
      width=w;

      for( int i=0 ; i<DIMENSION ; i++ )
      {
        x = ( center[i] - position[i] - width ) / width;
        dx[i][0] = 1.125 + 1.500*x + 0.500*x*x;
        x = ( center[i] - position[i] ) / width;
        dx[i][1] = 0.750           -       x*x;

        dx[i][2] = 1.0 - dx[i][1] - dx[i][0];
      }

      for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ )
      {
        dxdy = dx[0][i] * dx[1][j];
        for( int k=0 ; k<3 ; k++ ) if( neighbors.neighbors[i][j][k] )
          weight += Real( dxdy * dx[2][k] * neighbors.neighbors[i][j][k]->nodeData.centerWeightContribution );
      }
      return Real( 1.0 / weight );
    }

    template<int Degree>
    int Octree<Degree>::NonLinearUpdateWeightContribution( TreeOctNode* node , const Point3D<Real>& position , Real weight )
    {
      TreeOctNode::Neighbors3& neighbors = neighborKey.setNeighbors( node );
      double x,dxdy,dx[DIMENSION][3];
      double width;
      Point3D<Real> center;
      Real w;
      node->centerAndWidth( center , w );
      width=w;
      const double SAMPLE_SCALE = 1. / ( 0.125 * 0.125 + 0.75 * 0.75 + 0.125 * 0.125 );

      for( int i=0 ; i<DIMENSION ; i++ )
      {
        x = ( center[i] - position[i] - width ) / width;
        dx[i][0] = 1.125 + 1.500*x + 0.500*x*x;
        x = ( center[i] - position[i] ) / width;
        dx[i][1] = 0.750           -       x*x;
        dx[i][2] = 1. - dx[i][1] - dx[i][0];
        // Note that we are splatting along a co-dimension one manifold, so uniform point samples
        // do not generate a unit sample weight.
        dx[i][0] *= SAMPLE_SCALE;
      }

      for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ )
      {
        dxdy = dx[0][i] * dx[1][j] * weight;
        for( int k=0 ; k<3 ; k++ ) if( neighbors.neighbors[i][j][k] )
          neighbors.neighbors[i][j][k]->nodeData.centerWeightContribution += Real( dxdy * dx[2][k] );
      }
      return 0;
    }

    template< int Degree > template<typename PointNT> int
    Octree<Degree>::setTree( boost::shared_ptr<const pcl::PointCloud<PointNT> > input_, int maxDepth , int minDepth ,
                             int kernelDepth , Real samplesPerNode , Real scaleFactor , Point3D<Real>& center , Real& scale ,
                             int useConfidence , Real constraintWeight , bool adaptiveWeights )
    {
      _minDepth = std::min< int >( std::max< int >( 0 , minDepth ) , maxDepth );
      _constrainValues = (constraintWeight>0);
      double pointWeightSum = 0;
      Point3D<Real> min , max , position , normal , myCenter;
      Real myWidth;
      int i , cnt=0;
      TreeOctNode* temp;
      int splatDepth=0;

      TreeNodeData::UseIndex = 1;
      neighborKey.set( maxDepth );
      splatDepth = kernelDepth;
      if( splatDepth<0 ) splatDepth = 0;


      tree.setFullDepth( _minDepth );
      // Read through once to get the center and scale
      while (cnt != input_->size ())
      {
        Point3D< Real > p;
        p[0] = input_->points[cnt].x;
        p[1] = input_->points[cnt].y;
        p[2] = input_->points[cnt].z;

        for (i = 0; i < DIMENSION; i++)
        {
          if( !cnt || p[i]<min[i] ) min[i] = p[i];
          if( !cnt || p[i]>max[i] ) max[i] = p[i];
        }
        cnt++;
      }

      scale = std::max< Real >( max[0]-min[0] , std::max< Real >( max[1]-min[1] , max[2]-min[2] ) );
      center = ( max+min ) /2;

      scale *= scaleFactor;
      for( i=0 ; i<DIMENSION ; i++ ) center[i] -= scale/2;
      if( splatDepth>0 )
      {
        cnt = 0;
        while (cnt != input_->size ())
        {
          position[0] = input_->points[cnt].x;
          position[1] = input_->points[cnt].y;
          position[2] = input_->points[cnt].z;
          normal[0] = input_->points[cnt].normal_x;
          normal[1] = input_->points[cnt].normal_y;
          normal[2] = input_->points[cnt].normal_z;

          for( i=0 ; i<DIMENSION ; i++ ) position[i] = ( position[i]-center[i] ) / scale;
          myCenter[0] = myCenter[1] = myCenter[2] = Real(0.5);
          myWidth = Real(1.0);
          for( i=0 ; i<DIMENSION ; i++ ) if( position[i]<myCenter[i]-myWidth/2 || position[i]>myCenter[i]+myWidth/2 ) break;
          if( i!=DIMENSION ) continue;
          Real weight=Real( 1. );
          if( useConfidence ) weight = Real( Length(normal) );
          temp = &tree;
          int d=0;
          while( d<splatDepth )
          {
            NonLinearUpdateWeightContribution( temp , position , weight );
            if( !temp->children ) temp->initChildren();
            int cIndex=TreeOctNode::CornerIndex(myCenter,position);
            temp=&temp->children[cIndex];
            myWidth/=2;
            if(cIndex&1) myCenter[0] += myWidth/2;
            else		 myCenter[0] -= myWidth/2;
            if(cIndex&2) myCenter[1] += myWidth/2;
            else		 myCenter[1] -= myWidth/2;
            if(cIndex&4) myCenter[2] += myWidth/2;
            else		 myCenter[2] -= myWidth/2;
            d++;
          }
          NonLinearUpdateWeightContribution( temp , position , weight );
          cnt++;
        }
      }

      normals = new std::vector< Point3D<Real> >();
      cnt=0;
      while (cnt != input_->size ())
      {
        position[0] = input_->points[cnt].x;
        position[1] = input_->points[cnt].y;
        position[2] = input_->points[cnt].z;
        normal[0] = input_->points[cnt].normal_x;
        normal[1] = input_->points[cnt].normal_y;
        normal[2] = input_->points[cnt].normal_z;
        cnt ++;
        for( i=0 ; i<DIMENSION ; i++ ) position[i] = ( position[i]-center[i] ) / scale;
        myCenter[0] = myCenter[1] = myCenter[2] = Real(0.5);
        myWidth = Real(1.0);
        for( i=0 ; i<DIMENSION ; i++ ) if(position[i]<myCenter[i]-myWidth/2 || position[i]>myCenter[i]+myWidth/2) break;
        if( i!=DIMENSION ) continue;
        Real l = Real( Length( normal ) );
        if( l!=l || l<=EPSILON ) continue;
        if( !useConfidence ) normal /= l;

        l = Real(1.);
        Real pointWeight = Real(1.f);
        if( samplesPerNode>0 && splatDepth )
        {
          pointWeight = NonLinearSplatOrientedPoint( position , normal , splatDepth , samplesPerNode , _minDepth , maxDepth );
        }
        else
        {
          Real alpha=1;
          temp = &tree;
          int d=0;
          if( splatDepth )
          {
            while( d<splatDepth )
            {
              int cIndex=TreeOctNode::CornerIndex(myCenter,position);
              temp=&temp->children[cIndex];
              myWidth/=2;
              if(cIndex&1) myCenter[0]+=myWidth/2;
              else		 myCenter[0]-=myWidth/2;
              if(cIndex&2) myCenter[1]+=myWidth/2;
              else		 myCenter[1]-=myWidth/2;
              if(cIndex&4) myCenter[2]+=myWidth/2;
              else		 myCenter[2]-=myWidth/2;
              d++;
            }
            alpha = NonLinearGetSampleWeight( temp , position );
          }
          for( i=0 ; i<DIMENSION ; i++ ) normal[i]*=alpha;
          while( d<maxDepth )
          {
            if(!temp->children){temp->initChildren();}
            int cIndex=TreeOctNode::CornerIndex(myCenter,position);
            temp=&temp->children[cIndex];
            myWidth/=2;
            if(cIndex&1) myCenter[0]+=myWidth/2;
            else		 myCenter[0]-=myWidth/2;
            if(cIndex&2) myCenter[1]+=myWidth/2;
            else		 myCenter[1]-=myWidth/2;
            if(cIndex&4) myCenter[2]+=myWidth/2;
            else		 myCenter[2]-=myWidth/2;
            d++;
          }
          NonLinearSplatOrientedPoint( temp , position , normal );
          pointWeight = alpha;
        }
        pointWeight = 1;
        pointWeightSum += pointWeight;
        if( _constrainValues )
        {
          int d = 0;
          TreeOctNode* temp = &tree;
          myCenter[0] = myCenter[1] = myCenter[2] = Real(0.5);
          myWidth = Real(1.0);
          while( 1 )
          {
            int idx = temp->nodeData.pointIndex;
            if( idx==-1 )
            {
              Point3D< Real > p;
              p[0] = p[1] = p[2] = 0;
              idx = int( _points.size() );
              _points.push_back( PointData( position*pointWeight , pointWeight ) );
              temp->nodeData.pointIndex = idx;
            }
            else
            {
              _points[idx].weight += pointWeight;
              _points[idx].position += position * pointWeight;
            }

            int cIndex = TreeOctNode::CornerIndex( myCenter , position );
            if( !temp->children ) break;
            temp = &temp->children[cIndex];
            myWidth /= 2;
            if( cIndex&1 ) myCenter[0] += myWidth/2;
            else		   myCenter[0] -= myWidth/2;
            if( cIndex&2 ) myCenter[1] += myWidth/2;
            else		   myCenter[1] -= myWidth/2;
            if( cIndex&4 ) myCenter[2] += myWidth/2;
            else		   myCenter[2] -= myWidth/2;
            d++;
          }
        }
      }


      if( _constrainValues )
        for( TreeOctNode* n=tree.nextNode() ; n ; n=tree.nextNode(n) )
          if( n->nodeData.pointIndex!=-1 )
          {
            int idx = n->nodeData.pointIndex;
            _points[idx].position /= _points[idx].weight;
            if( adaptiveWeights ) _points[idx].weight *= (1<<n->d);
            else                  _points[idx].weight *= (1<<maxDepth);
            _points[idx].weight *= Real( constraintWeight / pointWeightSum );
          }
#if FORCE_NEUMANN_FIELD
      for( TreeOctNode* node=tree.nextNode() ; node ; node=tree.nextNode( node ) )
      {
        int d , off[3] , res;
        node->depthAndOffset( d , off );
        res = 1<<d;
        if( node->nodeData.normalIndex<0 ) continue;
        Point3D< Real >& normal = (*normals)[node->nodeData.normalIndex];
        for( int d=0 ; d<3 ; d++ ) if( off[d]==0 || off[d]==res-1 ) normal[d] = 0;
      }
#endif // FORCE_NEUMANN_FIELD
      _sNodes.set( tree );


      return cnt;
    }


    template<int Degree>
    void Octree<Degree>::setBSplineData( int maxDepth , Real normalSmooth , bool reflectBoundary )
    {
      radius = 0.5 + 0.5 * Degree;
      width=int(double(radius+0.5-EPSILON)*2);
      if( normalSmooth>0 ) postNormalSmooth = normalSmooth;
      fData.set( maxDepth , true , reflectBoundary );
    }

    template<int Degree>
    void Octree<Degree>::finalize( void )
    {
      int maxDepth = tree.maxDepth( );
      TreeOctNode::NeighborKey5 nKey;
      nKey.set( maxDepth );
      for( int d=maxDepth ; d>0 ; d-- )
        for( TreeOctNode* node=tree.nextNode() ; node ; node=tree.nextNode( node ) )
          if( node->d==d )
          {
            int xStart=0 , xEnd=5 , yStart=0 , yEnd=5 , zStart=0 , zEnd=5;
            int c = int( node - node->parent->children );
            int x , y , z;
            Cube::FactorCornerIndex( c , x , y , z );
            if( x ) xStart = 1;
            else    xEnd   = 4;
            if( y ) yStart = 1;
            else    yEnd   = 4;
            if( z ) zStart = 1;
            else    zEnd   = 4;
            nKey.setNeighbors( node->parent , xStart , xEnd , yStart , yEnd , zStart , zEnd );
          }
      _sNodes.set( tree );
      MemoryUsage();
    }
    template< int Degree >
    Real Octree< Degree >::GetValue( const PointInfo points[3][3][3] , const bool hasPoints[3][3] , const int d[3] ) const
    {
      double v = 0.;
      const int min[] = { std::max<int>( 0 , d[0]+0 ) , std::max<int>( 0 , d[1]+0 ) , std::max<int>( 0 , d[2]+0 ) };
      const int max[] = { std::min<int>( 2 , d[0]+2 ) , std::min<int>( 2 , d[1]+2 ) , std::min<int>( 2 , d[2]+2 ) };
      for( int i=min[0] ; i<=max[0] ; i++ ) for( int j=min[1] ; j<=max[1] ; j++ )
      {
        if( !hasPoints[i][j] ) continue;
        const PointInfo* pInfo = points[i][j];
        int ii = -d[0]+i;
        int jj = -d[1]+j;
        for( int k=min[2] ; k<=max[2] ; k++ )
          if( pInfo[k].weightedValue )
            v += pInfo[k].splineValues[0][ii] * pInfo[k].splineValues[1][jj] * pInfo[k].splineValues[2][-d[2]+k];
      }
      return Real( v );
    }
    template<int Degree>
    Real Octree<Degree>::GetLaplacian( const int idx[DIMENSION] ) const
    {
      return Real( fData.vvDotTable[idx[0]] * fData.vvDotTable[idx[1]] * fData.vvDotTable[idx[2]] * (fData.ddDotTable[idx[0]]+fData.ddDotTable[idx[1]]+fData.ddDotTable[idx[2]] ) );
    }
    template< int Degree >
    Real Octree< Degree >::GetLaplacian( const TreeOctNode* node1 , const TreeOctNode* node2 ) const
    {
      int symIndex[] =
      {
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[0] , node2->off[0] ) ,
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[1] , node2->off[1] ) ,
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[2] , node2->off[2] )
      };
      return GetLaplacian( symIndex );
    }
    template< int Degree >
    Real Octree< Degree >::GetDivergence( const TreeOctNode* node1 , const TreeOctNode* node2 , const Point3D< Real >& normal1 ) const
    {
      int symIndex[] =
      {
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[0] , node2->off[0] ) ,
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[1] , node2->off[1] ) ,
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[2] , node2->off[2] ) ,
      };
      int aSymIndex[] =
      {
  #if GRADIENT_DOMAIN_SOLUTION
        // Take the dot-product of the vector-field with the gradient of the basis function
        fData.Index( node2->off[0] , node1->off[0] ) ,
        fData.Index( node2->off[1] , node1->off[1] ) ,
        fData.Index( node2->off[2] , node1->off[2] )
  #else // !GRADIENT_DOMAIN_SOLUTION
        // Take the dot-product of the divergence of the vector-field with the basis function
        fData.Index( node1->off[0] , node2->off[0] ) ,
        fData.Index( node1->off[1] , node2->off[1] ) ,
        fData.Index( node1->off[2] , node2->off[2] )
  #endif // GRADIENT_DOMAIN_SOLUTION
      };
      double dot = fData.vvDotTable[symIndex[0]] * fData.vvDotTable[symIndex[1]] * fData.vvDotTable[symIndex[2]];
#if GRADIENT_DOMAIN_SOLUTION
      return  Real( dot * ( fData.dvDotTable[aSymIndex[0]]*normal1[0] + fData.dvDotTable[aSymIndex[1]]*normal1[1] + fData.dvDotTable[aSymIndex[2]]*normal1[2] ) );
#else // !GRADIENT_DOMAIN_SOLUTION
      return -Real( dot * ( fData.dvDotTable[aSymIndex[0]]*normal1[0] + fData.dvDotTable[aSymIndex[1]]*normal1[1] + fData.dvDotTable[aSymIndex[2]]*normal1[2] ) );
#endif // GRADIENT_DOMAIN_SOLUTION
    }
    template< int Degree >
    Real Octree< Degree >::GetDivergenceMinusLaplacian( const TreeOctNode* node1 , const TreeOctNode* node2 , Real value1 , const Point3D<Real>& normal1 ) const
    {
      int symIndex[] =
      {
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[0] , node2->off[0] ) ,
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[1] , node2->off[1] ) ,
        BSplineData< Degree , Real >::SymmetricIndex( node1->off[2] , node2->off[2] )
      };
      int aSymIndex[] =
      {
  #if GRADIENT_DOMAIN_SOLUTION
        // Take the dot-product of the vector-field with the gradient of the basis function
        fData.Index( node2->off[0] , node1->off[0] ) ,
        fData.Index( node2->off[1] , node1->off[1] ) ,
        fData.Index( node2->off[2] , node1->off[2] )
  #else // !GRADIENT_DOMAIN_SOLUTION
        // Take the dot-product of the divergence of the vector-field with the basis function
        fData.Index( node1->off[0] , node2->off[0] ) ,
        fData.Index( node1->off[1] , node2->off[1] ) ,
        fData.Index( node1->off[2] , node2->off[2] )
  #endif // GRADIENT_DOMAIN_SOLUTION
      };
      double dot = fData.vvDotTable[symIndex[0]] * fData.vvDotTable[symIndex[1]] * fData.vvDotTable[symIndex[2]];
      return Real( dot *
                   (
               #if GRADIENT_DOMAIN_SOLUTION
                     - ( fData.ddDotTable[ symIndex[0]]            + fData.ddDotTable[ symIndex[1]]            + fData.ddDotTable[ symIndex[2]] ) * value1
                     + ( fData.dvDotTable[aSymIndex[0]]*normal1[0] + fData.dvDotTable[aSymIndex[1]]*normal1[1] + fData.dvDotTable[aSymIndex[2]]*normal1[2] )
               #else // !GRADIENT_DOMAIN_SOLUTION
                     - ( fData.ddDotTable[ symIndex[0]]            + fData.ddDotTable[ symIndex[1]]            + fData.ddDotTable[ symIndex[2]] ) * value1
                     - ( fData.dvDotTable[aSymIndex[0]]*normal1[0] + fData.dvDotTable[aSymIndex[1]]*normal1[1] + fData.dvDotTable[aSymIndex[2]]*normal1[2] )
               #endif // GRADIENT_DOMAIN_SOLUTION
                     )
                   );
    }
    template< int Degree >
    void Octree< Degree >::SetMatrixRowBounds( const TreeOctNode* node , int rDepth , const int rOff[3] , int& xStart , int& xEnd , int& yStart , int& yEnd , int& zStart , int& zEnd ) const
    {
      xStart = 0 , yStart = 0 , zStart = 0 , xEnd = 5 , yEnd = 5 , zEnd = 5;
      int depth , off[3];
      node->depthAndOffset( depth , off );
      int width = 1 << ( depth-rDepth );
      off[0] -= rOff[0] << ( depth-rDepth ) , off[1] -= rOff[1] << ( depth-rDepth ) , off[2] -= rOff[2] << ( depth-rDepth );
      if( off[0]<0 ) xStart = -off[0];
      if( off[1]<0 ) yStart = -off[1];
      if( off[2]<0 ) zStart = -off[2];
      if( off[0]>=width ) xEnd = 4 - ( off[0]-width );
      if( off[1]>=width ) yEnd = 4 - ( off[1]-width );
      if( off[2]>=width ) zEnd = 4 - ( off[2]-width );
    }
    template< int Degree >
    int Octree< Degree >::GetMatrixRowSize( const OctNode< TreeNodeData , Real >::Neighbors5& neighbors5 ) const { return GetMatrixRowSize( neighbors5 , 0 , 5 , 0 , 5 , 0 , 5 ); }
    template< int Degree >
    int Octree< Degree >::GetMatrixRowSize( const OctNode< TreeNodeData , Real >::Neighbors5& neighbors5 , int xStart , int xEnd , int yStart , int yEnd , int zStart , int zEnd ) const
    {
      int count = 0;
      for( int x=xStart ; x<=2 ; x++ )
        for( int y=yStart ; y<yEnd ; y++ )
          if( x==2 && y>2 ) continue;
          else for( int z=zStart ; z<zEnd ; z++ )
            if( x==2 && y==2 && z>2 ) continue;
            else if( neighbors5.neighbors[x][y][z] && neighbors5.neighbors[x][y][z]->nodeData.nodeIndex>=0 )
              count++;
      return count;
    }
    template< int Degree >
    int Octree< Degree >::SetMatrixRow( const OctNode< TreeNodeData , Real >::Neighbors5& neighbors5 , MatrixEntry< float >* row , int offset , const double stencil[5][5][5] ) const
    {
      return SetMatrixRow( neighbors5 , row , offset , stencil , 0 , 5 , 0 , 5 , 0 , 5 );
    }
    template< int Degree >
    int Octree< Degree >::SetMatrixRow( const OctNode< TreeNodeData , Real >::Neighbors5& neighbors5 , MatrixEntry< float >* row , int offset , const double stencil[5][5][5] , int xStart , int xEnd , int yStart , int yEnd , int zStart , int zEnd ) const
    {
      bool hasPoints[3][3];
      Real diagonal = 0;
      PointInfo samples[3][3][3];

      int count = 0;
      const TreeOctNode* node = neighbors5.neighbors[2][2][2];
      int index[] = { int( node->off[0] ) , int( node->off[1] ), int( node->off[2] ) };

      if( _constrainValues )
      {
        int d , idx[3];
        node->depthAndOffset( d , idx );
        idx[0] = BinaryNode< double >::CenterIndex( d , idx[0] );
        idx[1] = BinaryNode< double >::CenterIndex( d , idx[1] );
        idx[2] = BinaryNode< double >::CenterIndex( d , idx[2] );
        for( int j=0 ; j<3 ; j++ ) for( int k=0 ; k<3 ; k++ )
        {
          hasPoints[j][k] = false;
          for( int l=0 ; l<3 ; l++ )
          {
            const TreeOctNode* _node = neighbors5.neighbors[j+1][k+1][l+1];
            if( _node && _node->nodeData.pointIndex!=-1 )
            {
              const PointData& pData = _points[ _node->nodeData.pointIndex ];
              PointInfo& pointInfo = samples[j][k][l];
              Real weight = pData.weight;
              Point3D< Real > p = pData.position;
              for( int s=0 ; s<3 ; s++ )
              {
                pointInfo.splineValues[0][s] = float( fData.baseBSplines[ idx[0]+j-s][s]( p[0] ) );
                pointInfo.splineValues[1][s] = float( fData.baseBSplines[ idx[1]+k-s][s]( p[1] ) );
                pointInfo.splineValues[2][s] = float( fData.baseBSplines[ idx[2]+l-s][s]( p[2] ) );
              }
              float value = pointInfo.splineValues[0][j] * pointInfo.splineValues[1][k] * pointInfo.splineValues[2][l];
              diagonal += value * value * weight;
              pointInfo.weightedValue  = value * weight;
              for( int s=0 ; s<3 ; s++ ) pointInfo.splineValues[0][s] *= pointInfo.weightedValue;
              hasPoints[j][k] = true;
            }
            else samples[j][k][l].weightedValue = 0;
          }
        }
      }

      bool isInterior;
      int d , off[3];
      neighbors5.neighbors[2][2][2]->depthAndOffset( d , off );
      int mn = 2 , mx = (1<<d)-2;
      isInterior = ( off[0]>=mn && off[0]<mx && off[1]>=mn && off[1]<mx && off[2]>=mn && off[2]<mx );
      for( int x=xStart ; x<=2 ; x++ )
        for( int y=yStart ; y<yEnd ; y++ )
          if( x==2 && y>2 ) continue;
          else for( int z=zStart ; z<zEnd ; z++ )
            if( x==2 && y==2 && z>2 ) continue;
            else if( neighbors5.neighbors[x][y][z] && neighbors5.neighbors[x][y][z]->nodeData.nodeIndex>=0 )
            {
              const TreeOctNode* _node = neighbors5.neighbors[x][y][z];
              int _index[] = { int( _node->off[0] ) , int( _node->off[1] ), int( _node->off[2] ) };
              Real temp;
              if( isInterior ) temp = Real( stencil[x][y][z] );
              else             temp = GetLaplacian( node , _node );
              if( _constrainValues )
              {
                int _d[] = { _index[0]-index[0] , _index[1]-index[1] , _index[2]-index[2] };
                if( x==2 && y==2 && z==2 ) temp += diagonal;
                else                       temp += GetValue( samples , hasPoints , _d );
              }
              if( x==2 && y==2 && z==2 ) temp /= 2;
              if( fabs(temp)>MATRIX_ENTRY_EPSILON )
              {
                row[count].N = _node->nodeData.nodeIndex-offset;
                row[count].Value = temp;
                count++;
              }
            }
      return count;
    }
    template< int Degree >
    void Octree< Degree >::SetDivergenceStencil( int depth , Point3D< double > *stencil , bool scatter ) const
    {
      int offset[] = { 2 , 2 , 2 };
      short d , off[3];
      TreeOctNode::Index( depth , offset , d , off );
      int index1[3] , index2[3];
      if( scatter ) index2[0] = int( off[0] ) , index2[1] = int( off[1] ) , index2[2] = int( off[2] );
      else          index1[0] = int( off[0] ) , index1[1] = int( off[1] ) , index1[2] = int( off[2] );
      for( int x=0 ; x<5 ; x++ ) for( int y=0 ; y<5 ; y++ ) for( int z=0 ; z<5 ; z++ )
      {
        int _offset[] = { x , y , z };
        TreeOctNode::Index( depth , _offset , d , off );
        if( scatter ) index1[0] = int( off[0] ) , index1[1] = int( off[1] ) , index1[2] = int( off[2] );
        else          index2[0] = int( off[0] ) , index2[1] = int( off[1] ) , index2[2] = int( off[2] );
        int symIndex[] =
        {
          BSplineData< Degree , Real >::SymmetricIndex( index1[0] , index2[0] ) ,
          BSplineData< Degree , Real >::SymmetricIndex( index1[1] , index2[1] ) ,
          BSplineData< Degree , Real >::SymmetricIndex( index1[2] , index2[2] ) ,
        };
        int aSymIndex[] =
        {
  #if GRADIENT_DOMAIN_SOLUTION
          // Take the dot-product of the vector-field with the gradient of the basis function
          fData.Index( index1[0] , index2[0] ) ,
          fData.Index( index1[1] , index2[1] ) ,
          fData.Index( index1[2] , index2[2] )
  #else // !GRADIENT_DOMAIN_SOLUTION
          // Take the dot-product of the divergence of the vector-field with the basis function
          fData.Index( index2[0] , index1[0] ) ,
          fData.Index( index2[1] , index1[1] ) ,
          fData.Index( index2[2] , index1[2] )
  #endif // GRADIENT_DOMAIN_SOLUTION
        };

        double dot = fData.vvDotTable[symIndex[0]] * fData.vvDotTable[symIndex[1]] * fData.vvDotTable[symIndex[2]];
#if GRADIENT_DOMAIN_SOLUTION
        Point3D<double> temp;
        temp[0] = fData.dvDotTable[aSymIndex[0]] * dot;
        temp[1] = fData.dvDotTable[aSymIndex[1]] * dot;
        temp[2] = fData.dvDotTable[aSymIndex[2]] * dot;
        stencil[25*x + 5*y + z] = temp;
        //        stencil[x][y][z][0] = fData.dvDotTable[aSymIndex[0]] * dot;
        //        stencil[x][y][z][1] = fData.dvDotTable[aSymIndex[1]] * dot;
        //        stencil[x][y][z][2] = fData.dvDotTable[aSymIndex[2]] * dot;
#else // !GRADIENT_DOMAIN_SOLUTION
        Point3D<double> temp;
        temp[0] = -fData.dvDotTable[aSymIndex[0]] * dot;
        temp[1] = -fData.dvDotTable[aSymIndex[1]] * dot;
        temp[2] = -fData.dvDotTable[aSymIndex[2]] * dot;
        stencil[25*x + 5*y + z] = temp;
        //        stencil[x][y][z][0] = -fData.dvDotTable[aSymIndex[0]] * dot;
        //        stencil[x][y][z][1] = -fData.dvDotTable[aSymIndex[1]] * dot;
        //        stencil[x][y][z][2] = -fData.dvDotTable[aSymIndex[2]] * dot;
#endif // GRADIENT_DOMAIN_SOLUTION
      }
    }
    template< int Degree >
    void Octree< Degree >::SetLaplacianStencil( int depth , double stencil[5][5][5] ) const
    {
      int offset[] = { 2 , 2 , 2 };
      short d , off[3];
      TreeOctNode::Index( depth , offset , d , off );
      int index[] = { int( off[0] ) , int( off[1] ) , int( off[2] ) };
      for( int x=0 ; x<5 ; x++ ) for( int y=0 ; y<5 ; y++ ) for( int z=0 ; z<5 ; z++ )
      {
        int _offset[] = { x , y , z };
        short _d , _off[3];
        TreeOctNode::Index( depth , _offset , _d , _off );
        int _index[] = { int( _off[0] ) , int( _off[1] ) , int( _off[2] ) };
        int symIndex[3];
        symIndex[0] = BSplineData< Degree , Real >::SymmetricIndex( index[0] , _index[0] );
        symIndex[1] = BSplineData< Degree , Real >::SymmetricIndex( index[1] , _index[1] );
        symIndex[2] = BSplineData< Degree , Real >::SymmetricIndex( index[2] , _index[2] );
        stencil[x][y][z] = GetLaplacian( symIndex );
      }
    }
    template< int Degree >
    void Octree< Degree >::SetLaplacianStencils( int depth , Stencil< double , 5 > stencils[2][2][2] ) const
    {
      if( depth<=1 ) return;
      for( int i=0 ; i<2 ; i++ ) for( int j=0 ; j<2 ; j++ ) for( int k=0 ; k<2 ; k++ )
      {
        short d , off[3];
        int offset[] = { 4+i , 4+j , 4+k };
        TreeOctNode::Index( depth , offset , d , off );
        int index[] = { int( off[0] ) , int( off[1] ) , int( off[2] ) };
        for( int x=0 ; x<5 ; x++ ) for( int y=0 ; y<5 ; y++ ) for( int z=0 ; z<5 ; z++ )
        {
          int _offset[] = { x , y , z };
          short _d , _off[3];
          TreeOctNode::Index( depth-1 , _offset , _d , _off );
          int _index[] = { int( _off[0] ) , int( _off[1] ) , int( _off[2] ) };
          int symIndex[3];
          symIndex[0] = BSplineData< Degree , Real >::SymmetricIndex( index[0] , _index[0] );
          symIndex[1] = BSplineData< Degree , Real >::SymmetricIndex( index[1] , _index[1] );
          symIndex[2] = BSplineData< Degree , Real >::SymmetricIndex( index[2] , _index[2] );
          stencils[i][j][k].values[x][y][z] = GetLaplacian( symIndex );
        }
      }
    }
    template< int Degree >
    void Octree< Degree >::SetDivergenceStencils( int depth , Stencil< Point3D< double > ,  5 > stencils[2][2][2] , bool scatter ) const
    {
      if( depth<=1 ) return;
      int index1[3] , index2[3];
      for( int i=0 ; i<2 ; i++ ) for( int j=0 ; j<2 ; j++ ) for( int k=0 ; k<2 ; k++ )
      {
        short d , off[3];
        int offset[] = { 4+i , 4+j , 4+k };
        TreeOctNode::Index( depth , offset , d , off );
        if( scatter ) index2[0] = int( off[0] ) , index2[1] = int( off[1] ) , index2[2] = int( off[2] );
        else          index1[0] = int( off[0] ) , index1[1] = int( off[1] ) , index1[2] = int( off[2] );
        for( int x=0 ; x<5 ; x++ ) for( int y=0 ; y<5 ; y++ ) for( int z=0 ; z<5 ; z++ )
        {
          int _offset[] = { x , y , z };
          TreeOctNode::Index( depth-1 , _offset , d , off );
          if( scatter ) index1[0] = int( off[0] ) , index1[1] = int( off[1] ) , index1[2] = int( off[2] );
          else          index2[0] = int( off[0] ) , index2[1] = int( off[1] ) , index2[2] = int( off[2] );

          int symIndex[] =
          {
            BSplineData< Degree , Real >::SymmetricIndex( index1[0] , index2[0] ) ,
            BSplineData< Degree , Real >::SymmetricIndex( index1[1] , index2[1] ) ,
            BSplineData< Degree , Real >::SymmetricIndex( index1[2] , index2[2] ) ,
          };
          int aSymIndex[] =
          {
  #if GRADIENT_DOMAIN_SOLUTION
            // Take the dot-product of the vector-field with the gradient of the basis function
            fData.Index( index1[0] , index2[0] ) ,
            fData.Index( index1[1] , index2[1] ) ,
            fData.Index( index1[2] , index2[2] )
  #else // !GRADIENT_DOMAIN_SOLUTION
            // Take the dot-product of the divergence of the vector-field with the basis function
            fData.Index( index2[0] , index1[0] ) ,
            fData.Index( index2[1] , index1[1] ) ,
            fData.Index( index2[2] , index1[2] )
  #endif // GRADIENT_DOMAIN_SOLUTION
          };
          double dot = fData.vvDotTable[symIndex[0]] * fData.vvDotTable[symIndex[1]] * fData.vvDotTable[symIndex[2]];
#if GRADIENT_DOMAIN_SOLUTION
          stencils[i][j][k].values[x][y][z][0] = fData.dvDotTable[aSymIndex[0]] * dot;
          stencils[i][j][k].values[x][y][z][1] = fData.dvDotTable[aSymIndex[1]] * dot;
          stencils[i][j][k].values[x][y][z][2] = fData.dvDotTable[aSymIndex[2]] * dot;
#else // !GRADIENT_DOMAIN_SOLUTION
          stencils[i][j][k].values[x][y][z][0] = -fData.dvDotTable[aSymIndex[0]] * dot;
          stencils[i][j][k].values[x][y][z][1] = -fData.dvDotTable[aSymIndex[1]] * dot;
          stencils[i][j][k].values[x][y][z][2] = -fData.dvDotTable[aSymIndex[2]] * dot;
#endif // GRADIENT_DOMAIN_SOLUTION
        }
      }
    }
    template< int Degree >
    void Octree< Degree >::SetEvaluationStencils( int depth , Stencil< double ,  3 > stencil1[8] , Stencil< double , 3 > stencil2[8][8] ) const
    {
      if( depth>2 )
      {
        int idx[3];
        int off[] = { 2 , 2 , 2 };
        for( int c=0 ; c<8 ; c++ )
        {
          VertexData::CornerIndex( depth , off , c , fData.depth , idx );
          idx[0] *= fData.functionCount , idx[1] *= fData.functionCount , idx[2] *= fData.functionCount;
          for( int x=0 ; x<3 ; x++ ) for( int y=0 ; y<3 ; y++ ) for( int z=0 ; z<3 ; z++ )
          {
            short _d , _off[3];
            int _offset[] = { x+1 , y+1 , z+1 };
            TreeOctNode::Index( depth , _offset , _d , _off );
            stencil1[c].values[x][y][z] = fData.valueTables[ idx[0]+int(_off[0]) ] * fData.valueTables[ idx[1]+int(_off[1]) ] * fData.valueTables[ idx[2]+int(_off[2]) ];
          }
        }
      }
      if( depth>3 )
        for( int _c=0 ; _c<8 ; _c++ )
        {
          int idx[3];
          int _cx , _cy , _cz;
          Cube::FactorCornerIndex( _c , _cx , _cy , _cz );
          int off[] = { 4+_cx , 4+_cy , 4+_cz };
          for( int c=0 ; c<8 ; c++ )
          {
            VertexData::CornerIndex( depth , off , c , fData.depth , idx );
            idx[0] *= fData.functionCount , idx[1] *= fData.functionCount , idx[2] *= fData.functionCount;
            for( int x=0 ; x<3 ; x++ ) for( int y=0 ; y<3 ; y++ ) for( int z=0 ; z<3 ; z++ )
            {
              short _d , _off[3];
              int _offset[] = { x+1 , y+1 , z+1 };
              TreeOctNode::Index( depth-1 , _offset , _d , _off );
              stencil2[_c][c].values[x][y][z] = fData.valueTables[ idx[0]+int(_off[0]) ] * fData.valueTables[ idx[1]+int(_off[1]) ] * fData.valueTables[ idx[2]+int(_off[2]) ];
            }
          }
        }
    }
    template< int Degree >
    void Octree< Degree >::UpdateCoarserSupportBounds( const TreeOctNode* node , int& startX , int& endX , int& startY , int& endY , int& startZ , int& endZ )
    {
      if( node->parent )
      {
        int x , y , z , c = int( node - node->parent->children );
        Cube::FactorCornerIndex( c , x , y , z );
        if( x==0 ) endX = 4;
        else     startX = 1;
        if( y==0 ) endY = 4;
        else     startY = 1;
        if( z==0 ) endZ = 4;
        else     startZ = 1;
      }
    }

    template< int Degree >
    void Octree< Degree >::UpdateConstraintsFromCoarser( const OctNode< TreeNodeData , Real >::NeighborKey5& neighborKey5 , TreeOctNode* node , Real* metSolution , const Stencil< double , 5 >& lapStencil ) const
    {
      bool isInterior;
      {
        int d , off[3];
        node->depthAndOffset( d , off );
        int mn = 4 , mx = (1<<d)-4;
        isInterior = ( off[0]>=mn && off[0]<mx && off[1]>=mn && off[1]<mx && off[2]>=mn && off[2]<mx );
      }
      Real constraint = Real( 0 );
      int depth = node->depth();
      if( depth<=_minDepth ) return;
      int i = node->nodeData.nodeIndex;
      // Offset the constraints using the solution from lower resolutions.
      int startX = 0 , endX = 5 , startY = 0 , endY = 5 , startZ = 0 , endZ = 5;
      UpdateCoarserSupportBounds( node , startX , endX , startY  , endY , startZ , endZ );

      const TreeOctNode::Neighbors5& neighbors5 = neighborKey5.neighbors[depth-1];
      for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
        if( neighbors5.neighbors[x][y][z] && neighbors5.neighbors[x][y][z]->nodeData.nodeIndex>=0 )
        {
          const TreeOctNode* _node = neighbors5.neighbors[x][y][z];
          Real _solution = metSolution[ _node->nodeData.nodeIndex ];
          {
            if( isInterior ) node->nodeData.constraint -= Real( lapStencil.values[x][y][z] * _solution );
            else             node->nodeData.constraint -= GetLaplacian( _node , node ) * _solution;
          }
        }
      if( _constrainValues )
      {
        int d , idx[3];
        node->depthAndOffset( d, idx );
        idx[0] = BinaryNode< double >::CenterIndex( d , idx[0] );
        idx[1] = BinaryNode< double >::CenterIndex( d , idx[1] );
        idx[2] = BinaryNode< double >::CenterIndex( d , idx[2] );
        const TreeOctNode::Neighbors5& neighbors5 = neighborKey5.neighbors[depth];
        for( int x=1 ; x<4 ; x++ ) for( int y=1 ; y<4 ; y++ ) for( int z=1 ; z<4 ; z++ )
          if( neighbors5.neighbors[x][y][z] && neighbors5.neighbors[x][y][z]->nodeData.pointIndex!=-1 )
          {
            const PointData& pData = _points[ neighbors5.neighbors[x][y][z]->nodeData.pointIndex ];
            Real pointValue = pData.value;
            Point3D< Real > p = pData.position;
            node->nodeData.constraint -=
                Real(
                  fData.baseBSplines[idx[0]][x-1]( p[0] ) *
                  fData.baseBSplines[idx[1]][y-1]( p[1] ) *
                  fData.baseBSplines[idx[2]][z-1]( p[2] ) *
                  pointValue
                  );
          }
      }
    }
    struct UpSampleData
    {
        int start;
        double v[2];
        UpSampleData( void ) { start = 0 , v[0] = v[1] = 0.; }
        UpSampleData( int s , double v1 , double v2 ) { start = s , v[0] = v1 , v[1] = v2; }
    };
    template< int Degree >
    void Octree< Degree >::UpSampleCoarserSolution( int depth , const SortedTreeNodes& sNodes , Vector< Real >& Solution ) const
    {
      int start = sNodes.nodeCount[depth] , end = sNodes.nodeCount[depth+1] , range = end-start;
      Solution.Resize( range );
      if( !depth ) return;
      else if( depth==1 ) for( int i=start ; i<end ; i++ ) Solution[i-start] += sNodes.treeNodes[0]->nodeData.solution;
      else
      {
        // For every node at the current depth
#pragma omp parallel for num_threads( threads )
        for( int t=0 ; t<threads ; t++ )
        {
          TreeOctNode::NeighborKey3 neighborKey;
          neighborKey.set( depth );
          for( int i=start+(range*t)/threads ; i<start+(range*(t+1))/threads ; i++ )
          {
            int d , off[3];
            UpSampleData usData[3];
            sNodes.treeNodes[i]->depthAndOffset( d , off );
            for( int d=0 ; d<3 ; d++ )
            {
              if     ( off[d]  ==0          ) usData[d] = UpSampleData( 1 , 1.00 , 0.00 );
              else if( off[d]+1==(1<<depth) ) usData[d] = UpSampleData( 0 , 0.00 , 1.00 );
              else if( off[d]%2             ) usData[d] = UpSampleData( 1 , 0.75 , 0.25 );
              else                            usData[d] = UpSampleData( 0 , 0.25 , 0.75 );
            }
            neighborKey.getNeighbors( sNodes.treeNodes[i] );
            for( int ii=0 ; ii<2 ; ii++ )
            {
              int _ii = ii + usData[0].start;
              double dx = usData[0].v[ii];
              for( int jj=0 ; jj<2 ; jj++ )
              {
                int _jj = jj + usData[1].start;
                double dxy = dx * usData[1].v[jj];
                for( int kk=0 ; kk<2 ; kk++ )
                {
                  int _kk = kk + usData[2].start;
                  double dxyz = dxy * usData[2].v[kk];
                  Solution[i-start] += Real( neighborKey.neighbors[depth-1].neighbors[_ii][_jj][_kk]->nodeData.solution * dxyz );
                }
              }
            }
          }
        }
      }
      // Clear the coarser solution
      start = sNodes.nodeCount[depth-1] , end = sNodes.nodeCount[depth] , range = end-start;
#pragma omp parallel for num_threads( threads )
      for( int i=start ; i<end ; i++ ) sNodes.treeNodes[i]->nodeData.solution = Real( 0. );
    }
    template< int Degree >
    void Octree< Degree >::DownSampleFinerConstraints( int depth , SortedTreeNodes& sNodes ) const
    {
      if( !depth ) return;
#pragma omp parallel for num_threads( threads )
      for( int i=sNodes.nodeCount[depth-1] ; i<sNodes.nodeCount[depth] ; i++ )
        sNodes.treeNodes[i]->nodeData.constraint = Real( 0 );

      if( depth==1 )
      {
        sNodes.treeNodes[0]->nodeData.constraint = Real( 0 );
        for( int i=sNodes.nodeCount[depth] ; i<sNodes.nodeCount[depth+1] ; i++ ) sNodes.treeNodes[0]->nodeData.constraint += sNodes.treeNodes[i]->nodeData.constraint;
        return;
      }
      std::vector< Vector< double > > constraints( threads );
      for( int t=0 ; t<threads ; t++ ) constraints[t].Resize( sNodes.nodeCount[depth] - sNodes.nodeCount[depth-1] ) , constraints[t].SetZero();
      int start = sNodes.nodeCount[depth] , end = sNodes.nodeCount[depth+1] , range = end-start;
      int lStart = sNodes.nodeCount[depth-1] , lEnd = sNodes.nodeCount[depth];
      // For every node at the current depth
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        TreeOctNode::NeighborKey3 neighborKey;
        neighborKey.set( depth );
        for( int i=start+(range*t)/threads ; i<start+(range*(t+1))/threads ; i++ )
        {
          int d , off[3];
          UpSampleData usData[3];
          sNodes.treeNodes[i]->depthAndOffset( d , off );
          for( int d=0 ; d<3 ; d++ )
          {
            if     ( off[d]  ==0          ) usData[d] = UpSampleData( 1 , 1.00 , 0.00 );
            else if( off[d]+1==(1<<depth) ) usData[d] = UpSampleData( 0 , 0.00 , 1.00 );
            else if( off[d]%2             ) usData[d] = UpSampleData( 1 , 0.75 , 0.25 );
            else                            usData[d] = UpSampleData( 0 , 0.25 , 0.75 );
          }
          neighborKey.getNeighbors( sNodes.treeNodes[i] );
          TreeOctNode::Neighbors3& neighbors = neighborKey.neighbors[depth-1];
          for( int ii=0 ; ii<2 ; ii++ )
          {
            int _ii = ii + usData[0].start;
            double dx = usData[0].v[ii];
            for( int jj=0 ; jj<2 ; jj++ )
            {
              int _jj = jj + usData[1].start;
              double dxy = dx * usData[1].v[jj];
              for( int kk=0 ; kk<2 ; kk++ )
              {
                int _kk = kk + usData[2].start;
                double dxyz = dxy * usData[2].v[kk];
                constraints[t][neighbors.neighbors[_ii][_jj][_kk]->nodeData.nodeIndex-lStart] += sNodes.treeNodes[i]->nodeData.constraint * dxyz;
              }
            }
          }
        }
      }
#pragma omp parallel for num_threads( threads )
      for( int i=lStart ; i<lEnd ; i++ )
      {
        Real cSum = Real(0.);
        for( int t=0 ; t<threads ; t++ ) cSum += constraints[t][i-lStart];
        sNodes.treeNodes[i]->nodeData.constraint += cSum;
      }
    }
    template< int Degree >
    template< class C >
    void Octree< Degree >::DownSample( int depth , const SortedTreeNodes& sNodes , C* constraints ) const
    {
      if( depth==0 ) return;
      if( depth==1 )
      {
        for( int i=sNodes.nodeCount[1] ; i<sNodes.nodeCount[2] ; i++ ) constraints[0] += constraints[i];
        return;
      }
      std::vector< Vector< C > > _constraints( threads );
      for( int t=0 ; t<threads ; t++ ) _constraints[t].Resize( sNodes.nodeCount[depth] - sNodes.nodeCount[depth-1] );
      int start = sNodes.nodeCount[depth] , end = sNodes.nodeCount[depth+1] , range = end-start , lStart = sNodes.nodeCount[depth-1] , lEnd = sNodes.nodeCount[depth];
      // For every node at the current depth
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        TreeOctNode::NeighborKey3 neighborKey;
        neighborKey.set( depth );
        for( int i=start+(range*t)/threads ; i<start+(range*(t+1))/threads ; i++ )
        {
          int d , off[3];
          UpSampleData usData[3];
          sNodes.treeNodes[i]->depthAndOffset( d , off );
          for( int d=0 ; d<3 ; d++ )
          {
            if     ( off[d]  ==0          ) usData[d] = UpSampleData( 1 , 1.00 , 0.00 );
            else if( off[d]+1==(1<<depth) ) usData[d] = UpSampleData( 0 , 0.00 , 1.00 );
            else if( off[d]%2             ) usData[d] = UpSampleData( 1 , 0.75 , 0.25 );
            else                            usData[d] = UpSampleData( 0 , 0.25 , 0.75 );
          }
          TreeOctNode::Neighbors3& neighbors = neighborKey.getNeighbors( sNodes.treeNodes[i]->parent );
          C c = constraints[i];
          for( int ii=0 ; ii<2 ; ii++ )
          {
            int _ii = ii + usData[0].start;
            C cx = C( c*usData[0].v[ii] );
            for( int jj=0 ; jj<2 ; jj++ )
            {
              int _jj = jj + usData[1].start;
              C cxy = C( cx*usData[1].v[jj] );
              for( int kk=0 ; kk<2 ; kk++ )
              {
                int _kk = kk + usData[2].start;
                if( neighbors.neighbors[_ii][_jj][_kk] )
                  _constraints[t][neighbors.neighbors[_ii][_jj][_kk]->nodeData.nodeIndex-lStart] += C( cxy*usData[2].v[kk] );
              }
            }
          }
        }
      }
#pragma omp parallel for num_threads( threads )
      for( int i=lStart ; i<lEnd ; i++ )
      {
        C cSum = C(0);
        for( int t=0 ; t<threads ; t++ ) cSum += _constraints[t][i-lStart];
        constraints[i] += cSum;
      }
    }
    template< int Degree >
    template< class C >
    void Octree< Degree >::UpSample( int depth , const SortedTreeNodes& sNodes , C* coefficients ) const
    {
      if     ( depth==0 ) return;
      else if( depth==1 )
      {
        for( int i=sNodes.nodeCount[1] ; i<sNodes.nodeCount[2] ; i++ ) coefficients[i] += coefficients[0];
        return;
      }

      int start = sNodes.nodeCount[depth] , end = sNodes.nodeCount[depth+1] , range = end-start;
      // For every node at the current depth
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        TreeOctNode::NeighborKey3 neighborKey;
        neighborKey.set( depth-1 );
        for( int i=start+(range*t)/threads ; i<start+(range*(t+1))/threads ; i++ )
        {
          TreeOctNode* node = sNodes.treeNodes[i];
          int d , off[3];
          UpSampleData usData[3];
          node->depthAndOffset( d , off );
          for( int d=0 ; d<3 ; d++ )
          {
            if     ( off[d]  ==0          ) usData[d] = UpSampleData( 1 , 1.00 , 0.00 );
            else if( off[d]+1==(1<<depth) ) usData[d] = UpSampleData( 0 , 0.00 , 1.00 );
            else if( off[d]%2             ) usData[d] = UpSampleData( 1 , 0.75 , 0.25 );
            else                            usData[d] = UpSampleData( 0 , 0.25 , 0.75 );
          }
          TreeOctNode::Neighbors3& neighbors = neighborKey.getNeighbors( node->parent );
          for( int ii=0 ; ii<2 ; ii++ )
          {
            int _ii = ii + usData[0].start;
            double dx = usData[0].v[ii];
            for( int jj=0 ; jj<2 ; jj++ )
            {
              int _jj = jj + usData[1].start;
              double dxy = dx * usData[1].v[jj];
              for( int kk=0 ; kk<2 ; kk++ )
              {
                int _kk = kk + usData[2].start;
                if( neighbors.neighbors[_ii][_jj][_kk] )
                {
                  double dxyz = dxy * usData[2].v[kk];
                  int _i = neighbors.neighbors[_ii][_jj][_kk]->nodeData.nodeIndex;
                  coefficients[i] += coefficients[_i] * Real( dxyz );
                }
              }
            }
          }
        }
      }
    }
    template< int Degree >
    void Octree< Degree >::SetCoarserPointValues( int depth , const SortedTreeNodes& sNodes , Real* metSolution )
    {
      int start = sNodes.nodeCount[depth] , end = sNodes.nodeCount[depth+1] , range = end-start;
      // For every node at the current depth
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        TreeOctNode::NeighborKey3 neighborKey;
        neighborKey.set( depth );
        for( int i=start+(range*t)/threads ; i<start+(range*(t+1))/threads ; i++ )
        {
          int pIdx = sNodes.treeNodes[i]->nodeData.pointIndex;
          if( pIdx!=-1 )
          {
            neighborKey.getNeighbors( sNodes.treeNodes[i] );
            _points[ pIdx ].value = WeightedCoarserFunctionValue( neighborKey , sNodes.treeNodes[i] , metSolution );
          }
        }
      }
    }
    template< int Degree >
    Real Octree< Degree >::WeightedCoarserFunctionValue( const OctNode< TreeNodeData , Real >::NeighborKey3& neighborKey , const TreeOctNode* pointNode , Real* metSolution ) const
    {
      int depth = pointNode->depth();
      if( !depth || pointNode->nodeData.pointIndex==-1 ) return Real(0.);
      double pointValue = 0;

      Real weight       = _points[ pointNode->nodeData.pointIndex ].weight;
      Point3D< Real > p = _points[ pointNode->nodeData.pointIndex ].position;

      // Iterate over all basis functions that overlap the point at the coarser resolutions
      {
        int d , _idx[3];
        const TreeOctNode::Neighbors3& neighbors = neighborKey.neighbors[depth-1];
        neighbors.neighbors[1][1][1]->depthAndOffset( d , _idx );
        _idx[0] = BinaryNode< double >::CenterIndex( d , _idx[0]-1 );
        _idx[1] = BinaryNode< double >::CenterIndex( d , _idx[1]-1 );
        _idx[2] = BinaryNode< double >::CenterIndex( d , _idx[2]-1 );
        for( int j=0 ; j<3 ; j++ ) for( int k=0 ; k<3 ; k++ ) for( int l=0 ; l<3 ; l++ )
          if( neighbors.neighbors[j][k][l] && neighbors.neighbors[j][k][l]->nodeData.nodeIndex>=0 )
          {
            // Accumulate the contribution from these basis nodes
            const TreeOctNode* basisNode = neighbors.neighbors[j][k][l];
            int idx[] = { _idx[0]+j , _idx[1]+k , _idx[2]+l };
            pointValue +=
                fData.baseBSplines[ idx[0] ][2-j]( p[0] ) *
                fData.baseBSplines[ idx[1] ][2-k]( p[1] ) *
                fData.baseBSplines[ idx[2] ][2-l]( p[2] ) *
                metSolution[basisNode->nodeData.nodeIndex];
          }
      }
      return Real( pointValue * weight );
    }
    template<int Degree>
    int Octree<Degree>::GetFixedDepthLaplacian( SparseSymmetricMatrix< Real >& matrix , int depth , const SortedTreeNodes& sNodes , Real* metSolution )
    {
      int start = sNodes.nodeCount[depth] , end = sNodes.nodeCount[depth+1] , range = end-start;
      double stencil[5][5][5];
      SetLaplacianStencil( depth , stencil );
      Stencil< double , 5 > stencils[2][2][2];
      SetLaplacianStencils( depth , stencils );
      matrix.Resize( range );
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        TreeOctNode::NeighborKey5 neighborKey5;
        neighborKey5.set( depth );
        for( int i=(range*t)/threads ; i<(range*(t+1))/threads ; i++ )
        {
          TreeOctNode* node = sNodes.treeNodes[i+start];
          neighborKey5.getNeighbors( node );

          // Get the matrix row size
          int count = GetMatrixRowSize( neighborKey5.neighbors[depth] );

          // Allocate memory for the row
#pragma omp critical (matrix_set_row_size)
          {
            matrix.SetRowSize( i , count );
          }

          // Set the row entries
          matrix.rowSizes[i] = SetMatrixRow( neighborKey5.neighbors[depth] , matrix[i] , start , stencil );

          // Offset the constraints using the solution from lower resolutions.
          int x , y , z , c;
          if( node->parent )
          {
            c = int( node - node->parent->children );
            Cube::FactorCornerIndex( c , x , y , z );
          }
          else x = y = z = 0;
          UpdateConstraintsFromCoarser( neighborKey5 , node , metSolution , stencils[x][y][z] );
        }
      }
      return 1;
    }
    template<int Degree>
    int Octree<Degree>::GetRestrictedFixedDepthLaplacian( SparseSymmetricMatrix< Real >& matrix,int depth,const int* entries,int entryCount,
                                                          const TreeOctNode* rNode , Real radius ,
                                                          const SortedTreeNodes& sNodes , Real* metSolution )
    {
      for( int i=0 ; i<entryCount ; i++ ) sNodes.treeNodes[ entries[i] ]->nodeData.nodeIndex = i;
      double stencil[5][5][5];
      int rDepth , rOff[3];
      rNode->depthAndOffset( rDepth , rOff );
      matrix.Resize( entryCount );
      SetLaplacianStencil( depth , stencil );
      Stencil< double , 5 > stencils[2][2][2];
      SetLaplacianStencils( depth , stencils );
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
      {
        TreeOctNode::NeighborKey5 neighborKey5;
        neighborKey5.set( depth );
        for( int i=(entryCount*t)/threads ; i<(entryCount*(t+1))/threads ; i++ )
        {
          TreeOctNode* node = sNodes.treeNodes[ entries[i] ];
          int d , off[3];
          node->depthAndOffset( d , off );
          off[0] >>= (depth-rDepth) , off[1] >>= (depth-rDepth) , off[2] >>= (depth-rDepth);
          bool isInterior = ( off[0]==rOff[0] && off[1]==rOff[1] && off[2]==rOff[2] );

          neighborKey5.getNeighbors( node );

          int xStart=0 , xEnd=5 , yStart=0 , yEnd=5 , zStart=0 , zEnd=5;
          if( !isInterior ) SetMatrixRowBounds( neighborKey5.neighbors[depth].neighbors[2][2][2] , rDepth , rOff , xStart , xEnd , yStart , yEnd , zStart , zEnd );

          // Get the matrix row size
          int count = GetMatrixRowSize( neighborKey5.neighbors[depth] , xStart , xEnd , yStart , yEnd , zStart , zEnd );

          // Allocate memory for the row
#pragma omp critical (matrix_set_row_size)
          {
            matrix.SetRowSize( i , count );
          }

          // Set the matrix row entries
          matrix.rowSizes[i] = SetMatrixRow( neighborKey5.neighbors[depth] , matrix[i] , 0 , stencil , xStart , xEnd , yStart , yEnd , zStart , zEnd );
          // Adjust the system constraints
          int x , y , z , c;
          if( node->parent )
          {
            c = int( node - node->parent->children );
            Cube::FactorCornerIndex( c , x , y , z );
          }
          else x = y = z = 0;
          UpdateConstraintsFromCoarser( neighborKey5 , node , metSolution , stencils[x][y][z] );
        }
      }
      for( int i=0 ; i<entryCount ; i++ ) sNodes.treeNodes[entries[i]]->nodeData.nodeIndex = entries[i];
      return 1;
    }

    template<int Degree>
    int Octree<Degree>::LaplacianMatrixIteration( int subdivideDepth , bool showResidual , int minIters , double accuracy )
    {
      int i,iter=0;
      double t = 0;
      fData.setDotTables( fData.DD_DOT_FLAG | fData.DV_DOT_FLAG );

      SparseMatrix< float >::SetAllocator( MEMORY_ALLOCATOR_BLOCK_SIZE );
      _sNodes.treeNodes[0]->nodeData.solution = 0;

      std::vector< Real > metSolution( _sNodes.nodeCount[ _sNodes.maxDepth ] , 0 );

      for( i=1 ; i<_sNodes.maxDepth ; i++ )
      {
        if( subdivideDepth>0 ) iter += SolveFixedDepthMatrix( i , _sNodes , &metSolution[0] , subdivideDepth , showResidual , minIters , accuracy );
        else                   iter += SolveFixedDepthMatrix( i , _sNodes , &metSolution[0] ,                  showResidual , minIters , accuracy );
      }
      SparseMatrix< float >::internalAllocator.reset();
      fData.clearDotTables( fData.VV_DOT_FLAG | fData.DV_DOT_FLAG | fData.DD_DOT_FLAG );

      return iter;
    }

    template<int Degree>
    int Octree<Degree>::SolveFixedDepthMatrix( int depth , const SortedTreeNodes& sNodes , Real* metSolution , bool showResidual , int minIters , double accuracy )
    {
      int iter = 0;
      Vector< Real > X , B;
      SparseSymmetricMatrix< Real > M;
      double systemTime=0. , solveTime=0. , updateTime=0. ,  evaluateTime = 0.;

      X.Resize( sNodes.nodeCount[depth+1]-sNodes.nodeCount[depth] );
      if( depth<=_minDepth ) UpSampleCoarserSolution( depth , sNodes , X );
      else
      {
        // Up-sample the cumulative solution into the previous depth
        UpSample( depth-1 , sNodes , metSolution );
        // Add in the solution from that depth
        if( depth )
#pragma omp parallel for num_threads( threads )
          for( int i=_sNodes.nodeCount[depth-1] ; i<_sNodes.nodeCount[depth] ; i++ ) metSolution[i] += _sNodes.treeNodes[i]->nodeData.solution;
      }
      if( _constrainValues )
      {
        SetCoarserPointValues( depth , sNodes , metSolution );
      }

      SparseSymmetricMatrix< Real >::internalAllocator.rollBack();
      {
        int maxECount = ( (2*Degree+1)*(2*Degree+1)*(2*Degree+1) + 1 ) / 2;
        maxECount = ( ( maxECount + 15 ) / 16 ) * 16;
        M.Resize( sNodes.nodeCount[depth+1]-sNodes.nodeCount[depth] );
        for( int i=0 ; i<M.rows ; i++ ) M.SetRowSize( i , maxECount );
      }

      {
        // Get the system matrix
        SparseSymmetricMatrix< Real >::internalAllocator.rollBack();
        GetFixedDepthLaplacian( M , depth , sNodes , metSolution );
        // Set the constraint vector
        B.Resize( sNodes.nodeCount[depth+1]-sNodes.nodeCount[depth] );
        for( int i=sNodes.nodeCount[depth] ; i<sNodes.nodeCount[depth+1] ; i++ ) B[i-sNodes.nodeCount[depth]] = sNodes.treeNodes[i]->nodeData.constraint;
      }

      // Solve the linear system
      iter += SparseSymmetricMatrix< Real >::Solve( M , B , std::max< int >( int( pow( M.rows , ITERATION_POWER ) ) , minIters ) , X , Real(accuracy) , 0 , threads , (depth<=_minDepth) && !_constrainValues );

      if( showResidual )
      {
        double mNorm = 0;
        for( int i=0 ; i<M.rows ; i++ ) for( int j=0 ; j<M.rowSizes[i] ; j++ ) mNorm += M[i][j].Value * M[i][j].Value;
        double bNorm = B.Norm( 2 ) , rNorm = ( B - M * X ).Norm( 2 );
        printf( "\tResidual: (%d %g) %g -> %g (%f) [%d]\n" , M.Entries() , sqrt(mNorm) , bNorm , rNorm , rNorm/bNorm , iter );
      }

      // Copy the solution back into the tree (over-writing the constraints)
      for( int i=sNodes.nodeCount[depth] ; i<sNodes.nodeCount[depth+1] ; i++ ) sNodes.treeNodes[i]->nodeData.solution = Real( X[i-sNodes.nodeCount[depth]] );

      return iter;
    }
    template<int Degree>
    int Octree<Degree>::SolveFixedDepthMatrix( int depth , const SortedTreeNodes& sNodes , Real* metSolution , int startingDepth , bool showResidual , int minIters , double accuracy )
    {
      if( startingDepth>=depth ) return SolveFixedDepthMatrix( depth , sNodes , metSolution , showResidual , minIters , accuracy );

      int i , j , d , tIter=0;
      SparseSymmetricMatrix< Real > _M;
      Vector< Real > B , B_ , X_;
      AdjacencySetFunction asf;
      AdjacencyCountFunction acf;
      double systemTime = 0 , solveTime = 0 , memUsage = 0 , evaluateTime = 0 , gTime = 0, sTime = 0;
      Real myRadius , myRadius2;

      if( depth>_minDepth )
      {
        // Up-sample the cumulative solution into the previous depth
        UpSample( depth-1 , sNodes , metSolution );
        // Add in the solution from that depth
        if( depth )
#pragma omp parallel for num_threads( threads )
          for( int i=_sNodes.nodeCount[depth-1] ; i<_sNodes.nodeCount[depth] ; i++ ) metSolution[i] += _sNodes.treeNodes[i]->nodeData.solution;
      }

      if( _constrainValues )
      {
        SetCoarserPointValues( depth , sNodes , metSolution );
      }
      B.Resize( sNodes.nodeCount[depth+1] - sNodes.nodeCount[depth] );

      // Back-up the constraints
      for( i=sNodes.nodeCount[depth] ; i<sNodes.nodeCount[depth+1] ; i++ )
      {
        B[ i-sNodes.nodeCount[depth] ] = sNodes.treeNodes[i]->nodeData.constraint;
        sNodes.treeNodes[i]->nodeData.constraint = 0;
      }

      myRadius = 2*radius-Real(0.5);
      myRadius = int(myRadius-ROUND_EPS)+ROUND_EPS;
      myRadius2 = Real(radius+ROUND_EPS-0.5);
      d = depth-startingDepth;
      std::vector< int > subDimension( sNodes.nodeCount[d+1]-sNodes.nodeCount[d] );
      int maxDimension = 0;
      for( i=sNodes.nodeCount[d] ; i<sNodes.nodeCount[d+1] ; i++ )
      {
        // Count the number of nodes at depth "depth" that lie under sNodes.treeNodes[i]
        acf.adjacencyCount = 0;
        for( TreeOctNode* temp=sNodes.treeNodes[i]->nextNode() ; temp ; )
        {
          if( temp->depth()==depth ) acf.adjacencyCount++ , temp = sNodes.treeNodes[i]->nextBranch( temp );
          else                                              temp = sNodes.treeNodes[i]->nextNode  ( temp );
        }
        for( j=sNodes.nodeCount[d] ; j<sNodes.nodeCount[d+1] ; j++ )
        {
          if( i==j ) continue;
          TreeOctNode::ProcessFixedDepthNodeAdjacentNodes( fData.depth , sNodes.treeNodes[i] , 1 , sNodes.treeNodes[j] , 2*width-1 , depth , &acf );
        }
        subDimension[i-sNodes.nodeCount[d]] = acf.adjacencyCount;
        maxDimension = std::max< int >( maxDimension , subDimension[i-sNodes.nodeCount[d]] );
      }
      asf.adjacencies = new int[maxDimension];
      MapReduceVector< Real > mrVector;
      mrVector.resize( threads , maxDimension );
      // Iterate through the coarse-level nodes
      for( i=sNodes.nodeCount[d] ; i<sNodes.nodeCount[d+1] ; i++ )
      {
        int iter = 0;
        // Count the number of nodes at depth "depth" that lie under sNodes.treeNodes[i]
        acf.adjacencyCount = subDimension[i-sNodes.nodeCount[d]];
        if( !acf.adjacencyCount ) continue;

        // Set the indices for the nodes under, or near, sNodes.treeNodes[i].
        asf.adjacencyCount = 0;
        for( TreeOctNode* temp=sNodes.treeNodes[i]->nextNode() ; temp ; )
        {
          if( temp->depth()==depth ) asf.adjacencies[ asf.adjacencyCount++ ] = temp->nodeData.nodeIndex , temp = sNodes.treeNodes[i]->nextBranch( temp );
          else                                                                                            temp = sNodes.treeNodes[i]->nextNode  ( temp );
        }
        for( j=sNodes.nodeCount[d] ; j<sNodes.nodeCount[d+1] ; j++ )
        {
          if( i==j ) continue;
          TreeOctNode::ProcessFixedDepthNodeAdjacentNodes( fData.depth , sNodes.treeNodes[i] , 1 , sNodes.treeNodes[j] , 2*width-1 , depth , &asf );
        }

        // Get the associated constraint vector
        B_.Resize( asf.adjacencyCount );
        for( j=0 ; j<asf.adjacencyCount ; j++ ) B_[j] = B[ asf.adjacencies[j]-sNodes.nodeCount[depth] ];

        X_.Resize( asf.adjacencyCount );
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( j=0 ; j<asf.adjacencyCount ; j++ )
        {
          X_[j] = sNodes.treeNodes[ asf.adjacencies[j] ]->nodeData.solution;
        }
        // Get the associated matrix
        SparseSymmetricMatrix< Real >::internalAllocator.rollBack();
        GetRestrictedFixedDepthLaplacian( _M , depth , asf.adjacencies , asf.adjacencyCount , sNodes.treeNodes[i] , myRadius , sNodes , metSolution );
#pragma omp parallel for num_threads( threads ) schedule( static )
        for( j=0 ; j<asf.adjacencyCount ; j++ )
        {
          B_[j] += sNodes.treeNodes[asf.adjacencies[j]]->nodeData.constraint;
          sNodes.treeNodes[ asf.adjacencies[j] ]->nodeData.constraint = 0;
        }

        // Solve the matrix
        // Since we don't have the full matrix, the system shouldn't be singular, so we shouldn't have to correct it
        iter += SparseSymmetricMatrix< Real >::Solve( _M , B_ , std::max< int >( int( pow( _M.rows , ITERATION_POWER ) ) , minIters ) , X_ , mrVector , Real(accuracy) , 0 );

        if( showResidual )
        {
          double mNorm = 0;
          for( int i=0 ; i<_M.rows ; i++ ) for( int j=0 ; j<_M.rowSizes[i] ; j++ ) mNorm += _M[i][j].Value * _M[i][j].Value;
          double bNorm = B_.Norm( 2 ) , rNorm = ( B_ - _M * X_ ).Norm( 2 );
          printf( "\t\tResidual: (%d %g) %g -> %g (%f) [%d]\n" , _M.Entries() , sqrt(mNorm) , bNorm , rNorm , rNorm/bNorm , iter );
        }

        // Update the solution for all nodes in the sub-tree
        for( j=0 ; j<asf.adjacencyCount ; j++ )
        {
          TreeOctNode* temp=sNodes.treeNodes[ asf.adjacencies[j] ];
          while( temp->depth()>sNodes.treeNodes[i]->depth() ) temp=temp->parent;
          if( temp->nodeData.nodeIndex>=sNodes.treeNodes[i]->nodeData.nodeIndex ) sNodes.treeNodes[ asf.adjacencies[j] ]->nodeData.solution = Real( X_[j] );
        }
        systemTime += gTime;
        solveTime += sTime;
        memUsage = std::max< double >( MemoryUsage() , memUsage );
        tIter += iter;
      }
      delete[] asf.adjacencies;
      return tIter;
    }
    template<int Degree>
    int Octree<Degree>::HasNormals(TreeOctNode* node,Real epsilon)
    {
      int hasNormals=0;
      if( node->nodeData.normalIndex>=0 && ( (*normals)[node->nodeData.normalIndex][0]!=0 || (*normals)[node->nodeData.normalIndex][1]!=0 || (*normals)[node->nodeData.normalIndex][2]!=0 ) ) hasNormals=1;
      if( node->children ) for(int i=0;i<Cube::CORNERS && !hasNormals;i++) hasNormals |= HasNormals(&node->children[i],epsilon);

      return hasNormals;
    }
    template<int Degree>
    void Octree<Degree>::ClipTree( void )
    {
      int maxDepth = tree.maxDepth();
      for( TreeOctNode* temp=tree.nextNode() ; temp ; temp=tree.nextNode(temp) )
        if( temp->children && temp->d>=_minDepth )
        {
          int hasNormals=0;
          for( int i=0 ; i<Cube::CORNERS && !hasNormals ; i++ ) hasNormals = HasNormals( &temp->children[i] , EPSILON/(1<<maxDepth) );
          if( !hasNormals ) temp->children=NULL;
        }
      MemoryUsage();
    }
    template<int Degree>
    void Octree<Degree>::SetLaplacianConstraints( void )
    {
      // To set the Laplacian constraints, we iterate over the
      // splatted normals and compute the dot-product of the
      // divergence of the normal field with all the basis functions.
      // Within the same depth: set directly as a gather
      // Coarser depths
      fData.setDotTables( fData.VV_DOT_FLAG | fData.DV_DOT_FLAG );

      int maxDepth = _sNodes.maxDepth-1;
      Point3D< Real > zeroPoint;
      zeroPoint[0] = zeroPoint[1] = zeroPoint[2] = 0;
      std::vector< Real > constraints( _sNodes.nodeCount[maxDepth] , Real(0) );
      std::vector< Point3D< Real > > coefficients( _sNodes.nodeCount[maxDepth] , zeroPoint );

      // Clear the constraints
#pragma omp parallel for num_threads( threads )
      for( int i=0 ; i<_sNodes.nodeCount[maxDepth+1] ; i++ ) _sNodes.treeNodes[i]->nodeData.constraint = Real( 0. );

      // For the scattering part of the operation, we parallelize by duplicating the constraints and then summing at the end.
      std::vector< std::vector< Real > > _constraints( threads );
      for( int t=0 ; t<threads ; t++ ) _constraints[t].resize( _sNodes.nodeCount[maxDepth] , 0 );

      for( int d=maxDepth ; d>=0 ; d-- )
      {
        Point3D< double > stencil[5][5][5];
        SetDivergenceStencil( d , &stencil[0][0][0] , false );
        Stencil< Point3D< double > , 5 > stencils[2][2][2];
        SetDivergenceStencils( d , stencils , true );
#pragma omp parallel for num_threads( threads )
        for( int t=0 ; t<threads ; t++ )
        {
          TreeOctNode::NeighborKey5 neighborKey5;
          neighborKey5.set( fData.depth );
          int start = _sNodes.nodeCount[d] , end = _sNodes.nodeCount[d+1] , range = end-start;
          for( int i=start+(range*t)/threads ; i<start+(range*(t+1))/threads ; i++ )
          {
            TreeOctNode* node = _sNodes.treeNodes[i];
            int startX=0 , endX=5 , startY=0 , endY=5 , startZ=0 , endZ=5;
            int depth = node->depth();
            neighborKey5.getNeighbors( node );

            bool isInterior , isInterior2;
            {
              int d , off[3];
              node->depthAndOffset( d , off );
              int mn = 2 , mx = (1<<d)-2;
              isInterior = ( off[0]>=mn && off[0]<mx && off[1]>=mn && off[1]<mx && off[2]>=mn && off[2]<mx );
              mn += 2 , mx -= 2;
              isInterior2 = ( off[0]>=mn && off[0]<mx && off[1]>=mn && off[1]<mx && off[2]>=mn && off[2]<mx );
            }
            int cx , cy , cz;
            if( d )
            {
              int c = int( node - node->parent->children );
              Cube::FactorCornerIndex( c , cx , cy , cz );
            }
            else cx = cy = cz = 0;
            Stencil< Point3D< double > , 5 >& _stencil = stencils[cx][cy][cz];

            // Set constraints from current depth
            {
              const TreeOctNode::Neighbors5& neighbors5 = neighborKey5.neighbors[depth];

              if( isInterior )
                for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
                {
                  const TreeOctNode* _node = neighbors5.neighbors[x][y][z];
                  if( _node && _node->nodeData.normalIndex>=0 )
                  {
                    const Point3D< Real >& _normal = (*normals)[_node->nodeData.normalIndex];
                    node->nodeData.constraint += Real( stencil[x][y][z][0] * _normal[0] + stencil[x][y][z][1] * _normal[1] + stencil[x][y][z][2] * _normal[2] );
                  }
                }
              else
                for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
                {
                  const TreeOctNode* _node = neighbors5.neighbors[x][y][z];
                  if( _node && _node->nodeData.normalIndex>=0 )
                  {
                    const Point3D< Real >& _normal = (*normals)[_node->nodeData.normalIndex];
                    node->nodeData.constraint += GetDivergence( _node , node ,  _normal );
                  }
                }
              UpdateCoarserSupportBounds( neighbors5.neighbors[2][2][2] , startX , endX , startY  , endY , startZ , endZ );
            }
            if( node->nodeData.nodeIndex<0 || node->nodeData.normalIndex<0 ) continue;
            const Point3D< Real >& normal = (*normals)[node->nodeData.normalIndex];
            if( normal[0]==0 && normal[1]==0 && normal[2]==0 ) continue;
            if( depth<maxDepth ) coefficients[i] += normal;

            if( depth )
            {
              const TreeOctNode::Neighbors5& neighbors5 = neighborKey5.neighbors[depth-1];

              for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
                if( neighbors5.neighbors[x][y][z] )
                {
                  TreeOctNode* _node = neighbors5.neighbors[x][y][z];
                  if( isInterior2 )
                  {
                    Point3D< double >& div = _stencil.values[x][y][z];
                    _constraints[t][ _node->nodeData.nodeIndex ] += Real( div[0] * normal[0] + div[1] * normal[1] + div[2] * normal[2] );
                  }
                  else _constraints[t][ _node->nodeData.nodeIndex ] += GetDivergence( node , _node , normal );
                }
            }
          }
        }
      }
#pragma omp parallel for num_threads( threads ) schedule( static )
      for( int i=0 ; i<_sNodes.nodeCount[maxDepth] ; i++ )
      {
        Real cSum = Real(0.);
        for( int t=0 ; t<threads ; t++ ) cSum += _constraints[t][i];
        constraints[i] = cSum;
      }
      // Fine-to-coarse down-sampling of constraints
      for( int d=maxDepth-1 ; d>=0 ; d-- ) DownSample( d , _sNodes , &constraints[0] );

      // Coarse-to-fine up-sampling of coefficients
      for( int d=0 ; d<maxDepth ; d++ ) UpSample( d , _sNodes , &coefficients[0] );

      // Add the accumulated constraints from all finer depths
#pragma omp parallel for num_threads( threads )
      for( int i=0 ; i<_sNodes.nodeCount[maxDepth] ; i++ ) _sNodes.treeNodes[i]->nodeData.constraint += constraints[i];

      // Compute the contribution from all coarser depths
      for( int d=0 ; d<=maxDepth ; d++ )
      {
        int start = _sNodes.nodeCount[d] , end = _sNodes.nodeCount[d+1] , range = end - start;
        Stencil< Point3D< double > , 5 > stencils[2][2][2];
        SetDivergenceStencils( d , stencils , false );
#pragma omp parallel for num_threads( threads )
        for( int t=0 ; t<threads ; t++ )
        {
          TreeOctNode::NeighborKey5 neighborKey5;
          neighborKey5.set( maxDepth );
          for( int i=start+(range*t)/threads ; i<start+(range*(t+1))/threads ; i++ )
          {
            TreeOctNode* node = _sNodes.treeNodes[i];
            int depth = node->depth();
            if( !depth ) continue;
            int startX=0 , endX=5 , startY=0 , endY=5 , startZ=0 , endZ=5;
            UpdateCoarserSupportBounds( node , startX , endX , startY  , endY , startZ , endZ );
            const TreeOctNode::Neighbors5& neighbors5 = neighborKey5.getNeighbors( node->parent );

            bool isInterior;
            {
              int d , off[3];
              node->depthAndOffset( d , off );
              int mn = 4 , mx = (1<<d)-4;
              isInterior = ( off[0]>=mn && off[0]<mx && off[1]>=mn && off[1]<mx && off[2]>=mn && off[2]<mx );
            }
            int cx , cy , cz;
            if( d )
            {
              int c = int( node - node->parent->children );
              Cube::FactorCornerIndex( c , cx , cy , cz );
            }
            else cx = cy = cz = 0;
            Stencil< Point3D< double > , 5 >& _stencil = stencils[cx][cy][cz];

            Real constraint = Real(0);
            for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
              if( neighbors5.neighbors[x][y][z] )
              {
                TreeOctNode* _node = neighbors5.neighbors[x][y][z];
                int _i = _node->nodeData.nodeIndex;
                if( isInterior )
                {
                  Point3D< double >& div = _stencil.values[x][y][z];
                  Point3D< Real >& normal = coefficients[_i];
                  constraint += Real( div[0] * normal[0] + div[1] * normal[1] + div[2] * normal[2] );
                }
                else constraint += GetDivergence( _node , node , coefficients[_i] );
              }
            node->nodeData.constraint += constraint;
          }
        }
      }

      fData.clearDotTables( fData.DV_DOT_FLAG );

      // Set the point weights for evaluating the iso-value
#pragma omp parallel for num_threads( threads )
      for( int t=0 ; t<threads ; t++ )
        for( int i=(_sNodes.nodeCount[maxDepth+1]*t)/threads ; i<(_sNodes.nodeCount[maxDepth+1]*(t+1))/threads ; i++ )
        {
          TreeOctNode* temp = _sNodes.treeNodes[i];
          if( temp->nodeData.nodeIndex<0 || temp->nodeData.normalIndex<0 ) temp->nodeData.centerWeightContribution = 0;
          else                                                             temp->nodeData.centerWeightContribution = Real( Length((*normals)[temp->nodeData.normalIndex]) );
        }
      MemoryUsage();
      delete normals;
      normals = NULL;
    }
    template<int Degree>
    void Octree<Degree>::AdjacencyCountFunction::Function(const TreeOctNode* node1,const TreeOctNode* node2){adjacencyCount++;}
    template<int Degree>
    void Octree<Degree>::AdjacencySetFunction::Function(const TreeOctNode* node1,const TreeOctNode* node2){adjacencies[adjacencyCount++]=node1->nodeData.nodeIndex;}
    template<int Degree>
    void Octree<Degree>::RefineFunction::Function( TreeOctNode* node1 , const TreeOctNode* node2 )
    {
      if( !node1->children && node1->depth()<depth ) node1->initChildren();
    }
    template< int Degree >
    void Octree< Degree >::FaceEdgesFunction::Function( const TreeOctNode* node1 , const TreeOctNode* node2 )
    {
      if( !node1->children && MarchingCubes::HasRoots( node1->nodeData.mcIndex ) )
      {
        RootInfo ri1 , ri2;
        hash_map< long long , std::pair< RootInfo , int > >::iterator iter;
        int isoTri[DIMENSION*MarchingCubes::MAX_TRIANGLES];
        int count=MarchingCubes::AddTriangleIndices( node1->nodeData.mcIndex , isoTri );

        for( int j=0 ; j<count ; j++ )
          for( int k=0 ; k<3 ; k++ )
            if( fIndex==Cube::FaceAdjacentToEdges( isoTri[j*3+k] , isoTri[j*3+((k+1)%3)] ) )
              if( GetRootIndex( node1 , isoTri[j*3+k] , maxDepth , ri1 ) && GetRootIndex( node1 , isoTri[j*3+((k+1)%3)] , maxDepth , ri2 ) )
              {
                long long key1=ri1.key , key2=ri2.key;
                edges->push_back( std::pair< RootInfo , RootInfo >( ri2 , ri1 ) );
                iter = vertexCount->find( key1 );
                if( iter==vertexCount->end() )
                {
                  (*vertexCount)[key1].first = ri1;
                  (*vertexCount)[key1].second=0;
                }
                iter=vertexCount->find(key2);
                if( iter==vertexCount->end() )
                {
                  (*vertexCount)[key2].first = ri2;
                  (*vertexCount)[key2].second=0;
                }
                (*vertexCount)[key1].second--;
                (*vertexCount)[key2].second++;
              }
              else fprintf( stderr , "Bad Edge 1: %d %d\n" , ri1.key , ri2.key );
      }
    }

    template< int Degree >
    void Octree< Degree >::RefineBoundary( int subdivideDepth )
    {
      // This implementation is somewhat tricky.
      // We would like to ensure that leaf-nodes across a subdivision boundary have the same depth.
      // We do this by calling the setNeighbors function.
      // The key is to implement this in a single pass through the leaves, ensuring that refinements don't propogate.
      // To this end, we do the minimal refinement that ensures that a cross boundary neighbor, and any of its cross-boundary
      // neighbors are all refined simultaneously.
      // For this reason, the implementation can only support nodes deeper than sDepth.
      bool flags[3][3][3];
      int maxDepth = tree.maxDepth();

      int sDepth;
      if( subdivideDepth<=0 ) sDepth = 0;
      else                    sDepth = maxDepth-subdivideDepth;
      if( sDepth<=0 ) return;

      // Ensure that face adjacent neighbors across the subdivision boundary exist to allow for
      // a consistent definition of the iso-surface
      TreeOctNode::NeighborKey3 nKey;
      nKey.set( maxDepth );
      for( TreeOctNode* leaf=tree.nextLeaf() ; leaf ; leaf=tree.nextLeaf( leaf ) )
        if( leaf->depth()>sDepth )
        {
          int d , off[3] , _off[3];
          leaf->depthAndOffset( d , off );
          int res = (1<<d)-1 , _res = ( 1<<(d-sDepth) )-1;
          _off[0] = off[0]&_res , _off[1] = off[1]&_res , _off[2] = off[2]&_res;
          bool boundary[3][2] =
          {
            { (off[0]!=0 && _off[0]==0) , (off[0]!=res && _off[0]==_res) } ,
            { (off[1]!=0 && _off[1]==0) , (off[1]!=res && _off[1]==_res) } ,
            { (off[2]!=0 && _off[2]==0) , (off[2]!=res && _off[2]==_res) }
          };

          if( boundary[0][0] || boundary[0][1] || boundary[1][0] || boundary[1][1] || boundary[2][0] || boundary[2][1] )
          {
            TreeOctNode::Neighbors3& neighbors = nKey.getNeighbors( leaf );
            for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ ) for( int k=0 ; k<3 ; k++ ) flags[i][j][k] = false;
            int x=0 , y=0 , z=0;
            if     ( boundary[0][0] && !neighbors.neighbors[0][1][1] ) x = -1;
            else if( boundary[0][1] && !neighbors.neighbors[2][1][1] ) x =  1;
            if     ( boundary[1][0] && !neighbors.neighbors[1][0][1] ) y = -1;
            else if( boundary[1][1] && !neighbors.neighbors[1][2][1] ) y =  1;
            if     ( boundary[2][0] && !neighbors.neighbors[1][1][0] ) z = -1;
            else if( boundary[2][1] && !neighbors.neighbors[1][1][2] ) z =  1;

            if( x || y || z )
            {
              // Corner case
              if( x && y && z ) flags[1+x][1+y][1+z] = true;
              // Edge cases
              if( x && y      ) flags[1+x][1+y][1  ] = true;
              if( x &&      z ) flags[1+x][1  ][1+z] = true;
              if(      y && z ) flags[1  ][1+y][1+1] = true;
              // Face cases
              if( x           ) flags[1+x][1  ][1  ] = true;
              if(      y      ) flags[1  ][1+y][1  ] = true;
              if(           z ) flags[1  ][1  ][1+z] = true;
              nKey.setNeighbors( leaf , flags );
            }
          }
        }
      _sNodes.set( tree );
      MemoryUsage();
    }
    template<int Degree>
    void Octree<Degree>::GetMCIsoTriangles( Real isoValue , int subdivideDepth , CoredMeshData* mesh , int fullDepthIso , int nonLinearFit , bool addBarycenter , bool polygonMesh )
    {
      fData.setValueTables( fData.VALUE_FLAG | fData.D_VALUE_FLAG , 0 , postNormalSmooth );

      // Ensure that the subtrees are self-contained
      RefineBoundary( subdivideDepth );

      RootData rootData , coarseRootData;
      std::vector< Point3D< float > >* interiorPoints;
      int maxDepth = tree.maxDepth();

      int sDepth = subdivideDepth<=0 ? 0 : std::max< int >( 0 , maxDepth-subdivideDepth );

      std::vector< Real > metSolution( _sNodes.nodeCount[maxDepth] , 0 );
#pragma omp parallel for num_threads( threads )
      for( int i=_sNodes.nodeCount[_minDepth] ; i<_sNodes.nodeCount[maxDepth] ; i++ ) metSolution[i] = _sNodes.treeNodes[i]->nodeData.solution;
      for( int d=0 ; d<maxDepth ; d++ ) UpSample( d , _sNodes , &metSolution[0] );

      // Clear the marching cube indices
#pragma omp parallel for num_threads( threads )
      for( int i=0 ; i<_sNodes.nodeCount[maxDepth+1] ; i++ ) _sNodes.treeNodes[i]->nodeData.mcIndex = 0;

      rootData.boundaryValues = new hash_map< long long , std::pair< Real , Point3D< Real > > >();
      int offSet = 0;

      int maxCCount = _sNodes.getMaxCornerCount( &tree , sDepth , maxDepth , threads );
      int maxECount = _sNodes.getMaxEdgeCount  ( &tree , sDepth , threads );
      rootData.cornerValues     = new          Real  [ maxCCount ];
      rootData.cornerNormals    = new Point3D< Real >[ maxCCount ];
      rootData.interiorRoots    = new int [ maxECount ];
      rootData.cornerValuesSet  = new char[ maxCCount ];
      rootData.cornerNormalsSet = new char[ maxCCount ];
      rootData.edgesSet         = new char[ maxECount ];
      _sNodes.setCornerTable( coarseRootData , &tree , sDepth , threads );
      coarseRootData.cornerValues     = new            Real[ coarseRootData.cCount ];
      coarseRootData.cornerNormals    = new Point3D< Real >[ coarseRootData.cCount ];
      coarseRootData.cornerValuesSet  = new            char[ coarseRootData.cCount ];
      coarseRootData.cornerNormalsSet = new            char[ coarseRootData.cCount ];
      memset( coarseRootData.cornerValuesSet  , 0 , sizeof( char ) * coarseRootData.cCount );
      memset( coarseRootData.cornerNormalsSet , 0 , sizeof( char ) * coarseRootData.cCount );
      MemoryUsage();

      std::vector< TreeOctNode::ConstNeighborKey3 > nKeys( threads );
      for( int t=0 ; t<threads ; t++ ) nKeys[t].set( maxDepth );
      TreeOctNode::ConstNeighborKey3 nKey;
      std::vector< TreeOctNode::ConstNeighborKey5 > nKeys5( threads );
      for( int t=0 ; t<threads ; t++ ) nKeys5[t].set( maxDepth );
      TreeOctNode::ConstNeighborKey5 nKey5;
      nKey5.set( maxDepth );
      nKey.set( maxDepth );
      // First process all leaf nodes at depths strictly finer than sDepth, one subtree at a time.
      for( int i=_sNodes.nodeCount[sDepth] ; i<_sNodes.nodeCount[sDepth+1] ; i++ )
      {
        if( !_sNodes.treeNodes[i]->children ) continue;

        _sNodes.setCornerTable( rootData , _sNodes.treeNodes[i] , threads );
        _sNodes.setEdgeTable  ( rootData , _sNodes.treeNodes[i] , threads );
        memset( rootData.cornerValuesSet  , 0 , sizeof( char ) * rootData.cCount );
        memset( rootData.cornerNormalsSet , 0 , sizeof( char ) * rootData.cCount );
        memset( rootData.edgesSet         , 0 , sizeof( char ) * rootData.eCount );
        interiorPoints = new std::vector< Point3D< float > >();
        for( int d=maxDepth ; d>sDepth ; d-- )
        {
          int leafNodeCount = 0;
          std::vector< TreeOctNode* > leafNodes;
          for( TreeOctNode* node=_sNodes.treeNodes[i]->nextLeaf() ; node ; node=_sNodes.treeNodes[i]->nextLeaf( node ) ) if( node->d==d ) leafNodeCount++;
          leafNodes.reserve( leafNodeCount );
          for( TreeOctNode* node=_sNodes.treeNodes[i]->nextLeaf() ; node ; node=_sNodes.treeNodes[i]->nextLeaf( node ) ) if( node->d==d ) leafNodes.push_back( node );
          Stencil< double , 3 > stencil1[8] , stencil2[8][8];
          SetEvaluationStencils( d , stencil1 , stencil2 );

          // First set the corner values and associated marching-cube indices
#pragma omp parallel for num_threads( threads )
          for( int t=0 ; t<threads ; t++ ) for( int i=(leafNodeCount*t)/threads ; i<(leafNodeCount*(t+1))/threads ; i++ )
          {
            TreeOctNode* leaf = leafNodes[i];
            SetIsoCorners( isoValue , leaf , rootData , rootData.cornerValuesSet , rootData.cornerValues , nKeys[t] , &metSolution[0] , stencil1 , stencil2 );

            // If this node shares a vertex with a coarser node, set the vertex value
            int d , off[3];
            leaf->depthAndOffset( d , off );
            int res = 1<<(d-sDepth);
            off[0] %= res , off[1] %=res , off[2] %= res;
            res--;
            if( !(off[0]%res) && !(off[1]%res) && !(off[2]%res) )
            {
              const TreeOctNode* temp = leaf;
              while( temp->d!=sDepth ) temp = temp->parent;
              int x = off[0]==0 ? 0 : 1 , y = off[1]==0 ? 0 : 1 , z = off[2]==0 ? 0 : 1;
              int c = Cube::CornerIndex( x , y , z );
              int idx = coarseRootData.cornerIndices( temp )[ c ];
              coarseRootData.cornerValues[ idx ] = rootData.cornerValues[ rootData.cornerIndices( leaf )[c] ];
              coarseRootData.cornerValuesSet[ idx ] = true;
            }

            // Compute the iso-vertices
            if( MarchingCubes::HasRoots( leaf->nodeData.mcIndex ) )
              SetMCRootPositions( leaf , sDepth , isoValue , nKeys5[t] , rootData , interiorPoints , mesh , &metSolution[0] , nonLinearFit );
          }
          // Note that this should be broken off for multi-threading as
          // the SetMCRootPositions writes to interiorPoints (with lockupdateing)
          // while GetMCIsoTriangles reads from interiorPoints (without locking)
#if MISHA_DEBUG
          std::vector< Point3D< float > > barycenters;
          std::vector< Point3D< float > >* barycenterPtr = addBarycenter ? & barycenters : NULL;
#endif // MISHA_DEBUG
#pragma omp parallel for num_threads( threads )
          for( int t=0 ; t<threads ; t++ ) for( int i=(leafNodeCount*t)/threads ; i<(leafNodeCount*(t+1))/threads ; i++ )
          {
            TreeOctNode* leaf = leafNodes[i];
            if( MarchingCubes::HasRoots( leaf->nodeData.mcIndex ) )
#if MISHA_DEBUG
              GetMCIsoTriangles( leaf , mesh , rootData , interiorPoints , offSet , sDepth , polygonMesh , barycenterPtr );
#else // !MISHA_DEBUG
              GetMCIsoTriangles( leaf , mesh , rootData , interiorPoints , offSet , sDepth , addBarycenter , polygonMesh );
#endif // MISHA_DEBUG
          }
#if MISHA_DEBUG
          for( int i=0 ; i<barycenters.size() ; i++ ) interiorPoints->push_back( barycenters[i] );
#endif // MISHA_DEBUG
        }
        offSet = mesh->outOfCorePointCount();
#if 1
        delete interiorPoints;
#endif
      }
      MemoryUsage();
      delete[] rootData.cornerValues , delete[] rootData.cornerNormals , rootData.cornerValues = NULL , rootData.cornerNormals = NULL;
      delete[] rootData.cornerValuesSet  , delete[] rootData.cornerNormalsSet , rootData.cornerValuesSet = NULL , rootData.cornerNormalsSet = NULL;
      delete[] rootData.interiorRoots ; rootData.interiorRoots = NULL;
      delete[] rootData.edgesSet ; rootData.edgesSet = NULL;
      coarseRootData.interiorRoots = NULL;
      coarseRootData.boundaryValues = rootData.boundaryValues;
      for( poisson::hash_map< long long , int >::iterator iter=rootData.boundaryRoots.begin() ; iter!=rootData.boundaryRoots.end() ; iter++ )
        coarseRootData.boundaryRoots[iter->first] = iter->second;

      for( int d=sDepth ; d>=0 ; d-- )
      {
        Stencil< double , 3 > stencil1[8] , stencil2[8][8];
        SetEvaluationStencils( d , stencil1 , stencil2 );
#if MISHA_DEBUG
        std::vector< Point3D< float > > barycenters;
        std::vector< Point3D< float > >* barycenterPtr = addBarycenter ? &barycenters : NULL;
#endif // MISHA_DEBUG
        for( int i=_sNodes.nodeCount[d] ; i<_sNodes.nodeCount[d+1] ; i++ )
        {
          TreeOctNode* leaf = _sNodes.treeNodes[i];
          if( leaf->children ) continue;

          // First set the corner values and associated marching-cube indices
          SetIsoCorners( isoValue , leaf , coarseRootData , coarseRootData.cornerValuesSet , coarseRootData.cornerValues , nKey , &metSolution[0] , stencil1 , stencil2 );

          // Now compute the iso-vertices
          if( MarchingCubes::HasRoots( leaf->nodeData.mcIndex ) )
          {
            SetMCRootPositions( leaf , 0 , isoValue , nKey5 , coarseRootData , NULL , mesh , &metSolution[0] , nonLinearFit );
#if MISHA_DEBUG
            GetMCIsoTriangles( leaf , mesh , coarseRootData , NULL , 0 , 0 , polygonMesh , barycenterPtr );
#else // !MISHA_DEBUG
            GetMCIsoTriangles( leaf , mesh , coarseRootData , NULL , 0 , 0 , addBarycenter , polygonMesh );
#endif // MISHA_DEBUG
          }
        }
      }
      MemoryUsage();

      delete[] coarseRootData.cornerValues , delete[] coarseRootData.cornerNormals;
      delete[] coarseRootData.cornerValuesSet  , delete[] coarseRootData.cornerNormalsSet;
      delete rootData.boundaryValues;
    }
    template<int Degree>
    Real Octree<Degree>::getCenterValue( const OctNode< TreeNodeData , Real >::ConstNeighborKey3& neighborKey , const TreeOctNode* node){
      int idx[3];
      Real value=0;

      VertexData::CenterIndex(node,fData.depth,idx);
      idx[0]*=fData.functionCount;
      idx[1]*=fData.functionCount;
      idx[2]*=fData.functionCount;
      int minDepth = std::max< int >( 0 , std::min< int >( _minDepth , node->depth()-1 ) );
      for( int i=minDepth ; i<=node->depth() ; i++ )
        for(int j=0;j<3;j++)
          for(int k=0;k<3;k++)
            for(int l=0;l<3;l++)
            {
              const TreeOctNode* n=neighborKey.neighbors[i].neighbors[j][k][l];
              if( n )
              {
                Real temp=n->nodeData.solution;
                value+=temp*Real(
                         fData.valueTables[idx[0]+int(n->off[0])]*
                         fData.valueTables[idx[1]+int(n->off[1])]*
                         fData.valueTables[idx[2]+int(n->off[2])]);
              }
            }
      if(node->children)
      {
        for(int i=0;i<Cube::CORNERS;i++){
          int ii=Cube::AntipodalCornerIndex(i);
          const TreeOctNode* n=&node->children[i];
          while(1){
            value+=n->nodeData.solution*Real(
                     fData.valueTables[idx[0]+int(n->off[0])]*
                     fData.valueTables[idx[1]+int(n->off[1])]*
                     fData.valueTables[idx[2]+int(n->off[2])]);
            if( n->children ) n=&n->children[ii];
            else break;
          }
        }
      }
      return value;
    }
    template< int Degree >
    Real Octree< Degree >::getCornerValue( const OctNode< TreeNodeData , Real >::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int corner , const Real* metSolution )
    {
      int idx[3];
      double value = 0;

      VertexData::CornerIndex( node , corner , fData.depth , idx );
      idx[0] *= fData.functionCount;
      idx[1] *= fData.functionCount;
      idx[2] *= fData.functionCount;

      int d = node->depth();
      int cx , cy , cz;
      int startX = 0 , endX = 3 , startY = 0 , endY = 3 , startZ = 0 , endZ = 3;
      Cube::FactorCornerIndex( corner , cx , cy , cz );
      {
        TreeOctNode::ConstNeighbors3& neighbors = neighborKey3.neighbors[d];
        if( cx==0 ) endX = 2;
        else      startX = 1;
        if( cy==0 ) endY = 2;
        else      startY = 1;
        if( cz==0 ) endZ = 2;
        else      startZ = 1;
        for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
        {
          const TreeOctNode* n=neighbors.neighbors[x][y][z];
          if( n )
          {
            double v =
                fData.valueTables[ idx[0]+int(n->off[0]) ]*
                fData.valueTables[ idx[1]+int(n->off[1]) ]*
                fData.valueTables[ idx[2]+int(n->off[2]) ];
            value += n->nodeData.solution * v;
          }
        }
      }
      if( d>0 && d>_minDepth )
      {
        int _corner = int( node - node->parent->children );
        int _cx , _cy , _cz;
        Cube::FactorCornerIndex( _corner , _cx , _cy , _cz );
        if( cx!=_cx ) startX = 0 , endX = 3;
        if( cy!=_cy ) startY = 0 , endY = 3;
        if( cz!=_cz ) startZ = 0 , endZ = 3;
        TreeOctNode::ConstNeighbors3& neighbors = neighborKey3.neighbors[d-1];
        for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
        {
          const TreeOctNode* n=neighbors.neighbors[x][y][z];
          if( n )
          {
            double v =
                fData.valueTables[ idx[0]+int(n->off[0]) ]*
                fData.valueTables[ idx[1]+int(n->off[1]) ]*
                fData.valueTables[ idx[2]+int(n->off[2]) ];
            value += metSolution[ n->nodeData.nodeIndex ] * v;
          }
        }
      }
      return Real( value );
    }
    template< int Degree >
    Real Octree< Degree >::getCornerValue( const OctNode< TreeNodeData , Real >::ConstNeighborKey3& neighborKey3 , const TreeOctNode* node , int corner , const Real* metSolution , const double stencil1[3][3][3] , const double stencil2[3][3][3] )
    {
      double value = 0;
      int d = node->depth();
      int cx , cy , cz;
      int startX = 0 , endX = 3 , startY = 0 , endY = 3 , startZ = 0 , endZ = 3;
      Cube::FactorCornerIndex( corner , cx , cy , cz );
      {
        TreeOctNode::ConstNeighbors3& neighbors = neighborKey3.neighbors[d];
        if( cx==0 ) endX = 2;
        else      startX = 1;
        if( cy==0 ) endY = 2;
        else      startY = 1;
        if( cz==0 ) endZ = 2;
        else      startZ = 1;
        for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
        {
          const TreeOctNode* n=neighbors.neighbors[x][y][z];
          if( n ) value += n->nodeData.solution * stencil1[x][y][z];
        }
      }
      if( d>0 && d>_minDepth )
      {
        int _corner = int( node - node->parent->children );
        int _cx , _cy , _cz;
        Cube::FactorCornerIndex( _corner , _cx , _cy , _cz );
        if( cx!=_cx ) startX = 0 , endX = 3;
        if( cy!=_cy ) startY = 0 , endY = 3;
        if( cz!=_cz ) startZ = 0 , endZ = 3;
        TreeOctNode::ConstNeighbors3& neighbors = neighborKey3.neighbors[d-1];
        for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
        {
          const TreeOctNode* n=neighbors.neighbors[x][y][z];
          if( n ) value += metSolution[ n->nodeData.nodeIndex ] * stencil2[x][y][z];
        }
      }
      return Real( value );
    }
    template< int Degree >
    Point3D< Real > Octree< Degree >::getCornerNormal( const OctNode< TreeNodeData , Real >::ConstNeighborKey5& neighborKey5 , const TreeOctNode* node , int corner , const Real* metSolution )
    {
      int idx[3];
      Point3D< Real > normal;
      normal[0] = normal[1] = normal[2] = 0.;

      VertexData::CornerIndex( node , corner , fData.depth , idx );
      idx[0] *= fData.functionCount;
      idx[1] *= fData.functionCount;
      idx[2] *= fData.functionCount;

      int d = node->depth();
      // Iterate over all ancestors that can overlap the corner
      {
        TreeOctNode::ConstNeighbors5& neighbors = neighborKey5.neighbors[d];
        for( int j=0 ; j<5 ; j++ ) for( int k=0 ; k<5 ; k++ ) for( int l=0 ; l<5 ; l++ )
        {
          const TreeOctNode* n=neighbors.neighbors[j][k][l];
          if( n )
          {
            int _idx[] = { idx[0] + n->off[0] , idx[1] + n->off[1] , idx[2] + n->off[2] };
            double  values[] = {  fData.valueTables[_idx[0]] ,  fData.valueTables[_idx[1]] ,  fData.valueTables[_idx[2]] };
            double dValues[] = { fData.dValueTables[_idx[0]] , fData.dValueTables[_idx[1]] , fData.dValueTables[_idx[2]] };
            Real solution = n->nodeData.solution;
            normal[0] += Real( dValues[0] *  values[1] *  values[2] * solution );
            normal[1] += Real(  values[0] * dValues[1] *  values[2] * solution );
            normal[2] += Real(  values[0] *  values[1] * dValues[2] * solution );
          }
        }
      }
      if( d>0 && d>_minDepth )
      {
        TreeOctNode::ConstNeighbors5& neighbors = neighborKey5.neighbors[d-1];
        for( int j=0 ; j<5 ; j++ ) for( int k=0 ; k<5 ; k++ ) for( int l=0 ; l<5 ; l++ )
        {
          const TreeOctNode* n=neighbors.neighbors[j][k][l];
          if( n )
          {
            int _idx[] = { idx[0] + n->off[0] , idx[1] + n->off[1] , idx[2] + n->off[2] };
            double  values[] = {  fData.valueTables[_idx[0]] ,  fData.valueTables[_idx[1]] ,  fData.valueTables[_idx[2]] };
            double dValues[] = { fData.dValueTables[_idx[0]] , fData.dValueTables[_idx[1]] , fData.dValueTables[_idx[2]] };
            Real solution = metSolution[ n->nodeData.nodeIndex ];
            normal[0] += Real( dValues[0] *  values[1] *  values[2] * solution );
            normal[1] += Real(  values[0] * dValues[1] *  values[2] * solution );
            normal[2] += Real(  values[0] *  values[1] * dValues[2] * solution );
          }
        }
      }
      return normal;
    }
    template< int Degree >
    Real Octree<Degree>::GetIsoValue( void )
    {
      Real isoValue , weightSum;

      neighborKey2.set( fData.depth );
      fData.setValueTables( fData.VALUE_FLAG , 0 );

      isoValue = weightSum = 0;
#pragma omp parallel for num_threads( threads ) reduction( + : isoValue , weightSum )
      for( int t=0 ; t<threads ; t++)
      {
        TreeOctNode::ConstNeighborKey3 nKey;
        nKey.set( _sNodes.maxDepth-1 );
        int nodeCount = _sNodes.nodeCount[ _sNodes.maxDepth ];
        for( int i=(nodeCount*t)/threads ; i<(nodeCount*(t+1))/threads ; i++ )
        {
          TreeOctNode* temp = _sNodes.treeNodes[i];
          nKey.getNeighbors( temp );
          Real w = temp->nodeData.centerWeightContribution;
          if( w!=0 )
          {
            isoValue += getCenterValue( nKey , temp ) * w;
            weightSum += w;
          }
        }
      }
      return isoValue/weightSum;
    }

    template< int Degree >
    void Octree< Degree >::SetIsoCorners( Real isoValue , TreeOctNode* leaf , SortedTreeNodes::CornerTableData& cData , char* valuesSet , Real* values , TreeOctNode::ConstNeighborKey3& nKey , const Real* metSolution , const Stencil< double , 3 > stencil1[8] , const Stencil< double , 3 > stencil2[8][8] )
    {
      Real cornerValues[ Cube::CORNERS ];
      const SortedTreeNodes::CornerIndices& cIndices = cData[ leaf ];

      bool isInterior;
      int d , off[3];
      leaf->depthAndOffset( d , off );
      int mn = 2 , mx = (1<<d)-2;
      isInterior = ( off[0]>=mn && off[0]<mx && off[1]>=mn && off[1]<mx && off[2]>=mn && off[2]<mx );
      nKey.getNeighbors( leaf );
      for( int c=0 ; c<Cube::CORNERS ; c++ )
      {
        int vIndex = cIndices[c];
        if( valuesSet[vIndex] ) cornerValues[c] = values[vIndex];
        else
        {
          if( isInterior ) cornerValues[c] = getCornerValue( nKey , leaf , c , metSolution , stencil1[c].values , stencil2[int(leaf - leaf->parent->children)][c].values );
          else             cornerValues[c] = getCornerValue( nKey , leaf , c , metSolution );
          values[vIndex] = cornerValues[c];
          valuesSet[vIndex] = 1;
        }
      }

      leaf->nodeData.mcIndex = MarchingCubes::GetIndex( cornerValues , isoValue );

      // Set the marching cube indices for all interior nodes.
      if( leaf->parent )
      {
        TreeOctNode* parent = leaf->parent;
        int c = int( leaf - leaf->parent->children );
        int mcid = leaf->nodeData.mcIndex & (1<<MarchingCubes::cornerMap()[c]);

        if( mcid )
        {
          poisson::atomicOr(parent->nodeData.mcIndex, mcid);

          while( 1 )
          {
            if( parent->parent && parent->parent->d>=_minDepth && (parent-parent->parent->children)==c )
            {
              poisson::atomicOr(parent->parent->nodeData.mcIndex, mcid);
              parent = parent->parent;
            }
            else break;
          }
        }
      }
    }


    template<int Degree>
    int Octree<Degree>::InteriorFaceRootCount(const TreeOctNode* node,const int &faceIndex,int maxDepth){
      int c1,c2,e1,e2,dir,off,cnt=0;
      int corners[Cube::CORNERS/2];
      if(node->children){
        Cube::FaceCorners(faceIndex,corners[0],corners[1],corners[2],corners[3]);
        Cube::FactorFaceIndex(faceIndex,dir,off);
        c1=corners[0];
        c2=corners[3];
        switch(dir){
        case 0:
          e1=Cube::EdgeIndex(1,off,1);
          e2=Cube::EdgeIndex(2,off,1);
          break;
        case 1:
          e1=Cube::EdgeIndex(0,off,1);
          e2=Cube::EdgeIndex(2,1,off);
          break;
        case 2:
          e1=Cube::EdgeIndex(0,1,off);
          e2=Cube::EdgeIndex(1,1,off);
          break;
        };
        cnt+=EdgeRootCount(&node->children[c1],e1,maxDepth)+EdgeRootCount(&node->children[c1],e2,maxDepth);
        switch(dir){
        case 0:
          e1=Cube::EdgeIndex(1,off,0);
          e2=Cube::EdgeIndex(2,off,0);
          break;
        case 1:
          e1=Cube::EdgeIndex(0,off,0);
          e2=Cube::EdgeIndex(2,0,off);
          break;
        case 2:
          e1=Cube::EdgeIndex(0,0,off);
          e2=Cube::EdgeIndex(1,0,off);
          break;
        };
        cnt+=EdgeRootCount(&node->children[c2],e1,maxDepth)+EdgeRootCount(&node->children[c2],e2,maxDepth);
        for(int i=0;i<Cube::CORNERS/2;i++){if(node->children[corners[i]].children){cnt+=InteriorFaceRootCount(&node->children[corners[i]],faceIndex,maxDepth);}}
      }
      return cnt;
    }

    template<int Degree>
    int Octree<Degree>::EdgeRootCount(const TreeOctNode* node,int edgeIndex,int maxDepth){
      int f1,f2,c1,c2;
      const TreeOctNode* temp;
      Cube::FacesAdjacentToEdge(edgeIndex,f1,f2);

      int eIndex;
      const TreeOctNode* finest=node;
      eIndex=edgeIndex;
      if(node->depth()<maxDepth){
        temp=node->faceNeighbor(f1);
        if(temp && temp->children){
          finest=temp;
          eIndex=Cube::FaceReflectEdgeIndex(edgeIndex,f1);
        }
        else{
          temp=node->faceNeighbor(f2);
          if(temp && temp->children){
            finest=temp;
            eIndex=Cube::FaceReflectEdgeIndex(edgeIndex,f2);
          }
          else{
            temp=node->edgeNeighbor(edgeIndex);
            if(temp && temp->children){
              finest=temp;
              eIndex=Cube::EdgeReflectEdgeIndex(edgeIndex);
            }
          }
        }
      }

      Cube::EdgeCorners(eIndex,c1,c2);
      if(finest->children) return EdgeRootCount(&finest->children[c1],eIndex,maxDepth)+EdgeRootCount(&finest->children[c2],eIndex,maxDepth);
      else return MarchingCubes::HasEdgeRoots(finest->nodeData.mcIndex,eIndex);
    }
    template<int Degree>
    int Octree<Degree>::IsBoundaryFace(const TreeOctNode* node,int faceIndex,int subdivideDepth){
      int dir,offset,d,o[3],idx;

      if(subdivideDepth<0){return 0;}
      if(node->d<=subdivideDepth){return 1;}
      Cube::FactorFaceIndex(faceIndex,dir,offset);
      node->depthAndOffset(d,o);

      idx=(int(o[dir])<<1) + (offset<<1);
      return !(idx%(2<<(int(node->d)-subdivideDepth)));
    }
    template<int Degree>
    int Octree<Degree>::IsBoundaryEdge(const TreeOctNode* node,int edgeIndex,int subdivideDepth){
      int dir,x,y;
      Cube::FactorEdgeIndex(edgeIndex,dir,x,y);
      return IsBoundaryEdge(node,dir,x,y,subdivideDepth);
    }
    template<int Degree>
    int Octree<Degree>::IsBoundaryEdge( const TreeOctNode* node , int dir , int x , int y , int subdivideDepth )
    {
      int d , o[3] , idx1 , idx2 , mask;

      if( subdivideDepth<0 ) return 0;
      if( node->d<=subdivideDepth ) return 1;
      node->depthAndOffset( d , o );

      switch( dir )
      {
      case 0:
        idx1 = o[1] + x;
        idx2 = o[2] + y;
        break;
      case 1:
        idx1 = o[0] + x;
        idx2 = o[2] + y;
        break;
      case 2:
        idx1 = o[0] + x;
        idx2 = o[1] + y;
        break;
      }
      mask = 1<<( int(node->d) - subdivideDepth );
      return !(idx1%(mask)) || !(idx2%(mask));
    }
    template< int Degree >
    void Octree< Degree >::GetRootSpan( const RootInfo& ri , Point3D< float >& start , Point3D< float >& end )
    {
      int o , i1 , i2;
      Real width;
      Point3D< Real > c;

      Cube::FactorEdgeIndex( ri.edgeIndex , o , i1 , i2 );
      ri.node->centerAndWidth( c , width );
      switch(o)
      {
      case 0:
        start[0]          = c[0] - width/2;
        end  [0]          = c[0] + width/2;
        start[1] = end[1] = c[1] - width/2 + width*i1;
        start[2] = end[2] = c[2] - width/2 + width*i2;
        break;
      case 1:
        start[0] = end[0] = c[0] - width/2 + width*i1;
        start[1]          = c[1] - width/2;
        end  [1]          = c[1] + width/2;
        start[2] = end[2] = c[2] - width/2 + width*i2;
        break;
      case 2:
        start[0] = end[0] = c[0] - width/2 + width*i1;
        start[1] = end[1] = c[1] - width/2 + width*i2;
        start[2]          = c[2] - width/2;
        end  [2]          = c[2] + width/2;
        break;
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////
    // The assumption made when calling this code is that the edge has at most one root //
    //////////////////////////////////////////////////////////////////////////////////////
    template< int Degree >
    int Octree< Degree >::GetRoot( const RootInfo& ri , Real isoValue , TreeOctNode::ConstNeighborKey5& neighborKey5 , Point3D< Real > & position , RootData& rootData , int sDepth , const Real* metSolution , int nonLinearFit )
    {
      if( !MarchingCubes::HasRoots( ri.node->nodeData.mcIndex ) ) return 0;
      int c1 , c2;
      Cube::EdgeCorners( ri.edgeIndex , c1 , c2 );
      if( !MarchingCubes::HasEdgeRoots( ri.node->nodeData.mcIndex , ri.edgeIndex ) ) return 0;

      long long key1 , key2;
      Point3D< Real > n[2];

      int i , o , i1 , i2 , rCount=0;
      Polynomial<2> P;
      std::vector< double > roots;
      double x0 , x1;
      Real center , width;
      Real averageRoot=0;
      Cube::FactorEdgeIndex( ri.edgeIndex , o , i1 , i2 );
      int idx1[3] , idx2[3];
      key1 = VertexData::CornerIndex( ri.node , c1 , fData.depth , idx1 );
      key2 = VertexData::CornerIndex( ri.node , c2 , fData.depth , idx2 );

      bool isBoundary = ( IsBoundaryEdge( ri.node , ri.edgeIndex , sDepth )!=0 );
      bool haveKey1 , haveKey2;
      std::pair< Real , Point3D< Real > > keyValue1 , keyValue2;
      int iter1 , iter2;
      {
        iter1 = rootData.cornerIndices( ri.node )[ c1 ];
        iter2 = rootData.cornerIndices( ri.node )[ c2 ];
        keyValue1.first = rootData.cornerValues[iter1];
        keyValue2.first = rootData.cornerValues[iter2];
        if( isBoundary )
        {
#pragma omp critical (normal_hash_access)
          {
            haveKey1 = ( rootData.boundaryValues->find( key1 )!=rootData.boundaryValues->end() );
            haveKey2 = ( rootData.boundaryValues->find( key2 )!=rootData.boundaryValues->end() );
            if( haveKey1 ) keyValue1 = (*rootData.boundaryValues)[key1];
            if( haveKey2 ) keyValue2 = (*rootData.boundaryValues)[key2];
          }
        }
        else
        {
          haveKey1 = ( rootData.cornerNormalsSet[ iter1 ] != 0 );
          haveKey2 = ( rootData.cornerNormalsSet[ iter2 ] != 0 );
          keyValue1.first = rootData.cornerValues[iter1];
          keyValue2.first = rootData.cornerValues[iter2];
          if( haveKey1 ) keyValue1.second = rootData.cornerNormals[iter1];
          if( haveKey2 ) keyValue2.second = rootData.cornerNormals[iter2];
        }
      }
      if( !haveKey1 || !haveKey2 ) neighborKey5.getNeighbors( ri.node );
      if( !haveKey1 ) keyValue1.second = getCornerNormal( neighborKey5 , ri.node , c1 , metSolution );
      x0 = keyValue1.first;
      n[0] = keyValue1.second;

      if( !haveKey2 ) keyValue2.second = getCornerNormal( neighborKey5 , ri.node , c2 , metSolution );
      x1 = keyValue2.first;
      n[1] = keyValue2.second;

      if( !haveKey1 || !haveKey2 )
      {
        if( isBoundary )
        {
#pragma omp critical (normal_hash_access)
          {
            if( !haveKey1 ) (*rootData.boundaryValues)[key1] = keyValue1;
            if( !haveKey2 ) (*rootData.boundaryValues)[key2] = keyValue2;
          }
        }
        else
        {
          if( !haveKey1 ) rootData.cornerNormals[iter1] = keyValue1.second , rootData.cornerNormalsSet[ iter1 ] = 1;
          if( !haveKey2 ) rootData.cornerNormals[iter2] = keyValue2.second , rootData.cornerNormalsSet[ iter2 ] = 1;
        }
      }

      Point3D< Real > c;
      ri.node->centerAndWidth(c,width);
      center=c[o];
      for( i=0 ; i<DIMENSION ; i++ ) n[0][i] *= width , n[1][i] *= width;

      switch(o)
      {
      case 0:
        position[1] = c[1]-width/2+width*i1;
        position[2] = c[2]-width/2+width*i2;
        break;
      case 1:
        position[0] = c[0]-width/2+width*i1;
        position[2] = c[2]-width/2+width*i2;
        break;
      case 2:
        position[0] = c[0]-width/2+width*i1;
        position[1] = c[1]-width/2+width*i2;
        break;
      }
      double dx0,dx1;
      dx0 = n[0][o];
      dx1 = n[1][o];

      // The scaling will turn the Hermite Spline into a quadratic
      double scl=(x1-x0)/((dx1+dx0)/2);
      dx0 *= scl;
      dx1 *= scl;

      // Hermite Spline
      P.coefficients[0] = x0;
      P.coefficients[1] = dx0;
      P.coefficients[2] = 3*(x1-x0)-dx1-2*dx0;

      P.getSolutions( isoValue , roots , EPSILON );
      for( i=0 ; i<int(roots.size()) ; i++ )
        if( roots[i]>=0 && roots[i]<=1 )
        {
          averageRoot += Real( roots[i] );
          rCount++;
        }
      if( rCount && nonLinearFit ) averageRoot /= rCount;
      else					     averageRoot  = Real((x0-isoValue)/(x0-x1));
      if( averageRoot<0 || averageRoot>1 )
      {
        fprintf( stderr , "[WARNING] Bad average root: %f\n" , averageRoot );
        fprintf( stderr , "\t(%f %f) , (%f %f) (%f)\n" , x0 , x1 , dx0 , dx1 , isoValue );
        if( averageRoot<0 ) averageRoot = 0;
        if( averageRoot>1 ) averageRoot = 1;
      }
      position[o] = Real(center-width/2+width*averageRoot);
      return 1;
    }
    template< int Degree >
    int Octree< Degree >::GetRootIndex( const TreeOctNode* node , int edgeIndex , int maxDepth , int sDepth,RootInfo& ri )
    {
      int c1,c2,f1,f2;
      const TreeOctNode *temp,*finest;
      int finestIndex;

      Cube::FacesAdjacentToEdge(edgeIndex,f1,f2);

      finest=node;
      finestIndex=edgeIndex;
      if(node->depth()<maxDepth){
        if(IsBoundaryFace(node,f1,sDepth)){temp=NULL;}
        else{temp=node->faceNeighbor(f1);}
        if(temp && temp->children){
          finest=temp;
          finestIndex=Cube::FaceReflectEdgeIndex(edgeIndex,f1);
        }
        else{
          if(IsBoundaryFace(node,f2,sDepth)){temp=NULL;}
          else{temp=node->faceNeighbor(f2);}
          if(temp && temp->children){
            finest=temp;
            finestIndex=Cube::FaceReflectEdgeIndex(edgeIndex,f2);
          }
          else{
            if(IsBoundaryEdge(node,edgeIndex,sDepth)){temp=NULL;}
            else{temp=node->edgeNeighbor(edgeIndex);}
            if(temp && temp->children){
              finest=temp;
              finestIndex=Cube::EdgeReflectEdgeIndex(edgeIndex);
            }
          }
        }
      }

      Cube::EdgeCorners(finestIndex,c1,c2);
      if(finest->children){
        if		(GetRootIndex(&finest->children[c1],finestIndex,maxDepth,sDepth,ri))	{return 1;}
        else if	(GetRootIndex(&finest->children[c2],finestIndex,maxDepth,sDepth,ri))	{return 1;}
        else
        {
          fprintf( stderr , "[WARNING] Couldn't find root index with either child\n" );
          return 0;
        }
      }
      else
      {
        if( !(MarchingCubes::edgeMask()[finest->nodeData.mcIndex] & (1<<finestIndex)) )
        {
          fprintf( stderr , "[WARNING] Finest node does not have iso-edge\n" );
          return 0;
        }

        int o,i1,i2;
        Cube::FactorEdgeIndex(finestIndex,o,i1,i2);
        int d,off[3];
        finest->depthAndOffset(d,off);
        ri.node=finest;
        ri.edgeIndex=finestIndex;
        int eIndex[2],offset;
        offset=BinaryNode<Real>::Index( d , off[o] );
        switch(o)
        {
        case 0:
          eIndex[0]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[1],i1);
          eIndex[1]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[2],i2);
          break;
        case 1:
          eIndex[0]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[0],i1);
          eIndex[1]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[2],i2);
          break;
        case 2:
          eIndex[0]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[0],i1);
          eIndex[1]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[1],i2);
          break;
        }
        ri.key = (long long)(o) | (long long)(eIndex[0])<<5 | (long long)(eIndex[1])<<25 | (long long)(offset)<<45;
        return 1;
      }
    }
    template<int Degree>
    int Octree<Degree>::GetRootIndex( const TreeOctNode* node , int edgeIndex , int maxDepth , RootInfo& ri )
    {
      int c1,c2,f1,f2;
      const TreeOctNode *temp,*finest;
      int finestIndex;


      // The assumption is that the super-edge has a root along it.
      if(!(MarchingCubes::edgeMask()[node->nodeData.mcIndex] & (1<<edgeIndex))){return 0;}

      Cube::FacesAdjacentToEdge(edgeIndex,f1,f2);

      finest = node;
      finestIndex = edgeIndex;
      if( node->depth()<maxDepth && !node->children )
      {
        temp=node->faceNeighbor( f1 );
        if( temp && temp->children ) finest = temp , finestIndex = Cube::FaceReflectEdgeIndex( edgeIndex , f1 );
        else
        {
          temp = node->faceNeighbor( f2 );
          if( temp && temp->children ) finest = temp , finestIndex = Cube::FaceReflectEdgeIndex( edgeIndex , f2 );
          else
          {
            temp = node->edgeNeighbor( edgeIndex );
            if( temp && temp->children ) finest = temp , finestIndex = Cube::EdgeReflectEdgeIndex( edgeIndex );
          }
        }
      }

      Cube::EdgeCorners( finestIndex , c1 , c2 );
      if( finest->children )
      {
        if     ( GetRootIndex( finest->children + c1 , finestIndex , maxDepth , ri ) ) return 1;
        else if( GetRootIndex( finest->children + c2 , finestIndex , maxDepth , ri ) ) return 1;
        else
        {
          int d1 , off1[3] , d2 , off2[3];
          node->depthAndOffset( d1 , off1 );
          finest->depthAndOffset( d2 , off2 );
          fprintf( stderr , "[WARNING] Couldn't find root index with either child [%d] (%d %d %d) -> [%d] (%d %d %d) (%d %d)\n" , d1 , off1[0] , off1[1] , off1[2] , d2 , off2[0] , off2[1] , off2[2] , node->children!=NULL , finest->children!=NULL );
          printf( "\t" );
          for( int i=0 ; i<8 ; i++ ) if( node->nodeData.mcIndex & (1<<i) ) printf( "1" ); else printf( "0" );
          printf( "\t" );
          for( int i=0 ; i<8 ; i++ ) if( finest->nodeData.mcIndex & (1<<i) ) printf( "1" ); else printf( "0" );
          printf( "\n" );
          return 0;
        }
      }
      else
      {
        int o,i1,i2;
        Cube::FactorEdgeIndex(finestIndex,o,i1,i2);
        int d,off[3];
        finest->depthAndOffset(d,off);
        ri.node=finest;
        ri.edgeIndex=finestIndex;
        int offset,eIndex[2];
        offset = BinaryNode< Real >::CenterIndex( d , off[o] );
        switch(o){
        case 0:
          eIndex[0]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[1],i1);
          eIndex[1]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[2],i2);
          break;
        case 1:
          eIndex[0]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[0],i1);
          eIndex[1]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[2],i2);
          break;
        case 2:
          eIndex[0]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[0],i1);
          eIndex[1]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[1],i2);
          break;
        }
        ri.key= (long long)(o) | (long long)(eIndex[0])<<5 | (long long)(eIndex[1])<<25 | (long long)(offset)<<45;
        return 1;
      }
    }
    template<int Degree>
    int Octree<Degree>::GetRootPair(const RootInfo& ri,int maxDepth,RootInfo& pair){
      const TreeOctNode* node=ri.node;
      int c1,c2,c;
      Cube::EdgeCorners(ri.edgeIndex,c1,c2);
      while(node->parent){
        c=int(node-node->parent->children);
        if(c!=c1 && c!=c2){return 0;}
        if(!MarchingCubes::HasEdgeRoots(node->parent->nodeData.mcIndex,ri.edgeIndex)){
          if(c==c1){return GetRootIndex(&node->parent->children[c2],ri.edgeIndex,maxDepth,pair);}
          else{return GetRootIndex(&node->parent->children[c1],ri.edgeIndex,maxDepth,pair);}
        }
        node=node->parent;
      }
      return 0;

    }
    template<int Degree>
    int Octree< Degree >::GetRootIndex( const RootInfo& ri , RootData& rootData , CoredPointIndex& index )
    {
      long long key = ri.key;
      hash_map< long long , int >::iterator rootIter;
      rootIter = rootData.boundaryRoots.find( key );
      if( rootIter!=rootData.boundaryRoots.end() )
      {
        index.inCore = 1;
        index.index = rootIter->second;
        return 1;
      }
      else if( rootData.interiorRoots )
      {
        int eIndex = rootData.edgeIndices( ri.node )[ ri.edgeIndex ];
        if( rootData.edgesSet[ eIndex ] )
        {
          index.inCore = 0;
          index.index = rootData.interiorRoots[ eIndex ];
          return 1;
        }
      }
      return 0;
    }
    template< int Degree >
    int Octree< Degree >::SetMCRootPositions( TreeOctNode* node , int sDepth , Real isoValue , TreeOctNode::ConstNeighborKey5& neighborKey5 , RootData& rootData ,
                                              std::vector< Point3D< float > >* interiorPositions , CoredMeshData* mesh , const Real* metSolution , int nonLinearFit )
    {
      Point3D< Real > position;
      int eIndex;
      RootInfo ri;
      int count=0;
      if( !MarchingCubes::HasRoots( node->nodeData.mcIndex ) ) return 0;
      for( int i=0 ; i<DIMENSION ; i++ ) for( int j=0 ; j<2 ; j++ ) for( int k=0 ; k<2 ; k++ )
      {
        long long key;
        eIndex = Cube::EdgeIndex( i , j , k );
        if( GetRootIndex( node , eIndex , fData.depth , ri ) )
        {
          key = ri.key;
          if( !rootData.interiorRoots || IsBoundaryEdge( node , i , j , k , sDepth ) )
          {
            poisson::hash_map< long long , int >::iterator iter , end;
            // Check if the root has already been set
#pragma omp critical (boundary_roots_hash_access)
            {
              iter = rootData.boundaryRoots.find( key );
              end  = rootData.boundaryRoots.end();
            }
            if( iter==end )
            {
              // Get the root information
              GetRoot( ri , isoValue , neighborKey5 , position , rootData , sDepth , metSolution , nonLinearFit );
              // Add the root if it hasn't been added already
#pragma omp critical (boundary_roots_hash_access)
              {
                iter = rootData.boundaryRoots.find( key );
                end  = rootData.boundaryRoots.end();
                if( iter==end )
                {
                  mesh->inCorePoints.push_back( position );
                  rootData.boundaryRoots[key] = int( mesh->inCorePoints.size() ) - 1;
                }
              }
              if( iter==end ) count++;
            }
          }
          else
          {
            int nodeEdgeIndex = rootData.edgeIndices( ri.node )[ ri.edgeIndex ];
            if( !rootData.edgesSet[ nodeEdgeIndex ] )
            {
              // Get the root information
              GetRoot( ri , isoValue , neighborKey5 , position , rootData , sDepth , metSolution , nonLinearFit );
              // Add the root if it hasn't been added already
#pragma omp critical (add_point_access)
              {
                if( !rootData.edgesSet[ nodeEdgeIndex ] )
                {
                  rootData.interiorRoots[ nodeEdgeIndex ] = mesh->addOutOfCorePoint( position );
                  interiorPositions->push_back( position );
                  rootData.edgesSet[ nodeEdgeIndex ] = 1;
                  count++;
                }
              }
            }
          }
        }
      }
      return count;
    }
    template<int Degree>
    int Octree< Degree >::SetBoundaryMCRootPositions( int sDepth , Real isoValue , RootData& rootData , CoredMeshData* mesh , int nonLinearFit )
    {
      Point3D< Real > position;
      int i,j,k,eIndex,hits=0;
      RootInfo ri;
      int count=0;
      TreeOctNode* node;

      node = tree.nextLeaf();
      while( node )
      {
        if( MarchingCubes::HasRoots( node->nodeData.mcIndex ) )
        {
          hits=0;
          for( i=0 ; i<DIMENSION ; i++ )
            for( j=0 ; j<2 ; j++ )
              for( k=0 ; k<2 ; k++ )
                if( IsBoundaryEdge( node , i , j , k , sDepth ) )
                {
                  hits++;
                  long long key;
                  eIndex = Cube::EdgeIndex( i , j , k );
                  if( GetRootIndex( node , eIndex , fData.depth , ri ) )
                  {
                    key = ri.key;
                    if( rootData.boundaryRoots.find(key)==rootData.boundaryRoots.end() )
                    {
                      GetRoot( ri , isoValue , position , rootData , sDepth , nonLinearFit );
                      mesh->inCorePoints.push_back( position );
                      rootData.boundaryRoots[key] = int( mesh->inCorePoints.size() )-1;
                      count++;
                    }
                  }
                }
        }
        if( hits ) node=tree.nextLeaf(node);
        else node=tree.nextBranch(node);
      }
      return count;
    }
    template<int Degree>
    void Octree< Degree >::GetMCIsoEdges( TreeOctNode* node , int sDepth , std::vector< std::pair< RootInfo , RootInfo > >& edges )
    {
      TreeOctNode* temp;
      int count=0 , tris=0;
      int isoTri[ DIMENSION * MarchingCubes::MAX_TRIANGLES ];
      FaceEdgesFunction fef;
      int ref , fIndex;
      hash_map< long long , std::pair< RootInfo , int > >::iterator iter;
      hash_map< long long , std::pair< RootInfo , int > > vertexCount;

      fef.edges = &edges;
      fef.maxDepth = fData.depth;
      fef.vertexCount = &vertexCount;
      count = MarchingCubes::AddTriangleIndices( node->nodeData.mcIndex , isoTri );
      for( fIndex=0 ; fIndex<Cube::NEIGHBORS ; fIndex++ )
      {
        ref = Cube::FaceReflectFaceIndex( fIndex , fIndex );
        fef.fIndex = ref;
        temp = node->faceNeighbor( fIndex );
        // If the face neighbor exists and has higher resolution than the current node,
        // get the iso-curve from the neighbor
        if( temp && temp->children && !IsBoundaryFace( node , fIndex , sDepth ) ) temp->processNodeFaces( temp , &fef , ref );
        // Otherwise, get it from the node
        else
        {
          RootInfo ri1 , ri2;
          for( int j=0 ; j<count ; j++ )
            for( int k=0 ; k<3 ; k++ )
              if( fIndex==Cube::FaceAdjacentToEdges( isoTri[j*3+k] , isoTri[j*3+((k+1)%3)] ) )
                if( GetRootIndex( node , isoTri[j*3+k] , fData.depth , ri1 ) && GetRootIndex( node , isoTri[j*3+((k+1)%3)] , fData.depth , ri2 ) )
                {
                  long long key1 = ri1.key , key2 = ri2.key;
                  edges.push_back( std::pair< RootInfo , RootInfo >( ri1 , ri2 ) );
                  iter=vertexCount.find( key1 );
                  if( iter==vertexCount.end() )
                  {
                    vertexCount[key1].first = ri1;
                    vertexCount[key1].second = 0;
                  }
                  iter=vertexCount.find( key2 );
                  if( iter==vertexCount.end() )
                  {
                    vertexCount[key2].first = ri2;
                    vertexCount[key2].second = 0;
                  }
                  vertexCount[key1].second++;
                  vertexCount[key2].second--;
                }
                else
                {
                  int r1 = MarchingCubes::HasEdgeRoots( node->nodeData.mcIndex , isoTri[j*3+k] );
                  int r2 = MarchingCubes::HasEdgeRoots( node->nodeData.mcIndex , isoTri[j*3+((k+1)%3)] );
                  fprintf( stderr , "Bad Edge 2: %d %d\t%d %d\n" , ri1.key , ri2.key , r1 , r2 );
                }
        }
      }
      for( int i=0 ; i<int(edges.size()) ; i++ )
      {
        iter = vertexCount.find( edges[i].first.key );
        if( iter==vertexCount.end() ) printf( "Could not find vertex: %lld\n" , edges[i].first );
        else if( vertexCount[ edges[i].first.key ].second )
        {
          RootInfo ri;
          GetRootPair( vertexCount[edges[i].first.key].first , fData.depth , ri );
          long long key = ri.key;
          iter = vertexCount.find( key );
          if( iter==vertexCount.end() )
          {
            int d , off[3];
            node->depthAndOffset( d , off );
            printf( "Vertex pair not in list 1 (%lld) %d\t[%d] (%d %d %d)\n" , key , IsBoundaryEdge( ri.node , ri.edgeIndex , sDepth ) , d , off[0] , off[1] , off[2] );
          }
          else
          {
            edges.push_back( std::pair< RootInfo , RootInfo >( ri , edges[i].first ) );
            vertexCount[ key ].second++;
            vertexCount[ edges[i].first.key ].second--;
          }
        }

        iter = vertexCount.find( edges[i].second.key );
        if( iter==vertexCount.end() ) printf( "Could not find vertex: %lld\n" , edges[i].second );
        else if( vertexCount[edges[i].second.key].second )
        {
          RootInfo ri;
          GetRootPair( vertexCount[edges[i].second.key].first , fData.depth , ri );
          long long key = ri.key;
          iter=vertexCount.find( key );
          if( iter==vertexCount.end() )
          {
            int d , off[3];
            node->depthAndOffset( d , off );
            printf( "Vertex pair not in list 2\t[%d] (%d %d %d)\n" , d , off[0] , off[1] , off[2] );
          }
          else
          {
            edges.push_back( std::pair< RootInfo , RootInfo >( edges[i].second , ri ) );
            vertexCount[key].second--;
            vertexCount[ edges[i].second.key ].second++;
          }
        }
      }
    }
    template<int Degree>
#if MISHA_DEBUG
    int Octree< Degree >::GetMCIsoTriangles( TreeOctNode* node , CoredMeshData* mesh , RootData& rootData , std::vector< Point3D< float > >* interiorPositions , int offSet , int sDepth , bool polygonMesh , std::vector< Point3D< float > >* barycenters )
#else // !MISHA_DEBUG
    int Octree< Degree >::GetMCIsoTriangles( TreeOctNode* node , CoredMeshData* mesh , RootData& rootData , std::vector< Point3D< float > >* interiorPositions , int offSet , int sDepth , bool addBarycenter , bool polygonMesh )
#endif // MISHA_DEBUG
    {
      int tris=0;
      std::vector< std::pair< RootInfo , RootInfo > > edges;
      std::vector< std::vector< std::pair< RootInfo , RootInfo > > > edgeLoops;
      GetMCIsoEdges( node , sDepth , edges );

      GetEdgeLoops( edges , edgeLoops );
      for( int i=0 ; i<int(edgeLoops.size()) ; i++ )
      {
        CoredPointIndex p;
        std::vector<CoredPointIndex> edgeIndices;
        for( int j=0 ; j<int(edgeLoops[i].size()) ; j++ )
        {
          if( !GetRootIndex( edgeLoops[i][j].first , rootData , p ) ) printf( "Bad Point Index\n" );
          else edgeIndices.push_back( p );
        }
#if MISHA_DEBUG
        tris += AddTriangles( mesh , edgeIndices , interiorPositions , offSet , polygonMesh , barycenters );
#else // !MISHA_DEBUG
        tris += AddTriangles( mesh , edgeIndices , interiorPositions , offSet , addBarycenter , polygonMesh );
#endif // MISHA_DEBUG
      }
      return tris;
    }

    template< int Degree >
    int Octree< Degree >::GetEdgeLoops( std::vector< std::pair< RootInfo , RootInfo > >& edges , std::vector< std::vector< std::pair< RootInfo , RootInfo > > >& loops )
    {
      int loopSize=0;
      long long frontIdx , backIdx;
      std::pair< RootInfo , RootInfo > e , temp;
      loops.clear();

      while( edges.size() )
      {
        std::vector< std::pair< RootInfo ,  RootInfo > > front , back;
        e = edges[0];
        loops.resize( loopSize+1 );
        edges[0] = edges.back();
        edges.pop_back();
        frontIdx = e.second.key;
        backIdx = e.first.key;
        for( int j=int(edges.size())-1 ; j>=0 ; j-- )
        {
          if( edges[j].first.key==frontIdx || edges[j].second.key==frontIdx )
          {
            if( edges[j].first.key==frontIdx ) temp = edges[j];
            else temp.first = edges[j].second , temp.second = edges[j].first;
            frontIdx = temp.second.key;
            front.push_back(temp);
            edges[j] = edges.back();
            edges.pop_back();
            j = int(edges.size());
          }
          else if( edges[j].first.key==backIdx || edges[j].second.key==backIdx )
          {
            if( edges[j].second.key==backIdx ) temp = edges[j];
            else temp.first = edges[j].second , temp.second = edges[j].first;
            backIdx = temp.first.key;
            back.push_back(temp);
            edges[j] = edges.back();
            edges.pop_back();
            j = int(edges.size());
          }
        }
        for( int j=int(back.size())-1 ; j>=0 ; j-- ) loops[loopSize].push_back( back[j] );
        loops[loopSize].push_back(e);
        for( int j=0 ; j<int(front.size()) ; j++ ) loops[loopSize].push_back( front[j] );
        loopSize++;
      }
      return int(loops.size());
    }
    template<int Degree>
#if MISHA_DEBUG
    int Octree<Degree>::AddTriangles( CoredMeshData* mesh , std::vector<CoredPointIndex>& edges , std::vector<Point3D<float> >* interiorPositions , int offSet , bool polygonMesh , std::vector< Point3D< float > >* barycenters )
#else // !MISHA_DEBUG
    int Octree<Degree>::AddTriangles( CoredMeshData* mesh , std::vector<CoredPointIndex>& edges , std::vector<Point3D<float> >* interiorPositions , int offSet , bool addBarycenter , bool polygonMesh )
#endif // MISHA_DEBUG
    {
      MinimalAreaTriangulation< float > MAT;
      std::vector< Point3D< float > > vertices;
      std::vector< TriangleIndex > triangles;
      if( polygonMesh )
      {
        std::vector< CoredVertexIndex > vertices( edges.size() );
        for( int i=0 ; i<int(edges.size()) ; i++ )
        {
          vertices[i].idx    =  edges[i].index;
          vertices[i].inCore = (edges[i].inCore!=0);
        }
#pragma omp critical (add_polygon_access)
        {
          mesh->addPolygon( vertices );
        }
        return 1;
      }
      if( edges.size()>3 )
      {
        bool isCoplanar = false;

#if MISHA_DEBUG
        if( barycenters )
#else // !MISHA_DEBUG
        if( addBarycenter )
#endif // MISHA_DEBUG
          for( int i=0 ; i<int(edges.size()) ; i++ )
            for( int j=0 ; j<i ; j++ )
              if( (i+1)%edges.size()!=j && (j+1)%edges.size()!=i )
              {
                Point3D< Real > v1 , v2;
                if( edges[i].inCore ) v1 =   mesh->inCorePoints[ edges[i].index        ];
                else                  v1 = (*interiorPositions)[ edges[i].index-offSet ];
                if( edges[j].inCore ) v2 =   mesh->inCorePoints[ edges[j].index        ];
                else                  v2 = (*interiorPositions)[ edges[j].index-offSet ];
                for( int k=0 ; k<3 ; k++ ) if( v1[k]==v2[k] ) isCoplanar = true;
              }
        if( isCoplanar )
        {
          Point3D< Real > c;
          c[0] = c[1] = c[2] = 0;
          for( int i=0 ; i<int(edges.size()) ; i++ )
          {
            Point3D<Real> p;
            if(edges[i].inCore)	p =   mesh->inCorePoints[edges[i].index       ];
            else				p = (*interiorPositions)[edges[i].index-offSet];
            c += p;
          }
          c /= Real( edges.size() );
          int cIdx;
#pragma omp critical (add_point_access)
          {
            cIdx = mesh->addOutOfCorePoint( c );
#if MISHA_DEBUG
            barycenters->push_back( c );
#else // !MISHA_DEBUG
            interiorPositions->push_back( c );
#endif // MISHA_DEBUG
          }
          for( int i=0 ; i<int(edges.size()) ; i++ )
          {
            std::vector< CoredVertexIndex > vertices( 3 );
            vertices[0].idx = edges[i                 ].index;
            vertices[1].idx = edges[(i+1)%edges.size()].index;
            vertices[2].idx = cIdx;
            vertices[0].inCore = (edges[i                 ].inCore!=0);
            vertices[1].inCore = (edges[(i+1)%edges.size()].inCore!=0);
            vertices[2].inCore = 0;
#pragma omp critical (add_polygon_access)
            {
              mesh->addPolygon( vertices );
            }
          }
          return int( edges.size() );
        }
        else
        {
          vertices.resize( edges.size() );
          // Add the points
          for( int i=0 ; i<int(edges.size()) ; i++ )
          {
            Point3D< Real > p;
            if( edges[i].inCore ) p =   mesh->inCorePoints[edges[i].index       ];
            else                  p = (*interiorPositions)[edges[i].index-offSet];
            vertices[i] = p;
          }
          MAT.GetTriangulation( vertices , triangles );
          for( int i=0 ; i<int(triangles.size()) ; i++ )
          {
            std::vector< CoredVertexIndex > _vertices( 3 );
            for( int j=0 ; j<3 ; j++ )
            {
              _vertices[j].idx    =  edges[ triangles[i].idx[j] ].index;
              _vertices[j].inCore = (edges[ triangles[i].idx[j] ].inCore!=0);
            }
#pragma omp critical (add_polygon_access)
            {
              mesh->addPolygon( _vertices );
            }
          }
        }
      }
      else if( edges.size()==3 )
      {
        std::vector< CoredVertexIndex > vertices( 3 );
        for( int i=0 ; i<3 ; i++ )
        {
          vertices[i].idx    =  edges[i].index;
          vertices[i].inCore = (edges[i].inCore!=0);
        }
#pragma omp critical (add_polygon_access)
        mesh->addPolygon( vertices );
      }
      return int(edges.size())-2;
    }
    template< int Degree >
    Real* Octree< Degree >::GetSolutionGrid( int& res , float isoValue , int depth )
    {
      if( depth<=0 || depth>tree.maxDepth() ) depth = tree.maxDepth();
      BSplineData< Degree , Real > fData;
      fData.set( depth );
      fData.setValueTables( fData.VALUE_FLAG );
      res = 1<<depth;
      Real* values = new float[ res * res * res ];
      memset( values , 0 , sizeof( float ) * res  * res * res );

      for( TreeOctNode* n=tree.nextNode() ; n ; n=tree.nextNode( n ) )
      {
        if( n->d>depth ) continue;
        if( n->d<_minDepth ) continue;
        int d , idx[3] , start[3] , end[3];
        n->depthAndOffset( d , idx );
        for( int i=0 ; i<3 ; i++ )
        {
          // Get the index of the functions
          idx[i] = BinaryNode< double >::CenterIndex( d , idx[i] );
          // Figure out which samples fall into the range
          fData.setSampleSpan( idx[i] , start[i] , end[i] );
          // We only care about the odd indices
          if( !(start[i]&1) ) start[i]++;
          if( !(  end[i]&1) )   end[i]--;
        }
        Real coefficient = n->nodeData.solution;
        for( int x=start[0] ; x<=end[0] ; x+=2 )
          for( int y=start[1] ; y<=end[1] ; y+=2 )
            for( int z=start[2] ; z<=end[2] ; z+=2 )
            {
              int xx = (x-1)>>1 , yy = (y-1)>>1 , zz = (z-1)>>1;
              values[ zz*res*res + yy*res + xx ] +=
                  coefficient *
                  fData.valueTables[ idx[0]+x*fData.functionCount ] *
                  fData.valueTables[ idx[1]+y*fData.functionCount ] *
                  fData.valueTables[ idx[2]+z*fData.functionCount ];
            }
      }
      for( int i=0 ; i<res*res*res ; i++ ) values[i] -= isoValue;

      return values;
    }
    template< int Degree >
    Real* Octree< Degree >::GetWeightGrid( int& res , int depth )
    {
      if( depth<=0 || depth>tree.maxDepth() ) depth = tree.maxDepth();
      res = 1<<tree.maxDepth();
      Real* values = new float[ res * res * res ];
      memset( values , 0 , sizeof( float ) * res  * res * res );

      for( TreeOctNode* n=tree.nextNode() ; n ; n=tree.nextNode( n ) )
      {
        if( n->d>depth ) continue;
        int d , idx[3] , start[3] , end[3];
        n->depthAndOffset( d , idx );
        for( int i=0 ; i<3 ; i++ )
        {
          // Get the index of the functions
          idx[i] = BinaryNode< double >::CenterIndex( d , idx[i] );
          // Figure out which samples fall into the range
          fData.setSampleSpan( idx[i] , start[i] , end[i] );
          // We only care about the odd indices
          if( !(start[i]&1) ) start[i]++;
          if( !(  end[i]&1) )   end[i]--;
        }
        for( int x=start[0] ; x<=end[0] ; x+=2 )
          for( int y=start[1] ; y<=end[1] ; y+=2 )
            for( int z=start[2] ; z<=end[2] ; z+=2 )
            {
              int xx = (x-1)>>1 , yy = (y-1)>>1 , zz = (z-1)>>1;
              values[ zz*res*res + yy*res + xx ] +=
                  n->nodeData.centerWeightContribution *
                  fData.valueTables[ idx[0]+x*fData.functionCount ] *
                  fData.valueTables[ idx[1]+y*fData.functionCount ] *
                  fData.valueTables[ idx[2]+z*fData.functionCount ];
            }
      }
      return values;
    }

    ////////////////
    // VertexData //
    ////////////////
    long long VertexData::CenterIndex(const TreeOctNode* node,int maxDepth){
      int idx[DIMENSION];
      return CenterIndex(node,maxDepth,idx);
    }
    long long VertexData::CenterIndex(const TreeOctNode* node,int maxDepth,int idx[DIMENSION]){
      int d,o[3];
      node->depthAndOffset(d,o);
      for(int i=0;i<DIMENSION;i++){idx[i]=BinaryNode<Real>::CornerIndex(maxDepth+1,d+1,o[i]<<1,1);}
      return (long long)(idx[0]) | (long long)(idx[1])<<15 | (long long)(idx[2])<<30;
    }
    long long VertexData::CenterIndex(int depth,const int offSet[DIMENSION],int maxDepth,int idx[DIMENSION]){
      for(int i=0;i<DIMENSION;i++){idx[i]=BinaryNode<Real>::CornerIndex(maxDepth+1,depth+1,offSet[i]<<1,1);}
      return (long long)(idx[0]) | (long long)(idx[1])<<15 | (long long)(idx[2])<<30;
    }
    long long VertexData::CornerIndex(const TreeOctNode* node,int cIndex,int maxDepth){
      int idx[DIMENSION];
      return CornerIndex(node,cIndex,maxDepth,idx);
    }
    long long VertexData::CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth , int idx[DIMENSION] )
    {
      int x[DIMENSION];
      Cube::FactorCornerIndex( cIndex , x[0] , x[1] , x[2] );
      int d , o[3];
      node->depthAndOffset( d , o );
      for( int i=0 ; i<DIMENSION ; i++ ) idx[i] = BinaryNode<Real>::CornerIndex( maxDepth+1 , d , o[i] , x[i] );
      return CornerIndexKey( idx );
    }
    long long VertexData::CornerIndex( int depth , const int offSet[DIMENSION] , int cIndex , int maxDepth , int idx[DIMENSION] )
    {
      int x[DIMENSION];
      Cube::FactorCornerIndex( cIndex , x[0] , x[1] , x[2] );
      for( int i=0 ; i<DIMENSION ; i++ ) idx[i] = BinaryNode<Real>::CornerIndex( maxDepth+1 , depth , offSet[i] , x[i] );
      return CornerIndexKey( idx );
    }
    long long VertexData::CornerIndexKey( const int idx[DIMENSION] )
    {
      return (long long)(idx[0]) | (long long)(idx[1])<<15 | (long long)(idx[2])<<30;
    }
    long long VertexData::FaceIndex(const TreeOctNode* node,int fIndex,int maxDepth){
      int idx[DIMENSION];
      return FaceIndex(node,fIndex,maxDepth,idx);
    }
    long long VertexData::FaceIndex(const TreeOctNode* node,int fIndex,int maxDepth,int idx[DIMENSION]){
      int dir,offset;
      Cube::FactorFaceIndex(fIndex,dir,offset);
      int d,o[3];
      node->depthAndOffset(d,o);
      for(int i=0;i<DIMENSION;i++){idx[i]=BinaryNode<Real>::CornerIndex(maxDepth+1,d+1,o[i]<<1,1);}
      idx[dir]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,o[dir],offset);
      return (long long)(idx[0]) | (long long)(idx[1])<<15 | (long long)(idx[2])<<30;
    }
    long long VertexData::EdgeIndex(const TreeOctNode* node,int eIndex,int maxDepth){
      int idx[DIMENSION];
      return EdgeIndex(node,eIndex,maxDepth,idx);
    }
    long long VertexData::EdgeIndex(const TreeOctNode* node,int eIndex,int maxDepth,int idx[DIMENSION]){
      int o,i1,i2;
      int d,off[3];
      node->depthAndOffset(d,off);
      for(int i=0;i<DIMENSION;i++){idx[i]=BinaryNode<Real>::CornerIndex(maxDepth+1,d+1,off[i]<<1,1);}
      Cube::FactorEdgeIndex(eIndex,o,i1,i2);
      switch(o){
      case 0:
        idx[1]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[1],i1);
        idx[2]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[2],i2);
        break;
      case 1:
        idx[0]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[0],i1);
        idx[2]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[2],i2);
        break;
      case 2:
        idx[0]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[0],i1);
        idx[1]=BinaryNode<Real>::CornerIndex(maxDepth+1,d,off[1],i2);
        break;
      };
      return (long long)(idx[0]) | (long long)(idx[1])<<15 | (long long)(idx[2])<<30;
    }

    
  }
}


