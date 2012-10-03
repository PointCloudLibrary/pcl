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

#include <stdlib.h>
#include <math.h>
#include <algorithm>

/////////////
// OctNode //
/////////////

namespace pcl
{
  namespace poisson
  {

    template<class NodeData,class Real> const int OctNode<NodeData,Real>::DepthShift=5;
    template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetShift=19;
    template<class NodeData,class Real> const int OctNode<NodeData,Real>::DepthMask=(1<<DepthShift)-1;
    template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetMask=(1<<OffsetShift)-1;
    template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetShift1=DepthShift;
    template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetShift2=OffsetShift1+OffsetShift;
    template<class NodeData,class Real> const int OctNode<NodeData,Real>::OffsetShift3=OffsetShift2+OffsetShift;

    template<class NodeData,class Real> int OctNode<NodeData,Real>::UseAlloc=0;
    template<class NodeData,class Real> Allocator<OctNode<NodeData,Real> > OctNode<NodeData,Real>::internalAllocator;

    template<class NodeData,class Real>
    void OctNode<NodeData,Real>::SetAllocator(int blockSize)
    {
      if(blockSize>0)
      {
        UseAlloc=1;
        internalAllocator.set(blockSize);
      }
      else{UseAlloc=0;}
    }
    template<class NodeData,class Real>
    int OctNode<NodeData,Real>::UseAllocator(void){return UseAlloc;}

    template <class NodeData,class Real>
    OctNode<NodeData,Real>::OctNode(void){
      parent=children=NULL;
      d=off[0]=off[1]=off[2]=0;
    }

    template <class NodeData,class Real>
    OctNode<NodeData,Real>::~OctNode(void){
      if(!UseAlloc){if(children){delete[] children;}}
      parent=children=NULL;
    }
    template <class NodeData,class Real>
    void OctNode<NodeData,Real>::setFullDepth(int maxDepth){
      if( maxDepth )
      {
        if( !children ) initChildren();
        for( int i=0 ; i<8 ; i++ ) children[i].setFullDepth( maxDepth-1 );
      }
    }

    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::initChildren(void){
      int i,j,k;

      if(UseAlloc){children=internalAllocator.newElements(8);}
      else{
        if(children){delete[] children;}
        children=NULL;
        children=new OctNode[Cube::CORNERS];
      }
      if(!children){
        fprintf(stderr,"Failed to initialize children in OctNode::initChildren\n");
        exit(0);
        return 0;
      }
      int d,off[3];
      depthAndOffset(d,off);
      for(i=0;i<2;i++){
        for(j=0;j<2;j++){
          for(k=0;k<2;k++){
            int idx=Cube::CornerIndex(i,j,k);
            children[idx].parent=this;
            children[idx].children=NULL;
            int off2[3];
            off2[0]=(off[0]<<1)+i;
            off2[1]=(off[1]<<1)+j;
            off2[2]=(off[2]<<1)+k;
            Index(d+1,off2,children[idx].d,children[idx].off);
          }
        }
      }
      return 1;
    }
    template <class NodeData,class Real>
    inline void OctNode<NodeData,Real>::Index(int depth,const int offset[3],short& d,short off[3]){
      d=short(depth);
      off[0]=short((1<<depth)+offset[0]-1);
      off[1]=short((1<<depth)+offset[1]-1);
      off[2]=short((1<<depth)+offset[2]-1);
    }

    template<class NodeData,class Real>
    inline void OctNode<NodeData,Real>::depthAndOffset(int& depth,int offset[3]) const {
      depth=int(d);
      offset[0]=(int(off[0])+1)&(~(1<<depth));
      offset[1]=(int(off[1])+1)&(~(1<<depth));
      offset[2]=(int(off[2])+1)&(~(1<<depth));
    }
    template<class NodeData,class Real>
    inline int OctNode<NodeData,Real>::depth(void) const {return int(d);}
    template<class NodeData,class Real>
    inline void OctNode<NodeData,Real>::DepthAndOffset(const long long& index,int& depth,int offset[3]){
      depth=int(index&DepthMask);
      offset[0]=(int((index>>OffsetShift1)&OffsetMask)+1)&(~(1<<depth));
      offset[1]=(int((index>>OffsetShift2)&OffsetMask)+1)&(~(1<<depth));
      offset[2]=(int((index>>OffsetShift3)&OffsetMask)+1)&(~(1<<depth));
    }
    template<class NodeData,class Real>
    inline int OctNode<NodeData,Real>::Depth(const long long& index){return int(index&DepthMask);}
    template <class NodeData,class Real>
    void OctNode<NodeData,Real>::centerAndWidth(Point3D<Real>& center,Real& width) const{
      int depth,offset[3];
      depth=int(d);
      offset[0]=(int(off[0])+1)&(~(1<<depth));
      offset[1]=(int(off[1])+1)&(~(1<<depth));
      offset[2]=(int(off[2])+1)&(~(1<<depth));
      width=Real(1.0/(1<<depth));
      for(int dim=0;dim<DIMENSION;dim++){center.coords[dim]=Real(0.5+offset[dim])*width;}
    }
    template< class NodeData , class Real >
    bool OctNode< NodeData , Real >::isInside( Point3D< Real > p ) const
    {
      Point3D< Real > c;
      Real w;
      centerAndWidth( c , w );
      w /= 2;
      return (c[0]-w)<p[0] && p[0]<=(c[0]+w) && (c[1]-w)<p[1] && p[1]<=(c[1]+w) && (c[2]-w)<p[2] && p[2]<=(c[2]+w);
    }
    template <class NodeData,class Real>
    inline void OctNode<NodeData,Real>::CenterAndWidth(const long long& index,Point3D<Real>& center,Real& width){
      int depth,offset[3];
      depth=index&DepthMask;
      offset[0]=(int((index>>OffsetShift1)&OffsetMask)+1)&(~(1<<depth));
      offset[1]=(int((index>>OffsetShift2)&OffsetMask)+1)&(~(1<<depth));
      offset[2]=(int((index>>OffsetShift3)&OffsetMask)+1)&(~(1<<depth));
      width=Real(1.0/(1<<depth));
      for(int dim=0;dim<DIMENSION;dim++){center.coords[dim]=Real(0.5+offset[dim])*width;}
    }

    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::maxDepth(void) const{
      if(!children){return 0;}
      else{
        int c,d;
        for(int i=0;i<Cube::CORNERS;i++){
          d=children[i].maxDepth();
          if(!i || d>c){c=d;}
        }
        return c+1;
      }
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::nodes(void) const{
      if(!children){return 1;}
      else{
        int c=0;
        for(int i=0;i<Cube::CORNERS;i++){c+=children[i].nodes();}
        return c+1;
      }
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::leaves(void) const{
      if(!children){return 1;}
      else{
        int c=0;
        for(int i=0;i<Cube::CORNERS;i++){c+=children[i].leaves();}
        return c;
      }
    }
    template<class NodeData,class Real>
    int OctNode<NodeData,Real>::maxDepthLeaves(int maxDepth) const{
      if(depth()>maxDepth){return 0;}
      if(!children){return 1;}
      else{
        int c=0;
        for(int i=0;i<Cube::CORNERS;i++){c+=children[i].maxDepthLeaves(maxDepth);}
        return c;
      }
    }
    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::root(void) const{
      const OctNode* temp=this;
      while(temp->parent){temp=temp->parent;}
      return temp;
    }


    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::nextBranch( const OctNode* current ) const
    {
      if( !current->parent || current==this ) return NULL;
      if(current-current->parent->children==Cube::CORNERS-1) return nextBranch( current->parent );
      else return current+1;
    }
    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::nextBranch(OctNode* current){
      if(!current->parent || current==this){return NULL;}
      if(current-current->parent->children==Cube::CORNERS-1){return nextBranch(current->parent);}
      else{return current+1;}
    }
    template< class NodeData , class Real >
    const OctNode< NodeData , Real >* OctNode< NodeData , Real >::prevBranch( const OctNode* current ) const
    {
      if( !current->parent || current==this ) return NULL;
      if( current-current->parent->children==0 ) return prevBranch( current->parent );
      else return current-1;
    }
    template< class NodeData , class Real >
    OctNode< NodeData , Real >* OctNode< NodeData , Real >::prevBranch( OctNode* current )
    {
      if( !current->parent || current==this ) return NULL;
      if( current-current->parent->children==0 ) return prevBranch( current->parent );
      else return current-1;
    }
    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::nextLeaf(const OctNode* current) const{
      if(!current){
        const OctNode<NodeData,Real>* temp=this;
        while(temp->children){temp=&temp->children[0];}
        return temp;
      }
      if(current->children){return current->nextLeaf();}
      const OctNode* temp=nextBranch(current);
      if(!temp){return NULL;}
      else{return temp->nextLeaf();}
    }
    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::nextLeaf(OctNode* current){
      if(!current){
        OctNode<NodeData,Real>* temp=this;
        while(temp->children){temp=&temp->children[0];}
        return temp;
      }
      if(current->children){return current->nextLeaf();}
      OctNode* temp=nextBranch(current);
      if(!temp){return NULL;}
      else{return temp->nextLeaf();}
    }

    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::nextNode( const OctNode* current ) const
    {
      if( !current ) return this;
      else if( current->children ) return &current->children[0];
      else return nextBranch(current);
    }
    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::nextNode( OctNode* current )
    {
      if( !current ) return this;
      else if( current->children ) return &current->children[0];
      else return nextBranch( current );
    }

    template <class NodeData,class Real>
    void OctNode<NodeData,Real>::printRange(void) const{
      Point3D<Real> center;
      Real width;
      centerAndWidth(center,width);
      for(int dim=0;dim<DIMENSION;dim++){
        printf("%[%f,%f]",center.coords[dim]-width/2,center.coords[dim]+width/2);
        if(dim<DIMENSION-1){printf("x");}
        else printf("\n");
      }
    }

    template <class NodeData,class Real>
    void OctNode<NodeData,Real>::AdjacencyCountFunction::Function(const OctNode<NodeData,Real>* node1,const OctNode<NodeData,Real>* node2){count++;}

    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::processNodeNodes(OctNode* node,NodeAdjacencyFunction* F,int processCurrent){
      if(processCurrent){F->Function(this,node);}
      if(children){__processNodeNodes(node,F);}
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::processNodeFaces(OctNode* node,NodeAdjacencyFunction* F,int fIndex,int processCurrent){
      if(processCurrent){F->Function(this,node);}
      if(children){
        int c1,c2,c3,c4;
        Cube::FaceCorners(fIndex,c1,c2,c3,c4);
        __processNodeFaces(node,F,c1,c2,c3,c4);
      }
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::processNodeEdges(OctNode* node,NodeAdjacencyFunction* F,int eIndex,int processCurrent){
      if(processCurrent){F->Function(this,node);}
      if(children){
        int c1,c2;
        Cube::EdgeCorners(eIndex,c1,c2);
        __processNodeEdges(node,F,c1,c2);
      }
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::processNodeCorners(OctNode* node,NodeAdjacencyFunction* F,int cIndex,int processCurrent){
      if(processCurrent){F->Function(this,node);}
      OctNode<NodeData,Real>* temp=this;
      while(temp->children){
        temp=&temp->children[cIndex];
        F->Function(temp,node);
      }
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::__processNodeNodes(OctNode* node,NodeAdjacencyFunction* F){
      F->Function(&children[0],node);
      F->Function(&children[1],node);
      F->Function(&children[2],node);
      F->Function(&children[3],node);
      F->Function(&children[4],node);
      F->Function(&children[5],node);
      F->Function(&children[6],node);
      F->Function(&children[7],node);
      if(children[0].children){children[0].__processNodeNodes(node,F);}
      if(children[1].children){children[1].__processNodeNodes(node,F);}
      if(children[2].children){children[2].__processNodeNodes(node,F);}
      if(children[3].children){children[3].__processNodeNodes(node,F);}
      if(children[4].children){children[4].__processNodeNodes(node,F);}
      if(children[5].children){children[5].__processNodeNodes(node,F);}
      if(children[6].children){children[6].__processNodeNodes(node,F);}
      if(children[7].children){children[7].__processNodeNodes(node,F);}
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::__processNodeEdges(OctNode* node,NodeAdjacencyFunction* F,int cIndex1,int cIndex2){
      F->Function(&children[cIndex1],node);
      F->Function(&children[cIndex2],node);
      if(children[cIndex1].children){children[cIndex1].__processNodeEdges(node,F,cIndex1,cIndex2);}
      if(children[cIndex2].children){children[cIndex2].__processNodeEdges(node,F,cIndex1,cIndex2);}
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::__processNodeFaces(OctNode* node,NodeAdjacencyFunction* F,int cIndex1,int cIndex2,int cIndex3,int cIndex4){
      F->Function(&children[cIndex1],node);
      F->Function(&children[cIndex2],node);
      F->Function(&children[cIndex3],node);
      F->Function(&children[cIndex4],node);
      if(children[cIndex1].children){children[cIndex1].__processNodeFaces(node,F,cIndex1,cIndex2,cIndex3,cIndex4);}
      if(children[cIndex2].children){children[cIndex2].__processNodeFaces(node,F,cIndex1,cIndex2,cIndex3,cIndex4);}
      if(children[cIndex3].children){children[cIndex3].__processNodeFaces(node,F,cIndex1,cIndex2,cIndex3,cIndex4);}
      if(children[cIndex4].children){children[cIndex4].__processNodeFaces(node,F,cIndex1,cIndex2,cIndex3,cIndex4);}
    }
    template<class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessNodeAdjacentNodes(int maxDepth,OctNode* node1,int width1,OctNode* node2,int width2,NodeAdjacencyFunction* F,int processCurrent){
      int c1[3],c2[3],w1,w2;
      node1->centerIndex(maxDepth+1,c1);
      node2->centerIndex(maxDepth+1,c2);
      w1=node1->width(maxDepth+1);
      w2=node2->width(maxDepth+1);

      ProcessNodeAdjacentNodes(c1[0]-c2[0],c1[1]-c2[1],c1[2]-c2[2],node1,(width1*w1)>>1,node2,(width2*w2)>>1,w2,F,processCurrent);
    }
    template<class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessNodeAdjacentNodes(int dx,int dy,int dz,
                                                          OctNode<NodeData,Real>* node1,int radius1,
                                                          OctNode<NodeData,Real>* node2,int radius2,int width2,
                                                          NodeAdjacencyFunction* F,int processCurrent){
      if(!Overlap(dx,dy,dz,radius1+radius2)){return;}
      if(processCurrent){F->Function(node2,node1);}
      if(!node2->children){return;}
      __ProcessNodeAdjacentNodes(-dx,-dy,-dz,node1,radius1,node2,radius2,width2/2,F);
    }
    template<class NodeData,class Real>
    template<class TerminatingNodeAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessTerminatingNodeAdjacentNodes(int maxDepth,OctNode* node1,int width1,OctNode* node2,int width2,TerminatingNodeAdjacencyFunction* F,int processCurrent){
      int c1[3],c2[3],w1,w2;
      node1->centerIndex(maxDepth+1,c1);
      node2->centerIndex(maxDepth+1,c2);
      w1=node1->width(maxDepth+1);
      w2=node2->width(maxDepth+1);

      ProcessTerminatingNodeAdjacentNodes(c1[0]-c2[0],c1[1]-c2[1],c1[2]-c2[2],node1,(width1*w1)>>1,node2,(width2*w2)>>1,w2,F,processCurrent);
    }
    template<class NodeData,class Real>
    template<class TerminatingNodeAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessTerminatingNodeAdjacentNodes(int dx,int dy,int dz,
                                                                     OctNode<NodeData,Real>* node1,int radius1,
                                                                     OctNode<NodeData,Real>* node2,int radius2,int width2,
                                                                     TerminatingNodeAdjacencyFunction* F,int processCurrent)
    {
      if(!Overlap(dx,dy,dz,radius1+radius2)){return;}
      if(processCurrent){F->Function(node2,node1);}
      if(!node2->children){return;}
      __ProcessTerminatingNodeAdjacentNodes(-dx,-dy,-dz,node1,radius1,node2,radius2,width2/2,F);
    }
    template<class NodeData,class Real>
    template<class PointAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessPointAdjacentNodes( int maxDepth , const int c1[3] , OctNode* node2 , int width2 , PointAdjacencyFunction* F , int processCurrent )
    {
      int c2[3] , w2;
      node2->centerIndex( maxDepth+1 , c2 );
      w2 = node2->width( maxDepth+1 );
      ProcessPointAdjacentNodes( c1[0]-c2[0] , c1[1]-c2[1] , c1[2]-c2[2] , node2 , (width2*w2)>>1 , w2 , F , processCurrent );
    }
    template<class NodeData,class Real>
    template<class PointAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessPointAdjacentNodes(int dx,int dy,int dz,
                                                           OctNode<NodeData,Real>* node2,int radius2,int width2,
                                                           PointAdjacencyFunction* F,int processCurrent)
    {
      if( !Overlap(dx,dy,dz,radius2) ) return;
      if( processCurrent ) F->Function(node2);
      if( !node2->children ) return;
      __ProcessPointAdjacentNodes( -dx , -dy , -dz , node2 , radius2 , width2>>1 , F );
    }
    template<class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessFixedDepthNodeAdjacentNodes(int maxDepth,
                                                                    OctNode<NodeData,Real>* node1,int width1,
                                                                    OctNode<NodeData,Real>* node2,int width2,
                                                                    int depth,NodeAdjacencyFunction* F,int processCurrent)
    {
      int c1[3],c2[3],w1,w2;
      node1->centerIndex(maxDepth+1,c1);
      node2->centerIndex(maxDepth+1,c2);
      w1=node1->width(maxDepth+1);
      w2=node2->width(maxDepth+1);

      ProcessFixedDepthNodeAdjacentNodes(c1[0]-c2[0],c1[1]-c2[1],c1[2]-c2[2],node1,(width1*w1)>>1,node2,(width2*w2)>>1,w2,depth,F,processCurrent);
    }
    template<class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessFixedDepthNodeAdjacentNodes(int dx,int dy,int dz,
                                                                    OctNode<NodeData,Real>* node1,int radius1,
                                                                    OctNode<NodeData,Real>* node2,int radius2,int width2,
                                                                    int depth,NodeAdjacencyFunction* F,int processCurrent)
    {
      int d=node2->depth();
      if(d>depth){return;}
      if(!Overlap(dx,dy,dz,radius1+radius2)){return;}
      if(d==depth){if(processCurrent){F->Function(node2,node1);}}
      else{
        if(!node2->children){return;}
        __ProcessFixedDepthNodeAdjacentNodes(-dx,-dy,-dz,node1,radius1,node2,radius2,width2/2,depth-1,F);
      }
    }
    template<class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessMaxDepthNodeAdjacentNodes(int maxDepth,
                                                                  OctNode<NodeData,Real>* node1,int width1,
                                                                  OctNode<NodeData,Real>* node2,int width2,
                                                                  int depth,NodeAdjacencyFunction* F,int processCurrent)
    {
      int c1[3],c2[3],w1,w2;
      node1->centerIndex(maxDepth+1,c1);
      node2->centerIndex(maxDepth+1,c2);
      w1=node1->width(maxDepth+1);
      w2=node2->width(maxDepth+1);
      ProcessMaxDepthNodeAdjacentNodes(c1[0]-c2[0],c1[1]-c2[1],c1[2]-c2[2],node1,(width1*w1)>>1,node2,(width2*w2)>>1,w2,depth,F,processCurrent);
    }
    template<class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::ProcessMaxDepthNodeAdjacentNodes(int dx,int dy,int dz,
                                                                  OctNode<NodeData,Real>* node1,int radius1,
                                                                  OctNode<NodeData,Real>* node2,int radius2,int width2,
                                                                  int depth,NodeAdjacencyFunction* F,int processCurrent)
    {
      int d=node2->depth();
      if(d>depth){return;}
      if(!Overlap(dx,dy,dz,radius1+radius2)){return;}
      if(processCurrent){F->Function(node2,node1);}
      if(d<depth && node2->children){__ProcessMaxDepthNodeAdjacentNodes(-dx,-dy,-dz,node1,radius1,node2,radius2,width2>>1,depth-1,F);}
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::__ProcessNodeAdjacentNodes(int dx,int dy,int dz,
                                                            OctNode* node1,int radius1,
                                                            OctNode* node2,int radius2,int cWidth2,
                                                            NodeAdjacencyFunction* F)
    {
      int cWidth=cWidth2>>1;
      int radius=radius2>>1;
      int o=ChildOverlap(dx,dy,dz,radius1+radius,cWidth);
      if(o){
        int dx1=dx-cWidth;
        int dx2=dx+cWidth;
        int dy1=dy-cWidth;
        int dy2=dy+cWidth;
        int dz1=dz-cWidth;
        int dz2=dz+cWidth;
        if(o&  1){F->Function(&node2->children[0],node1);if(node2->children[0].children){__ProcessNodeAdjacentNodes(dx1,dy1,dz1,node1,radius1,&node2->children[0],radius,cWidth,F);}}
        if(o&  2){F->Function(&node2->children[1],node1);if(node2->children[1].children){__ProcessNodeAdjacentNodes(dx2,dy1,dz1,node1,radius1,&node2->children[1],radius,cWidth,F);}}
        if(o&  4){F->Function(&node2->children[2],node1);if(node2->children[2].children){__ProcessNodeAdjacentNodes(dx1,dy2,dz1,node1,radius1,&node2->children[2],radius,cWidth,F);}}
        if(o&  8){F->Function(&node2->children[3],node1);if(node2->children[3].children){__ProcessNodeAdjacentNodes(dx2,dy2,dz1,node1,radius1,&node2->children[3],radius,cWidth,F);}}
        if(o& 16){F->Function(&node2->children[4],node1);if(node2->children[4].children){__ProcessNodeAdjacentNodes(dx1,dy1,dz2,node1,radius1,&node2->children[4],radius,cWidth,F);}}
        if(o& 32){F->Function(&node2->children[5],node1);if(node2->children[5].children){__ProcessNodeAdjacentNodes(dx2,dy1,dz2,node1,radius1,&node2->children[5],radius,cWidth,F);}}
        if(o& 64){F->Function(&node2->children[6],node1);if(node2->children[6].children){__ProcessNodeAdjacentNodes(dx1,dy2,dz2,node1,radius1,&node2->children[6],radius,cWidth,F);}}
        if(o&128){F->Function(&node2->children[7],node1);if(node2->children[7].children){__ProcessNodeAdjacentNodes(dx2,dy2,dz2,node1,radius1,&node2->children[7],radius,cWidth,F);}}
      }
    }
    template <class NodeData,class Real>
    template<class TerminatingNodeAdjacencyFunction>
    void OctNode<NodeData,Real>::__ProcessTerminatingNodeAdjacentNodes(int dx,int dy,int dz,
                                                                       OctNode* node1,int radius1,
                                                                       OctNode* node2,int radius2,int cWidth2,
                                                                       TerminatingNodeAdjacencyFunction* F)
    {
      int cWidth=cWidth2>>1;
      int radius=radius2>>1;
      int o=ChildOverlap(dx,dy,dz,radius1+radius,cWidth);
      if(o){
        int dx1=dx-cWidth;
        int dx2=dx+cWidth;
        int dy1=dy-cWidth;
        int dy2=dy+cWidth;
        int dz1=dz-cWidth;
        int dz2=dz+cWidth;
        if(o&  1){if(F->Function(&node2->children[0],node1) && node2->children[0].children){__ProcessTerminatingNodeAdjacentNodes(dx1,dy1,dz1,node1,radius1,&node2->children[0],radius,cWidth,F);}}
        if(o&  2){if(F->Function(&node2->children[1],node1) && node2->children[1].children){__ProcessTerminatingNodeAdjacentNodes(dx2,dy1,dz1,node1,radius1,&node2->children[1],radius,cWidth,F);}}
        if(o&  4){if(F->Function(&node2->children[2],node1) && node2->children[2].children){__ProcessTerminatingNodeAdjacentNodes(dx1,dy2,dz1,node1,radius1,&node2->children[2],radius,cWidth,F);}}
        if(o&  8){if(F->Function(&node2->children[3],node1) && node2->children[3].children){__ProcessTerminatingNodeAdjacentNodes(dx2,dy2,dz1,node1,radius1,&node2->children[3],radius,cWidth,F);}}
        if(o& 16){if(F->Function(&node2->children[4],node1) && node2->children[4].children){__ProcessTerminatingNodeAdjacentNodes(dx1,dy1,dz2,node1,radius1,&node2->children[4],radius,cWidth,F);}}
        if(o& 32){if(F->Function(&node2->children[5],node1) && node2->children[5].children){__ProcessTerminatingNodeAdjacentNodes(dx2,dy1,dz2,node1,radius1,&node2->children[5],radius,cWidth,F);}}
        if(o& 64){if(F->Function(&node2->children[6],node1) && node2->children[6].children){__ProcessTerminatingNodeAdjacentNodes(dx1,dy2,dz2,node1,radius1,&node2->children[6],radius,cWidth,F);}}
        if(o&128){if(F->Function(&node2->children[7],node1) && node2->children[7].children){__ProcessTerminatingNodeAdjacentNodes(dx2,dy2,dz2,node1,radius1,&node2->children[7],radius,cWidth,F);}}
      }
    }
    template <class NodeData,class Real>
    template<class PointAdjacencyFunction>
    void OctNode<NodeData,Real>::__ProcessPointAdjacentNodes(int dx,int dy,int dz,
                                                             OctNode* node2,int radius2,int cWidth2,
                                                             PointAdjacencyFunction* F)
    {
      int cWidth=cWidth2>>1;
      int radius=radius2>>1;
      int o=ChildOverlap(dx,dy,dz,radius,cWidth);
      if( o )
      {
        int dx1=dx-cWidth;
        int dx2=dx+cWidth;
        int dy1=dy-cWidth;
        int dy2=dy+cWidth;
        int dz1=dz-cWidth;
        int dz2=dz+cWidth;
        if(o&  1){F->Function(&node2->children[0]);if(node2->children[0].children){__ProcessPointAdjacentNodes(dx1,dy1,dz1,&node2->children[0],radius,cWidth,F);}}
        if(o&  2){F->Function(&node2->children[1]);if(node2->children[1].children){__ProcessPointAdjacentNodes(dx2,dy1,dz1,&node2->children[1],radius,cWidth,F);}}
        if(o&  4){F->Function(&node2->children[2]);if(node2->children[2].children){__ProcessPointAdjacentNodes(dx1,dy2,dz1,&node2->children[2],radius,cWidth,F);}}
        if(o&  8){F->Function(&node2->children[3]);if(node2->children[3].children){__ProcessPointAdjacentNodes(dx2,dy2,dz1,&node2->children[3],radius,cWidth,F);}}
        if(o& 16){F->Function(&node2->children[4]);if(node2->children[4].children){__ProcessPointAdjacentNodes(dx1,dy1,dz2,&node2->children[4],radius,cWidth,F);}}
        if(o& 32){F->Function(&node2->children[5]);if(node2->children[5].children){__ProcessPointAdjacentNodes(dx2,dy1,dz2,&node2->children[5],radius,cWidth,F);}}
        if(o& 64){F->Function(&node2->children[6]);if(node2->children[6].children){__ProcessPointAdjacentNodes(dx1,dy2,dz2,&node2->children[6],radius,cWidth,F);}}
        if(o&128){F->Function(&node2->children[7]);if(node2->children[7].children){__ProcessPointAdjacentNodes(dx2,dy2,dz2,&node2->children[7],radius,cWidth,F);}}
      }
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::__ProcessFixedDepthNodeAdjacentNodes(int dx,int dy,int dz,
                                                                      OctNode* node1,int radius1,
                                                                      OctNode* node2,int radius2,int cWidth2,
                                                                      int depth,NodeAdjacencyFunction* F)
    {
      int cWidth=cWidth2>>1;
      int radius=radius2>>1;
      int o=ChildOverlap(dx,dy,dz,radius1+radius,cWidth);
      if(o){
        int dx1=dx-cWidth;
        int dx2=dx+cWidth;
        int dy1=dy-cWidth;
        int dy2=dy+cWidth;
        int dz1=dz-cWidth;
        int dz2=dz+cWidth;
        if(node2->depth()==depth){
          if(o&  1){F->Function(&node2->children[0],node1);}
          if(o&  2){F->Function(&node2->children[1],node1);}
          if(o&  4){F->Function(&node2->children[2],node1);}
          if(o&  8){F->Function(&node2->children[3],node1);}
          if(o& 16){F->Function(&node2->children[4],node1);}
          if(o& 32){F->Function(&node2->children[5],node1);}
          if(o& 64){F->Function(&node2->children[6],node1);}
          if(o&128){F->Function(&node2->children[7],node1);}
        }
        else{
          if(o&  1){if(node2->children[0].children){__ProcessFixedDepthNodeAdjacentNodes(dx1,dy1,dz1,node1,radius1,&node2->children[0],radius,cWidth,depth,F);}}
          if(o&  2){if(node2->children[1].children){__ProcessFixedDepthNodeAdjacentNodes(dx2,dy1,dz1,node1,radius1,&node2->children[1],radius,cWidth,depth,F);}}
          if(o&  4){if(node2->children[2].children){__ProcessFixedDepthNodeAdjacentNodes(dx1,dy2,dz1,node1,radius1,&node2->children[2],radius,cWidth,depth,F);}}
          if(o&  8){if(node2->children[3].children){__ProcessFixedDepthNodeAdjacentNodes(dx2,dy2,dz1,node1,radius1,&node2->children[3],radius,cWidth,depth,F);}}
          if(o& 16){if(node2->children[4].children){__ProcessFixedDepthNodeAdjacentNodes(dx1,dy1,dz2,node1,radius1,&node2->children[4],radius,cWidth,depth,F);}}
          if(o& 32){if(node2->children[5].children){__ProcessFixedDepthNodeAdjacentNodes(dx2,dy1,dz2,node1,radius1,&node2->children[5],radius,cWidth,depth,F);}}
          if(o& 64){if(node2->children[6].children){__ProcessFixedDepthNodeAdjacentNodes(dx1,dy2,dz2,node1,radius1,&node2->children[6],radius,cWidth,depth,F);}}
          if(o&128){if(node2->children[7].children){__ProcessFixedDepthNodeAdjacentNodes(dx2,dy2,dz2,node1,radius1,&node2->children[7],radius,cWidth,depth,F);}}
        }
      }
    }
    template <class NodeData,class Real>
    template<class NodeAdjacencyFunction>
    void OctNode<NodeData,Real>::__ProcessMaxDepthNodeAdjacentNodes(int dx,int dy,int dz,
                                                                    OctNode* node1,int radius1,
                                                                    OctNode* node2,int radius2,int cWidth2,
                                                                    int depth,NodeAdjacencyFunction* F)
    {
      int cWidth=cWidth2>>1;
      int radius=radius2>>1;
      int o=ChildOverlap(dx,dy,dz,radius1+radius,cWidth);
      if(o){
        int dx1=dx-cWidth;
        int dx2=dx+cWidth;
        int dy1=dy-cWidth;
        int dy2=dy+cWidth;
        int dz1=dz-cWidth;
        int dz2=dz+cWidth;
        if(node2->depth()<=depth){
          if(o&  1){F->Function(&node2->children[0],node1);}
          if(o&  2){F->Function(&node2->children[1],node1);}
          if(o&  4){F->Function(&node2->children[2],node1);}
          if(o&  8){F->Function(&node2->children[3],node1);}
          if(o& 16){F->Function(&node2->children[4],node1);}
          if(o& 32){F->Function(&node2->children[5],node1);}
          if(o& 64){F->Function(&node2->children[6],node1);}
          if(o&128){F->Function(&node2->children[7],node1);}
        }
        if(node2->depth()<depth){
          if(o&  1){if(node2->children[0].children){__ProcessMaxDepthNodeAdjacentNodes(dx1,dy1,dz1,node1,radius1,&node2->children[0],radius,cWidth,depth,F);}}
          if(o&  2){if(node2->children[1].children){__ProcessMaxDepthNodeAdjacentNodes(dx2,dy1,dz1,node1,radius1,&node2->children[1],radius,cWidth,depth,F);}}
          if(o&  4){if(node2->children[2].children){__ProcessMaxDepthNodeAdjacentNodes(dx1,dy2,dz1,node1,radius1,&node2->children[2],radius,cWidth,depth,F);}}
          if(o&  8){if(node2->children[3].children){__ProcessMaxDepthNodeAdjacentNodes(dx2,dy2,dz1,node1,radius1,&node2->children[3],radius,cWidth,depth,F);}}
          if(o& 16){if(node2->children[4].children){__ProcessMaxDepthNodeAdjacentNodes(dx1,dy1,dz2,node1,radius1,&node2->children[4],radius,cWidth,depth,F);}}
          if(o& 32){if(node2->children[5].children){__ProcessMaxDepthNodeAdjacentNodes(dx2,dy1,dz2,node1,radius1,&node2->children[5],radius,cWidth,depth,F);}}
          if(o& 64){if(node2->children[6].children){__ProcessMaxDepthNodeAdjacentNodes(dx1,dy2,dz2,node1,radius1,&node2->children[6],radius,cWidth,depth,F);}}
          if(o&128){if(node2->children[7].children){__ProcessMaxDepthNodeAdjacentNodes(dx2,dy2,dz2,node1,radius1,&node2->children[7],radius,cWidth,depth,F);}}
        }
      }
    }
    template <class NodeData,class Real>
    inline int OctNode<NodeData,Real>::ChildOverlap(int dx,int dy,int dz,int d,int cRadius2)
    {
      int w1=d-cRadius2;
      int w2=d+cRadius2;
      int overlap=0;

      int test=0,test1=0;
      if(dx<w2 && dx>-w1){test =1;}
      if(dx<w1 && dx>-w2){test|=2;}

      if(!test){return 0;}
      if(dz<w2 && dz>-w1){test1 =test;}
      if(dz<w1 && dz>-w2){test1|=test<<4;}

      if(!test1){return 0;}
      if(dy<w2 && dy>-w1){overlap =test1;}
      if(dy<w1 && dy>-w2){overlap|=test1<<2;}
      return overlap;
    }

    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::getNearestLeaf(const Point3D<Real>& p){
      Point3D<Real> center;
      Real width;
      OctNode<NodeData,Real>* temp;
      int cIndex;
      if(!children){return this;}
      centerAndWidth(center,width);
      temp=this;
      while(temp->children){
        cIndex=CornerIndex(center,p);
        temp=&temp->children[cIndex];
        width/=2;
        if(cIndex&1){center.coords[0]+=width/2;}
        else		{center.coords[0]-=width/2;}
        if(cIndex&2){center.coords[1]+=width/2;}
        else		{center.coords[1]-=width/2;}
        if(cIndex&4){center.coords[2]+=width/2;}
        else		{center.coords[2]-=width/2;}
      }
      return temp;
    }
    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::getNearestLeaf(const Point3D<Real>& p) const{
      int nearest;
      Real temp,dist2;
      if(!children){return this;}
      for(int i=0;i<Cube::CORNERS;i++){
        temp=SquareDistance(children[i].center,p);
        if(!i || temp<dist2){
          dist2=temp;
          nearest=i;
        }
      }
      return children[nearest].getNearestLeaf(p);
    }

    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::CommonEdge(const OctNode<NodeData,Real>* node1,int eIndex1,const OctNode<NodeData,Real>* node2,int eIndex2){
      int o1,o2,i1,i2,j1,j2;

      Cube::FactorEdgeIndex(eIndex1,o1,i1,j1);
      Cube::FactorEdgeIndex(eIndex2,o2,i2,j2);
      if(o1!=o2){return 0;}

      int dir[2];
      int idx1[2];
      int idx2[2];
      switch(o1){
      case 0:	dir[0]=1;	dir[1]=2;	break;
      case 1:	dir[0]=0;	dir[1]=2;	break;
      case 2:	dir[0]=0;	dir[1]=1;	break;
      };
      int d1,d2,off1[3],off2[3];
      node1->depthAndOffset(d1,off1);
      node2->depthAndOffset(d2,off2);
      idx1[0]=off1[dir[0]]+(1<<d1)+i1;
      idx1[1]=off1[dir[1]]+(1<<d1)+j1;
      idx2[0]=off2[dir[0]]+(1<<d2)+i2;
      idx2[1]=off2[dir[1]]+(1<<d2)+j2;
      if(d1>d2){
        idx2[0]<<=(d1-d2);
        idx2[1]<<=(d1-d2);
      }
      else{
        idx1[0]<<=(d2-d1);
        idx1[1]<<=(d2-d1);
      }
      if(idx1[0]==idx2[0] && idx1[1]==idx2[1]){return 1;}
      else									{return 0;}
    }
    template<class NodeData,class Real>
    int OctNode<NodeData,Real>::CornerIndex(const Point3D<Real>& center,const Point3D<Real>& p){
      int cIndex=0;
      if(p.coords[0]>center.coords[0]){cIndex|=1;}
      if(p.coords[1]>center.coords[1]){cIndex|=2;}
      if(p.coords[2]>center.coords[2]){cIndex|=4;}
      return cIndex;
    }
    template <class NodeData,class Real>
    template<class NodeData2>
    OctNode<NodeData,Real>& OctNode<NodeData,Real>::operator = (const OctNode<NodeData2,Real>& node){
      int i;
      if(children){delete[] children;}
      children=NULL;

      d=node.depth ();
      for(i=0;i<DIMENSION;i++){this->offset[i] = node.offset[i];}
      if(node.children){
        initChildren();
        for(i=0;i<Cube::CORNERS;i++){children[i] = node.children[i];}
      }
      return *this;
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::CompareForwardDepths(const void* v1,const void* v2){
      return ((const OctNode<NodeData,Real>*)v1)->depth-((const OctNode<NodeData,Real>*)v2)->depth;
    }
    template< class NodeData , class Real >
    int OctNode< NodeData , Real >::CompareByDepthAndXYZ( const void* v1 , const void* v2 )
    {
      const OctNode<NodeData,Real> *n1 = (*(const OctNode< NodeData , Real >**)v1);
      const OctNode<NodeData,Real> *n2 = (*(const OctNode< NodeData , Real >**)v2);
      if( n1->d!=n2->d ) return int(n1->d)-int(n2->d);
      else if( n1->off[0]!=n2->off[0] ) return int(n1->off[0]) - int(n2->off[0]);
      else if( n1->off[1]!=n2->off[1] ) return int(n1->off[1]) - int(n2->off[1]);
      else if( n1->off[2]!=n2->off[2] ) return int(n1->off[2]) - int(n2->off[2]);
      return 0;
    }

    long long _InterleaveBits( int p[3] )
    {
      long long key = 0;
      for( int i=0 ; i<32 ; i++ ) key |= ( ( p[0] & (1<<i) )<<(2*i) ) | ( ( p[1] & (1<<i) )<<(2*i+1) ) | ( ( p[2] & (1<<i) )<<(2*i+2) );
      return key;
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::CompareByDepthAndZIndex( const void* v1 , const void* v2 )
    {
      const OctNode<NodeData,Real>* n1 = (*(const OctNode<NodeData,Real>**)v1);
      const OctNode<NodeData,Real>* n2 = (*(const OctNode<NodeData,Real>**)v2);
      int d1 , off1[3] , d2 , off2[3];
      n1->depthAndOffset( d1 , off1 ) , n2->depthAndOffset( d2 , off2 );
      if     ( d1>d2 ) return  1;
      else if( d1<d2 ) return -1;
      long long k1 = _InterleaveBits( off1 ) , k2 = _InterleaveBits( off2 );
      if     ( k1>k2 ) return  1;
      else if( k1<k2 ) return -1;
      else             return  0;
    }

    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::CompareForwardPointerDepths( const void* v1 , const void* v2 )
    {
      const OctNode<NodeData,Real>* n1 = (*(const OctNode<NodeData,Real>**)v1);
      const OctNode<NodeData,Real>* n2 = (*(const OctNode<NodeData,Real>**)v2);
      if(n1->d!=n2->d){return int(n1->d)-int(n2->d);}
      while( n1->parent!=n2->parent )
      {
        n1=n1->parent;
        n2=n2->parent;
      }
      if(n1->off[0]!=n2->off[0]){return int(n1->off[0])-int(n2->off[0]);}
      if(n1->off[1]!=n2->off[1]){return int(n1->off[1])-int(n2->off[1]);}
      return int(n1->off[2])-int(n2->off[2]);
      return 0;
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::CompareBackwardDepths(const void* v1,const void* v2){
      return ((const OctNode<NodeData,Real>*)v2)->depth-((const OctNode<NodeData,Real>*)v1)->depth;
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::CompareBackwardPointerDepths(const void* v1,const void* v2){
      return (*(const OctNode<NodeData,Real>**)v2)->depth()-(*(const OctNode<NodeData,Real>**)v1)->depth();
    }
    template <class NodeData,class Real>
    inline int OctNode<NodeData,Real>::Overlap2(const int &depth1,const int offSet1[DIMENSION],const Real& multiplier1,const int &depth2,const int offSet2[DIMENSION],const Real& multiplier2){
      int d=depth2-depth1;
      Real w=multiplier2+multiplier1*(1<<d);
      Real w2=Real((1<<(d-1))-0.5);
      if(
         fabs(Real(offSet2[0]-(offSet1[0]<<d))-w2)>=w ||
         fabs(Real(offSet2[1]-(offSet1[1]<<d))-w2)>=w ||
         fabs(Real(offSet2[2]-(offSet1[2]<<d))-w2)>=w
         ){return 0;}
      return 1;
    }
    template <class NodeData,class Real>
    inline int OctNode<NodeData,Real>::Overlap(int c1,int c2,int c3,int dWidth){
      if(c1>=dWidth || c1<=-dWidth || c2>=dWidth || c2<=-dWidth || c3>=dWidth || c3<=-dWidth){return 0;}
      else{return 1;}
    }
    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::faceNeighbor(int faceIndex,int forceChildren){return __faceNeighbor(faceIndex>>1,faceIndex&1,forceChildren);}
    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::faceNeighbor(int faceIndex) const {return __faceNeighbor(faceIndex>>1,faceIndex&1);}
    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::__faceNeighbor(int dir,int off,int forceChildren){
      if(!parent){return NULL;}
      int pIndex=int(this-parent->children);
      pIndex^=(1<<dir);
      if((pIndex & (1<<dir))==(off<<dir)){return &parent->children[pIndex];}
      else{
        OctNode* temp=parent->__faceNeighbor(dir,off,forceChildren);
        if(!temp){return NULL;}
        if(!temp->children){
          if(forceChildren){temp->initChildren();}
          else{return temp;}
        }
        return &temp->children[pIndex];
      }
    }
    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::__faceNeighbor(int dir,int off) const {
      if(!parent){return NULL;}
      int pIndex=int(this-parent->children);
      pIndex^=(1<<dir);
      if((pIndex & (1<<dir))==(off<<dir)){return &parent->children[pIndex];}
      else{
        const OctNode* temp=parent->__faceNeighbor(dir,off);
        if(!temp || !temp->children){return temp;}
        else{return &temp->children[pIndex];}
      }
    }

    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::edgeNeighbor(int edgeIndex,int forceChildren){
      int idx[2],o,i[2];
      Cube::FactorEdgeIndex(edgeIndex,o,i[0],i[1]);
      switch(o){
      case 0:	idx[0]=1;	idx[1]=2;	break;
      case 1:	idx[0]=0;	idx[1]=2;	break;
      case 2:	idx[0]=0;	idx[1]=1;	break;
      };
      return __edgeNeighbor(o,i,idx,forceChildren);
    }
    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::edgeNeighbor(int edgeIndex) const {
      int idx[2],o,i[2];
      Cube::FactorEdgeIndex(edgeIndex,o,i[0],i[1]);
      switch(o){
      case 0:	idx[0]=1;	idx[1]=2;	break;
      case 1:	idx[0]=0;	idx[1]=2;	break;
      case 2:	idx[0]=0;	idx[1]=1;	break;
      };
      return __edgeNeighbor(o,i,idx);
    }
    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::__edgeNeighbor(int o,const int i[2],const int idx[2]) const{
      if(!parent){return NULL;}
      int pIndex=int(this-parent->children);
      int aIndex,x[DIMENSION];

      Cube::FactorCornerIndex(pIndex,x[0],x[1],x[2]);
      aIndex=(~((i[0] ^ x[idx[0]]) | ((i[1] ^ x[idx[1]])<<1))) & 3;
      pIndex^=(7 ^ (1<<o));
      if(aIndex==1)	{	// I can get the neighbor from the parent's face adjacent neighbor
        const OctNode* temp=parent->__faceNeighbor(idx[0],i[0]);
        if(!temp || !temp->children){return NULL;}
        else{return &temp->children[pIndex];}
      }
      else if(aIndex==2)	{	// I can get the neighbor from the parent's face adjacent neighbor
        const OctNode* temp=parent->__faceNeighbor(idx[1],i[1]);
        if(!temp || !temp->children){return NULL;}
        else{return &temp->children[pIndex];}
      }
      else if(aIndex==0)	{	// I can get the neighbor from the parent
        return &parent->children[pIndex];
      }
      else if(aIndex==3)	{	// I can get the neighbor from the parent's edge adjacent neighbor
        const OctNode* temp=parent->__edgeNeighbor(o,i,idx);
        if(!temp || !temp->children){return temp;}
        else{return &temp->children[pIndex];}
      }
      else{return NULL;}
    }
    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::__edgeNeighbor(int o,const int i[2],const int idx[2],int forceChildren){
      if(!parent){return NULL;}
      int pIndex=int(this-parent->children);
      int aIndex,x[DIMENSION];

      Cube::FactorCornerIndex(pIndex,x[0],x[1],x[2]);
      aIndex=(~((i[0] ^ x[idx[0]]) | ((i[1] ^ x[idx[1]])<<1))) & 3;
      pIndex^=(7 ^ (1<<o));
      if(aIndex==1)	{	// I can get the neighbor from the parent's face adjacent neighbor
        OctNode* temp=parent->__faceNeighbor(idx[0],i[0],0);
        if(!temp || !temp->children){return NULL;}
        else{return &temp->children[pIndex];}
      }
      else if(aIndex==2)	{	// I can get the neighbor from the parent's face adjacent neighbor
        OctNode* temp=parent->__faceNeighbor(idx[1],i[1],0);
        if(!temp || !temp->children){return NULL;}
        else{return &temp->children[pIndex];}
      }
      else if(aIndex==0)	{	// I can get the neighbor from the parent
        return &parent->children[pIndex];
      }
      else if(aIndex==3)	{	// I can get the neighbor from the parent's edge adjacent neighbor
        OctNode* temp=parent->__edgeNeighbor(o,i,idx,forceChildren);
        if(!temp){return NULL;}
        if(!temp->children){
          if(forceChildren){temp->initChildren();}
          else{return temp;}
        }
        return &temp->children[pIndex];
      }
      else{return NULL;}
    }

    template <class NodeData,class Real>
    const OctNode<NodeData,Real>* OctNode<NodeData,Real>::cornerNeighbor(int cornerIndex) const {
      int pIndex,aIndex=0;
      if(!parent){return NULL;}

      pIndex=int(this-parent->children);
      aIndex=(cornerIndex ^ pIndex);	// The disagreement bits
      pIndex=(~pIndex)&7;				// The antipodal point
      if(aIndex==7){					// Agree on no bits
        return &parent->children[pIndex];
      }
      else if(aIndex==0){				// Agree on all bits
        const OctNode* temp=((const OctNode*)parent)->cornerNeighbor(cornerIndex);
        if(!temp || !temp->children){return temp;}
        else{return &temp->children[pIndex];}
      }
      else if(aIndex==6){				// Agree on face 0
        const OctNode* temp=((const OctNode*)parent)->__faceNeighbor(0,cornerIndex & 1);
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==5){				// Agree on face 1
        const OctNode* temp=((const OctNode*)parent)->__faceNeighbor(1,(cornerIndex & 2)>>1);
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==3){				// Agree on face 2
        const OctNode* temp=((const OctNode*)parent)->__faceNeighbor(2,(cornerIndex & 4)>>2);
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==4){				// Agree on edge 2
        const OctNode* temp=((const OctNode*)parent)->edgeNeighbor(8 | (cornerIndex & 1) | (cornerIndex & 2) );
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==2){				// Agree on edge 1
        const OctNode* temp=((const OctNode*)parent)->edgeNeighbor(4 | (cornerIndex & 1) | ((cornerIndex & 4)>>1) );
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==1){				// Agree on edge 0
        const OctNode* temp=((const OctNode*)parent)->edgeNeighbor(((cornerIndex & 2) | (cornerIndex & 4))>>1 );
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else{return NULL;}
    }
    template <class NodeData,class Real>
    OctNode<NodeData,Real>* OctNode<NodeData,Real>::cornerNeighbor(int cornerIndex,int forceChildren){
      int pIndex,aIndex=0;
      if(!parent){return NULL;}

      pIndex=int(this-parent->children);
      aIndex=(cornerIndex ^ pIndex);	// The disagreement bits
      pIndex=(~pIndex)&7;				// The antipodal point
      if(aIndex==7){					// Agree on no bits
        return &parent->children[pIndex];
      }
      else if(aIndex==0){				// Agree on all bits
        OctNode* temp=((OctNode*)parent)->cornerNeighbor(cornerIndex,forceChildren);
        if(!temp){return NULL;}
        if(!temp->children){
          if(forceChildren){temp->initChildren();}
          else{return temp;}
        }
        return &temp->children[pIndex];
      }
      else if(aIndex==6){				// Agree on face 0
        OctNode* temp=((OctNode*)parent)->__faceNeighbor(0,cornerIndex & 1,0);
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==5){				// Agree on face 1
        OctNode* temp=((OctNode*)parent)->__faceNeighbor(1,(cornerIndex & 2)>>1,0);
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==3){				// Agree on face 2
        OctNode* temp=((OctNode*)parent)->__faceNeighbor(2,(cornerIndex & 4)>>2,0);
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==4){				// Agree on edge 2
        OctNode* temp=((OctNode*)parent)->edgeNeighbor(8 | (cornerIndex & 1) | (cornerIndex & 2) );
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==2){				// Agree on edge 1
        OctNode* temp=((OctNode*)parent)->edgeNeighbor(4 | (cornerIndex & 1) | ((cornerIndex & 4)>>1) );
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else if(aIndex==1){				// Agree on edge 0
        OctNode* temp=((OctNode*)parent)->edgeNeighbor(((cornerIndex & 2) | (cornerIndex & 4))>>1 );
        if(!temp || !temp->children){return NULL;}
        else{return & temp->children[pIndex];}
      }
      else{return NULL;}
    }
    ////////////////////////
    // OctNodeNeighborKey //
    ////////////////////////
    template<class NodeData,class Real>
    OctNode<NodeData,Real>::Neighbors3::Neighbors3(void){clear();}
    template<class NodeData,class Real>
    void OctNode<NodeData,Real>::Neighbors3::clear(void){
      for(int i=0;i<3;i++){for(int j=0;j<3;j++){for(int k=0;k<3;k++){neighbors[i][j][k]=NULL;}}}
    }
    template<class NodeData,class Real>
    OctNode<NodeData,Real>::NeighborKey3::NeighborKey3(void){ neighbors=NULL; }
    template<class NodeData,class Real>
    OctNode<NodeData,Real>::NeighborKey3::~NeighborKey3(void)
    {
      if( neighbors ) delete[] neighbors;
      neighbors = NULL;
    }

    template<class NodeData,class Real>
    void OctNode<NodeData,Real>::NeighborKey3::set( int d )
    {
      if( neighbors ) delete[] neighbors;
      neighbors = NULL;
      if( d<0 ) return;
      neighbors = new Neighbors3[d+1];
    }
    template< class NodeData , class Real >
    typename OctNode<NodeData,Real>::Neighbors3& OctNode<NodeData,Real>::NeighborKey3::setNeighbors( OctNode<NodeData,Real>* root , Point3D< Real > p , int d )
    {
      if( !neighbors[d].neighbors[1][1][1] || !neighbors[d].neighbors[1][1][1]->isInside( p ) )
      {
        neighbors[d].clear();

        if( !d ) neighbors[d].neighbors[1][1][1] = root;
        else
        {
          Neighbors3& temp = setNeighbors( root , p , d-1 );

          int i , j , k , x1 , y1 , z1 , x2 , y2 , z2;
          Point3D< Real > c;
          Real w;
          temp.neighbors[1][1][1]->centerAndWidth( c , w );
          int idx = CornerIndex( c , p );
          Cube::FactorCornerIndex(   idx    , x1 , y1 , z1 );
          Cube::FactorCornerIndex( (~idx)&7 , x2 , y2 , z2 );

          if( !temp.neighbors[1][1][1]->children ) temp.neighbors[1][1][1]->initChildren();
          for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
            neighbors[d].neighbors[x2+i][y2+j][z2+k] = &temp.neighbors[1][1][1]->children[Cube::CornerIndex(i,j,k)];


          // Set the neighbors from across the faces
          i=x1<<1;
          if( temp.neighbors[i][1][1] )
          {
            if( !temp.neighbors[i][1][1]->children ) temp.neighbors[i][1][1]->initChildren();
            for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[i][y2+j][z2+k] = &temp.neighbors[i][1][1]->children[Cube::CornerIndex(x2,j,k)];
          }
          j=y1<<1;
          if( temp.neighbors[1][j][1] )
          {
            if( !temp.neighbors[1][j][1]->children ) temp.neighbors[1][j][1]->initChildren();
            for( i=0 ; i<2 ; i++ ) for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[x2+i][j][z2+k] = &temp.neighbors[1][j][1]->children[Cube::CornerIndex(i,y2,k)];
          }
          k=z1<<1;
          if( temp.neighbors[1][1][k] )
          {
            if( !temp.neighbors[1][1][k]->children ) temp.neighbors[1][1][k]->initChildren();
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) neighbors[d].neighbors[x2+i][y2+j][k] = &temp.neighbors[1][1][k]->children[Cube::CornerIndex(i,j,z2)];
          }

          // Set the neighbors from across the edges
          i=x1<<1 , j=y1<<1;
          if( temp.neighbors[i][j][1] )
          {
            if( !temp.neighbors[i][j][1]->children ) temp.neighbors[i][j][1]->initChildren();
            for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[i][j][z2+k] = &temp.neighbors[i][j][1]->children[Cube::CornerIndex(x2,y2,k)];
          }
          i=x1<<1 , k=z1<<1;
          if( temp.neighbors[i][1][k] )
          {
            if( !temp.neighbors[i][1][k]->children ) temp.neighbors[i][1][k]->initChildren();
            for( j=0 ; j<2 ; j++ ) neighbors[d].neighbors[i][y2+j][k] = &temp.neighbors[i][1][k]->children[Cube::CornerIndex(x2,j,z2)];
          }
          j=y1<<1 , k=z1<<1;
          if( temp.neighbors[1][j][k] )
          {
            if( !temp.neighbors[1][j][k]->children ) temp.neighbors[1][j][k]->initChildren();
            for( i=0 ; i<2 ; i++ ) neighbors[d].neighbors[x2+i][j][k] = &temp.neighbors[1][j][k]->children[Cube::CornerIndex(i,y2,z2)];
          }

          // Set the neighbor from across the corner
          i=x1<<1 , j=y1<<1 , k=z1<<1;
          if( temp.neighbors[i][j][k] )
          {
            if( !temp.neighbors[i][j][k]->children ) temp.neighbors[i][j][k]->initChildren();
            neighbors[d].neighbors[i][j][k] = &temp.neighbors[i][j][k]->children[Cube::CornerIndex(x2,y2,z2)];
          }
        }
      }
      return neighbors[d];
    }
    template< class NodeData , class Real >
    typename OctNode<NodeData,Real>::Neighbors3& OctNode<NodeData,Real>::NeighborKey3::getNeighbors( OctNode<NodeData,Real>* root , Point3D< Real > p , int d )
    {
      if( !neighbors[d].neighbors[1][1][1] || !neighbors[d].neighbors[1][1][1]->isInside( p ) )
      {
        neighbors[d].clear();

        if( !d ) neighbors[d].neighbors[1][1][1] = root;
        else
        {
          Neighbors3& temp = getNeighbors( root , p , d-1 );

          int i , j , k , x1 , y1 , z1 , x2 , y2 , z2;
          Point3D< Real > c;
          Real w;
          temp.neighbors[1][1][1]->centerAndWidth( c , w );
          int idx = CornerIndex( c , p );
          Cube::FactorCornerIndex(   idx    , x1 , y1 , z1 );
          Cube::FactorCornerIndex( (~idx)&7 , x2 , y2 , z2 );

          if( !temp.neighbors[1][1][1] || !temp.neighbors[1][1][1]->children )
          {
            fprintf( stderr , "[ERROR] Couldn't find node at appropriate depth\n" );
            exit( 0 );
          }
          for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
            neighbors[d].neighbors[x2+i][y2+j][z2+k] = &temp.neighbors[1][1][1]->children[Cube::CornerIndex(i,j,k)];


          // Set the neighbors from across the faces
          i=x1<<1;
          if( temp.neighbors[i][1][1] )
            for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[i][y2+j][z2+k] = &temp.neighbors[i][1][1]->children[Cube::CornerIndex(x2,j,k)];
          j=y1<<1;
          if( temp.neighbors[1][j][1] )
            for( i=0 ; i<2 ; i++ ) for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[x2+i][j][z2+k] = &temp.neighbors[1][j][1]->children[Cube::CornerIndex(i,y2,k)];
          k=z1<<1;
          if( temp.neighbors[1][1][k] )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) neighbors[d].neighbors[x2+i][y2+j][k] = &temp.neighbors[1][1][k]->children[Cube::CornerIndex(i,j,z2)];

          // Set the neighbors from across the edges
          i=x1<<1 , j=y1<<1;
          if( temp.neighbors[i][j][1] )
            for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[i][j][z2+k] = &temp.neighbors[i][j][1]->children[Cube::CornerIndex(x2,y2,k)];
          i=x1<<1 , k=z1<<1;
          if( temp.neighbors[i][1][k] )
            for( j=0 ; j<2 ; j++ ) neighbors[d].neighbors[i][y2+j][k] = &temp.neighbors[i][1][k]->children[Cube::CornerIndex(x2,j,z2)];
          j=y1<<1 , k=z1<<1;
          if( temp.neighbors[1][j][k] )
            for( i=0 ; i<2 ; i++ ) neighbors[d].neighbors[x2+i][j][k] = &temp.neighbors[1][j][k]->children[Cube::CornerIndex(i,y2,z2)];

          // Set the neighbor from across the corner
          i=x1<<1 , j=y1<<1 , k=z1<<1;
          if( temp.neighbors[i][j][k] )
            neighbors[d].neighbors[i][j][k] = &temp.neighbors[i][j][k]->children[Cube::CornerIndex(x2,y2,z2)];
        }
      }
      return neighbors[d];
    }

    template< class NodeData , class Real >
    typename OctNode<NodeData,Real>::Neighbors3& OctNode<NodeData,Real>::NeighborKey3::setNeighbors( OctNode<NodeData,Real>* node )
    {
      int d = node->depth();
      if( node==neighbors[d].neighbors[1][1][1] )
      {
        bool reset = false;
        for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ ) for( int k=0 ; k<3 ; k++ ) if( !neighbors[d].neighbors[i][j][k] ) reset = true;
        if( reset ) neighbors[d].neighbors[1][1][1] = NULL;
      }
      if( node!=neighbors[d].neighbors[1][1][1] )
      {
        neighbors[d].clear();

        if( !node->parent ) neighbors[d].neighbors[1][1][1] = node;
        else
        {
          int i,j,k,x1,y1,z1,x2,y2,z2;
          int idx=int(node-node->parent->children);
          Cube::FactorCornerIndex(  idx   ,x1,y1,z1);
          Cube::FactorCornerIndex((~idx)&7,x2,y2,z2);
          for(i=0;i<2;i++){
            for(j=0;j<2;j++){
              for(k=0;k<2;k++){
                neighbors[d].neighbors[x2+i][y2+j][z2+k]=&node->parent->children[Cube::CornerIndex(i,j,k)];
              }
            }
          }
          Neighbors3& temp=setNeighbors(node->parent);

          // Set the neighbors from across the faces
          i=x1<<1;
          if(temp.neighbors[i][1][1]){
            if(!temp.neighbors[i][1][1]->children){temp.neighbors[i][1][1]->initChildren();}
            for(j=0;j<2;j++){for(k=0;k<2;k++){neighbors[d].neighbors[i][y2+j][z2+k]=&temp.neighbors[i][1][1]->children[Cube::CornerIndex(x2,j,k)];}}
          }
          j=y1<<1;
          if(temp.neighbors[1][j][1]){
            if(!temp.neighbors[1][j][1]->children){temp.neighbors[1][j][1]->initChildren();}
            for(i=0;i<2;i++){for(k=0;k<2;k++){neighbors[d].neighbors[x2+i][j][z2+k]=&temp.neighbors[1][j][1]->children[Cube::CornerIndex(i,y2,k)];}}
          }
          k=z1<<1;
          if(temp.neighbors[1][1][k]){
            if(!temp.neighbors[1][1][k]->children){temp.neighbors[1][1][k]->initChildren();}
            for(i=0;i<2;i++){for(j=0;j<2;j++){neighbors[d].neighbors[x2+i][y2+j][k]=&temp.neighbors[1][1][k]->children[Cube::CornerIndex(i,j,z2)];}}
          }

          // Set the neighbors from across the edges
          i=x1<<1;	j=y1<<1;
          if(temp.neighbors[i][j][1]){
            if(!temp.neighbors[i][j][1]->children){temp.neighbors[i][j][1]->initChildren();}
            for(k=0;k<2;k++){neighbors[d].neighbors[i][j][z2+k]=&temp.neighbors[i][j][1]->children[Cube::CornerIndex(x2,y2,k)];}
          }
          i=x1<<1;	k=z1<<1;
          if(temp.neighbors[i][1][k]){
            if(!temp.neighbors[i][1][k]->children){temp.neighbors[i][1][k]->initChildren();}
            for(j=0;j<2;j++){neighbors[d].neighbors[i][y2+j][k]=&temp.neighbors[i][1][k]->children[Cube::CornerIndex(x2,j,z2)];}
          }
          j=y1<<1;	k=z1<<1;
          if(temp.neighbors[1][j][k]){
            if(!temp.neighbors[1][j][k]->children){temp.neighbors[1][j][k]->initChildren();}
            for(i=0;i<2;i++){neighbors[d].neighbors[x2+i][j][k]=&temp.neighbors[1][j][k]->children[Cube::CornerIndex(i,y2,z2)];}
          }

          // Set the neighbor from across the corner
          i=x1<<1;	j=y1<<1;	k=z1<<1;
          if(temp.neighbors[i][j][k]){
            if(!temp.neighbors[i][j][k]->children){temp.neighbors[i][j][k]->initChildren();}
            neighbors[d].neighbors[i][j][k]=&temp.neighbors[i][j][k]->children[Cube::CornerIndex(x2,y2,z2)];
          }
        }
      }
      return neighbors[d];
    }
    // Note the assumption is that if you enable an edge, you also enable adjacent faces.
    // And, if you enable a corner, you enable adjacent edges and faces.
    template< class NodeData , class Real >
    typename OctNode<NodeData,Real>::Neighbors3& OctNode<NodeData,Real>::NeighborKey3::setNeighbors( OctNode<NodeData,Real>* node , bool flags[3][3][3] )
    {
      int d = node->depth();
      if( node==neighbors[d].neighbors[1][1][1] )
      {
        bool reset = false;
        for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ ) for( int k=0 ; k<3 ; k++ ) if( flags[i][j][k] && !neighbors[d].neighbors[i][j][k] ) reset = true;
        if( reset ) neighbors[d].neighbors[1][1][1] = NULL;
      }
      if( node!=neighbors[d].neighbors[1][1][1] )
      {
        neighbors[d].clear();

        if( !node->parent ) neighbors[d].neighbors[1][1][1] = node;
        else
        {
          int x1,y1,z1,x2,y2,z2;
          int idx=int(node-node->parent->children);
          Cube::FactorCornerIndex(  idx   ,x1,y1,z1);
          Cube::FactorCornerIndex((~idx)&7,x2,y2,z2);
          for( int i=0 ; i<2 ; i++ )
            for( int j=0 ; j<2 ; j++ )
              for( int k=0 ; k<2 ; k++ )
                neighbors[d].neighbors[x2+i][y2+j][z2+k]=&node->parent->children[Cube::CornerIndex(i,j,k)];

          Neighbors3& temp=setNeighbors( node->parent , flags );

          // Set the neighbors from across the faces
          {
            int i=x1<<1;
            if( temp.neighbors[i][1][1] )
            {
              if( flags[i][1][1] && !temp.neighbors[i][1][1]->children ) temp.neighbors[i][1][1]->initChildren();
              if( temp.neighbors[i][1][1]->children ) for( int j=0 ; j<2 ; j++ ) for( int k=0 ; k<2 ; k++ ) neighbors[d].neighbors[i][y2+j][z2+k] = &temp.neighbors[i][1][1]->children[Cube::CornerIndex(x2,j,k)];
            }
          }
          {
            int j = y1<<1;
            if( temp.neighbors[1][j][1] )
            {
              if( flags[1][j][1] && !temp.neighbors[1][j][1]->children ) temp.neighbors[1][j][1]->initChildren();
              if( temp.neighbors[1][j][1]->children ) for( int i=0 ; i<2 ; i++ ) for( int k=0 ; k<2 ; k++ ) neighbors[d].neighbors[x2+i][j][z2+k] = &temp.neighbors[1][j][1]->children[Cube::CornerIndex(i,y2,k)];
            }
          }
          {
            int k = z1<<1;
            if( temp.neighbors[1][1][k] )
            {
              if( flags[1][1][k] && !temp.neighbors[1][1][k]->children ) temp.neighbors[1][1][k]->initChildren();
              if( temp.neighbors[1][1][k]->children ) for( int i=0 ; i<2 ; i++ ) for( int j=0 ; j<2 ; j++ ) neighbors[d].neighbors[x2+i][y2+j][k] = &temp.neighbors[1][1][k]->children[Cube::CornerIndex(i,j,z2)];
            }
          }

          // Set the neighbors from across the edges
          {
            int i=x1<<1 , j=y1<<1;
            if( temp.neighbors[i][j][1] )
            {
              if( flags[i][j][1] && !temp.neighbors[i][j][1]->children ) temp.neighbors[i][j][1]->initChildren();
              if( temp.neighbors[i][j][1]->children ) for( int k=0 ; k<2 ; k++ ) neighbors[d].neighbors[i][j][z2+k] = &temp.neighbors[i][j][1]->children[Cube::CornerIndex(x2,y2,k)];
            }
          }
          {
            int i=x1<<1 , k=z1<<1;
            if( temp.neighbors[i][1][k] )
            {
              if( flags[i][1][k] && !temp.neighbors[i][1][k]->children ) temp.neighbors[i][1][k]->initChildren();
              if( temp.neighbors[i][1][k]->children ) for( int j=0 ; j<2 ; j++ ) neighbors[d].neighbors[i][y2+j][k] = &temp.neighbors[i][1][k]->children[Cube::CornerIndex(x2,j,z2)];
            }
          }
          {
            int j=y1<<1 , k=z1<<1;
            if( temp.neighbors[1][j][k] )
            {
              if( flags[1][j][k] && !temp.neighbors[1][j][k]->children ) temp.neighbors[1][j][k]->initChildren();
              if( temp.neighbors[1][j][k]->children ) for( int i=0 ; i<2 ; i++ ) neighbors[d].neighbors[x2+i][j][k] = &temp.neighbors[1][j][k]->children[Cube::CornerIndex(i,y2,z2)];
            }
          }

          // Set the neighbor from across the corner
          {
            int i=x1<<1 , j=y1<<1 , k=z1<<1;
            if( temp.neighbors[i][j][k] )
            {
              if( flags[i][j][k] && !temp.neighbors[i][j][k]->children ) temp.neighbors[i][j][k]->initChildren();
              if( temp.neighbors[i][j][k]->children ) neighbors[d].neighbors[i][j][k] = &temp.neighbors[i][j][k]->children[Cube::CornerIndex(x2,y2,z2)];
            }
          }
        }
      }
      return neighbors[d];
    }

    template<class NodeData,class Real>
    typename OctNode<NodeData,Real>::Neighbors3& OctNode<NodeData,Real>::NeighborKey3::getNeighbors(OctNode<NodeData,Real>* node){
      int d=node->depth();
      if(node!=neighbors[d].neighbors[1][1][1]){
        neighbors[d].clear();

        if(!node->parent){neighbors[d].neighbors[1][1][1]=node;}
        else{
          int i,j,k,x1,y1,z1,x2,y2,z2;
          int idx=int(node-node->parent->children);
          Cube::FactorCornerIndex(  idx   ,x1,y1,z1);
          Cube::FactorCornerIndex((~idx)&7,x2,y2,z2);
          for(i=0;i<2;i++){
            for(j=0;j<2;j++){
              for(k=0;k<2;k++){
                neighbors[d].neighbors[x2+i][y2+j][z2+k]=&node->parent->children[Cube::CornerIndex(i,j,k)];
              }
            }
          }
          Neighbors3& temp=getNeighbors(node->parent);

          // Set the neighbors from across the faces
          i=x1<<1;
          if(temp.neighbors[i][1][1] && temp.neighbors[i][1][1]->children){
            for(j=0;j<2;j++){for(k=0;k<2;k++){neighbors[d].neighbors[i][y2+j][z2+k]=&temp.neighbors[i][1][1]->children[Cube::CornerIndex(x2,j,k)];}}
          }
          j=y1<<1;
          if(temp.neighbors[1][j][1] && temp.neighbors[1][j][1]->children){
            for(i=0;i<2;i++){for(k=0;k<2;k++){neighbors[d].neighbors[x2+i][j][z2+k]=&temp.neighbors[1][j][1]->children[Cube::CornerIndex(i,y2,k)];}}
          }
          k=z1<<1;
          if(temp.neighbors[1][1][k] && temp.neighbors[1][1][k]->children){
            for(i=0;i<2;i++){for(j=0;j<2;j++){neighbors[d].neighbors[x2+i][y2+j][k]=&temp.neighbors[1][1][k]->children[Cube::CornerIndex(i,j,z2)];}}
          }

          // Set the neighbors from across the edges
          i=x1<<1;	j=y1<<1;
          if(temp.neighbors[i][j][1] && temp.neighbors[i][j][1]->children){
            for(k=0;k<2;k++){neighbors[d].neighbors[i][j][z2+k]=&temp.neighbors[i][j][1]->children[Cube::CornerIndex(x2,y2,k)];}
          }
          i=x1<<1;	k=z1<<1;
          if(temp.neighbors[i][1][k] && temp.neighbors[i][1][k]->children){
            for(j=0;j<2;j++){neighbors[d].neighbors[i][y2+j][k]=&temp.neighbors[i][1][k]->children[Cube::CornerIndex(x2,j,z2)];}
          }
          j=y1<<1;	k=z1<<1;
          if(temp.neighbors[1][j][k] && temp.neighbors[1][j][k]->children){
            for(i=0;i<2;i++){neighbors[d].neighbors[x2+i][j][k]=&temp.neighbors[1][j][k]->children[Cube::CornerIndex(i,y2,z2)];}
          }

          // Set the neighbor from across the corner
          i=x1<<1;	j=y1<<1;	k=z1<<1;
          if(temp.neighbors[i][j][k] && temp.neighbors[i][j][k]->children){
            neighbors[d].neighbors[i][j][k]=&temp.neighbors[i][j][k]->children[Cube::CornerIndex(x2,y2,z2)];
          }
        }
      }
      return neighbors[node->depth()];
    }

    ///////////////////////
    // ConstNeighborKey3 //
    ///////////////////////
    template<class NodeData,class Real>
    OctNode<NodeData,Real>::ConstNeighbors3::ConstNeighbors3(void){clear();}
    template<class NodeData,class Real>
    void OctNode<NodeData,Real>::ConstNeighbors3::clear(void){
      for(int i=0;i<3;i++){for(int j=0;j<3;j++){for(int k=0;k<3;k++){neighbors[i][j][k]=NULL;}}}
    }
    template<class NodeData,class Real>
    OctNode<NodeData,Real>::ConstNeighborKey3::ConstNeighborKey3(void){neighbors=NULL;}
    template<class NodeData,class Real>
    OctNode<NodeData,Real>::ConstNeighborKey3::~ConstNeighborKey3(void){
      if(neighbors){delete[] neighbors;}
      neighbors=NULL;
    }

    template<class NodeData,class Real>
    void OctNode<NodeData,Real>::ConstNeighborKey3::set(int d){
      if(neighbors){delete[] neighbors;}
      neighbors=NULL;
      if(d<0){return;}
      neighbors=new ConstNeighbors3[d+1];
    }
    template<class NodeData,class Real>
    typename OctNode<NodeData,Real>::ConstNeighbors3& OctNode<NodeData,Real>::ConstNeighborKey3::getNeighbors(const OctNode<NodeData,Real>* node){
      int d=node->depth();
      if(node!=neighbors[d].neighbors[1][1][1]){
        neighbors[d].clear();

        if(!node->parent){neighbors[d].neighbors[1][1][1]=node;}
        else{
          int i,j,k,x1,y1,z1,x2,y2,z2;
          int idx=int(node-node->parent->children);
          Cube::FactorCornerIndex(  idx   ,x1,y1,z1);
          Cube::FactorCornerIndex((~idx)&7,x2,y2,z2);
          for(i=0;i<2;i++){
            for(j=0;j<2;j++){
              for(k=0;k<2;k++){
                neighbors[d].neighbors[x2+i][y2+j][z2+k]=&node->parent->children[Cube::CornerIndex(i,j,k)];
              }
            }
          }
          ConstNeighbors3& temp=getNeighbors(node->parent);

          // Set the neighbors from across the faces
          i=x1<<1;
          if(temp.neighbors[i][1][1] && temp.neighbors[i][1][1]->children){
            for(j=0;j<2;j++){for(k=0;k<2;k++){neighbors[d].neighbors[i][y2+j][z2+k]=&temp.neighbors[i][1][1]->children[Cube::CornerIndex(x2,j,k)];}}
          }
          j=y1<<1;
          if(temp.neighbors[1][j][1] && temp.neighbors[1][j][1]->children){
            for(i=0;i<2;i++){for(k=0;k<2;k++){neighbors[d].neighbors[x2+i][j][z2+k]=&temp.neighbors[1][j][1]->children[Cube::CornerIndex(i,y2,k)];}}
          }
          k=z1<<1;
          if(temp.neighbors[1][1][k] && temp.neighbors[1][1][k]->children){
            for(i=0;i<2;i++){for(j=0;j<2;j++){neighbors[d].neighbors[x2+i][y2+j][k]=&temp.neighbors[1][1][k]->children[Cube::CornerIndex(i,j,z2)];}}
          }

          // Set the neighbors from across the edges
          i=x1<<1;	j=y1<<1;
          if(temp.neighbors[i][j][1] && temp.neighbors[i][j][1]->children){
            for(k=0;k<2;k++){neighbors[d].neighbors[i][j][z2+k]=&temp.neighbors[i][j][1]->children[Cube::CornerIndex(x2,y2,k)];}
          }
          i=x1<<1;	k=z1<<1;
          if(temp.neighbors[i][1][k] && temp.neighbors[i][1][k]->children){
            for(j=0;j<2;j++){neighbors[d].neighbors[i][y2+j][k]=&temp.neighbors[i][1][k]->children[Cube::CornerIndex(x2,j,z2)];}
          }
          j=y1<<1;	k=z1<<1;
          if(temp.neighbors[1][j][k] && temp.neighbors[1][j][k]->children){
            for(i=0;i<2;i++){neighbors[d].neighbors[x2+i][j][k]=&temp.neighbors[1][j][k]->children[Cube::CornerIndex(i,y2,z2)];}
          }

          // Set the neighbor from across the corner
          i=x1<<1;	j=y1<<1;	k=z1<<1;
          if(temp.neighbors[i][j][k] && temp.neighbors[i][j][k]->children){
            neighbors[d].neighbors[i][j][k]=&temp.neighbors[i][j][k]->children[Cube::CornerIndex(x2,y2,z2)];
          }
        }
      }
      return neighbors[node->depth()];
    }
    template<class NodeData,class Real>
    typename OctNode<NodeData,Real>::ConstNeighbors3& OctNode<NodeData,Real>::ConstNeighborKey3::getNeighbors( const OctNode<NodeData,Real>* node , int minDepth )
    {
      int d=node->depth();
      if( d<minDepth ) fprintf( stderr , "[ERROR] Node depth lower than min-depth: %d < %d\n" , d , minDepth ) , exit( 0 );
      if( node!=neighbors[d].neighbors[1][1][1] )
      {
        neighbors[d].clear();

        if( d==minDepth ) neighbors[d].neighbors[1][1][1]=node;
        else
        {
          int i,j,k,x1,y1,z1,x2,y2,z2;
          int idx = int(node-node->parent->children);
          Cube::FactorCornerIndex(  idx   ,x1,y1,z1);
          Cube::FactorCornerIndex((~idx)&7,x2,y2,z2);

          ConstNeighbors3& temp=getNeighbors( node->parent , minDepth );

          //  Set the syblings
          for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
            neighbors[d].neighbors[x2+i][y2+j][z2+k] = &node->parent->children[ Cube::CornerIndex(i,j,k) ];

          // Set the neighbors from across the faces
          i=x1<<1;
          if( temp.neighbors[i][1][1] && temp.neighbors[i][1][1]->children )
            for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[i][y2+j][z2+k] = &temp.neighbors[i][1][1]->children[Cube::CornerIndex(x2,j,k)];

          j=y1<<1;
          if( temp.neighbors[1][j][1] && temp.neighbors[1][j][1]->children )
            for( i=0 ; i<2 ; i++ ) for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[x2+i][j][z2+k] = &temp.neighbors[1][j][1]->children[Cube::CornerIndex(i,y2,k)];

          k=z1<<1;
          if( temp.neighbors[1][1][k] && temp.neighbors[1][1][k]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) neighbors[d].neighbors[x2+i][y2+j][k] = &temp.neighbors[1][1][k]->children[Cube::CornerIndex(i,j,z2)];

          // Set the neighbors from across the edges
          i=x1<<1 , j=y1<<1;
          if( temp.neighbors[i][j][1] && temp.neighbors[i][j][1]->children )
            for( k=0 ; k<2 ; k++ ) neighbors[d].neighbors[i][j][z2+k] = &temp.neighbors[i][j][1]->children[Cube::CornerIndex(x2,y2,k)];

          i=x1<<1 , k=z1<<1;
          if( temp.neighbors[i][1][k] && temp.neighbors[i][1][k]->children )
            for( j=0 ; j<2 ; j++ ) neighbors[d].neighbors[i][y2+j][k] = &temp.neighbors[i][1][k]->children[Cube::CornerIndex(x2,j,z2)];

          j=y1<<1 , k=z1<<1;
          if( temp.neighbors[1][j][k] && temp.neighbors[1][j][k]->children )
            for( i=0 ; i<2 ; i++ ) neighbors[d].neighbors[x2+i][j][k] = &temp.neighbors[1][j][k]->children[Cube::CornerIndex(i,y2,z2)];

          // Set the neighbor from across the corner
          i=x1<<1 , j=y1<<1 , k=z1<<1;
          if( temp.neighbors[i][j][k] && temp.neighbors[i][j][k]->children )
            neighbors[d].neighbors[i][j][k] = &temp.neighbors[i][j][k]->children[Cube::CornerIndex(x2,y2,z2)];
        }
      }
      return neighbors[node->depth()];
    }

    template< class NodeData , class Real > OctNode< NodeData , Real >::Neighbors5::Neighbors5( void ){ clear(); }
    template< class NodeData , class Real > OctNode< NodeData , Real >::ConstNeighbors5::ConstNeighbors5( void ){ clear(); }
    template< class NodeData , class Real >
    void OctNode< NodeData , Real >::Neighbors5::clear( void )
    {
      for( int i=0 ; i<5 ; i++ ) for( int j=0 ; j<5 ; j++ ) for( int k=0 ; k<5 ; k++ ) neighbors[i][j][k] = NULL;
    }
    template< class NodeData , class Real >
    void OctNode< NodeData , Real >::ConstNeighbors5::clear( void )
    {
      for( int i=0 ; i<5 ; i++ ) for( int j=0 ; j<5 ; j++ ) for( int k=0 ; k<5 ; k++ ) neighbors[i][j][k] = NULL;
    }
    template< class NodeData , class Real >
    OctNode< NodeData , Real >::NeighborKey5::NeighborKey5( void )
    {
      _depth = -1;
      neighbors = NULL;
    }
    template< class NodeData , class Real >
    OctNode< NodeData , Real >::ConstNeighborKey5::ConstNeighborKey5( void )
    {
      _depth = -1;
      neighbors = NULL;
    }
    template< class NodeData , class Real >
    OctNode< NodeData , Real >::NeighborKey5::~NeighborKey5( void )
    {
      if( neighbors ) delete[] neighbors;
      neighbors = NULL;
    }
    template< class NodeData , class Real >
    OctNode< NodeData , Real >::ConstNeighborKey5::~ConstNeighborKey5( void )
    {
      if( neighbors ) delete[] neighbors;
      neighbors = NULL;
    }
    template< class NodeData , class Real >
    void OctNode< NodeData , Real >::NeighborKey5::set( int d )
    {
      if( neighbors ) delete[] neighbors;
      neighbors = NULL;
      if(d<0) return;
      _depth = d;
      neighbors=new Neighbors5[d+1];
    }
    template< class NodeData , class Real >
    void OctNode< NodeData , Real >::ConstNeighborKey5::set( int d )
    {
      if( neighbors ) delete[] neighbors;
      neighbors = NULL;
      if(d<0) return;
      _depth = d;
      neighbors=new ConstNeighbors5[d+1];
    }
    template< class NodeData , class Real >
    typename OctNode< NodeData , Real >::Neighbors5& OctNode< NodeData , Real >::NeighborKey5::getNeighbors( OctNode* node )
    {
      int d=node->depth();
      if( node!=neighbors[d].neighbors[2][2][2] )
      {
        neighbors[d].clear();

        if( !node->parent ) neighbors[d].neighbors[2][2][2]=node;
        else
        {
          getNeighbors( node->parent );
          Neighbors5& temp = neighbors[d-1];
          int x1 , y1 , z1 , x2 , y2 , z2;
          int idx = int( node - node->parent->children );
          Cube::FactorCornerIndex( idx , x1 , y1 , z1 );

          Neighbors5& n = neighbors[d];
          Cube::FactorCornerIndex( (~idx)&7 , x2 , y2 , z2 );
          int i , j , k;
          int fx0 = x2+1 , fy0 = y2+1 , fz0 = z2+1;	// Indices of the bottom left corner of the parent within the 5x5x5
          int cx1 = x1*2+1 , cy1 = y1*2+1 , cz1 = z1*2+1;
          int cx2 = x2*2+1 , cy2 = y2*2+1 , cz2 = z2*2+1;
          int fx1 = x1*3 , fy1 = y1*3 , fz1 = z1*3;
          int fx2 = x2*4 , fy2 = y2*4 , fz2 = z2*4;

          //  Set the syblings
          for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
            n.neighbors[fx0+i][fy0+j][fz0+k] = node->parent->children + Cube::CornerIndex( i , j , k );

          // Set the neighbors from across the faces
          if( temp.neighbors[cx1][2][2] && temp.neighbors[cx1][2][2]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx1+i][fy0+j][fz0+k] = temp.neighbors[cx1][2][2]->children + Cube::CornerIndex( i , j , k );
          if( temp.neighbors[2][cy1][2] && temp.neighbors[2][cy1][2]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx0+i][fy1+j][fz0+k] = temp.neighbors[2][cy1][2]->children + Cube::CornerIndex( i , j , k );
          if( temp.neighbors[2][2][cz1] && temp.neighbors[2][2][cz1]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx0+i][fy0+j][fz1+k] = temp.neighbors[2][2][cz1]->children + Cube::CornerIndex( i , j , k );
          if( temp.neighbors[cx2][2][2] && temp.neighbors[cx2][2][2]->children )
            for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx2  ][fy0+j][fz0+k] = temp.neighbors[cx2][2][2]->children + Cube::CornerIndex( x1 , j , k );
          if( temp.neighbors[2][cy2][2] && temp.neighbors[2][cy2][2]->children )
            for( i=0 ; i<2 ; i++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx0+i][fy2  ][fz0+k] = temp.neighbors[2][cy2][2]->children + Cube::CornerIndex( i , y1 , k );
          if( temp.neighbors[2][2][cz2] && temp.neighbors[2][2][cz2]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ )
              n.neighbors[fx0+i][fy0+j][fz2  ] = temp.neighbors[2][2][cz2]->children + Cube::CornerIndex( i , j , z1 );

          // Set the neighbors from across the edges
          if( temp.neighbors[cx1][cy1][2] && temp.neighbors[cx1][cy1][2]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx1+i][fy1+j][fz0+k] = temp.neighbors[cx1][cy1][2]->children + Cube::CornerIndex( i , j , k );
          if( temp.neighbors[cx1][2][cz1] && temp.neighbors[cx1][2][cz1]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx1+i][fy0+j][fz1+k] = temp.neighbors[cx1][2][cz1]->children + Cube::CornerIndex( i , j , k );
          if( temp.neighbors[2][cy1][cz1] && temp.neighbors[2][cy1][cz1]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx0+i][fy1+j][fz1+k] = temp.neighbors[2][cy1][cz1]->children + Cube::CornerIndex( i , j , k );
          if( temp.neighbors[cx1][cy2][2] && temp.neighbors[cx1][cy2][2]->children )
            for( i=0 ; i<2 ; i++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx1+i][fy2  ][fz0+k] = temp.neighbors[cx1][cy2][2]->children + Cube::CornerIndex( i , y1 , k );
          if( temp.neighbors[cx1][2][cz2] && temp.neighbors[cx1][2][cz2]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ )
              n.neighbors[fx1+i][fy0+j][fz2  ] = temp.neighbors[cx1][2][cz2]->children + Cube::CornerIndex( i , j , z1 );
          if( temp.neighbors[cx2][cy1][2] && temp.neighbors[cx2][cy1][2]->children )
            for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx2  ][fy1+j][fz0+k] = temp.neighbors[cx2][cy1][2]->children + Cube::CornerIndex( x1 , j , k );
          if( temp.neighbors[2][cy1][cz2] && temp.neighbors[2][cy1][cz2]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ )
              n.neighbors[fx0+i][fy1+j][fz2  ] = temp.neighbors[2][cy1][cz2]->children + Cube::CornerIndex( i , j , z1 );
          if( temp.neighbors[cx2][2][cz1] && temp.neighbors[cx2][2][cz1]->children )
            for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx2  ][fy0+j][fz1+k] = temp.neighbors[cx2][2][cz1]->children + Cube::CornerIndex( x1 , j , k );
          if( temp.neighbors[2][cy2][cz1] && temp.neighbors[2][cy2][cz1]->children )
            for( i=0 ; i<2 ; i++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx0+i][fy2  ][fz1+k] = temp.neighbors[2][cy2][cz1]->children + Cube::CornerIndex( i , y1 , k );
          if( temp.neighbors[cx2][cy2][2] && temp.neighbors[cx2][cy2][2]->children )
            for( k=0 ; k<2 ; k++ )
              n.neighbors[fx2  ][fy2  ][fz0+k] = temp.neighbors[cx2][cy2][2]->children + Cube::CornerIndex( x1 , y1 , k );
          if( temp.neighbors[cx2][2][cz2] && temp.neighbors[cx2][2][cz2]->children )
            for( j=0 ; j<2 ; j++ )
              n.neighbors[fx2  ][fy0+j][fz2  ] = temp.neighbors[cx2][2][cz2]->children + Cube::CornerIndex( x1 , j , z1 );
          if( temp.neighbors[2][cy2][cz2] && temp.neighbors[2][cy2][cz2]->children )
            for( i=0 ; i<2 ; i++ )
              n.neighbors[fx0+i][fy2  ][fz2  ] = temp.neighbors[2][cy2][cz2]->children + Cube::CornerIndex( i , y1 , z1 );

          // Set the neighbor from across the corners
          if( temp.neighbors[cx1][cy1][cz1] && temp.neighbors[cx1][cy1][cz1]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx1+i][fy1+j][fz1+k] = temp.neighbors[cx1][cy1][cz1]->children + Cube::CornerIndex( i , j , k );
          if( temp.neighbors[cx1][cy1][cz2] && temp.neighbors[cx1][cy1][cz2]->children )
            for( i=0 ; i<2 ; i++ ) for( j=0 ; j<2 ; j++ )
              n.neighbors[fx1+i][fy1+j][fz2  ] = temp.neighbors[cx1][cy1][cz2]->children + Cube::CornerIndex( i , j , z1 );
          if( temp.neighbors[cx1][cy2][cz1] && temp.neighbors[cx1][cy2][cz1]->children )
            for( i=0 ; i<2 ; i++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx1+i][fy2  ][fz1+k] = temp.neighbors[cx1][cy2][cz1]->children + Cube::CornerIndex( i , y1 , k );
          if( temp.neighbors[cx2][cy1][cz1] && temp.neighbors[cx2][cy1][cz1]->children )
            for( j=0 ; j<2 ; j++ ) for( k=0 ; k<2 ; k++ )
              n.neighbors[fx2  ][fy1+j][fz1+k] = temp.neighbors[cx2][cy1][cz1]->children + Cube::CornerIndex( x1 , j , k );
          if( temp.neighbors[cx1][cy2][cz2] && temp.neighbors[cx1][cy2][cz2]->children )
            for( i=0 ; i<2 ; i++ )
              n.neighbors[fx1+i][fy2  ][fz2  ] = temp.neighbors[cx1][cy2][cz2]->children + Cube::CornerIndex( i , y1 , z1 );
          if( temp.neighbors[cx2][cy1][cz2] && temp.neighbors[cx2][cy1][cz2]->children )
            for( j=0 ; j<2 ; j++ )
              n.neighbors[fx2  ][fy1+j][fz2  ] = temp.neighbors[cx2][cy1][cz2]->children + Cube::CornerIndex( x1 , j , z1 );
          if( temp.neighbors[cx2][cy2][cz1] && temp.neighbors[cx2][cy2][cz1]->children )
            for( k=0 ; k<2 ; k++ )
              n.neighbors[fx2  ][fy2  ][fz1+k] = temp.neighbors[cx2][cy2][cz1]->children + Cube::CornerIndex( x1 , y1 , k );
          if( temp.neighbors[cx2][cy2][cz2] && temp.neighbors[cx2][cy2][cz2]->children )
            n.neighbors[fx2  ][fy2  ][fz2  ] = temp.neighbors[cx2][cy2][cz2]->children + Cube::CornerIndex( x1 , y1 , z1 );
        }
      }
      return neighbors[d];
    }
    template< class NodeData , class Real >
    typename OctNode< NodeData , Real >::Neighbors5& OctNode< NodeData , Real >::NeighborKey5::setNeighbors( OctNode* node , int xStart , int xEnd , int yStart , int yEnd , int zStart , int zEnd )
    {
      int d=node->depth();
      if( node!=neighbors[d].neighbors[2][2][2] )
      {
        neighbors[d].clear();

        if( !node->parent ) neighbors[d].neighbors[2][2][2]=node;
        else
        {
          setNeighbors( node->parent , xStart , xEnd , yStart , yEnd , zStart , zEnd );
          Neighbors5& temp = neighbors[d-1];
          int x1 , y1 , z1 , x2 , y2 , z2 , ii , jj , kk;
          int idx = int( node-node->parent->children );
          Cube::FactorCornerIndex( idx , x1 , y1 , z1 );

          for( int i=xStart ; i<xEnd ; i++ )
          {
            x2 = i+x1;
            ii = x2&1;
            x2 = 1+(x2>>1);
            for( int j=yStart ; j<yEnd ; j++ )
            {
              y2 = j+y1;
              jj = y2&1;
              y2 = 1+(y2>>1);
              for( int k=zStart ; k<zEnd ; k++ )
              {
                z2 = k+z1;
                kk = z2&1;
                z2 = 1+(z2>>1);
                if(temp.neighbors[x2][y2][z2] )
                {
                  if( !temp.neighbors[x2][y2][z2]->children ) temp.neighbors[x2][y2][z2]->initChildren();
                  neighbors[d].neighbors[i][j][k] = temp.neighbors[x2][y2][z2]->children + Cube::CornerIndex(ii,jj,kk);
                }
              }
            }
          }
        }
      }
      return neighbors[d];
    }
    template< class NodeData , class Real >
    typename OctNode< NodeData , Real >::ConstNeighbors5& OctNode< NodeData , Real >::ConstNeighborKey5::getNeighbors( const OctNode* node )
    {
      int d=node->depth();
      if( node!=neighbors[d].neighbors[2][2][2] )
      {
        neighbors[d].clear();

        if(!node->parent) neighbors[d].neighbors[2][2][2]=node;
        else
        {
          getNeighbors( node->parent );
          ConstNeighbors5& temp = neighbors[d-1];
          int x1,y1,z1,x2,y2,z2,ii,jj,kk;
          int idx=int(node-node->parent->children);
          Cube::FactorCornerIndex(idx,x1,y1,z1);

          for(int i=0;i<5;i++)
          {
            x2=i+x1;
            ii=x2&1;
            x2=1+(x2>>1);
            for(int j=0;j<5;j++)
            {
              y2=j+y1;
              jj=y2&1;
              y2=1+(y2>>1);
              for(int k=0;k<5;k++)
              {
                z2=k+z1;
                kk=z2&1;
                z2=1+(z2>>1);
                if(temp.neighbors[x2][y2][z2] && temp.neighbors[x2][y2][z2]->children)
                  neighbors[d].neighbors[i][j][k] = temp.neighbors[x2][y2][z2]->children + Cube::CornerIndex(ii,jj,kk);
              }
            }
          }
        }
      }
      return neighbors[d];
    }


    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::write(const char* fileName) const{
      FILE* fp=fopen(fileName,"wb");
      if(!fp){return 0;}
      int ret=write(fp);
      fclose(fp);
      return ret;
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::write(FILE* fp) const{
      fwrite(this,sizeof(OctNode<NodeData,Real>),1,fp);
      if(children){for(int i=0;i<Cube::CORNERS;i++){children[i].write(fp);}}
      return 1;
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::read(const char* fileName){
      FILE* fp=fopen(fileName,"rb");
      if(!fp){return 0;}
      int ret=read(fp);
      fclose(fp);
      return ret;
    }
    template <class NodeData,class Real>
    int OctNode<NodeData,Real>::read(FILE* fp){
      fread(this,sizeof(OctNode<NodeData,Real>),1,fp);
      parent=NULL;
      if(children){
        children=NULL;
        initChildren();
        for(int i=0;i<Cube::CORNERS;i++){
          children[i].read(fp);
          children[i].parent=this;
        }
      }
      return 1;
    }
    template<class NodeData,class Real>
    int OctNode<NodeData,Real>::width(int maxDepth) const {
      int d=depth();
      return 1<<(maxDepth-d);
    }
    template<class NodeData,class Real>
    void OctNode<NodeData,Real>::centerIndex(int maxDepth,int index[DIMENSION]) const {
      int d,o[3];
      depthAndOffset(d,o);
      for(int i=0;i<DIMENSION;i++){index[i]=BinaryNode<Real>::CornerIndex(maxDepth,d+1,o[i]<<1,1);}
    }

  }
}
