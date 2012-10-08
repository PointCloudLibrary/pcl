/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2012 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////
*/

#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

// Dimension of tree bounding boxes
#define ON_RTree_NODE_DIM 3

/// A link list of nodes for reinsertion after a delete operation
struct ON_RTreeListNode
{
  struct ON_RTreeListNode* m_next;      ///< Next in list
  struct ON_RTreeNode* m_node;  ///< ON_RTreeNode
};

/// Variables for finding a split partition
struct ON_RTreePartitionVars
{
  int m_partition[ON_RTree_MAX_NODE_COUNT+1];
  int m_total;
  int m_minFill;
  int m_taken[ON_RTree_MAX_NODE_COUNT+1];
  int m_count[2];
  ON_RTreeBBox m_cover[2];
  double m_area[2];

  ON_RTreeBranch m_branchBuf[ON_RTree_MAX_NODE_COUNT+1];
  int m_branchCount;
  ON_RTreeBBox m_coverSplit;
  double m_coverSplitArea;
}; 

typedef bool (*ON_RTreeSearchCallback)(void* a_context, ON__INT_PTR a_id );

struct ON_RTreeSearchResultCallback
{
  void* m_context;
  ON_RTreeSearchCallback m_resultCallback;
};

static void ChoosePartition(ON_RTreePartitionVars* a_parVars, int a_minFill);
static void CountRec(ON_RTreeNode* a_node, int& a_count);
static void PickSeeds(ON_RTreePartitionVars* a_parVars);
static void InitParVars(ON_RTreePartitionVars* a_parVars, int a_maxRects, int a_minFill);
static void GetBranches(ON_RTreeNode* a_node, ON_RTreeBranch* a_branch, ON_RTreePartitionVars* a_parVars);
static int PickBranch(ON_RTreeBBox* a_rect, ON_RTreeNode* a_node);
static void DisconnectBranch(ON_RTreeNode* a_node, int a_index);
static ON_RTreeBBox NodeCover(ON_RTreeNode* a_node);
static void InitRect(ON_RTreeBBox* a_rect);
static ON_RTreeBBox CombineRectHelper(const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB);
static double CalcRectVolumeHelper(const ON_RTreeBBox* a_rect);
static bool OverlapHelper(const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB);
static double DistanceToCapsuleAxisHelper(const struct ON_RTreeCapsule* a_capsule, const ON_RTreeBBox* a_rect);
static void ClassifyHelper(int a_index, int a_group, struct ON_RTreePartitionVars* a_parVars);
static bool SearchHelper(const ON_RTreeNode* a_node, ON_RTreeBBox* a_rect, ON_RTreeSearchResultCallback& a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_RTreeSearchResult& a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_SimpleArray<ON_RTreeLeaf> &a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_SimpleArray<int> &a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_SimpleArray<void*> &a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, struct ON_RTreeSphere* a_sphere, ON_RTreeSearchResultCallback& a_result );
static bool SearchHelper(const ON_RTreeNode* a_node, struct ON_RTreeCapsule* a_capsule, ON_RTreeSearchResultCallback& a_result );

////////////////////////////////////////////////////////////////
//
// ON_RTreeMemPool
//

size_t ON_MemoryPageSize()
{
#if defined(ON_COMPILER_MSC)
  static size_t pagesize = 0;
  if ( 0 == pagesize )
  {
    SYSTEM_INFO system_info;
    memset(&system_info,0,sizeof(system_info));
    ::GetSystemInfo(&system_info);
    pagesize = system_info.dwPageSize;
    if ( pagesize <= 0 )
      pagesize = 4096;
  }
  return pagesize;
#else
  return 4096;
#endif
}


static size_t SizeofBlkLink()
{
  // Room to reserve for struct ON_RTreeMemPool::Blk.
  // This number should be >= sizeof(struct ON_RTreeMemPool::Blk)
  // and a multiple of 16 to insure proper alignment on all CPUs.
  return 16;
}

static size_t MemPoolBlkSize( size_t leaf_count )
{
  // sizeof_blklink = number of bytes to reserve for the
  // struct ON_RTreeMemPool::Blk header used to keep track
  // of our allocations.
  const size_t sizeof_blklink = SizeofBlkLink();

  // pagesize = OS memory manager page size.  We want the
  // allocated blocks to be some smallish mulitples of pagesize
  // to the active sections of the tree will end up in CPU cache 
  // when the tree is being repeatedly searched.
  size_t pagesize = ON_MemoryPageSize();
  if ( pagesize <= sizeof_blklink )
    pagesize = 4096;

  size_t node_count = 32;
  if ( leaf_count > 0 )
  {
    // When the user can estimate the number of leaves,
    // node_count is an estimate of the number of nodes needed
    // to build the tree.  When node_count is small, we avoid
    // allocating too much memory for a tiny tree.  The goal
    // is to avoid wasting lots of memory when there are thousands
    // of individual trees, most with a less than six leaves.
    // If there are only a few trees, or most of the trees have
    // lots of leaves, then hard coding node_count = 32 works
    // just fine.
    if ( 5*leaf_count < 4*ON_RTree_MAX_NODE_COUNT )
      node_count = 3;
    else if ( 5*leaf_count < 4*ON_RTree_MAX_NODE_COUNT*ON_RTree_MAX_NODE_COUNT )
      node_count = ON_RTree_MAX_NODE_COUNT+1;
  }
  
  // Set "sz" to an multiple of pagesize that is big enough
  // to hold node_count nodes.
  const size_t sizeof_node = sizeof(ON_RTreeNode);
  size_t sz = pagesize;
  size_t nodes_per_blk = ( node_count < 32 )
                       ? node_count
                       : (sz-sizeof_blklink)/sizeof_node;
  while ( nodes_per_blk < node_count )
  {
    sz += pagesize;
    nodes_per_blk = (sz-sizeof_blklink)/sizeof_node;
  }

  // Some lame allocators pad each allocation request and use the extra space
  // to store allocation bookkeeping information.  The "+ 2*sizeof(void*) allows 
  // for up to two pointers of "lame" system overhead per allocation.  The goal
  // is to prevent the "real" allocation from being just a hair bigger that a 
  // multiple of pagesize.
  if ( sz < sizeof_blklink + nodes_per_blk*sizeof_node + 2*sizeof(void*)  ) 
    nodes_per_blk--; // prevent memory manager overhead from allocating another page

  // Return the minimum number of bytes we need for each block. An decent
  // OS should assign this allocation an efficient set of pages.
  return (sizeof_blklink + nodes_per_blk*sizeof_node);
}

ON_RTreeMemPool::ON_RTreeMemPool( ON_MEMORY_POOL* heap, size_t leaf_count  )
: m_nodes(0)
, m_list_nodes(0)
, m_buffer(0)
, m_buffer_capacity(0)
, m_blk_list(0)
, m_sizeof_blk(0)
, m_heap(heap)
, m_sizeof_heap(0)
{
  m_sizeof_blk = MemPoolBlkSize(leaf_count); 
}

ON_RTreeMemPool::~ON_RTreeMemPool()
{
  DeallocateAll();
}

void ON_RTreeMemPool::GrowBuffer()
{
  if ( (0 == m_sizeof_blk) || (0 != m_blk_list && 0 == m_blk_list->m_next) )
  {
    // Setting m_sizeof_blk happens twice per ON_RTreeMemPool.
    // The first time is typically at construction and not here.
    // The (0 == m_sizeof_blk) check is here to support cases 
    // where the ON_RTreeMemPool class is initialized by a "memset(...,0)
    // instead of calling its constructor.  The first block can be small
    // if the caller passed in a leaf_count estimate.  For the second
    // (0 != m_blk_list && 0 == m_blk_list->m_next) and subsequent calls
    // to GrowBuffer(), we use the default block size.
    m_sizeof_blk = MemPoolBlkSize(0); 
  }

  struct Blk* blk = (struct Blk*)onmalloc_from_pool(m_heap,m_sizeof_blk);
  if ( blk )
  {
    m_sizeof_heap += m_sizeof_blk;
    blk->m_next = m_blk_list;
    m_blk_list = blk;
    const size_t sizeof_blklink = SizeofBlkLink();
    m_buffer = ((unsigned char*)m_blk_list) + sizeof_blklink;
    m_buffer_capacity = m_sizeof_blk - sizeof_blklink;
  }
  else
  {
    m_buffer = 0;
    m_buffer_capacity = 0;
    ON_ERROR("ON_RTreeMemPool::GrowBuffer - out of memory");
  }
}

ON_RTreeNode* ON_RTreeMemPool::AllocNode()
{
  ON_RTreeNode* node = (ON_RTreeNode*)m_nodes;
  if ( node )
  {
    m_nodes = m_nodes->m_next;
  }
  else
  {
    const size_t node_sz = sizeof(*node);
    if ( m_buffer_capacity < node_sz )
      GrowBuffer();

    if ( 0 == (node = (ON_RTreeNode*)m_buffer) )
    {
      ON_ERROR("ON_RTreeMemPool::AllocNode() - out of memory");
      return 0;
    }

    m_buffer += node_sz;
    m_buffer_capacity -= node_sz;
  }

  // initialize node
  node->m_count = 0;
  node->m_level = -1;

  return node;
}

void ON_RTreeMemPool::FreeNode(ON_RTreeNode* node)
{
  if ( node )
  {
    struct Blk* blk = (struct Blk*)node;
    blk->m_next = m_nodes;
    m_nodes = blk;
  }
}

struct ON_RTreeListNode* ON_RTreeMemPool::AllocListNode()
{
  struct ON_RTreeListNode* list_node = (struct ON_RTreeListNode*)m_list_nodes;
  if ( list_node )
  {
    m_list_nodes = m_list_nodes->m_next;
  }
  else
  {
    size_t list_node_sz = sizeof(*list_node);
    if ( m_buffer_capacity < list_node_sz )
    {
      GrowBuffer();
    }
    list_node = (struct ON_RTreeListNode*)m_buffer;
    if ( list_node )
    {
      m_buffer += list_node_sz;
      m_buffer_capacity -= list_node_sz;
    }
  }
  return list_node;
}

void ON_RTreeMemPool::FreeListNode(struct ON_RTreeListNode* list_node)
{
  if ( list_node )
  {
    struct Blk* blk = (struct Blk*)list_node;
    blk->m_next = m_list_nodes;
    m_list_nodes = blk;
  }
}

size_t ON_RTreeMemPool::SizeOf() const
{
  return m_sizeof_heap;
}

size_t ON_RTreeMemPool::SizeOfUnusedBuffer() const
{
  const struct Blk* blk;
  size_t sz = m_buffer_capacity;
  for ( blk = m_nodes; blk; blk = blk->m_next )
  {
    sz += sizeof(struct ON_RTreeNode);
  }
  for ( blk = m_list_nodes; blk; blk = blk->m_next )
  {
    sz += sizeof(struct ON_RTreeListNode);
  }
  return sz;
}

void ON_RTreeMemPool::DeallocateAll()
{
  struct Blk* p = m_blk_list;

  m_nodes = 0;
  m_list_nodes = 0;
  m_buffer = 0;
  m_buffer_capacity = 0;
  m_blk_list = 0;
  m_sizeof_blk = 0;
  m_sizeof_heap = 0;

  while(p)
  {
    struct Blk* next = p->m_next; 
    onfree(p);
    p = next; 
  }
}

////////////////////////////////////////////////////////////////
//
// ON_RTreeIterator
//

ON_RTreeIterator::ON_RTreeIterator()
{ 
  Initialize(0);
}

ON_RTreeIterator::ON_RTreeIterator(const class ON_RTree& rtree)
{ 
  Initialize(rtree);
}

ON_RTreeIterator::~ON_RTreeIterator()
{ 
}

const ON_RTreeBranch* ON_RTreeIterator::Value() const
{
  return ( 0 != m_sp )
         ? &m_sp->m_node->m_branch[m_sp->m_branchIndex]
         : 0;
}

bool ON_RTreeIterator::Initialize(const ON_RTree& a_rtree)
{
  return Initialize(a_rtree.Root());
}

bool ON_RTreeIterator::Initialize(const ON_RTreeNode* a_node)
{ 
  m_sp = 0;
  m_root = ( 0 != a_node && a_node->m_count > 0 ) ? a_node : 0;
  return First();
}

bool ON_RTreeIterator::PushChildren(StackElement* sp, bool bFirstChild )
{ 
  StackElement* spmax = &m_stack[0] + MAX_STACK;
  const ON_RTreeNode* node = sp->m_node;
  m_sp = 0;
  // push first leaf coverted by this node onto the stack
  while( 0 != node && node->m_level >= 0 && node->m_count > 0 )
  {
    if ( 0 == node->m_level )
    {
      m_sp = sp;
      return true;
    }
    node = node->m_branch[sp->m_branchIndex].m_child;
    if( ++sp == spmax )
    {
      // Either this is a GIGANTIC R-tree, or, more likely, there is
      // a bug in the code that creates the R-tree and this R-tree
      // is horribly unbalanced. If the case is valid, then we must 
      // increase MAX_STACK and ship a service release.
      ON_ERROR("ON_RTreeIterator::PushFirstChild - stack overflow");
      return false;
    }
    sp->m_node = node;
    sp->m_branchIndex = bFirstChild ? 0 : node->m_count-1; // 0 for first child
  }
  return false;
}

bool ON_RTreeIterator::First()
{ 
  m_sp = 0;
  if ( 0 == m_root || m_root->m_level < 0 || m_root->m_count <= 0 )
    return false;
  m_stack[0].m_node = m_root;
  m_stack[0].m_branchIndex = 0;
  return PushChildren(&m_stack[0],true);
}

bool ON_RTreeIterator::Last()
{ 
  m_sp = 0;
  if ( 0 == m_root || m_root->m_level < 0 || m_root->m_count <= 0 )
    return false;
  m_stack[0].m_node = m_root;
  m_stack[0].m_branchIndex = m_root->m_count - 1;
  return PushChildren(&m_stack[0],false);
}

bool ON_RTreeIterator::Next()
{ 
  if ( 0 == m_sp )
    return false; // invalid iterator

  if ( ++(m_sp->m_branchIndex) < m_sp->m_node->m_count )
    return true; // m_sp->m_node is always at leaf level

  // pop the stack until we find an element with room to move over.
  StackElement* sp0 = &m_stack[0];
  StackElement* sp = m_sp;
  m_sp = 0;
  while ( sp > sp0 )
  {        
    sp--; // pop the stack

    // ++(sp->m_branchIndex) moves to the next element in sp->m_node.
    if ( ++(sp->m_branchIndex) >= sp->m_node->m_count )
      continue; // this element is used up

    // Since we've popped the stack, we cannot be at the leaf level.
    // PushFirst() pushes the first child onto the stack until 
    // it reaches the leaf level.
    return PushChildren(sp,true);
  }
  return false; // we were at the last element and now there are no more.
}

bool ON_RTreeIterator::Prev()
{ 
  if ( 0 == m_sp )
    return false; // invalid iterator

  if ( --(m_sp->m_branchIndex) >= 0 )
    return true; // m_sp->m_node is always at leaf level

  // pop the stack until we find an element with room to move over.
  StackElement* sp0 = &m_stack[0];
  StackElement* sp = m_sp;
  m_sp = 0;
  while ( sp > sp0 )
  {        
    sp--; // pop the stack

    // --(sp->m_branchIndex) moves to the previous element in sp->m_node.
    if ( --(sp->m_branchIndex) < 0 )
      continue; // this element is used up

    // Since we've popped the stack, we cannot be at the leaf level.
    // PushFirst() pushes the first child onto the stack until 
    // it reaches the leaf level.
    return PushChildren(sp,false);
  }
  return false; // we were at the last element and now there are no more.
}


////////////////////////////////////////////////////////////////
//
// ON_RTree
//


ON_RTree::ON_RTree( ON_MEMORY_POOL* heap, size_t leaf_count )
: m_root(0)
, m_reserved(0)
, m_mem_pool(heap,leaf_count)
{
}



ON_RTree::~ON_RTree()
{
  RemoveAll();
}

bool ON_RTree::CreateMeshFaceTree( const ON_Mesh* mesh )
{
  double fmin[3], fmax[3];
  ON_3dPoint V;
  unsigned int fi, fcount;
  const int* fvi;
  const ON_MeshFace* meshF;
  const ON_3fPoint* meshfV;
  const ON_3dPoint* meshdV;

  RemoveAll();

  if ( 0 == mesh )
    return false;

  fcount = mesh->m_F.UnsignedCount();
  if ( fcount <= 0 )
    return false;

  meshF = mesh->m_F.Array();
  if ( 0 == meshF )
    return false;

  meshfV = mesh->m_V.Array();

  meshdV = mesh->HasDoublePrecisionVertices() 
         ? mesh->DoublePrecisionVertices().Array() 
         : 0;

  if ( 0 != meshfV )
  {
    if ( 0 != meshdV )
    {
      for ( fi = 0; fi < fcount; fi++ )
      {
        fvi = meshF[fi].vi;

        V = meshfV[fvi[0]];
        fmin[0] = fmax[0] = V.x;
        fmin[1] = fmax[1] = V.y;
        fmin[2] = fmax[2] = V.z;
        V = meshdV[fvi[0]];
        if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
        if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
        if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  

        V = meshfV[fvi[1]];
        if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
        if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
        if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  
        V = meshdV[fvi[1]];
        if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
        if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
        if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  

        V = meshfV[fvi[2]];
        if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
        if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
        if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  
        V = meshdV[fvi[2]];
        if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
        if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
        if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  

        if ( fvi[2] != fvi[3] )
        {
          V = meshfV[fvi[3]];
          if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
          if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
          if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;      
          V = meshdV[fvi[3]];
          if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
          if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
          if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  
        }

        if ( !Insert(fmin,fmax,fi) )
        {
          RemoveAll();
          return false;
        }
      }
    }
    else
    {
      for ( fi = 0; fi < fcount; fi++ )
      {
        fvi = meshF[fi].vi;

        V = meshfV[fvi[0]];
        fmin[0] = fmax[0] = V.x;
        fmin[1] = fmax[1] = V.y;
        fmin[2] = fmax[2] = V.z;

        V = meshfV[fvi[1]];
        if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
        if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
        if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  

        V = meshfV[fvi[2]];
        if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
        if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
        if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  

        if ( fvi[2] != fvi[3] )
        {
          V = meshfV[fvi[3]];
          if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
          if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
          if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;      
        }

        if ( !Insert(fmin,fmax,fi) )
        {
          RemoveAll();
          return false;
        }
      }
    }
  }
  else if ( 0 != meshdV )
  {
    for ( fi = 0; fi < fcount; fi++ )
    {
      fvi = meshF[fi].vi;

      V = meshdV[fvi[0]];
      fmin[0] = fmax[0] = V.x;
      fmin[1] = fmax[1] = V.y;
      fmin[2] = fmax[2] = V.z;

      V = meshdV[fvi[1]];
      if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
      if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
      if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  

      V = meshdV[fvi[2]];
      if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
      if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
      if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;  

      if ( fvi[2] != fvi[3] )
      {
        V = meshdV[fvi[3]];
        if ( V.x < fmin[0] ) fmin[0] = V.x; else if ( V.x > fmax[0] ) fmax[0] = V.x;
        if ( V.y < fmin[1] ) fmin[1] = V.y; else if ( V.y > fmax[1] ) fmax[1] = V.y;
        if ( V.z < fmin[2] ) fmin[2] = V.z; else if ( V.z > fmax[2] ) fmax[2] = V.z;      
      }

      if ( !Insert(fmin,fmax,fi) )
      {
        RemoveAll();
        return false;
      }
    }
  }
  else
  {
    // no vertices
    return false;
  }

  return (0 != m_root);
}

bool ON_RTree::Insert2d(const double a_min[2], const double a_max[2], int a_element_id)
{
  const double min3d[3] = {a_min[0],a_min[1],0.0};
  const double max3d[3] = {a_max[0],a_max[1],0.0};
  return Insert(min3d,max3d,a_element_id);
}

bool ON_RTree::Insert(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM], int a_element_id)
{
  bool rc;
  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));
  if ( rect.m_min[0] <= rect.m_max[0] && rect.m_min[1] <= rect.m_max[1] && rect.m_min[2] <= rect.m_max[2] )
  {  
    if ( 0 == m_root )
    {
      m_root = m_mem_pool.AllocNode();
      m_root->m_level = 0;
    }
    InsertRect(&rect, a_element_id, &m_root, 0);
    rc = true;
  }
  else
  {
    // invalid bounding box - don't let this corrupt the tree
    rc = false;
    ON_ERROR("ON_RTree::Insert - invalid a_min[] or a_max[] input.");
  }
  return rc;
}

bool ON_RTree::Insert2d(const double a_min[2], const double a_max[2], void* a_element_id)
{
  const double min3d[3] = {a_min[0],a_min[1],0.0};
  const double max3d[3] = {a_max[0],a_max[1],0.0};
  return Insert(min3d,max3d,a_element_id);
}

bool ON_RTree::Insert(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM], void* a_element_id)
{
  bool rc;
  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));
  if ( rect.m_min[0] <= rect.m_max[0] && rect.m_min[1] <= rect.m_max[1] && rect.m_min[2] <= rect.m_max[2] )
  {  
    if ( 0 == m_root )
    {
      m_root = m_mem_pool.AllocNode();
      m_root->m_level = 0;
    }

    // The ON__INT_PTR cast is safe because ON__INT_PTR == sizeof(void*)
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma warning( push )
// Disable warning C4311: 'type cast' : pointer truncation from 'void *' to 'ON__INT_PTR'
#pragma warning( disable : 4311 )
#endif
    InsertRect(&rect, (ON__INT_PTR)a_element_id, &m_root, 0);
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma warning( pop )
#endif

    rc = true;
  }
  else
  {
    // invalid bounding box - don't let this corrupt the tree
    rc = false;
    ON_ERROR("ON_RTree::Insert - invalid a_min[] or a_max[] input.");
  }
  return rc;
}

bool ON_RTree::Remove2d(const double a_min[2], const double a_max[2], int a_dataId)
{
  const double min3d[3] = {a_min[0],a_min[1],0.0};
  const double max3d[3] = {a_max[0],a_max[1],0.0};
  return Remove(min3d,max3d,a_dataId);
}

bool ON_RTree::Remove(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM], int a_dataId)
{
  bool rc = false;
  if ( 0 != m_root )
  {
    ON_RTreeBBox rect;
    memcpy(rect.m_min,a_min,sizeof(rect.m_min));
    memcpy(rect.m_max,a_max,sizeof(rect.m_max));
    if ( rect.m_min[0] <= rect.m_max[0] && rect.m_min[1] <= rect.m_max[1] && rect.m_min[2] <= rect.m_max[2] )
    {  
      // RemoveRect() returns 0 on success
      rc = (0 ==  RemoveRect(&rect, a_dataId, &m_root));
    }
    else
    {
      // invalid bounding box - don't let this corrupt the tree
      ON_ERROR("ON_RTree::Remove - invalid a_min[] or a_max[] input.");
    }
  }
  return rc;
}

bool ON_RTree::Remove2d(const double a_min[2], const double a_max[2],  void* a_dataId)
{
  const double min3d[3] = {a_min[0],a_min[1],0.0};
  const double max3d[3] = {a_max[0],a_max[1],0.0};
  return Remove(min3d,max3d,a_dataId);
}

bool ON_RTree::Remove(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM], void* a_dataId)
{
  bool rc = false;
  if ( 0 != m_root )
  {
    ON_RTreeBBox rect;
    memcpy(rect.m_min,a_min,sizeof(rect.m_min));
    memcpy(rect.m_max,a_max,sizeof(rect.m_max));
    if ( rect.m_min[0] <= rect.m_max[0] && rect.m_min[1] <= rect.m_max[1] && rect.m_min[2] <= rect.m_max[2] )
    {  
      // RemoveRect() returns 0 on success
      // The ON__INT_PTR cast is save because ON__INT_PTR == sizeof(void*)
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma warning( push )
// Disable warning C4311: 'type cast' : pointer truncation from 'void *' to 'ON__INT_PTR'
#pragma warning( disable : 4311 )
#endif
      rc = (0 ==  RemoveRect(&rect, (ON__INT_PTR)a_dataId, &m_root));
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma warning( pop )
#endif

    }
    else
    {
      // invalid bounding box - don't let this corrupt the tree
      ON_ERROR("ON_RTree::Remove - invalid a_min[] or a_max[] input.");
    }
  }
  return rc;
}


bool ON_RTree::Search2d(const double a_min[2], const double a_max[2], 
                          bool ON_MSC_CDECL a_resultCallback(void* a_context, ON__INT_PTR a_data), 
                          void* a_context
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  ON_RTreeSearchResultCallback result;
  result.m_context = a_context;
  result.m_resultCallback = a_resultCallback;
  return SearchHelper(m_root, &rect, result);
}

bool ON_RTree::Search(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM], 
                          bool ON_MSC_CDECL a_resultCallback(void* a_context, ON__INT_PTR a_data), 
                          void* a_context
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));
  return Search( &rect, a_resultCallback, a_context );
}

bool ON_RTree::Search( ON_RTreeBBox* a_rect,
                       bool ON_MSC_CDECL a_resultCallback(void* a_context, ON__INT_PTR a_data), 
                       void* a_context
                      ) const
{
  if ( 0 == m_root || 0 == a_rect )
    return false;

  ON_RTreeSearchResultCallback result;
  result.m_context = a_context;
  result.m_resultCallback = a_resultCallback;
  return SearchHelper(m_root, a_rect, result);
}

bool ON_RTree::Search( 
    struct ON_RTreeSphere* a_sphere,
    bool ON_MSC_CDECL a_resultCallback(void* a_context, ON__INT_PTR a_id), 
    void* a_context
    ) const
{
  if ( 0 == m_root || 0 == a_sphere )
    return false;

  ON_RTreeSearchResultCallback result;
  result.m_context = a_context;
  result.m_resultCallback = a_resultCallback;

  return SearchHelper(m_root, a_sphere, result);
}

bool ON_RTree::Search( 
  struct ON_RTreeCapsule* a_capsule,
  bool ON_MSC_CDECL a_resultCallback(void* a_context, ON__INT_PTR a_id), 
  void* a_context
  ) const
{
  
  if ( 0 == m_root || 0 == a_capsule )
    return false;

  ON_RTreeSearchResultCallback result;
  result.m_context = a_context;
  result.m_resultCallback = a_resultCallback;

  return SearchHelper(m_root, a_capsule, result);
}

bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
                          ON_SimpleArray<ON_RTreeLeaf>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  return SearchHelper(m_root, &rect, a_result);
}


bool ON_RTree::Search(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM],
                          ON_SimpleArray<ON_RTreeLeaf>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));

  return SearchHelper(m_root, &rect, a_result);
}


bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
                          ON_SimpleArray<void*>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  return SearchHelper(m_root, &rect,  a_result);
}


bool ON_RTree::Search(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM],
                          ON_SimpleArray<void*>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));

  return SearchHelper(m_root, &rect,  a_result);
}


bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
                          ON_SimpleArray<int>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  return SearchHelper(m_root, &rect,  a_result);
}

bool ON_RTree::Search(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM],
                          ON_SimpleArray<int>& a_result 
                          ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));

  return SearchHelper(m_root, &rect,  a_result);
}

bool ON_RTree::Search2d(const double a_min[2], const double a_max[2],
                          ON_RTreeSearchResult& a_result ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,2*sizeof(a_min[0]));
  rect.m_min[2] = 0.0;
  memcpy(rect.m_max,a_max,2*sizeof(a_max[0]));
  rect.m_max[2] = 0.0;

  return SearchHelper(m_root, &rect, a_result);
}

bool ON_RTree::Search(const double a_min[ON_RTree_NODE_DIM], const double a_max[ON_RTree_NODE_DIM],
                          ON_RTreeSearchResult& a_result ) const
{
  if ( 0 == m_root )
    return false;

  ON_RTreeBBox rect;
  memcpy(rect.m_min,a_min,sizeof(rect.m_min));
  memcpy(rect.m_max,a_max,sizeof(rect.m_max));

  return SearchHelper(m_root, &rect, a_result);
}

struct ON_RTreePairSearchResult
{
  double m_tolerance;
  ON_SimpleArray<ON_2dex>* m_result;
};


static bool PairSearchOverlapHelper( const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB, double tolerance )
{
  double dx,dy,dz,d;
  const double* mn;
  const double* mx;


  mx = a_rectA->m_max;
  mn = a_rectB->m_min;
  dx = *mn++ - *mx++;
  if ( dx > tolerance ) return false;
  dy = *mn++ - *mx++;
  if ( dy > tolerance ) return false;
  dz = *mn - *mx;
  if ( dz > tolerance ) return false;

  mx = a_rectB->m_max;
  mn = a_rectA->m_min;
  d = *mn++ - *mx++;
  if ( d > tolerance ) return false;
  if ( d > dx ) dx = d;
  d = *mn++ - *mx++;
  if ( d > tolerance ) return false;
  if ( d > dy ) dy = d;
  d = *mn++ - *mx++;
  if ( d > tolerance ) return false;
  if ( d > dz ) dz = d;

  d  = (dx > 0.0) ? dx*dx : 0.0;
  d += (dy > 0.0) ? dy*dy : 0.0;
  d += (dz > 0.0) ? dz*dz : 0.0;

  return (d <= tolerance*tolerance);
}


static void PairSearchHelper( const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;
  while(branchB < branchBmax)
  {
    if ( PairSearchOverlapHelper( &a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeB->m_level > 0 )
      {
        PairSearchHelper(a_branchA,branchB->m_child,a_result);
      }
      else
      {
        ON_2dex& r = a_result->m_result->AppendNew();
        r.i = (int)a_branchA->m_id;
        r.j = (int)branchB->m_id;
      }
    }
    branchB++;
  }
}

static void PairSearchHelper( const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  while(branchA < branchAmax)
  {
    if ( PairSearchOverlapHelper( &branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeA->m_level > 0 )
      {
        PairSearchHelper(branchA->m_child,a_branchB,a_result);
      }
      else
      {
        ON_2dex& r = a_result->m_result->AppendNew();
        r.i = (int)branchA->m_id;
        r.j = (int)a_branchB->m_id;
      }
    }
    branchA++;
  }
}


static void PairSearchHelper( const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;
  while(branchA < branchAmax)
  {
    for ( branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++ )
    {
      if ( PairSearchOverlapHelper( &branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
      {
        if ( a_nodeA->m_level > 0 )
        {
          if ( a_nodeB->m_level > 0 )
            PairSearchHelper(branchA->m_child,branchB->m_child,a_result);
          else
            PairSearchHelper(branchA->m_child,branchB,a_result);
        }
        else if ( a_nodeB->m_level > 0 )
        {
          PairSearchHelper(branchA,branchB->m_child,a_result);
        }
        else
        {
          ON_2dex& r = a_result->m_result->AppendNew();
          r.i = (int)branchA->m_id;
          r.j = (int)branchB->m_id;
        }
      }
    }
    branchA++;
  }
}


bool ON_RTree::Search( 
          const ON_RTree& a_rtreeA,
          const ON_RTree& a_rtreeB, 
          double tolerance,
          ON_SimpleArray<ON_2dex>& a_result
          )
{
  if ( 0 == a_rtreeA.m_root )
    return false;
  if ( 0 == a_rtreeB.m_root )
    return false;
  ON_RTreePairSearchResult r;
  r.m_tolerance = (ON_IsValid(tolerance) && tolerance > 0.0) ? tolerance : 0.0;
  r.m_result = &a_result;
  PairSearchHelper(a_rtreeA.m_root,a_rtreeB.m_root,&r);
  return true;
}

typedef void (*ON_RTreePairSearchCallback)(void*, ON__INT_PTR, ON__INT_PTR);

struct ON_RTreePairSearchCallbackResult
{
  double m_tolerance;
  void* m_context;
  ON_RTreePairSearchCallback m_resultCallback;
};

typedef bool (*ON_RTreePairSearchCallbackBool)(void*, ON__INT_PTR, ON__INT_PTR);

struct ON_RTreePairSearchCallbackResultBool
{
  double m_tolerance;
  void* m_context;
  ON_RTreePairSearchCallbackBool m_resultCallbackBool;
};

static void PairSearchHelper( const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;
  while(branchB < branchBmax)
  {
    if ( PairSearchOverlapHelper( &a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeB->m_level > 0 )
      {
        PairSearchHelper(a_branchA,branchB->m_child,a_result);
      }
      else
      {
        a_result->m_resultCallback(a_result->m_context,a_branchA->m_id,branchB->m_id);
      }
    }
    branchB++;
  }
}

static bool PairSearchHelperBool( const ON_RTreeBranch* a_branchA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBool* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchB, *branchBmax;

  branchB = a_nodeB->m_branch;
  branchBmax = branchB + a_nodeB->m_count;
  while(branchB < branchBmax)
  {
    if ( PairSearchOverlapHelper( &a_branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeB->m_level > 0 )
      {
        if ( !PairSearchHelperBool(a_branchA,branchB->m_child,a_result) )
          return false;
      }
      else
      {
        if ( !a_result->m_resultCallbackBool(a_result->m_context,a_branchA->m_id,branchB->m_id) )
          return false;
      }
    }
    branchB++;
  }
  return true;
}

static void PairSearchHelper( const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchCallbackResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  while(branchA < branchAmax)
  {
    if ( PairSearchOverlapHelper( &branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeA->m_level > 0 )
      {
        PairSearchHelper(branchA->m_child,a_branchB,a_result);
      }
      else
      {
        a_result->m_resultCallback(a_result->m_context,branchA->m_id,a_branchB->m_id);
      }
    }
    branchA++;
  }
}

static bool PairSearchHelperBool( const ON_RTreeNode* a_nodeA, const ON_RTreeBranch* a_branchB, ON_RTreePairSearchCallbackResultBool* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  while(branchA < branchAmax)
  {
    if ( PairSearchOverlapHelper( &branchA->m_rect, &a_branchB->m_rect, a_result->m_tolerance ) )
    {
      if ( a_nodeA->m_level > 0 )
      {
        if ( !PairSearchHelperBool(branchA->m_child,a_branchB,a_result) )
          return false;
      }
      else
      {
        if ( !a_result->m_resultCallbackBool(a_result->m_context,branchA->m_id,a_branchB->m_id) )
          return false;
      }
    }
    branchA++;
  }
  return true;
}


static void PairSearchHelper( const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResult* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;
  while(branchA < branchAmax)
  {
    for ( branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++ )
    {
      if ( PairSearchOverlapHelper( &branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
      {
        if ( a_nodeA->m_level > 0 )
        {
          if ( a_nodeB->m_level > 0 )
            PairSearchHelper(branchA->m_child,branchB->m_child,a_result);
          else
            PairSearchHelper(branchA->m_child,branchB,a_result);
        }
        else if ( a_nodeB->m_level > 0 )
        {
          PairSearchHelper(branchA,branchB->m_child,a_result);
        }
        else
        {
          a_result->m_resultCallback(a_result->m_context,branchA->m_id,branchB->m_id);
        }
      }
    }
    branchA++;
  }
}


static bool PairSearchHelperBool( const ON_RTreeNode* a_nodeA, const ON_RTreeNode* a_nodeB, ON_RTreePairSearchCallbackResultBool* a_result )
{
  // DO NOT ADD ANYTHING TO THIS FUNCTION
  const ON_RTreeBranch *branchA, *branchAmax, *branchB, *branchBmax;

  branchA = a_nodeA->m_branch;
  branchAmax = branchA + a_nodeA->m_count;
  branchBmax = a_nodeB->m_branch + a_nodeB->m_count;
  while(branchA < branchAmax)
  {
    for ( branchB = a_nodeB->m_branch; branchB < branchBmax; branchB++ )
    {
      if ( PairSearchOverlapHelper( &branchA->m_rect, &branchB->m_rect, a_result->m_tolerance ) )
      {
        if ( a_nodeA->m_level > 0 )
        {
          if ( a_nodeB->m_level > 0 )
          {
            if ( !PairSearchHelperBool(branchA->m_child,branchB->m_child,a_result) )
              return false;
          }
          else
          {
            if ( !PairSearchHelperBool(branchA->m_child,branchB,a_result) )
              return false;
          }
        }
        else if ( a_nodeB->m_level > 0 )
        {
          if ( !PairSearchHelperBool(branchA,branchB->m_child,a_result) )
            return false;
        }
        else
        {
          if ( !a_result->m_resultCallbackBool(a_result->m_context,branchA->m_id,branchB->m_id) )
            return false;
        }
      }
    }
    branchA++;
  }
  return true;
}

bool ON_RTree::Search( 
          const ON_RTree& a_rtreeA,
          const ON_RTree& a_rtreeB, 
          double tolerance,
          void ON_MSC_CDECL resultCallback(void* a_context,ON__INT_PTR a_idA, ON__INT_PTR a_idB),
          void* a_context
          )
{
  if ( 0 == a_rtreeA.m_root )
    return false;
  if ( 0 == a_rtreeB.m_root )
    return false;
  ON_RTreePairSearchCallbackResult r;
  r.m_tolerance = (ON_IsValid(tolerance) && tolerance > 0.0) ? tolerance : 0.0;
  r.m_context = a_context;
  r.m_resultCallback = resultCallback;
  PairSearchHelper(a_rtreeA.m_root,a_rtreeB.m_root,&r);
  return true;
}

bool ON_RTree::Search( 
          const ON_RTree& a_rtreeA,
          const ON_RTree& a_rtreeB, 
          double tolerance,
          bool ON_MSC_CDECL resultCallback(void* a_context,ON__INT_PTR a_idA, ON__INT_PTR a_idB),
          void* a_context
          )
{
  if ( 0 == a_rtreeA.m_root )
    return false;
  if ( 0 == a_rtreeB.m_root )
    return false;
  ON_RTreePairSearchCallbackResultBool r;
  r.m_tolerance = (ON_IsValid(tolerance) && tolerance > 0.0) ? tolerance : 0.0;
  r.m_context = a_context;
  r.m_resultCallbackBool = resultCallback;

  // Do not return false if PairSearchHelperBool() returns false.  The only reason
  // PairSearchHelperBool() returns false is that the user specified resultCallback()
  // terminated the search. This way a programmer with the ability to reason can
  // distinguish between a terminiation and a failure to start because input is
  // missing.
  PairSearchHelperBool(a_rtreeA.m_root,a_rtreeB.m_root,&r);

  return true;
}




int ON_RTree::ElementCount()
{
  int count = 0;

  if ( 0 != m_root )
    CountRec(m_root, count);
  
  return count;
}

const ON_RTreeNode* ON_RTree::Root() const
{
  return m_root;
}

ON_BoundingBox ON_RTree::BoundingBox() const
{
  ON_BoundingBox bbox;
  if ( 0 != m_root && m_root->m_count > 0 )
  {
    bbox.m_min = m_root->m_branch[0].m_rect.m_min;
    bbox.m_max = m_root->m_branch[0].m_rect.m_max;
    for ( int i = 1; i < m_root->m_count; i++ )
    {
      if ( m_root->m_branch[i].m_rect.m_min[0] < bbox.m_min.x )
        bbox.m_min.x = m_root->m_branch[i].m_rect.m_min[0];
      if ( m_root->m_branch[i].m_rect.m_min[1] < bbox.m_min.y )
        bbox.m_min.y = m_root->m_branch[i].m_rect.m_min[1];
      if ( m_root->m_branch[i].m_rect.m_min[2] < bbox.m_min.z )
        bbox.m_min.z = m_root->m_branch[i].m_rect.m_min[2];

      if ( m_root->m_branch[i].m_rect.m_max[0] > bbox.m_max.x )
        bbox.m_max.x = m_root->m_branch[i].m_rect.m_max[0];
      if ( m_root->m_branch[i].m_rect.m_max[1] > bbox.m_max.y )
        bbox.m_max.y = m_root->m_branch[i].m_rect.m_max[1];
      if ( m_root->m_branch[i].m_rect.m_max[2] > bbox.m_max.z )
        bbox.m_max.z = m_root->m_branch[i].m_rect.m_max[2];
    }
  }
  return bbox;
}

static void CountRec(ON_RTreeNode* a_node, int& a_count)
{
  if(a_node->IsInternalNode())  // not a leaf node
  {
    for(int index = 0; index < a_node->m_count; ++index)
    {
      CountRec(a_node->m_branch[index].m_child, a_count);
    }
  }
  else // A leaf node
  {
    a_count += a_node->m_count;
  }
}

size_t ON_RTree::SizeOf() const
{
  return m_mem_pool.SizeOf();
}


static void NodeCountHelper( const ON_RTreeNode* node, size_t& node_count, size_t& wasted_branch_count, size_t& leaf_count )
{
  if ( 0 == node )
    return;
  node_count++;
  wasted_branch_count += (ON_RTree_MAX_NODE_COUNT - node->m_count);
  if ( node->m_level > 0 )
  {
    for ( int i = 0; i < node->m_count; i++ )
    {
      NodeCountHelper(node->m_branch[i].m_child,node_count,wasted_branch_count,leaf_count);
    }
  }
  else
    leaf_count += node->m_count;
}

void ON_RTree::RemoveAll()
{
  m_root = 0;
  m_mem_pool.DeallocateAll();
}

void ON_RTree::RemoveAllRec(ON_RTreeNode* a_node)
{
  if(a_node->IsInternalNode()) // This is an internal node in the tree
  {
    for(int index=0; index < a_node->m_count; ++index)
    {
      RemoveAllRec(a_node->m_branch[index].m_child);
    }
  }
  m_mem_pool.FreeNode(a_node); 
}



static void InitRect(ON_RTreeBBox* a_rect)
{
  for(int index = 0; index < ON_RTree_NODE_DIM; ++index)
  {
    a_rect->m_min[index] = (double)0;
    a_rect->m_max[index] = (double)0;
  }
}


// Inserts a new data rectangle into the index structure.
// Recursively descends tree, propagates splits back up.
// Returns 0 if node was not split.  Old node updated.
// If node was split, returns 1 and sets the pointer pointed to by
// new_node to point to the new node.  Old node updated to become one of two.
// The level argument specifies the number of steps up from the leaf
// level to insert; e.g. a data rectangle goes in at level = 0.

bool ON_RTree::InsertRectRec(ON_RTreeBBox* a_rect, ON__INT_PTR a_id, ON_RTreeNode* a_node, ON_RTreeNode** a_newNode, int a_level)
{
  int index;
  ON_RTreeBranch branch;
  ON_RTreeNode* otherNode;

  // Still above level for insertion, go down tree recursively
  if(a_node->m_level > a_level)
  {
    index = PickBranch(a_rect, a_node);
    if ( index < 0 )
    {
      return false;
    }
    if (!InsertRectRec(a_rect, a_id, a_node->m_branch[index].m_child, &otherNode, a_level))
    {
      // Child was not split
      a_node->m_branch[index].m_rect = CombineRectHelper(a_rect, &(a_node->m_branch[index].m_rect));
      return false;
    }
    else // Child was split
    {
      a_node->m_branch[index].m_rect = NodeCover(a_node->m_branch[index].m_child);
      branch.m_child = otherNode;
      branch.m_rect = NodeCover(otherNode);
      return AddBranch(&branch, a_node, a_newNode);
    }
  }
  else if(a_node->m_level == a_level) // Have reached level for insertion. Add rect, split if necessary
  {
    branch.m_rect = *a_rect;

    // The (ON_RTreeNode*) cast is safe because ON__INT_PTR == sizeof(void*)
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma warning( push )
// Disable warning C4312: 'type cast' : conversion from 'ON__INT_PTR' to 'ON_RTreeNode *' of greater size
#pragma warning( disable : 4312 )
#endif
    branch.m_child = (ON_RTreeNode*)a_id;
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma warning( pop )
#endif

    // Child field of leaves contains id of data record
    return AddBranch(&branch, a_node, a_newNode);
  }

  // We should never get here
  ON_ERROR("ON_RTree::InsertRectRec - bug in algorithm");
  return false;
}


// Insert a data rectangle into an index structure.
// InsertRect provides for splitting the root;
// returns 1 if root was split, 0 if it was not.
// The level argument specifies the number of steps up from the leaf
// level to insert; e.g. a data rectangle goes in at level = 0.
// InsertRect2 does the recursion.
//

bool ON_RTree::InsertRect(ON_RTreeBBox* a_rect, ON__INT_PTR a_id, ON_RTreeNode** a_root, int a_level)
{
  ON_RTreeNode* newRoot;
  ON_RTreeNode* newNode;
  ON_RTreeBranch branch;

  if(InsertRectRec(a_rect, a_id, *a_root, &newNode, a_level))  // Root split
  {
    newRoot = m_mem_pool.AllocNode();  // Grow tree taller and new root
    newRoot->m_level = (*a_root)->m_level + 1;
    branch.m_rect = NodeCover(*a_root);
    branch.m_child = *a_root;
    AddBranch(&branch, newRoot, NULL);
    branch.m_rect = NodeCover(newNode);
    branch.m_child = newNode;
    AddBranch(&branch, newRoot, NULL);
    *a_root = newRoot;
    return true;
  }

  return false;
}


// Find the smallest rectangle that includes all rectangles in branches of a node.

static ON_RTreeBBox NodeCover(ON_RTreeNode* a_node)
{
  int i;
  const ON_RTreeBranch* branch;
  ON_RTreeBBox rect;

  if ( (i = a_node->m_count) > 0 )
  {
    rect = a_node->m_branch[--i].m_rect;
    for ( branch = a_node->m_branch; i; i--, branch++ )
    {
#if (3 == ON_RTree_NODE_DIM)
      if ( rect.m_min[0] > branch->m_rect.m_min[0] )
        rect.m_min[0] = branch->m_rect.m_min[0];
      if ( rect.m_min[1] > branch->m_rect.m_min[1] )
        rect.m_min[1] = branch->m_rect.m_min[1];
      if ( rect.m_min[2] > branch->m_rect.m_min[2] )
        rect.m_min[2] = branch->m_rect.m_min[2];
      if ( rect.m_max[0] < branch->m_rect.m_max[0] )
        rect.m_max[0] = branch->m_rect.m_max[0];
      if ( rect.m_max[1] < branch->m_rect.m_max[1] )
        rect.m_max[1] = branch->m_rect.m_max[1];
      if ( rect.m_max[2] < branch->m_rect.m_max[2] )
        rect.m_max[2] = branch->m_rect.m_max[2];
#else
      for( int j = 0; j < ON_RTree_NODE_DIM; j++ )
      {
        if ( rect.m_min[j] > branch->m_rect.m_min[j] )
          rect.m_min[j] = branch->m_rect.m_min[j];
        if ( rect.m_max[j] < branch->m_rect.m_max[j] )
          rect.m_max[j] = branch->m_rect.m_max[j];
      }
#endif
    }
  }
  else
  {
    InitRect(&rect);
  }

  return rect;
}


// Add a branch to a node.  Split the node if necessary.
// Returns 0 if node not split.  Old node updated.
// Returns 1 if node split, sets *new_node to address of new node.
// Old node updated, becomes one of two.

bool ON_RTree::AddBranch(ON_RTreeBranch* a_branch, ON_RTreeNode* a_node, ON_RTreeNode** a_newNode)
{
  if(a_node->m_count < ON_RTree_MAX_NODE_COUNT)  // Split won't be necessary
  {
    a_node->m_branch[a_node->m_count] = *a_branch;
    ++a_node->m_count;

    return false;
  }
  else
  {
    SplitNode(a_node, a_branch, a_newNode);
    return true;
  }
}


// Disconnect a dependent node.
// Caller must return (or stop using iteration index) after this as count has changed

static void DisconnectBranch(ON_RTreeNode* a_node, int a_index)
{
  // Remove element by swapping with the last element to prevent gaps in array
  a_node->m_branch[a_index] = a_node->m_branch[a_node->m_count - 1];
  
  --a_node->m_count;
}


// Pick a branch.  Pick the one that will need the smallest increase
// in area to accomodate the new rectangle.  This will result in the
// least total area for the covering rectangles in the current node.
// In case of a tie, pick the one which was smaller before, to get
// the best resolution when searching.

static int PickBranch(ON_RTreeBBox* a_rect, ON_RTreeNode* a_node)
{
  bool firstTime = true;
  double increase;
  double bestIncr = -1.0;
  double area;
  double bestArea;
  int best;
  ON_RTreeBBox tempRect;

  best = -1;
  bestArea = -1.0;

  for(int index=0; index < a_node->m_count; ++index)
  {
    ON_RTreeBBox* curRect = &a_node->m_branch[index].m_rect;
    area = CalcRectVolumeHelper(curRect);
    tempRect = CombineRectHelper(a_rect, curRect);
    increase = CalcRectVolumeHelper(&tempRect) - area;
    if((increase < bestIncr) || firstTime)
    {
      best = index;
      bestArea = area;
      bestIncr = increase;
      firstTime = false;
    }
    else if((increase == bestIncr) && (area <=bestArea))
    {
      best = index;
      bestArea = area;
      bestIncr = increase;
    }
  }
  return best;
}


// Combine two rectangles into larger one containing both

ON_RTreeBBox CombineRectHelper(const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB)
{
  ON_RTreeBBox rect = *a_rectA;

#if (3 == ON_RTree_NODE_DIM)
  if ( rect.m_min[0] > a_rectB->m_min[0] )
    rect.m_min[0] = a_rectB->m_min[0];
  if ( rect.m_min[1] > a_rectB->m_min[1] )
    rect.m_min[1] = a_rectB->m_min[1];
  if ( rect.m_min[2] > a_rectB->m_min[2] )
    rect.m_min[2] = a_rectB->m_min[2];
  if ( rect.m_max[0] < a_rectB->m_max[0] )
    rect.m_max[0] = a_rectB->m_max[0];
  if ( rect.m_max[1] < a_rectB->m_max[1] )
    rect.m_max[1] = a_rectB->m_max[1];
  if ( rect.m_max[2] < a_rectB->m_max[2] )
    rect.m_max[2] = a_rectB->m_max[2];
#else
  for( int j = 0; j < ON_RTree_NODE_DIM; j++ )
  {
    if ( rect.m_min[j] > a_rectB->m_min[j] )
      rect.m_min[j] = a_rectB->m_min[j];
    if ( rect.m_max[j] < a_rectB->m_max[j] )
      rect.m_max[j] = a_rectB->m_max[j];
  }
#endif

  return rect;
}



// Split a node.
// Divides the nodes branches and the extra one between two nodes.
// Old node is one of the new ones, and one really new one is created.
// Tries more than one method for choosing a partition, uses best result.

void ON_RTree::SplitNode(ON_RTreeNode* a_node, ON_RTreeBranch* a_branch, ON_RTreeNode** a_newNode)
{
  ON_RTreePartitionVars localVars;
  int level;

  // Load all the branches into a buffer, initialize a_node to be empty
  level = a_node->m_level; // save m_level (The InitNode() call in GetBranches will set it to -1)
  GetBranches(a_node, a_branch, &localVars);

  // Find partition
  ChoosePartition(&localVars, ON_RTree_MIN_NODE_COUNT);

  // Put branches from buffer into 2 nodes according to chosen partition
  *a_newNode = m_mem_pool.AllocNode();
  (*a_newNode)->m_level = a_node->m_level = level; // restore m_level
  LoadNodes(a_node, *a_newNode, &localVars);
}

double CalcRectVolumeHelper(const ON_RTreeBBox* a_rect)
{
  double d, r;

  // Bounding sphere volume calculation is slower, but helps certain merge cases
#if ( 3 == ON_RTree_NODE_DIM)
  // 3d bounding sphere volume
  d = (a_rect->m_max[0] - a_rect->m_min[0]);
  r = d * d;
  d = (a_rect->m_max[1] - a_rect->m_min[1]);
  r += d * d;
  d = (a_rect->m_max[2] - a_rect->m_min[2]);
  r += d * d;
  r = sqrt(r*0.5); // r = sqrt((dx^2 + dy^2 + dz^2)/2);
  return (r * r * r * 4.1887902047863909846168578443727); // 4/3 pi r^3
#elif ( 2 == ON_RTree_NODE_DIM )
  // 2d bounding circle volume
  d = (a_rect->m_max[0] - a_rect->m_min[0]);
  r = d * d;
  d = (a_rect->m_max[1] - a_rect->m_min[1]);
  r += d * d;
  r = sqrt(r*0.5); // r = sqrt((dx^2 + dy^2)/2);
  return (r * r * ON_PI);
#else

  // n-dim unit sphere volumes
  //  0.000000f, 2.000000f, 3.141593f, // Dimension  0,1,2
  //  4.188790f, 4.934802f, 5.263789f, // Dimension  3,4,5
  //  5.167713f, 4.724766f, 4.058712f, // Dimension  6,7,8
  //  3.298509f, 2.550164f, 1.884104f, // Dimension  9,10,11
  //  1.335263f, 0.910629f, 0.599265f, // Dimension  12,13,14
  //  0.381443f, 0.235331f, 0.140981f, // Dimension  15,16,17
  //  0.082146f, 0.046622f, 0.025807f, // Dimension  18,19,20 
  //return (unit_sphere_volume * radius^ON_RTree_NODE_DIM);

  // Faster rectangle volume calculation, but can cause poor merges
  d = a_rect->m_max[0] - a_rect->m_min[0];
  for(int i = 1; i < ON_RTree_NODE_DIM; ++i)
  {
    d *= a_rect->m_max[i] - a_rect->m_min[i];
  }
  return d;
#endif
}


// Load branch buffer with branches from full node plus the extra branch.

static void GetBranches(ON_RTreeNode* a_node, ON_RTreeBranch* a_branch, ON_RTreePartitionVars* a_parVars)
{
  // Load the branch buffer
  for(int index=0; index < ON_RTree_MAX_NODE_COUNT; ++index)
  {
    a_parVars->m_branchBuf[index] = a_node->m_branch[index];
  }
  a_parVars->m_branchBuf[ON_RTree_MAX_NODE_COUNT] = *a_branch;
  a_parVars->m_branchCount = ON_RTree_MAX_NODE_COUNT + 1;

  // Calculate rect containing all in the set
  a_parVars->m_coverSplit = a_parVars->m_branchBuf[0].m_rect;
  for(int index=1; index < ON_RTree_MAX_NODE_COUNT+1; ++index)
  {
    a_parVars->m_coverSplit = CombineRectHelper(&a_parVars->m_coverSplit, &a_parVars->m_branchBuf[index].m_rect);
  }
  a_parVars->m_coverSplitArea = CalcRectVolumeHelper(&a_parVars->m_coverSplit);

  a_node->m_count = 0;
  a_node->m_level = -1;
}


// Method #0 for choosing a partition:
// As the seeds for the two groups, pick the two rects that would waste the
// most area if covered by a single rectangle, i.e. evidently the worst pair
// to have in the same group.
// Of the remaining, one at a time is chosen to be put in one of the two groups.
// The one chosen is the one with the greatest difference in area expansion
// depending on which group - the rect most strongly attracted to one group
// and repelled from the other.
// If one group gets too full (more would force other group to violate min
// fill requirement) then other group gets the rest.
// These last are the ones that can go in either group most easily.

static void ChoosePartition(ON_RTreePartitionVars* a_parVars, int a_minFill)
{
  double biggestDiff;
  int group, chosen, betterGroup;
  
  InitParVars(a_parVars, a_parVars->m_branchCount, a_minFill);
  PickSeeds(a_parVars);

  while (((a_parVars->m_count[0] + a_parVars->m_count[1]) < a_parVars->m_total)
       && (a_parVars->m_count[0] < (a_parVars->m_total - a_parVars->m_minFill))
       && (a_parVars->m_count[1] < (a_parVars->m_total - a_parVars->m_minFill)))
  {
    biggestDiff = -1.0;
    chosen = 0;
    betterGroup = 0;
    for(int index=0; index<a_parVars->m_total; ++index)
    {
      if(!a_parVars->m_taken[index])
      {
        ON_RTreeBBox* curRect = &a_parVars->m_branchBuf[index].m_rect;
        ON_RTreeBBox rect0 = CombineRectHelper(curRect, &a_parVars->m_cover[0]);
        ON_RTreeBBox rect1 = CombineRectHelper(curRect, &a_parVars->m_cover[1]);
        double growth0 = CalcRectVolumeHelper(&rect0) - a_parVars->m_area[0];
        double growth1 = CalcRectVolumeHelper(&rect1) - a_parVars->m_area[1];
        double diff = growth1 - growth0;
        if(diff >= 0)
        {
          group = 0;
        }
        else
        {
          group = 1;
          diff = -diff;
        }

        if(diff > biggestDiff)
        {
          biggestDiff = diff;
          chosen = index;
          betterGroup = group;
        }
        else if((diff == biggestDiff) && (a_parVars->m_count[group] < a_parVars->m_count[betterGroup]))
        {
          chosen = index;
          betterGroup = group;
        }
      }
    }
    ClassifyHelper(chosen, betterGroup, a_parVars);
  }

  // If one group too full, put remaining rects in the other
  if((a_parVars->m_count[0] + a_parVars->m_count[1]) < a_parVars->m_total)
  {
    if(a_parVars->m_count[0] >= a_parVars->m_total - a_parVars->m_minFill)
    {
      group = 1;
    }
    else
    {
      group = 0;
    }
    for(int index=0; index<a_parVars->m_total; ++index)
    {
      if(!a_parVars->m_taken[index])
      {
        ClassifyHelper(index, group, a_parVars);
      }
    }
  }
}


// Copy branches from the buffer into two nodes according to the partition.

void ON_RTree::LoadNodes(ON_RTreeNode* a_nodeA, ON_RTreeNode* a_nodeB, ON_RTreePartitionVars* a_parVars)
{
  for(int index=0; index < a_parVars->m_total; ++index)
  {
    if(a_parVars->m_partition[index] == 0)
    {
      AddBranch(&a_parVars->m_branchBuf[index], a_nodeA, NULL);
    }
    else if(a_parVars->m_partition[index] == 1)
    {
      AddBranch(&a_parVars->m_branchBuf[index], a_nodeB, NULL);
    }
  }
}


// Initialize a ON_RTreePartitionVars structure.

static void InitParVars(ON_RTreePartitionVars* a_parVars, int a_maxRects, int a_minFill)
{
  a_parVars->m_count[0] = a_parVars->m_count[1] = 0;
  a_parVars->m_area[0] = a_parVars->m_area[1] = (double)0;
  a_parVars->m_total = a_maxRects;
  a_parVars->m_minFill = a_minFill;
  for(int index=0; index < a_maxRects; ++index)
  {
    a_parVars->m_taken[index] = false;
    a_parVars->m_partition[index] = -1;
  }
}



static void PickSeeds(ON_RTreePartitionVars* a_parVars)
{
  int seed0 = 0, seed1 = 1;
  double worst, waste;
  double area[ON_RTree_MAX_NODE_COUNT+1];

  for(int index=0; index<a_parVars->m_total; ++index)
  {
    area[index] = CalcRectVolumeHelper(&a_parVars->m_branchBuf[index].m_rect);
  }

  worst = -a_parVars->m_coverSplitArea - 1;
  for(int indexA=0; indexA < a_parVars->m_total-1; ++indexA)
  {
    for(int indexB = indexA+1; indexB < a_parVars->m_total; ++indexB)
    {
      ON_RTreeBBox oneRect = CombineRectHelper(&a_parVars->m_branchBuf[indexA].m_rect, &a_parVars->m_branchBuf[indexB].m_rect);
      waste = CalcRectVolumeHelper(&oneRect) - area[indexA] - area[indexB];
      if(waste > worst)
      {
        worst = waste;
        seed0 = indexA;
        seed1 = indexB;
      }
    }
  }
  ClassifyHelper(seed0, 0, a_parVars);
  ClassifyHelper(seed1, 1, a_parVars);
}

// Put a branch in one of the groups.

void ClassifyHelper(int a_index, int a_group, ON_RTreePartitionVars* a_parVars)
{
  a_parVars->m_partition[a_index] = a_group;
  a_parVars->m_taken[a_index] = true;

  if (a_parVars->m_count[a_group] == 0)
  {
    a_parVars->m_cover[a_group] = a_parVars->m_branchBuf[a_index].m_rect;
  }
  else
  {
    a_parVars->m_cover[a_group] = CombineRectHelper(&a_parVars->m_branchBuf[a_index].m_rect, &a_parVars->m_cover[a_group]);
  }
  a_parVars->m_area[a_group] = CalcRectVolumeHelper(&a_parVars->m_cover[a_group]);
  ++a_parVars->m_count[a_group];
}


// Delete a data rectangle from an index structure.
// Pass in a pointer to a ON_RTreeBBox, the tid of the record, ptr to ptr to root node.
// Returns 1 if record not found, 0 if success.
// RemoveRect provides for eliminating the root.

bool ON_RTree::RemoveRect(ON_RTreeBBox* a_rect, ON__INT_PTR a_id, ON_RTreeNode** a_root)
{
  ON_RTreeNode* tempNode;
  ON_RTreeListNode* reInsertList = NULL;

  if(!RemoveRectRec(a_rect, a_id, *a_root, &reInsertList))
  {
    // Found and deleted a data item
    // Reinsert any branches from eliminated nodes
    while(reInsertList)
    {
      tempNode = reInsertList->m_node;

      for(int index = 0; index < tempNode->m_count; ++index)
      {
        InsertRect(&(tempNode->m_branch[index].m_rect),
                   tempNode->m_branch[index].m_id,
                   a_root,
                   tempNode->m_level);
      }
      
      ON_RTreeListNode* remLNode = reInsertList;
      reInsertList = reInsertList->m_next;
      
      m_mem_pool.FreeNode(remLNode->m_node);
      m_mem_pool.FreeListNode(remLNode);
    }
    
    // Check for redundant root (not leaf, 1 child) and eliminate
    if((*a_root)->m_count == 1 && (*a_root)->IsInternalNode())
    {
      tempNode = (*a_root)->m_branch[0].m_child;
      m_mem_pool.FreeNode(*a_root);
      *a_root = tempNode;
    }
    return false;
  }
  else
  {
    return true;
  }
}


// Delete a rectangle from non-root part of an index structure.
// Called by RemoveRect.  Descends tree recursively,
// merges branches on the way back up.
// Returns 1 if record not found, 0 if success.

bool ON_RTree::RemoveRectRec(ON_RTreeBBox* a_rect, ON__INT_PTR a_id, ON_RTreeNode* a_node, ON_RTreeListNode** a_listNode)
{
  if(a_node->IsInternalNode())  // not a leaf node
  {
    for(int index = 0; index < a_node->m_count; ++index)
    {
      if(OverlapHelper(a_rect, &(a_node->m_branch[index].m_rect)))
      {
        if(!RemoveRectRec(a_rect, a_id, a_node->m_branch[index].m_child, a_listNode))
        {
          if(a_node->m_branch[index].m_child->m_count >= ON_RTree_MIN_NODE_COUNT)
          {
            // child removed, just resize parent rect
            a_node->m_branch[index].m_rect = NodeCover(a_node->m_branch[index].m_child);
          }
          else
          {
            // child removed, not enough entries in node, eliminate node
            ReInsert(a_node->m_branch[index].m_child, a_listNode);
            DisconnectBranch(a_node, index); // Must return after this call as count has changed
          }
          return false;
        }
      }
    }
    return true;
  }
  else // A leaf node
  {
    for(int index = 0; index < a_node->m_count; ++index)
    {
      if(a_node->m_branch[index].m_id == a_id)
      {
        DisconnectBranch(a_node, index); // Must return after this call as count has changed
        return false;
      }
    }
    return true;
  }
}


// Decide whether two rectangles overlap.
bool OverlapHelper(const ON_RTreeBBox* a_rectA, const ON_RTreeBBox* a_rectB)
{
  const double* mn;
  const double* mx;

  mx = a_rectA->m_max;
  mn = a_rectB->m_min;
  if ( *mx++ < *mn++ ) return false;
  if ( *mx++ < *mn++ ) return false;
  if ( *mx   < *mn   ) return false;

  mx = a_rectB->m_max;
  mn = a_rectA->m_min;
  if ( *mx++ < *mn++ ) return false;
  if ( *mx++ < *mn++ ) return false;
  if ( *mx   < *mn   ) return false;

  return true;
}

//static bool OverlapHelper(const struct ON_RTreeSphere* a_sphere, const ON_RTreeBBox* a_rect)
//{
//  double d[3], t, r;
//  const double* mn;
//  const double* mx;
//  const double* pt;
//
//  pt = a_sphere->m_point;
//  r  = a_sphere->m_radius;
//  mn = a_rect->m_min;
//  mx = a_rect->m_max;
//
//  if ( *pt < *mn )
//  {
//    d[0] = *mn - *pt;
//    if ( d[0] > r )
//      return false;
//  }
//  else if ( *pt > *mx )
//  {
//    d[0] = *pt - *mx;
//    if ( d[0] > r )
//      return false;
//  }
//  else
//  {
//    d[0] = 0.0;
//  }
//
//  mn++;
//  mx++;
//  pt++;
//  if ( *pt < *mn )
//  {
//    d[1] = *mn - *pt;
//    if ( d[1] > r )
//      return false;
//    if ( d[1] > d[0] )
//    {
//      t = d[0]; d[0] = d[1]; d[1] = t;
//    }
//  }
//  else if ( *pt > *mx )
//  {
//    d[1] = *pt - *mx;
//    if ( d[1] > r )
//      return false;
//    if ( d[1] > d[0] )
//    {
//      t = d[0]; d[0] = d[1]; d[1] = t;
//    }
//  }
//  else
//  {
//    d[1] = 0.0;
//  }
//
//  mn++;
//  mx++;
//  pt++;
//  if ( *pt < *mn )
//  {
//    d[2] = *mn - *pt;
//    if ( d[2] > r )
//      return false;
//    if ( d[2] > d[0] )
//    {
//      t = d[0]; d[0] = d[2]; d[2] = t;
//    }
//  }
//  else if ( *pt > *mx )
//  {
//    d[2] = *pt - *mx;
//    if ( d[2] > r )
//      return false;
//    if ( d[2] > d[0] )
//    {
//      t = d[0]; d[0] = d[2]; d[2] = t;
//    }
//  }
//  else
//  {
//    d[2] = 0.0;
//  }
//
//  if ( d[0] > 0.0 )
//  {
//    d[1] /= d[0];
//    d[2] /= d[0];
//    d[0] *= sqrt(1.0 + d[1]*d[1] + d[2]*d[2]);
//    return (d[0] <= r);
//  }
//
//  return true;
//}

static double DistanceToBoxHelper( 
  const double* pt, 
  double r, 
  const ON_RTreeBBox* a_rect
  )
{
  // If the sphere with center at pt and radius r intersects a_rect, then
  // the distance from pt to a_rect is returned. A value of 0.0 indicates
  // that pt is inside a_rect.  If the distance from pt to a_rect is
  // greater than r, then some number > r and <= actual distance from
  // pt to a_rect is returned as quickly as possible.

  double d[3], t;
  const double* mn;
  const double* mx;

  mn = a_rect->m_min;
  mx = a_rect->m_max;

  if ( *pt < *mn )
  {
    d[0] = *mn - *pt;
    if ( d[0] > r )
      return d[0];
  }
  else if ( *pt > *mx )
  {
    d[0] = *pt - *mx;
    if ( d[0] > r )
      return d[0];
  }
  else
  {
    d[0] = 0.0;
  }

  mn++;
  mx++;
  pt++;
  if ( *pt < *mn )
  {
    d[1] = *mn - *pt;
    if ( d[1] > r )
      return d[1];
    if ( d[1] > d[0] )
    {
      t = d[0]; d[0] = d[1]; d[1] = t;
    }
  }
  else if ( *pt > *mx )
  {
    d[1] = *pt - *mx;
    if ( d[1] > r )
      return d[1];
    if ( d[1] > d[0] )
    {
      t = d[0]; d[0] = d[1]; d[1] = t;
    }
  }
  else
  {
    d[1] = 0.0;
  }

  mn++;
  mx++;
  pt++;
  if ( *pt < *mn )
  {
    d[2] = *mn - *pt;
    if ( d[2] > r )
      return d[2];
    if ( d[2] > d[0] )
    {
      t = d[0]; d[0] = d[2]; d[2] = t;
    }
  }
  else if ( *pt > *mx )
  {
    d[2] = *pt - *mx;
    if ( d[2] > r )
      return d[2];
    if ( d[2] > d[0] )
    {
      t = d[0]; d[0] = d[2]; d[2] = t;
    }
  }
  else
  {
    d[2] = 0.0;
  }

  if ( d[0] > 0.0 )
  {
    d[1] /= d[0];
    d[2] /= d[0];
    d[0] *= sqrt(1.0 + d[1]*d[1] + d[2]*d[2]);
  }

  return d[0];
}

static double DistanceToCapsuleAxisHelper(const struct ON_RTreeCapsule* a_capsule, const ON_RTreeBBox* a_rect)
{
  double L[2][3], s[2];
  if ( 0.0 == a_capsule->m_domain[0] && 1.0 == a_capsule->m_domain[1] )
    return ((const ON_BoundingBox*)a_rect->m_min)->MinimumDistanceTo( *((const ON_Line*)a_capsule->m_point[0]) );

  if ( 0.0 == a_capsule->m_domain[0] )
  {
    L[0][0] = a_capsule->m_point[0][0];
    L[0][1] = a_capsule->m_point[0][1];
    L[0][2] = a_capsule->m_point[0][2];
  }
  else
  {
    s[0] = 1.0 - a_capsule->m_domain[0];
    s[1] = a_capsule->m_domain[0];
    L[0][0] = s[0]*a_capsule->m_point[0][0] + s[1]*a_capsule->m_point[1][0];
    L[0][1] = s[0]*a_capsule->m_point[0][1] + s[1]*a_capsule->m_point[1][1];
    L[0][2] = s[0]*a_capsule->m_point[0][2] + s[1]*a_capsule->m_point[1][2];
  }

  if ( 0.0 == a_capsule->m_domain[1] )
  {
    L[1][0] = a_capsule->m_point[1][0];
    L[1][1] = a_capsule->m_point[1][1];
    L[1][2] = a_capsule->m_point[1][2];
  }
  else
  {
    s[0] = 1.0 - a_capsule->m_domain[1];
    s[1] = a_capsule->m_domain[1];
    L[1][0] = s[0]*a_capsule->m_point[0][0] + s[1]*a_capsule->m_point[1][0];
    L[1][1] = s[0]*a_capsule->m_point[0][1] + s[1]*a_capsule->m_point[1][1];
    L[1][2] = s[0]*a_capsule->m_point[0][2] + s[1]*a_capsule->m_point[1][2];
  }

  return ((const ON_BoundingBox*)a_rect->m_min)->MinimumDistanceTo( *((const ON_Line*)L[0]) );
}

// Add a node to the reinsertion list.  All its branches will later
// be reinserted into the index structure.

void ON_RTree::ReInsert(ON_RTreeNode* a_node, ON_RTreeListNode** a_listNode)
{
  ON_RTreeListNode* newListNode;

  newListNode = m_mem_pool.AllocListNode();
  newListNode->m_node = a_node;
  newListNode->m_next = *a_listNode;
  *a_listNode = newListNode;
}


static
bool OverlapBoundedPlaneXYZHelper( const double* a_bounded_plane, const ON_RTreeBBox* a_rect )
{
  unsigned char flag = 0;
  double x, y, z, d, v;
  
  // check the 8 corners of the box minimizing the number of evaluations
  // and unrolling the loop for speed

  // corner = (min, min, min)
  x = a_bounded_plane[0]*a_rect->m_min[0]; 
  y = a_bounded_plane[1]*a_rect->m_min[1];
  z = a_bounded_plane[2]*a_rect->m_min[2];
  d = a_bounded_plane[3];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
    flag = 1;
  else if ( v > a_bounded_plane[5] )
    flag = 2;
  else
    return true;
  
  // corner = (max, min, min)
  x = a_bounded_plane[0]*a_rect->m_max[0]; 
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;
  
  // corner = (max, max, min)
  y = a_bounded_plane[1]*a_rect->m_max[1];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;
    
  // corner = (max, max, max)
  z = a_bounded_plane[2]*a_rect->m_max[2];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;

  // corner = (min, max, max)
  x = a_bounded_plane[0]*a_rect->m_min[0]; 
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;
  
  // corner = (min, min, max)
  y = a_bounded_plane[1]*a_rect->m_min[1];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;

  // corner = (max, min, max)
  x = a_bounded_plane[0]*a_rect->m_max[0]; 
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;

  // corner = (min, max, min)
  x = a_bounded_plane[0]*a_rect->m_min[0]; 
  y = a_bounded_plane[1]*a_rect->m_max[1];
  z = a_bounded_plane[2]*a_rect->m_min[2];
  v = x + y + z + d;
  if ( v < a_bounded_plane[4] )
  {
    flag |= 1;
    if ( 3 == flag )
      return true;
  }
  else if ( v > a_bounded_plane[5] )
  {
    flag |= 2;
    if ( 3 == flag )
      return true;
  }
  else
    return true;
  
  // Either all 8 box corners 
  // are below the min plane (flag=1)
  // or above the max plane (flag=2).
  return false;
}

static
bool SearchBoundedPlaneXYZHelper(const ON_RTreeNode* a_node, const double* a_bounded_plane, ON_RTreeSearchResultCallback& a_result ) 
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapBoundedPlaneXYZHelper(a_bounded_plane, &branch[i].m_rect))
        {
          if(!SearchBoundedPlaneXYZHelper(branch[i].m_child, a_bounded_plane, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapBoundedPlaneXYZHelper(a_bounded_plane, &branch[i].m_rect))
        {
          if ( !a_result.m_resultCallback( a_result.m_context, branch[i].m_id ) )
          {
            // callback canceled search
            return false;
          }
        }
      }
    }
  }

  return true; // Continue searching
}

bool ON_RTree::Search(
  const double a_plane_eqn[4],
  double a_min,
  double a_max,
  bool ON_MSC_CDECL a_resultCallback(void* a_context, ON__INT_PTR a_id), 
  void* a_context
  ) const
{
  if (    0 == m_root 
       || 0 == a_plane_eqn 
       || !(a_min <= a_max) 
       || (0.0 == a_plane_eqn[0] && 0.0 == a_plane_eqn[1] && 0.0 == a_plane_eqn[2])
     )
    return false;

  double bounded_plane[6];
  bounded_plane[0] = a_plane_eqn[0];
  bounded_plane[1] = a_plane_eqn[1];
  bounded_plane[2] = a_plane_eqn[2];
  bounded_plane[3] = a_plane_eqn[3];
  bounded_plane[4] = a_min;
  bounded_plane[5] = a_max;

  ON_RTreeSearchResultCallback result;
  result.m_context = a_context;
  result.m_resultCallback = a_resultCallback;

  return SearchBoundedPlaneXYZHelper(m_root, bounded_plane, result);
}

// Search in an index tree or subtree for all data retangles that overlap the argument rectangle.

static
bool SearchHelper(const ON_RTreeNode* a_node, ON_RTreeBBox* a_rect, ON_RTreeSearchResultCallback& a_result ) 
{
  // NOTE: 
  //  Some versions of ON_RTree::Search shrink a_rect as the search progresses.
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if ( !a_result.m_resultCallback( a_result.m_context, branch[i].m_id ) )
          {
            // callback canceled search
            return false;
          }
        }
      }
    }
  }

  return true; // Continue searching
}

static
bool SearchHelper(const ON_RTreeNode* a_node, struct ON_RTreeSphere* a_sphere, ON_RTreeSearchResultCallback& a_result ) 
{
  // NOTE: 
  //  Some versions of ON_RTree::Search shrink a_sphere as the search progresses.
  int i, closest_i, count;
  const double* sphere_center;
  const ON_RTreeBranch* branch;
  double r[ON_RTree_MAX_NODE_COUNT], sphere_radius, closest_d;
  
  if ( (count = a_node->m_count) > 0 )
  {
    branch = a_node->m_branch;
    sphere_center = a_sphere->m_point;
    sphere_radius = a_sphere->m_radius;
    closest_i = -1;
    closest_d = sphere_radius;

    for( i = 0; i < count; ++i )
    {
      // The radius parameter passed to DistanceToBoxHelper()
      // needs to be sphere_radius and not closest_d in order 
      // for the for() loops below to work correctly.
      r[i] = DistanceToBoxHelper( sphere_center, sphere_radius, &branch[i].m_rect );
      if ( r[i] <= closest_d )
      {
        closest_d = r[i];
        closest_i = i;
      }
    }

    // If all of the branches rectangles do not intersect the sphere,
    // then closest_i = -1.
    if ( closest_i >= 0 )
    {
      if(a_node->IsInternalNode()) 
      {
        // a_node is an internal node - search m_branch[].m_child as needed.
        // Search a closer node first to avoid worst case search times
        // in calculations where the calls to a_result.m_resultCallback()
        // reduce a_sphere->m_radius as results are found.  Closest point
        // calculations are an example.
        if ( !SearchHelper(branch[closest_i].m_child, a_sphere, a_result) )
        {
          // callback canceled search
          return false;
        }

        for( i = 0; i < count; ++i )
        {
          // Note that the calls to SearchHelper() can reduce the
          // value of a_sphere->m_radius.
          if( i != closest_i && r[i] <= a_sphere->m_radius )
          {
            if(!SearchHelper(branch[i].m_child, a_sphere, a_result) )
            {
              return false; // Don't continue searching
            }
          }
        }
      }
      else
      {
        // a_node is a leaf node - return m_branch[].m_id values
        // Search a closer node first to avoid worst case search times
        // in calculations where the calls to a_result.m_resultCallback()
        // reduce a_sphere->m_radius as results are found.  Closest point
        // calculations are an example.
        if ( !a_result.m_resultCallback( a_result.m_context, branch[closest_i].m_id ) ) 
        {
          // callback canceled search
          return false;
        }

        for( i = 0; i < count; ++i)
        {
          // Note that the calls to a_result.m_resultCallback() can reduce
          // the value of a_sphere->m_radius.
          if( i != closest_i && r[i] <= a_sphere->m_radius )
          {
            if ( !a_result.m_resultCallback( a_result.m_context, branch[i].m_id ) )
            {
              // callback canceled search
              return false;
            }
          }
        }
      }
    }
  }

  return true; // Continue searching
}


static
bool SearchHelper(const ON_RTreeNode* a_node, struct ON_RTreeCapsule* a_capsule, ON_RTreeSearchResultCallback& a_result ) 
{
  // NOTE: 
  //  Some versions of ON_RTree::Search shrink a_sphere as the search progresses.
  int i, count;
  double r[2];
  
  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      if ( count > 1 )
      {
        // search a closer node first to avoid worst case search times
        // in closest point style calculations
        r[0] = DistanceToCapsuleAxisHelper( a_capsule, &branch[0].m_rect );
        r[1] = DistanceToCapsuleAxisHelper( a_capsule, &branch[count-1].m_rect );
        i = ( r[0] <= r[1] ) ? 0 : count-1;
        if (    (r[i?1:0] <= a_capsule->m_radius && !SearchHelper(branch[i].m_child, a_capsule, a_result)) 
             || (r[i?0:1] <= a_capsule->m_radius && !SearchHelper(branch[count-1 - i].m_child, a_capsule, a_result))
          )
        {
          // callback canceled search
          return false;
        }
        count -= 2;
        branch++;
      }

      r[1] = a_capsule->m_radius;
      for( i = 0; i < count; ++i )
      {
        r[0] = DistanceToCapsuleAxisHelper(a_capsule, &branch[i].m_rect);
        if(r[0] <= r[1])
        {
          if(!SearchHelper(branch[i].m_child, a_capsule, a_result) )
          {
            return false; // Don't continue searching
          }
          // a_result.m_resultCallback can shrink the capsule
          r[1] = a_capsule->m_radius;
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      if ( count > 1 )
      {
        // search a closer node first to avoid worst case search times
        // in closest point style calculations
        r[0] = DistanceToCapsuleAxisHelper( a_capsule, &branch[0].m_rect );
        r[1] = DistanceToCapsuleAxisHelper( a_capsule, &branch[count-1].m_rect );
        i = ( r[0] <= r[1] ) ? 0 : count-1;
        if (    (r[i?1:0] <= a_capsule->m_radius && !a_result.m_resultCallback( a_result.m_context, branch[i].m_id )) 
             || (r[i?0:1] <= a_capsule->m_radius && !a_result.m_resultCallback( a_result.m_context, branch[count-1 - i].m_id ))
          )
        {
          // callback canceled search
          return false;
        }
        count -= 2;
        branch++;
      }

      r[1] = a_capsule->m_radius;
      for( i = 0; i < count; ++i)
      {
        r[0] = DistanceToCapsuleAxisHelper(a_capsule, &branch[i].m_rect);
        if(r[0] <= r[1])
        {
          if ( !a_result.m_resultCallback( a_result.m_context, branch[i].m_id ) )
          {
            // callback canceled search
            return false;
          }
          // a_result.m_resultCallback can shrink the capsule
          r[1] = a_capsule->m_radius;
        }
      }
    }
  }

  return true; // Continue searching
}


// Search in an index tree or subtree for all data retangles that overlap the argument rectangle.

static bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_SimpleArray<ON_RTreeLeaf> &a_result)
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return point to the branch
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          ON_RTreeLeaf& leaf = a_result.AppendNew();
          leaf.m_rect = branch[i].m_rect;
          leaf.m_id = branch[i].m_id;
        }
      }
    }
  }

  return true; // Continue searching
}


static
bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_SimpleArray<void*> &a_result)
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          // The (void*) cast is safe because branch[i].m_id is an ON__INT_PTR
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma warning( push )
// Disable warning C4312: 'type cast' : conversion from 'const ON__INT_PTR' to 'void *' of greater size
#pragma warning( disable : 4312 )
#endif
          a_result.Append( (void*)branch[i].m_id );
#if defined(ON_COMPILER_MSC) && 4 == ON_SIZEOF_POINTER
#pragma warning( pop )
#endif
        }
      }
    }
  }

  return true; // Continue searching
}

static
bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_SimpleArray<int> &a_result)
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          a_result.Append( (int)branch[i].m_id );
        }
      }
    }
  }

  return true; // Continue searching
}

static
bool SearchHelper(const ON_RTreeNode* a_node, const ON_RTreeBBox* a_rect, ON_RTreeSearchResult& a_result)
{
  int i, count;

  if ( (count = a_node->m_count) > 0 )
  {
    const ON_RTreeBranch* branch = a_node->m_branch;
    if(a_node->IsInternalNode()) 
    {
      // a_node is an internal node - search m_branch[].m_child as needed
      for( i=0; i < count; ++i )
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if(!SearchHelper(branch[i].m_child, a_rect, a_result) )
          {
            return false; // Don't continue searching
          }
        }
      }
    }
    else
    {
      // a_node is a leaf node - return m_branch[].m_id values
      for(i=0; i < count; ++i)
      {
        if(OverlapHelper(a_rect, &branch[i].m_rect))
        {
          if ( a_result.m_count >= a_result.m_capacity )
            return false; // No more space for results
          a_result.m_id[a_result.m_count++] = branch[i].m_id;
        }
      }
    }
  }

  return true; // Continue searching
}

