#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

ON_FixedSizePool::ON_FixedSizePool()
{
  memset(this,0,sizeof(*this));
}

ON_FixedSizePool::~ON_FixedSizePool()
{
  Destroy();
}

size_t ON_FixedSizePool::SizeofElement() const
{
  return m_sizeof_element;
}


bool ON_FixedSizePool::Create( 
        size_t sizeof_element, 
        size_t element_count_estimate,
        size_t block_element_capacity
        )
{
  if ( sizeof_element <= 0 )
  {
    ON_ERROR( "ON_FixedSizePool::Create - sizeof_element <= 0" );
    return false;
  }

  if ( m_sizeof_element != 0 || 0 != m_first_block )
  {
    ON_ERROR( "ON_FixedSizePool::Create - called on a pool that is in use." );
    return false;
  }

  memset(this,0,sizeof(*this));

  m_sizeof_element = sizeof_element;

  if ( block_element_capacity <= 0 )
  {
    size_t page_size = ON_MemoryPageSize();
    if ( page_size < 512 )
      page_size = 512;

    // The "overhead" is for the 2*sizeof(void*) ON_FixedSizePool uses at
    // the start of each block + 32 bytes extra for the heap manager
    // to keep the total allocation not exceeding multiple of page_size.
    const size_t overhead = 2*sizeof(void*) + 32;

    size_t page_count = 1;
    block_element_capacity = (page_count*page_size - overhead)/m_sizeof_element;
    while ( block_element_capacity < 1000 )
    {
      page_count *= 2;
      block_element_capacity = (page_count*page_size - overhead)/m_sizeof_element;
      if (page_count > 8 && block_element_capacity > 64)
      {
        // for pools with large elements
        break;
      }
    }
  }

  // capacity for the the 2nd and subsequent blocks
  m_block_element_count = block_element_capacity;

  // Set m_al_count = capacity of the first block.

  // If the estimated number of elements is not too big, 
  // then make the first block that size.
  if ( element_count_estimate > 0 )
  {
    // this is the first block and it has a custom size
    if ( 8*m_block_element_count >= element_count_estimate )
      m_al_count = element_count_estimate;
    else 
      m_al_count = 8*m_block_element_count; // first block will be large
  }
  else
  {
    m_al_count = m_block_element_count;
  }

  return true;
}

void ON_FixedSizePool::ReturnAll()
{
  if ( 0 != m_first_block )
  {
    // initialize
    m_al_element_stack = 0;
    m_qwerty_it_block = 0;
    m_qwerty_it_element = 0;
    m_al_block = m_first_block;
    m_al_element_array = (void*)(((char*)m_al_block) + 2*sizeof(void*));
    m_al_count = BlockElementCapacity(m_first_block); 
    m_active_element_count = 0;
    m_total_element_count = 0;
  }
}

void ON_FixedSizePool::Destroy()
{
  void* p;
  void* next;
  next = m_first_block;
  memset(this,0,sizeof(*this));
  for ( p = next; 0 != p; p = next )
  {
    next = *((void**)p);
    onfree(p);
  }
}

void ON_FixedSizePool::EmergencyDestroy()
{
  memset(this,0,sizeof(*this));
}

void ON_FixedSizePool::SetHeap( ON_MEMORY_POOL* heap )
{
  m_heap = heap;
}

ON_MEMORY_POOL* ON_FixedSizePool::Heap()
{
  return m_heap;
}

size_t ON_FixedSizePool::ActiveElementCount() const
{
  return m_active_element_count;
}

size_t ON_FixedSizePool::TotalElementCount() const
{
  return m_total_element_count;
}

void* ON_FixedSizePool::AllocateElement()
{
  void* p;

  if ( 0 != m_al_element_stack )
  {
    // use item on the returned stack first.
    p = m_al_element_stack;
    m_al_element_stack = *((void**)m_al_element_stack);
  }
  else
  {
    if ( 0 == m_al_block || 0 == m_al_count )
    {
      // No more memory left in m_al_block.
      void* next_block = (0 != m_al_block)
                       ? *((void**)m_al_block)
                       : 0;
      if ( 0 == next_block )
      {
        // This if clause is used when we need to allocate a new block from the heap
        if ( 0 == m_sizeof_element )
        {
          ON_ERROR("ON_FixedSizePool::AllocateElement - you must call ON_FixedSizePool::Create with a valid element size before using ON_FixedSizePool");
          return 0;
        }
        // allocate a new block
        if ( 0 == m_al_count )
          m_al_count = m_block_element_count;

        if ( m_al_count <= 0 )
        {
          ON_ERROR("ON_FixedSizePool::AllocateElement - you must call ON_FixedSizePool::Create with a valid element size before using ON_FixedSizePool");
          return 0;
        }

        p = onmalloc_from_pool( m_heap, 2*sizeof(void*) + m_al_count*m_sizeof_element ); // get some heap

        // set "next" pointer to zero
        *((void**)p) = 0;

        // set "end" pointer to address after last byte in the block
        *((void**)(((char*)p) + sizeof(void*))) = ((char*)p) + (2*sizeof(void*) + m_al_count*m_sizeof_element);
        if ( 0 == m_first_block )
        {
          m_first_block = p;
          // If the call to Create() specified a positive element_count_estimate,
          // then m_sizeof_block needs to be reset for any future block allocations.
          
        }
        else
        {
          // If m_first_block != 0, then m_al_block is nonzero (or memory for this class has been trashed)
          *((void**)m_al_block) = p;
        }
        m_al_block = p;
      }
      else
      {
        // If we get here, ReturnAll() was used at some point in
        // the past, m_al_block != 0, m_al_count = zero, and we are
        // reusing blocks that were allocated early.
        m_al_block = next_block;
        m_al_count = BlockElementCapacity(m_al_block);
      }

      m_al_element_array = (void*)(((char*)m_al_block)+2*sizeof(void*));
    }
    m_al_count--;
    p = m_al_element_array;
    m_al_element_array = (void*)(((char*)m_al_element_array) + m_sizeof_element);
    m_total_element_count++;
  }

  memset(p,0,m_sizeof_element);
  m_active_element_count++;

  return p;
}

void ON_FixedSizePool::ReturnElement(void* p)
{
  if ( p )
  {
    if ( m_active_element_count <= 0 )
    {
      // If you get this error, something is seriously wrong.
      // You may be returning the same element multiple times or 
      // you may be returning pointers that are not from this pool.
      // In any case, you're probably going to be crashing sometime soon.
      ON_ERROR("ON_FixedSizePool::ReturnElement - no active elements exist.");
    }
    else
    {
      m_active_element_count--;
      *((void**)p) = m_al_element_stack;
      m_al_element_stack = p;
    }
  }
}


ON_FixedSizePoolIterator::ON_FixedSizePoolIterator( const ON_FixedSizePool& fsp )
: m_fsp(fsp)
, m_it_block(0)
, m_it_element(0)
{}

void* ON_FixedSizePool::FirstElement()
{
  if ( m_first_block && m_total_element_count > 0 )
  {
    m_qwerty_it_block = m_first_block;
    m_qwerty_it_element = (void*)(((char*)m_qwerty_it_block)+2*sizeof(void*)); // m_qwerty_it_element points to first element in m_first_block
  }
  else
  {
    m_qwerty_it_block = 0;
    m_qwerty_it_element = 0;
  }
  return m_qwerty_it_element;
}

void* ON_FixedSizePoolIterator::FirstElement()
{
  if ( m_fsp.m_first_block && m_fsp.m_total_element_count > 0 )
  {
    m_it_block = m_fsp.m_first_block;
    m_it_element = (void*)(((char*)m_it_block)+2*sizeof(void*)); // m_it_element points to first element in m_first_block
  }
  else
  {
    m_it_block = 0;
    m_it_element = 0;
  }
  return m_it_element;
}

void* ON_FixedSizePool::NextElement()
{
  if ( m_qwerty_it_element )
  {
    m_qwerty_it_element = (void*)(((char*)m_qwerty_it_element) + m_sizeof_element);
    if ( m_qwerty_it_element == m_al_element_array )
    {
      m_qwerty_it_block = 0;
      m_qwerty_it_element = 0;
    }
    else if ( m_qwerty_it_element == *((void**)(((char*)m_qwerty_it_block) + sizeof(void*))) )
    {
      // m_qwerty_it_element  = "end" pointer which means we are at the end of m_qwerty_it_block
      m_qwerty_it_block = *((void**)m_qwerty_it_block); // m_qwerty_it_block = "next" block
      m_qwerty_it_element = (0 != m_qwerty_it_block)    // m_qwerty_it_element points to first element in m_qwerty_it_block
                   ? (void*)(((char*)m_qwerty_it_block)+2*sizeof(void*))
                   : 0;
      if ( m_qwerty_it_element == m_al_element_array )
      {
        m_qwerty_it_block = 0;
        m_qwerty_it_element = 0;
      }
    }
  }
  return m_qwerty_it_element;
}

void* ON_FixedSizePoolIterator::NextElement()
{
  if ( m_it_element )
  {
    m_it_element = (void*)(((char*)m_it_element) + m_fsp.m_sizeof_element);
    if ( m_it_element == m_fsp.m_al_element_array )
    {
      m_it_block = 0;
      m_it_element = 0;
    }
    else if ( m_it_element == *((void**)(((char*)m_it_block) + sizeof(void*))) )
    {
      // m_it_element  = "end" pointer which means we are at the end of m_it_block
      m_it_block = *((void**)m_it_block); // m_it_block = "next" block
      m_it_element = (0 != m_it_block)    // m_it_element points to first element in m_it_block
                   ? (void*)(((char*)m_it_block)+2*sizeof(void*))
                   : 0;
      if ( m_it_element == m_fsp.m_al_element_array )
      {
        m_it_block = 0;
        m_it_element = 0;
      }
    }
  }
  return m_it_element;
}

void* ON_FixedSizePool::FirstElement(size_t element_index)
{
  const char* block;
  const char* block_end;
  const char* next_block;
  size_t block_count;

  m_qwerty_it_block = 0;
  m_qwerty_it_element = 0;
  if ( element_index < m_total_element_count )
  {
    for ( block = (const char*)m_first_block; 0 != block; block = next_block )
    {
      if ( block == m_al_block )
      {
        next_block = 0;
        block_end = (const char*)m_al_element_array;
      }
      else
      {
        next_block = *((const char**)block);
        block_end =  *((const char**)(block + sizeof(void*)));
      }
      block_count = (block_end - block)/m_sizeof_element;
      if ( element_index < block_count )
      {
        m_qwerty_it_block = (void*)block;
        m_qwerty_it_element = ((void*)(block + (2*sizeof(void*) + element_index*m_sizeof_element)));
        break;
      }
      element_index -= block_count;
    }
  }
  return m_qwerty_it_element;
}

void* ON_FixedSizePoolIterator::FirstElement(size_t element_index)
{
  const char* block;
  const char* block_end;
  const char* next_block;
  size_t block_count;

  m_it_block = 0;
  m_it_element = 0;
  if ( element_index < m_fsp.m_total_element_count )
  {
    for ( block = (const char*)m_fsp.m_first_block; 0 != block; block = next_block )
    {
      if ( block == m_fsp.m_al_block )
      {
        next_block = 0;
        block_end = (const char*)m_fsp.m_al_element_array;
      }
      else
      {
        next_block = *((const char**)block);
        block_end =  *((const char**)(block + sizeof(void*)));
      }
      block_count = (block_end - block)/m_fsp.m_sizeof_element;
      if ( element_index < block_count )
      {
        m_it_block = (void*)block;
        m_it_element = ((void*)(block + (2*sizeof(void*) + element_index*m_fsp.m_sizeof_element)));
        break;
      }
      element_index -= block_count;
    }
  }
  return m_it_element;
}

size_t ON_FixedSizePool::BlockElementCapacity( const void* block ) const
{
  // returns number of items that can be allocated from block
  if ( 0 == block || m_sizeof_element <= 0 )
    return 0;
  char* block_end = *((char**)(((char*)block)+sizeof(void*)));
  return (block_end - ((char*)block))/m_sizeof_element;
}

size_t ON_FixedSizePool::BlockElementCount( const void* block ) const
{
  // returns number of items allocated from block
  if ( 0 == block || m_sizeof_element <= 0 )
    return 0;
  char* block_end = (block == m_al_block && m_al_count > 0)
      ? ((char*)m_al_element_array)
      : *((char**)(((char*)block)+sizeof(void*)));
  return (block_end - ((char*)block))/m_sizeof_element;
}

void* ON_FixedSizePool::FirstBlock( size_t* block_element_count )
{
  if ( m_first_block && m_total_element_count > 0 )
  {
    m_qwerty_it_block = m_first_block;
    m_qwerty_it_element = (void*)(((char*)m_qwerty_it_block)+2*sizeof(void*)); // m_qwerty_it_element points to first element in m_first_block
    if ( 0 != block_element_count )
      *block_element_count = BlockElementCount(m_qwerty_it_block);
  }
  else
  {
    m_qwerty_it_block = 0;
    m_qwerty_it_element = 0;
    if ( 0 != block_element_count )
      *block_element_count = 0;
  }
  return m_qwerty_it_element;
}

void* ON_FixedSizePoolIterator::FirstBlock( size_t* block_element_count )
{
  if ( m_fsp.m_first_block && m_fsp.m_total_element_count > 0 )
  {
    m_it_block = m_fsp.m_first_block;
    m_it_element = (void*)(((char*)m_it_block)+2*sizeof(void*)); // m_it_element points to first element in m_first_block
    if ( 0 != block_element_count )
      *block_element_count = m_fsp.BlockElementCount(m_it_block);
  }
  else
  {
    m_it_block = 0;
    m_it_element = 0;
    if ( 0 != block_element_count )
      *block_element_count = 0;
  }
  return m_it_element;
}

void* ON_FixedSizePool::NextBlock( size_t* block_element_count )
{
  if ( 0 != m_qwerty_it_block 
       && m_qwerty_it_block != m_al_block
       && m_qwerty_it_element == (void*)(((char*)m_qwerty_it_block)+2*sizeof(void*)) )
  {
    m_qwerty_it_block = *((void**)m_qwerty_it_block);
    if ( m_qwerty_it_block == m_al_element_array )
    {
      m_qwerty_it_block = 0;
      m_qwerty_it_element = 0;
      if ( 0 != block_element_count )
        *block_element_count = 0;
    }
    else
    {
      m_qwerty_it_element = (void*)(((char*)m_qwerty_it_block)+2*sizeof(void*)); // m_qwerty_it_element points to first element in m_first_block
      if ( 0 != block_element_count )
        *block_element_count = BlockElementCount(m_qwerty_it_block);
    }
  }
  else
  {
    m_qwerty_it_block = 0;
    m_qwerty_it_element = 0;
    if ( 0 != block_element_count )
      *block_element_count = 0;
  }
  return m_qwerty_it_element;
}

void* ON_FixedSizePoolIterator::NextBlock( size_t* block_element_count )
{
  if ( 0 != m_it_block 
       && m_it_block != m_fsp.m_al_block
       && m_it_element == (void*)(((char*)m_it_block)+2*sizeof(void*)) )
  {
    m_it_block = *((void**)m_it_block);
    if ( m_it_block == m_fsp.m_al_element_array )
    {
      m_it_block = 0;
      m_it_element = 0;
      if ( 0 != block_element_count )
        *block_element_count = 0;
    }
    else
    {
      m_it_element = (void*)(((char*)m_it_block)+2*sizeof(void*)); // m_it_element points to first element in m_first_block
      if ( 0 != block_element_count )
        *block_element_count = m_fsp.BlockElementCount(m_it_block);
    }
  }
  else
  {
    m_it_block = 0;
    m_it_element = 0;
    if ( 0 != block_element_count )
      *block_element_count = 0;
  }
  return m_it_element;
}

void* ON_FixedSizePool::Element(size_t element_index) const
{
  const char* block;
  const char* block_end;
  const char* next_block;
  size_t block_count;

  for ( block = (const char*)m_first_block; 0 != block; block = next_block )
  {
    if ( block == m_al_block )
    {
      next_block = 0;
      block_end = (const char*)m_al_element_array;
    }
    else
    {
      next_block = *((const char**)block);
      block_end =  *((const char**)(block + sizeof(void*)));
    }
    block_count = (block_end - block)/m_sizeof_element;
    if ( element_index < block_count )
      return ((void*)(block + (2*sizeof(void*) + element_index*m_sizeof_element)));
    element_index -= block_count;
  }

  return 0;
}
