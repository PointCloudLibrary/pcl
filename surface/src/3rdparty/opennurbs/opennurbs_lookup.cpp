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

static bool IdIsNotZero(const ON_UUID* id)
{
  // This is a fast static function to check for
  // zero ids.  The caller always checks that
  // id is not null.
  const unsigned char* p = (const unsigned char*)id;
  unsigned int i = (unsigned int)sizeof(ON_UUID);
  while (i--) 
  {
    if (0 != *p++) 
      return true;
  }
  return false;
}

static ON__UINT16 IdCRC(const ON_UUID* id)
{
  // This is a private member function and the caller
  // insures the id pointer is not null.
  static const ON__UINT16 ON_CRC16_CCITT_TABLE[256] =
   {0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0};

  const unsigned char* b = (const unsigned char*)id;
  ON__UINT16 current_remainder;
  ON__UINT16 r1;

  // crc calculation loop unrolled for speed

  //current_remainder=0;
  //r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  //current_remainder = (current_remainder << 8) ^ (*b++);
  //current_remainder ^= r1;  
  //r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  //current_remainder = (current_remainder << 8) ^ (*b++);
  //current_remainder ^= r1;  

  // The commented out lines above reduce to these two lines.
  current_remainder = *b++; 
  current_remainder = (current_remainder << 8) ^ (*b++);

  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b++);
  current_remainder ^= r1;  
  r1 = ON_CRC16_CCITT_TABLE[(current_remainder & ((ON__UINT16)0xff00))>>8];
  current_remainder = (current_remainder << 8) ^ (*b);
  current_remainder ^= r1;  

  return current_remainder;
}

ON_SerialNumberMap::ON_SerialNumberMap(ON_MEMORY_POOL* pool)
{
  m_maxsn = 0;
  m_reserved = 0;
  m_pool = pool;
  m_sn_count = 0;
  m_sn_purged = 0;
  m_snblk_list = 0;
  m_snblk_list_capacity = 0;
  m_snblk_list_count = 0;
  m_e_blk = 0;
  m_sn_block0.EmptyBlock();
  m_bHashTableIsValid = true;
  m_active_id_count = 0;
  memset(&m_inactive_id,0,sizeof(m_inactive_id));
  memset(m_hash_table,0,sizeof(m_hash_table));
}

ON_SerialNumberMap::~ON_SerialNumberMap()
{
  EmptyList();
}

void ON_SerialNumberMap::EmptyList()
{
  m_maxsn = 0;
  m_sn_count = 0;
  m_sn_purged = 0;
  m_sn_block0.EmptyBlock();
  if (m_snblk_list)
  {
    size_t i = m_snblk_list_capacity;
    while(i--)
    {
      if ( 0 != m_snblk_list[i] )
        onfree(m_snblk_list[i]);
    }
    onfree(m_snblk_list);
    m_snblk_list = 0;
    m_snblk_list_capacity = 0;
    m_snblk_list_count = 0;
  }
  m_bHashTableIsValid = true;
  m_active_id_count = 0;
  memset(m_hash_table,0,sizeof(m_hash_table));
}


void ON_SerialNumberMap::SN_BLOCK::EmptyBlock()
{
  m_count = 0;
  m_purged = 0;
  m_sorted = 1;
  m_sn0 = 0;
  m_sn1 = 0;
}

void ON_SerialNumberMap::SN_BLOCK::CullBlockHelper()
{
  // Search the m_an[] array for elements whose m_u_type
  // value is zero and remove them from the array.
  //
  // This is a high speed helper function.  
  // The calling function must verfy m_purged > 0.
  //
  // This function removes all m_sn[] elements
  // that have 0 == m_sn_active.

  size_t i, j;
  for (i = 0; i < m_count; i++ )
  {
    if ( 0 == m_sn[i].m_sn_active )
    {
      for ( j = i+1; j < m_count; j++ )
      {
        if ( m_sn[j].m_sn_active )
        {
          m_sn[i++] = m_sn[j];
        }
      }
      if ( 0 == i )
      {
        EmptyBlock();
      }
      else
      {
        m_count = i;
        m_purged = 0;
        if ( m_sorted )
        {
          m_sn0 = m_sn[0].m_sn;
          m_sn1 = m_sn[m_count-1].m_sn;
        }
      }
      break;
    }
  }
}



/*
The defines and #include generates a fast sorting function
static void ON_qsort_SN_ELEMENT( struct ON_SerialNumberMap::SN_ELEMENT* base, size_t nel );
*/

#define ON_SORT_TEMPLATE_COMPARE compare_SN_ELEMENT_sn
#define ON_COMPILING_OPENNURBS_QSORT_FUNCTIONS
#define ON_SORT_TEMPLATE_STATIC_FUNCTION
#define ON_SORT_TEMPLATE_TYPE struct ON_SerialNumberMap::SN_ELEMENT
#define ON_QSORT_FNAME ON_qsort_SN_ELEMENT

static int ON_SORT_TEMPLATE_COMPARE(const struct ON_SerialNumberMap::SN_ELEMENT * a, const struct ON_SerialNumberMap::SN_ELEMENT * b)
{
  unsigned int asn, bsn;
  return ( ( (asn = a->m_sn) < (bsn = b->m_sn) ) ? -1 : (asn > bsn) ? 1 : 0 );
}

#include "pcl/surface/3rdparty/opennurbs/opennurbs_qsort_template.h"

#undef ON_COMPILING_OPENNURBS_QSORT_FUNCTIONS
#undef ON_SORT_TEMPLATE_STATIC_FUNCTION
#undef ON_SORT_TEMPLATE_TYPE
#undef ON_QSORT_FNAME
#undef ON_SORT_TEMPLATE_COMPARE


void ON_SerialNumberMap::SN_BLOCK::SortBlockHelper()
{
  // Sort m_sn[] by serial number.
  //
  // This is a high speed helper function.  
  //
  // The calling function verify:
  //   m_sorted is zero
  //   m_purged is zero
  //   m_count > 0
  //
  // The array m_sn[] is almost always nearly sorted,
  // so the sort used here should efficiently
  // handle almost sorted arrays. In the past,
  // heap sort was the best choice, but the qsort()
  // in VS 2010 is now a better choice than heap sort.
   
  if ( m_count > 1 )
  {
#if 1
    // Quick sort
    ON_qsort_SN_ELEMENT(m_sn, m_count);
#else
    // Heap sort
    size_t i, j, k, i_end;
    struct SN_ELEMENT e_tmp;
    struct SN_ELEMENT* e;

    e = m_sn;

    k = m_count >> 1;
    i_end = m_count-1;
    for (;;) 
    {
      if (k)
      {
        --k;
        e_tmp = e[k];
      } 
      else 
      {
        e_tmp = e[i_end];
        e[i_end] = e[0];
        if (!(--i_end)) 
        {
          e[0] = e_tmp;
          break;
        }
      }

      i = k;
      j = (k<<1) + 1;
      while (j <= i_end)
      {
        if (j < i_end && e[j].m_sn < e[j + 1].m_sn)
          j++;
        if (e_tmp.m_sn < e[j].m_sn)
        {
          e[i] = e[j];
          i = j;
          j = (j<<1) + 1;
        } 
        else 
          j = i_end + 1;
      }
      e[i] = e_tmp;
    }
#endif
    m_sn0 = m_sn[0].m_sn;
    m_sn1 = m_sn[m_count-1].m_sn;
  }
  else
  {
    m_sn0 = m_sn1 = ((1 == m_count) ? m_sn[0].m_sn : 0);
  }
  m_sorted = 1;
}


static bool ON_SerialNumberMap_IsNotValidBlock()
{
  return ON_IsNotValid(); // <- Good place for a debugger breakpoint
}

bool ON_SerialNumberMap::SN_BLOCK::IsValidBlock(ON_TextLog* textlog, 
                                                struct SN_ELEMENT*const* hash_table,
                                                size_t* active_id_count) const
{
  unsigned int sn0, sn;
  size_t i, pc, aidcnt;

  if ( m_count > SN_BLOCK_CAPACITY )
  {
    if (textlog)
      textlog->Print("SN_BLOCK m_count = %u (should be >=0 and <%u).\n",
                      m_count,SN_BLOCK_CAPACITY);
    return ON_SerialNumberMap_IsNotValidBlock();
  }

  if ( m_purged > m_count )
  {
    if (textlog)
      textlog->Print("SN_BLOCK m_purged = %u (should be >0 and <=%u).\n",
                      m_purged,m_count);
    return ON_SerialNumberMap_IsNotValidBlock();
  }

  if ( m_count < 2 && 1 != m_sorted )
  {
    if (textlog)
      textlog->Print("SN_BLOCK m_count = %u but m_sorted is not 1.\n",m_count);
    return ON_SerialNumberMap_IsNotValidBlock();
  }

  if ( 0 == m_count )
  {
    if ( 0 != m_sn0 )
    {
      if (textlog)
        textlog->Print("SN_BLOCK m_count = 0 but m_sn0 != 0.\n");
      return ON_SerialNumberMap_IsNotValidBlock();
    }
    if ( 0 != m_sn1 )
    {
      if (textlog)
        textlog->Print("SN_BLOCK m_count = 0 but m_sn1 != 0.\n");
      return ON_SerialNumberMap_IsNotValidBlock();
    }
    return true;
  }

  if ( m_sn1 < m_sn0 )
  {
    if (textlog)
      textlog->Print("SN_BLOCK m_sn1 < m_sn0.\n");
    return ON_SerialNumberMap_IsNotValidBlock();
  }

  if ( m_count > m_purged )
  {
    if ( m_sn1 - m_sn0 < m_count-m_purged-1 )
    {
      if (textlog)
        textlog->Print("SN_BLOCK m_sn1 < m_sn0.\n");
      return ON_SerialNumberMap_IsNotValidBlock();
    }
  }

  sn0 = 0;
  pc = 0;
  aidcnt = 0;
  for (i = 0; i < m_count; i++)
  {
    // validate m_sn_active and m_id_active flags
    if (0 == m_sn[i].m_sn_active)
    {
      // The element serial number is not active.
      // The id must also be not active.
      pc++;
      if ( 0 != m_sn[i].m_id_active )
      {
        if (textlog)
          textlog->Print("SN_BLOCK m_sn[%d].m_sn_active = 0 but m_id_active != 0.\n",i);
        return ON_SerialNumberMap_IsNotValidBlock();
      }
    }
    else if ( 0 != m_sn[i].m_id_active )
    {
      // The element has active serial number and active id.
      // It must have a nonzero m_id and be in the hash table.
      aidcnt++;
      if ( IdIsNotZero(&m_sn[i].m_id) )
      {
        const SN_ELEMENT* e;
        for (e = hash_table[IdCRC(&m_sn[i].m_id) % ID_HASH_TABLE_COUNT]; 0!=e; e = e->m_next)
        {
          if ( e == &m_sn[i] )
          {
            // found m_sn[i] in the hash table
            break;
          }
        }
        if ( 0 == e )
        {
          // m_sn[i] is not in the hash table but m_id_active indicates
          // it should be.
          if (textlog)
            textlog->Print("SN_BLOCK m_sn[%d].m_id_active != 0 but the element is not in m_hash_table[].\n",i);
          return ON_SerialNumberMap_IsNotValidBlock();
        }
      }
      else 
      {
        if (textlog)
          textlog->Print("SN_BLOCK m_sn[%d].m_id_active != 0 but m_id = 0.\n",i);
        return ON_SerialNumberMap_IsNotValidBlock();
      }
    }

    // verify the serial number is in the range m_sn0 to m_sn1
    sn = m_sn[i].m_sn;
    if ( sn < m_sn0 )
    {
      if (textlog)
        textlog->Print("SN_BLOCK m_sn[%d] < m_sn0.\n",i);
      return ON_SerialNumberMap_IsNotValidBlock();
    }
    if ( sn > m_sn1 )
    {
      if (textlog)
        textlog->Print("SN_BLOCK m_sn[%d] > m_sn1.\n",i);
      return ON_SerialNumberMap_IsNotValidBlock();
    }

    if ( m_sorted )
    {
      // Verify this sn is bigger than the previous one
      if (sn <= sn0 )
      {
        if (textlog)
          textlog->Print("SN_BLOCK m_sn[%d] > m_sn[%d].\n",i,i-1);
        return ON_SerialNumberMap_IsNotValidBlock();
      }
      sn0 = sn;
    }
  }

  if ( pc != m_purged )
  {
    if (textlog)
      textlog->Print("SN_BLOCK m_purged = %u (should be %u)\n",m_purged,pc);
    return ON_SerialNumberMap_IsNotValidBlock();
  }

  // Update the active id count to include
  // the active ids from this block.
  if ( 0 != active_id_count )
    *active_id_count += aidcnt;

  return true;
}

struct ON_SerialNumberMap::SN_ELEMENT* ON_SerialNumberMap::SN_BLOCK::BinarySearchBlockHelper(unsigned int sn)
{
  // Perform a binary search on the serial number values in the m_sn[] array.
  //
  // This is a high speed helper function.  
  //
  // The calling function verify:
  //   m_sn[] is sorted by serial number (1 == m_sorted)
  //   m_count > 0
  //   m_sn0 <= sn <= m_sn1.

  size_t i, j;
  struct SN_ELEMENT* e;
  unsigned int midsn;

  i = m_count;
  e = m_sn;
  while (i > 0 )
  {
    midsn = e[(j=i/2)].m_sn;
    if ( sn < midsn )
    {
      i = j;
    }
    else if ( sn > midsn )
    {
      j++;
      e += j;
      i -= j;
    }
    else 
    {
      return e + j;
    }
  }
  return 0;
}

void ON_SerialNumberMap::UpdateMaxSNHelper()
{
  m_maxsn = (m_snblk_list_count > 0) ? m_snblk_list[m_snblk_list_count-1]->m_sn1 : 0;
  if ( m_maxsn < m_sn_block0.m_sn1 )
    m_maxsn = m_sn_block0.m_sn1;
}

struct ON_SerialNumberMap::SN_ELEMENT* ON_SerialNumberMap::FindElementHelper(unsigned int sn)
{
  struct SN_BLOCK** eblk_array;
  struct SN_BLOCK* eblk;
  size_t i, j;

  if ( m_maxsn < sn )
    return 0; // happens almost every time an object is added to the doc

  if ( sn <= 0 )
    return 0;

  // First check m_sn_block0. For small models this
  // is the only place we need to look.
  if ( sn <= m_sn_block0.m_sn1 && m_sn_block0.m_sn0 <= sn )
  {
    SN_ELEMENT* e;

    m_e_blk = &m_sn_block0;
    if ( !m_sn_block0.m_sorted )
    {
      // m_sn_block0.m_sn[] needs to be sorted
      //
      // This is a rare occurance.  It only happens
      // when commands add new objects in an order that
      // is different from that in which the CRhinoObject
      // class constructor was called.  In testing,
      // I could not find a real command that did this
      // and had to write TestAddBackwards to test this code.
      if ( m_sn_block0.m_purged > 0 )
      {
        InvalidateHashTableHelper();
        m_sn_count -= m_sn_block0.m_purged;
        m_sn_purged -= m_sn_block0.m_purged;
        m_sn_block0.CullBlockHelper();
        UpdateMaxSNHelper();
      }
      if ( m_sn_block0.m_count > 0 )
      {
        InvalidateHashTableHelper();
        m_sn_block0.SortBlockHelper();
      }
      e = ( sn <= m_sn_block0.m_sn1 && m_sn_block0.m_sn0 <= sn )
        ? m_sn_block0.BinarySearchBlockHelper(sn)
        : 0;
    }
    else if ( m_sn_block0.m_purged > m_sn_block0.m_count/SN_PURGE_RATIO )
    {
      InvalidateHashTableHelper();
      m_sn_count -= m_sn_block0.m_purged;
      m_sn_purged -= m_sn_block0.m_purged;
      m_sn_block0.CullBlockHelper();
      UpdateMaxSNHelper();
      e = ( sn <= m_sn_block0.m_sn1 && m_sn_block0.m_sn0 <= sn )
        ? m_sn_block0.BinarySearchBlockHelper(sn)
        : 0;
    }
    else
    {
      e = m_sn_block0.BinarySearchBlockHelper(sn);
    }
    if (e)
      return e;
  }

  // look in the blocks kept in the m_sn_list[] array.
  // The m_sn[] arrays in these blocks are always sorted.
  // In addition the blocks are disjoint and stored in
  // the m_sn_list[] array in sorted order.
  if ( 0 == (i = m_snblk_list_count) )
  {
    return 0;
  }
  eblk_array = m_snblk_list;
  while (i > 0 )
  {
    eblk = eblk_array[(j=i/2)];

    // The culling code is here rather than
    // in the binary search clause so the
    // entire ON_SerialNumberMap tends to
    // be tidy.
    if ( eblk->m_purged > eblk->m_count/SN_PURGE_RATIO )
    {
      // cull purged items from eblk
      InvalidateHashTableHelper();
      m_sn_count -= eblk->m_purged;
      m_sn_purged -= eblk->m_purged;
      eblk->CullBlockHelper();
      if ( 0 == eblk->m_count )
      {
        // put empty block at the end of the list
        for( j += ((eblk_array-m_snblk_list)+1); j < m_snblk_list_count; j++ )
        {
          m_snblk_list[j-1] = m_snblk_list[j];
        }
        m_snblk_list_count--;
        m_snblk_list[m_snblk_list_count] = eblk;
        i--;
        UpdateMaxSNHelper();
        continue;
      }
      UpdateMaxSNHelper();
    }

    if ( sn < eblk->m_sn0 )
    {
      i = j;
    }
    else if ( sn > eblk->m_sn1 )
    {
      j++;
      eblk_array += j;
      i -= j;
    }
    else 
    {
      m_e_blk = eblk;
      return eblk->BinarySearchBlockHelper(sn);
    }
  }

  return 0;
}

size_t ON_SerialNumberMap::ActiveSerialNumberCount() const
{
  return m_sn_count - m_sn_purged;
}

size_t ON_SerialNumberMap::ActiveIdCount() const
{
  return m_active_id_count;
}

struct ON_SerialNumberMap::SN_ELEMENT* ON_SerialNumberMap::FirstElement() const
{
  struct SN_ELEMENT* e=0;
  size_t i,j;

  // The first element is likely to be m_snblk_list[0]->m_sn[0]
  // so start looking there.
  for(i = 0; i < m_snblk_list_count; i++)
  {
    if ( m_snblk_list[i]->m_count > m_snblk_list[i]->m_purged )
    {
      for ( j = 0; j < m_snblk_list[i]->m_count; j++ )
      {
        if ( m_snblk_list[i]->m_sn[j].m_sn_active )
        {
          e = &m_snblk_list[i]->m_sn[j];
          break;
        }
      }
      break;
    }
  }

  if ( m_sn_block0.m_count > m_sn_block0.m_purged 
       && (!e || m_sn_block0.m_sn0 < e->m_sn) 
     )
  {
    // It's possible the element is in m_sn_block0.
    if ( m_sn_block0.m_purged > 0 )
    {
      // remove purged elements from m_sn_block0.
      const_cast<ON_SerialNumberMap*>(this)->InvalidateHashTableHelper();
      const_cast<ON_SerialNumberMap*>(this)->m_sn_count -= m_sn_block0.m_purged;
      const_cast<ON_SerialNumberMap*>(this)->m_sn_purged -= m_sn_block0.m_purged;
      const_cast<ON_SerialNumberMap*>(this)->m_sn_block0.CullBlockHelper();
    }
    if ( !m_sn_block0.m_sorted )
    {
      // sort elements in m_sn_block0.
      const_cast<ON_SerialNumberMap*>(this)->InvalidateHashTableHelper();
      const_cast<ON_SerialNumberMap*>(this)->m_sn_block0.SortBlockHelper();      
    }
    if ( !e || m_sn_block0.m_sn0 <  e->m_sn )
    {
      // first element in m_sn_block0 is the
      // one with the smallest serial number.
      e = const_cast<struct SN_ELEMENT*>(&m_sn_block0.m_sn[0]);
    }
  }
  return e;
}

struct ON_SerialNumberMap::SN_ELEMENT* ON_SerialNumberMap::LastElement() const
{
  struct SN_ELEMENT* e=0;
  size_t i,j;

  // Last element is likely to be m_sn_block0.m_sn[m_sn_block0.m_count-1]
  // so start looking there.
  if ( m_sn_block0.m_count > m_sn_block0.m_purged )
  {
    if ( m_sn_block0.m_purged > 0 )
    {
      // remove purged elements from m_sn_block0
      const_cast<ON_SerialNumberMap*>(this)->InvalidateHashTableHelper();
      const_cast<ON_SerialNumberMap*>(this)->m_sn_count -= m_sn_block0.m_purged;
      const_cast<ON_SerialNumberMap*>(this)->m_sn_purged -= m_sn_block0.m_purged;
      const_cast<ON_SerialNumberMap*>(this)->m_sn_block0.CullBlockHelper();
    }
    if ( !m_sn_block0.m_sorted )
    {
      // sort m_sn_block0
      const_cast<ON_SerialNumberMap*>(this)->InvalidateHashTableHelper();
      const_cast<ON_SerialNumberMap*>(this)->m_sn_block0.SortBlockHelper();      
    }
    e = const_cast<struct SN_ELEMENT*>(&m_sn_block0.m_sn[m_sn_block0.m_count-1]);
  }

  i = m_snblk_list_count;
  while(i--)
  {
    if ( m_snblk_list[i]->m_count > m_snblk_list[i]->m_purged )
    {
      if (e && e->m_sn > m_snblk_list[i]->m_sn1 )
        break;
      j = m_snblk_list[i]->m_count;
      while(j--)
      {
        if ( m_snblk_list[i]->m_sn[j].m_sn_active )
        {
          e = &m_snblk_list[i]->m_sn[j];
          break;
        }
      }
      break;
    }
  }

  return e;
}

struct ON_SerialNumberMap::SN_ELEMENT* ON_SerialNumberMap::FindSerialNumber(unsigned int sn) const
{
  struct SN_ELEMENT* e = const_cast<ON_SerialNumberMap*>(this)->FindElementHelper(sn);
  return ( (e && e->m_sn_active) ? e : 0);
}

struct ON_SerialNumberMap::SN_ELEMENT* ON_SerialNumberMap::FindId(ON_UUID id) const
{
  struct SN_ELEMENT* e = 0;
  size_t i;

  if ( m_active_id_count > 0 )
  {
    i = HashIndex(&id);
    if ( 0 != i || IdIsNotZero(&id) )
    {
      if ( !m_bHashTableIsValid )
      {
        const_cast<ON_SerialNumberMap*>(this)->BuildHashTableHelper();
      }
      e = m_hash_table[i];
      while(e)
      {
        if ( 0 == memcmp(&e->m_id,&id,sizeof(e->m_id)) )
          break;
        e = e->m_next;
      }
    }
  }
  return e;
}


size_t ON_SerialNumberMap::GetElements(
        unsigned int sn0,
        unsigned int sn1, 
        size_t max_count,
        ON_SimpleArray<SN_ELEMENT>& elements
        ) const
{
  size_t i,j,k,c;
  const SN_ELEMENT *ei, *ek;

  const int elements_count0 = elements.Count();

  if ( sn1 < sn0 || max_count <= 0 || m_sn_count <= m_sn_purged )
    return 0;

  if ( sn0+3 <= sn1 )
  {
    elements.Reserve(elements_count0+3);
    while ( sn0 <= sn1 )
    {
      ei = const_cast<ON_SerialNumberMap*>(this)->FindElementHelper(sn0++);
      if ( ei && ei->m_sn_active )
        elements.Append(*ei);
    }
    return (elements.Count() - elements_count0); 
  }

  ek = 0;
  k = 0;
  for ( j = 0; j < m_snblk_list_count; j++ )
  {
    if ( m_snblk_list[j]->m_sn1 < sn0 )
      continue;
    if ( sn1 < m_snblk_list[j]->m_sn0 )
      break;

    k = m_snblk_list[j]->m_count; // always > 0
    ek = &m_snblk_list[j]->m_sn[0];
    while ( ek->m_sn < sn0 || !ek->m_sn_active )
    {
      if ( --k )
      {
        if ((++ek)->m_sn > sn1)
        {
          ek = 0;
          break;
        }
      }
      else if ( ++j < m_snblk_list_count && m_snblk_list[j]->m_sn0 <= sn1 )
      {
        k = m_snblk_list[j]->m_count; // always > 0
        ek = &m_snblk_list[j]->m_sn[0];
      }
      else
      {
        ek = 0;
        break;
      }
    }
    if ( ek && ek->m_sn > sn1 )
      ek = 0;
    break;
  }

  // set c = estimate of number of items in m_snblk_list[]
  if ( ek )
  {
    c = m_snblk_list[j]->ActiveElementEstimate(ek->m_sn,sn1);
    for ( i = j+1; i < m_snblk_list_count && m_snblk_list[i]->m_sn0 <= sn1; i++ )
    {
      c += m_snblk_list[j]->ActiveElementEstimate(ek->m_sn,sn1);
    }
  }
  else
    c = 0;

  // determine where to begin searching in m_sn_block0
  i = 0;
  ei = 0;
  if (    m_sn_block0.m_count > m_sn_block0.m_purged
       && sn1 >= m_sn_block0.m_sn0
       && sn0 <= m_sn_block0.m_sn1
       && !m_sn_block0.m_sorted
     )
  {
    if ( !m_sn_block0.m_sorted )
    {
      if ( m_sn_block0.m_purged > 0 )
      {
        const_cast<ON_SerialNumberMap*>(this)->InvalidateHashTableHelper();
        const_cast<ON_SerialNumberMap*>(this)->m_sn_count -= m_sn_block0.m_purged;
        const_cast<ON_SerialNumberMap*>(this)->m_sn_purged -= m_sn_block0.m_purged;
        const_cast<ON_SerialNumberMap*>(this)->m_sn_block0.CullBlockHelper();
        const_cast<ON_SerialNumberMap*>(this)->UpdateMaxSNHelper();
      }
      if ( m_sn_block0.m_count > 0 )
      {
        const_cast<ON_SerialNumberMap*>(this)->InvalidateHashTableHelper();
        const_cast<ON_SerialNumberMap*>(this)->m_sn_block0.SortBlockHelper();
        if ( sn1 >= m_sn_block0.m_sn0 && sn0 <= m_sn_block0.m_sn1 )
        {
          i = m_sn_block0.m_count;
          ei = &m_sn_block0.m_sn[0];
        }
      }
    }
    else
    {
      i = m_sn_block0.m_count;
      ei = &m_sn_block0.m_sn[0];
      while ( ei->m_sn < sn0 || !ei->m_sn_active )
      {
        if ( --i )
          ei++;
        else
        {
          ei = 0;
          break;
        }
      }
      if ( ei && ei->m_sn > sn1 )
      {
        ei = 0;
      }
    }
  }

  // adjust c = estimate of number of items in m_snblk_list[]
  if ( ei )
    c += m_sn_block0.ActiveElementEstimate(ei->m_sn,sn1);
  if (c > (sn1-sn0+1) )
    c = (sn1-sn0+1);
  if ( c > 2*4096 )
    c = 2*4096;

  // reserve room for elements so we don't thrash memory
  // while growing a large dynamic array.
  elements.Reserve(elements.Count()+((int)c));

  // Add appropriate elements to elements[] array.
  while (ei || ek)
  {
    if (ei && (!ek || ei->m_sn < ek->m_sn) )
    {
      if ( ei->m_sn_active )
        elements.Append(*ei);
      if ( --i )
      {
        if ( (++ei)->m_sn > sn1 )
        {
          ei = 0;
        }
      }
      else
      {
        ei = 0;
      }
    }
    else 
    {
      if ( ek->m_sn_active )
        elements.Append(*ei);
      if ( --k )
      {
        if ( (++ek)->m_sn > sn1 )
        {
          ek = 0;
        }
      }
      else if (++j < m_snblk_list_count && sn1 <= m_snblk_list[j]->m_sn0 )
      {
        k = m_snblk_list[j]->m_count; // always > 0
        ek = &m_snblk_list[j]->m_sn[0];
      }
      else 
      {
        ek = 0;
      }
    }
  }
  
  return (elements.Count() - elements_count0);
}

struct ON_SerialNumberMap::SN_ELEMENT* 
ON_SerialNumberMap::RemoveSerialNumberAndId(unsigned int sn)
{
  struct SN_ELEMENT* e = FindElementHelper(sn);
  if ( e && e->m_sn_active )
  {
    if ( e->m_id_active )
    {
      if ( m_bHashTableIsValid )
      {
        // Hash table is valid - remove the element from the table
        size_t i = HashIndex(&e->m_id);
        struct SN_ELEMENT* prev = 0;
        struct SN_ELEMENT* h;
        for ( h = m_hash_table[i]; h; h = h->m_next )
        {
          if (h == e)
          {
            if ( prev )
            {
              prev->m_next = e->m_next;
            }
            else
            {
              m_hash_table[i] = e->m_next;
            }
            break;
          }
          prev = h;
        }
      }
      e->m_next = 0;
      e->m_id_active = 0;
      if ( m_active_id_count > 0 )
      {
        m_active_id_count--;
      }
      else
      {
        ON_ERROR("ON_SerialNumberMap - m_active_id_count corruption");
      }

      // save this id.  When objects are replaced, this id will
      // be added back and saving it in m_inactive_id will 
      // prevent having to check for it in the hash table.
      m_inactive_id = e->m_id;
    }

    e->m_sn_active = 0;
    m_sn_purged++;
    if ( m_e_blk->m_count == ++m_e_blk->m_purged )
    {
      // every item in the block is purged
      if ( m_e_blk == &m_sn_block0 )
      {
        // Every element in m_sn_block0 is purged.
        // Empyt m_sn_block0.
        m_sn_count -= m_sn_block0.m_count;
        m_sn_purged -= m_sn_block0.m_count;
        m_sn_block0.EmptyBlock();
      }
      else if ( m_e_blk->m_count > 1 )
      {
        // m_e_blk is in m_sn_list[] and every element
        // in m_e_blk is purged. Remove all but
        // except one of these elements.
        // Note: We cannot empty blocks in the m_sn_list[] because
        //       this class has code that assumes the blocks
        //       in m_sn_list[] have m_count >= 1.  This makes
        //       the class generally faster.  There is code in 
        //       FindElementHelper() the keeps the m_sn_list[]
        //       blocks relatively tidy.
        m_sn_count  -= (m_e_blk->m_count-1);
        m_sn_purged -= (m_e_blk->m_count-1);
        m_e_blk->m_count = 1;
        m_e_blk->m_purged = 1;
        m_e_blk->m_sn0 = m_e_blk->m_sn1 = m_e_blk->m_sn[0].m_sn;
      }
    }
    return e;
  }

  return 0;
}

struct ON_SerialNumberMap::SN_ELEMENT* 
ON_SerialNumberMap::RemoveId(unsigned int sn, ON_UUID id)
{
  struct SN_ELEMENT* e=0;
  struct SN_ELEMENT* prev;
  size_t i;
  if ( m_active_id_count > 0 )
  {
    i = HashIndex(&id);
    if ( i != 0 || IdIsNotZero(&id) )
    {
      if ( !m_bHashTableIsValid && sn > 0 )
      {
        // We can avoid rebuilding the hash table.
        // Use the serial number to find the element.
        e = FindSerialNumber(sn);
        if (e)
        {
          if (e->m_id_active && 0 == memcmp(&e->m_id,&id,sizeof(e->m_id)) )
          {
            e->m_next = 0;
            e->m_id_active = 0;
            m_active_id_count--;
            m_inactive_id = e->m_id;
          }
          else
          {
            // id is not active
            e = 0;
          }
        }
      }
      else
      {
        BuildHashTableHelper();
        prev = 0;
        for( e = m_hash_table[i]; e; e = e->m_next )
        {
          if ( 0 == memcmp(&e->m_id,&id,sizeof(e->m_id)) )
          {
            if ( prev )
              prev->m_next = e->m_next;
            else
              m_hash_table[i] = e->m_next;
            e->m_next = 0;
            e->m_id_active = 0;
            m_active_id_count--;
            m_inactive_id = e->m_id;
            break;
          }
          prev = e;
        }
      }
    }
  }
  return e;
}


int ON_SerialNumberMap::SN_BLOCK::CompareMaxSN(const void* a, const void* b)
{
  const unsigned int sna = (*((const SN_BLOCK*const*)a))->m_sn1;
  const unsigned int snb = (*((const SN_BLOCK*const*)b))->m_sn1;
  if ( sna < snb )
  {
    return (0 == sna) ? 1 : -1;
  }
  if ( snb < sna )
  {
    return (0 == snb) ? -1 : 1;
  }
  return 0;
}

size_t ON_SerialNumberMap::SN_BLOCK::ActiveElementEstimate(unsigned int sn0, unsigned int sn1) const
{
  size_t c = m_count-m_purged;
  if ( c > 0 )
  {
    if ( sn0 < m_sn0 )
      sn0 = m_sn0;
    if ( sn1 > m_sn1 )
      sn1 = m_sn1;
    sn1 -= sn0;
    sn1++;
    if (c > sn1)
      c = sn1;
  }
  return c;
}

static bool ON_SerialNumberMap_IsNotValid()
{
  return ON_IsNotValid(); // <- Good place for a debugger breakpoint
}

bool ON_SerialNumberMap::IsValid(ON_TextLog* textlog) const
{
  size_t i, c, pc, aic;

  aic = 0;

  const_cast<ON_SerialNumberMap*>(this)->BuildHashTableHelper();

  if ( !m_sn_block0.IsValidBlock(textlog,m_hash_table,&aic) )
  {
    if ( textlog )
      textlog->Print("m_sn_block0 is not valid\n");
    return ON_SerialNumberMap_IsNotValid();
  }

  c = m_sn_block0.m_count;
  pc = m_sn_block0.m_purged;

  for ( i = 0; i < m_snblk_list_count; i++ )
  {
    if ( 0 == m_snblk_list[i]->m_count )
    {
      if ( textlog )
        textlog->Print("m_snblk_list[%d] is empty\n",i);
      return ON_SerialNumberMap_IsNotValid();
    }
    if ( 1 != m_snblk_list[i]->m_sorted )
    {
      if ( textlog )
        textlog->Print("m_snblk_list[%d] is not sorted\n",i);
      return ON_SerialNumberMap_IsNotValid();
    }
    if ( !m_snblk_list[i]->IsValidBlock(textlog,m_hash_table,&aic) )
    {
      if ( textlog )
        textlog->Print("m_snblk_list[%d] is not valid\n",i);
      return ON_SerialNumberMap_IsNotValid();
    }
    c += m_snblk_list[i]->m_count;
    pc += m_snblk_list[i]->m_purged;
    if ( i>0 && m_snblk_list[i]->m_sn0 <= m_snblk_list[i-1]->m_sn1 )
    {
      if ( textlog )
        textlog->Print("m_snblk_list[%d]->m_sn0 <= m_snblk_list[%d]->m_sn1\n",i,i-1);
      return ON_SerialNumberMap_IsNotValid();
    }
  }

  if ( c != m_sn_count )
  {
    if ( textlog )
      textlog->Print("m_sn_count=%d (should be %d) is not correct\n",m_sn_count,c);
    return ON_SerialNumberMap_IsNotValid();
  }

  if ( pc != m_sn_purged )
  {
    if ( textlog )
      textlog->Print("m_sn_purged=%d (should be %d) is not correct\n",m_sn_purged,pc);
    return ON_SerialNumberMap_IsNotValid();
  }

  if ( m_active_id_count != aic )
  {
    if ( textlog )
      textlog->Print("m_active_id_count=%d (should be %d) is not correct\n",m_active_id_count,aic);
    return ON_SerialNumberMap_IsNotValid();
  }

  if ( m_active_id_count + m_sn_purged > m_sn_count )
  {
    if ( textlog )
      textlog->Print("m_active_id_count=%d > %d = (m_sn_count-m_sn_purged)\n",m_active_id_count,m_sn_count-m_sn_purged);
    return ON_SerialNumberMap_IsNotValid();
  }

  c = 0;
  for ( i = 0; i < ID_HASH_TABLE_COUNT; i++ )
  {
    const SN_ELEMENT* e;
    for (e = m_hash_table[i]; e && c <= m_active_id_count; e = e->m_next)
    {
      c++;
    }
  }

  if ( c > m_active_id_count )
  {
    if ( textlog )
      textlog->Print("m_hash_table[] linked lists have too many elements.\n");
    return ON_SerialNumberMap_IsNotValid();
  }

  return true;
}

size_t ON_SerialNumberMap::GarbageCollectMoveHelper(ON_SerialNumberMap::SN_BLOCK* dst,ON_SerialNumberMap::SN_BLOCK* src)
{
  // This helper is used by GarbageCollectHelper and moves
  // as many entries as possible from src to dst.  
  // Returns: the number of entries transfered.
  size_t i,j,n;
  if ( src && dst )
  {
    n = SN_BLOCK_CAPACITY - dst->m_count;
    if ( src->m_count < n )
      n = src->m_count;
    if ( n > 0 )
    {
      if ( 0 == dst->m_count )
      {
        dst->EmptyBlock();
      }
      if ( 0 == src->m_sorted )
      {
        dst->m_sorted = 0;
        if ( 0 == dst->m_count )
        {
          dst->m_sn0 = src->m_sn0;
          dst->m_sn1 = src->m_sn1;
        }
      }
      memcpy(&dst->m_sn[dst->m_count],&src->m_sn[0],n*sizeof(src->m_sn[0]));
      dst->m_count += n;
      if ( dst->m_sorted )
      {
        dst->m_sn0 = dst->m_sn[0].m_sn; // set m_sn0 because input dst could have count 0
        dst->m_sn1 = dst->m_sn[dst->m_count-1].m_sn;
      }
      else 
      {
        if ( dst->m_sn0 > src->m_sn0 )
          dst->m_sn0 = src->m_sn0;
        if ( dst->m_sn1 < src->m_sn1 )
          dst->m_sn1 = src->m_sn1;
      }
      i = 0; j = n;
      while ( j < src->m_count )
      {
        src->m_sn[i++] = src->m_sn[j++];
      }
      src->m_count = i;
      if ( src->m_count > 0 )
      {
        if ( src->m_sorted )
          src->m_sn0 = src->m_sn[0].m_sn;
      }
      else
      {
        src->EmptyBlock();
      }
    }
  }
  else
  {
    n = 0;
  }
  return n;
}

void ON_SerialNumberMap::GarbageCollectHelper()
{
  size_t i,j,k,n;

  // This is a helper function.  The caller
  // should check that SN_BLOCK_CAPACITY == m_sn_block0.m_count
  // before calling it.
  InvalidateHashTableHelper();

  if ( m_sn_block0.m_purged > 0 )
  {
    m_sn_count -= m_sn_block0.m_purged;
    m_sn_purged -= m_sn_block0.m_purged;
    m_sn_block0.CullBlockHelper();
    if ( !m_sn_block0.m_sorted )
      m_sn_block0.SortBlockHelper();
    if ( 0 == m_snblk_list_count )
      m_maxsn = m_sn_block0.m_sn1; 
    if ( m_sn_block0.m_count < 7*(SN_BLOCK_CAPACITY/8) )
      return;
  }
  else if ( !m_sn_block0.m_sorted )
  {
    m_sn_block0.SortBlockHelper();
    if ( 0 == m_snblk_list_count )
      m_maxsn = m_sn_block0.m_sn1; 
  }

  // Remove all purged serial numbers from every block
  // and make sure every block is sorted.
  m_sn_purged = 0;
  m_sn_count = m_sn_block0.m_count;
  i = m_snblk_list_count;
  while (i--)
  {
    if ( m_snblk_list[i]->m_purged > 0 )
    {
      // The next call may empty m_snblk_list[i].
      m_snblk_list[i]->CullBlockHelper();
    }
    m_sn_count += m_snblk_list[i]->m_count;
  }

  // Put empty blocks at the end of the list
  for ( i = 0; i < m_snblk_list_count; i++ )
  {
    if ( 0 == m_snblk_list[i]->m_count )
    {
      // m_snblk_list[i] is empty ...
      for(j = i+1; j < m_snblk_list_count; j++ )
      {
        if ( m_snblk_list[j]->m_count > 0 )
        {
          // ... and m_snblk_list[j] is not empty
          ON_qsort(m_snblk_list+i,m_snblk_list_count-i,sizeof(*m_snblk_list),SN_BLOCK::CompareMaxSN);
          break;
        }
      }
      while ( m_snblk_list_count > 0 && 0 == m_snblk_list[m_snblk_list_count-1]->m_count )
        m_snblk_list_count--;
      break;
    }
  }

  if (    m_snblk_list_count > 0 
       && m_snblk_list[m_snblk_list_count-1]->m_sn1 > m_sn_block0.m_sn0 
     )
  {
    // Merge the serial number lists so the blocks in m_sn_list[]
    // have the lowest serial numbers and m_sn_block0.m_sn[] contains
    // the largest.
    size_t snarray_count = 0;
    struct SN_ELEMENT* snarray = (struct SN_ELEMENT*)onmalloc_from_pool(m_pool,2*SN_BLOCK_CAPACITY*sizeof(snarray[0]));
    for ( i = 0; i < m_snblk_list_count && m_sn_block0.m_count > 0; i++ )
    {
      if ( m_snblk_list[i]->m_sn1 < m_sn_block0.m_sn0 )
        continue;

      // Move some entries in m_sn_block0.m_sn[] 
      // to m_snblk_list[i]->m_sn[].
      SN_BLOCK* blk = m_snblk_list[i];
      const unsigned int sn1 = (i < m_snblk_list_count-1) 
                             ? m_snblk_list[i+1]->m_sn0 
                             : (m_sn_block0.m_sn1+1);
      snarray_count = j = k = 0;
      while(j < blk->m_count && k < m_sn_block0.m_count )
      {
        if ( blk->m_sn[j].m_sn < m_sn_block0.m_sn[k].m_sn )
        {
          snarray[snarray_count++] = blk->m_sn[j++];
        }
        else if ( m_sn_block0.m_sn[k].m_sn < sn1 )
        {
          snarray[snarray_count++] = m_sn_block0.m_sn[k++];
        }
        else
        {
          // If m_sn_block0.m_sn[m_sn_block0.m_count-1].m_sn is the largest
          // value, then we should get j == blk->m_count exit.
          // If blk->m_sn[blk->m_count-1].m_sn is the largest value,
          // then we should get k == m_sn_block0.m_count and exit.
          ON_ERROR("Bogus information - should never get here");
          break;
        }
      }
      n = blk->m_count-j;
      if ( n > 0 )
      {
        memcpy(&snarray[snarray_count],&blk->m_sn[j],n*sizeof(snarray[0]));
        snarray_count += n;
      }
      else
      {
        while ( k < m_sn_block0.m_count && m_sn_block0.m_sn[k].m_sn < sn1 )
          snarray[snarray_count++] = m_sn_block0.m_sn[k++];
      }
      n = (snarray_count > SN_BLOCK_CAPACITY) 
        ? SN_BLOCK_CAPACITY 
        : snarray_count;
      if ( k < m_sn_block0.m_count )
      {
        memcpy(&snarray[snarray_count],
               &m_sn_block0.m_sn[k],
               (m_sn_block0.m_count-k)*sizeof(snarray[0])
               );
        snarray_count += (m_sn_block0.m_count-k);
      }
      blk->m_count = n;
      memcpy(&blk->m_sn[0],snarray,blk->m_count*sizeof(blk->m_sn[0]));
      blk->m_sn0 = blk->m_sn[0].m_sn;
      blk->m_sn1 = blk->m_sn[blk->m_count-1].m_sn;
      if ( snarray_count > n )
      {
        // put the end of snarray[] (the largest serial numbers)
        // in m_sn_block0.m_sn[].
        m_sn_block0.m_count = (snarray_count-n);
        memcpy(&m_sn_block0.m_sn[0],
               &snarray[n],
               m_sn_block0.m_count*sizeof(m_sn_block0.m_sn[0])
               );
        m_sn_block0.m_sn0 = m_sn_block0.m_sn[0].m_sn;
        m_sn_block0.m_sn1 = m_sn_block0.m_sn[m_sn_block0.m_count-1].m_sn;
      }
      else
      {
        m_sn_block0.EmptyBlock();
      }
    }
    onfree(snarray);
    snarray = 0;
  }

  // Compact the blocks in m_sn_list[]
  for ( i = 0; i < m_snblk_list_count; i++ )
  {
    for ( j = i+1; j < m_snblk_list_count; j++ )
    {
      if ( m_snblk_list[i]->m_count >= SN_BLOCK_CAPACITY )
        break;
      GarbageCollectMoveHelper(m_snblk_list[i],m_snblk_list[j]);
    }
  }
  while ( m_snblk_list_count > 0 && 0 == m_snblk_list[m_snblk_list_count-1]->m_count )
  {
    m_snblk_list_count--;
  }

  // Make sure m_sn_block0.m_an[] is has plenty of room
  if ( m_sn_block0.m_count > SN_BLOCK_CAPACITY/4 )
  {
    if ( m_snblk_list_count > 0 )
    {
      GarbageCollectMoveHelper(m_snblk_list[m_snblk_list_count-1],&m_sn_block0);
    }
    if ( m_sn_block0.m_count > SN_BLOCK_CAPACITY/2 )
    {
      // Need to add a new block to m_snblk_list[]
      if ( m_snblk_list_count == m_snblk_list_capacity )
      {
        // Add room to store more pointers to blocks in the m_sn_list[]
        i = m_snblk_list_capacity;
        m_snblk_list_capacity += 32;
        n = m_snblk_list_capacity*sizeof(m_snblk_list[0]);
        m_snblk_list = (SN_BLOCK**)((0 == m_snblk_list)
                     ? onmalloc_from_pool(m_pool,n)
                     : onrealloc(m_snblk_list,n));
        while ( i < m_snblk_list_capacity )
          m_snblk_list[i++] = 0;
      }
      if ( 0 == m_snblk_list[m_snblk_list_count] )
      {
        // add room to store at more serial numbers
        m_snblk_list[m_snblk_list_count] = (SN_BLOCK*)onmalloc_from_pool(m_pool,sizeof(*(m_snblk_list[m_snblk_list_count])));
      }
      *m_snblk_list[m_snblk_list_count++] = m_sn_block0;
      m_sn_block0.EmptyBlock();
    }
  }
}

struct ON_SerialNumberMap::SN_ELEMENT* ON_SerialNumberMap::AddSerialNumber(unsigned int sn)
{
  if ( sn <= 0 )
    return 0;
  struct SN_ELEMENT* e = FindElementHelper(sn);
  if ( e )
  {
    if ( 0 == e->m_sn_active )
    {
      m_sn_purged--;
      m_e_blk->m_purged--;
      memset(e,0,sizeof(*e));
      e->m_sn = sn;
      e->m_sn_active = 1;
    }
  }
  else
  {
    if ( SN_BLOCK_CAPACITY == m_sn_block0.m_count )
    {
      // make room in m_sn_block0 for the new serial number
      GarbageCollectHelper();    
    }

    if ( 0 == m_sn_block0.m_count )
    {
      m_sn_block0.m_sn0 = m_sn_block0.m_sn1 = sn;
      m_sn_block0.m_sorted = 1;
    }
    else
    {
      if ( sn > m_sn_block0.m_sn1 )
      {
        m_sn_block0.m_sn1 = sn;
      }
      else
      {
        if ( sn < m_sn_block0.m_sn0 )
          m_sn_block0.m_sn0 = sn;
        m_sn_block0.m_sorted = 0;
      }        
    }
    if ( sn > m_maxsn )
      m_maxsn = sn;
    m_sn_count++;
    e = &m_sn_block0.m_sn[m_sn_block0.m_count++];
    memset(e,0,sizeof(*e));
    e->m_sn = sn;
    e->m_sn_active = 1;
  }

  return e;
}

struct ON_SerialNumberMap::SN_ELEMENT* ON_SerialNumberMap::AddSerialNumberAndId(unsigned int sn,ON_UUID id)
{
  struct SN_ELEMENT* e = AddSerialNumber(sn);
  size_t i;

  if ( 0 != e && 0 == e->m_id_active )
  {
    if ( IdIsNotZero(&id) )
    {
      if (    m_active_id_count > 0 
           && 0 != memcmp(&m_inactive_id,&id,sizeof(m_inactive_id)) 
         )
      {
        // Need to determine if id is already in use.
        BuildHashTableHelper();
        i = HashIndex(&id);
        for(const struct SN_ELEMENT* h = m_hash_table[i];h;h = h->m_next)
        {
          if ( 0 == memcmp(&h->m_id,&id,sizeof(h->m_id)) )
          {
            // This id is already in use. Create a new id.
            ON_CreateUuid(id);
            break;
          }
        }
      }        
    }
    else
    {
      // The input id was zero. Create a new id.
      ON_CreateUuid(id);
    }

    // Add id to the hash table
    e->m_id = id;
    e->m_id_active = 1;
    if ( m_bHashTableIsValid )
    {
      // the table isn't dirty - go ahead
      // and add the element.  If the table
      // is dirty, it will get rebuilt before
      // it needs to be used and this element
      // will be included at that time.
      i = HashIndex(&id);
      e->m_next = m_hash_table[i];
      m_hash_table[i] = e;
    }
    m_active_id_count++;
    memset(&m_inactive_id,0,sizeof(m_inactive_id));
  }

  return e;
}

size_t ON_SerialNumberMap::HashIndex(const ON_UUID* id) const
{
  // this is a private member function and the caller
  // insures the id pointer is not null.
  return (IdCRC(id) % ID_HASH_TABLE_COUNT);
}

void ON_SerialNumberMap::InvalidateHashTableHelper()
{
  // This helper function is called when the memory
  // locations of SN_ELEMENTs are going to change.
  // and the pointers saved in m_hash_table[] and
  // SN_ELEMENT::next may become invalid. When a
  // valid m_hash_table[] is needed at a later time,
  // BuildHashTableHelper() will restore it.
  if ( m_bHashTableIsValid )
  {
    m_bHashTableIsValid = false;
    memset(m_hash_table,0,sizeof(m_hash_table));
  }
}

bool ON_SerialNumberMap::RemoveBlockFromHashTableHelper(const struct ON_SerialNumberMap::SN_BLOCK* blk)
{
  // quickly remove every id in the block from the hash table.
  // This is done when the block is small compared to the
  // size of the hash table and removing the elements from
  // the hash table, rearranging the block, and then adding
  // the elements back into the hash table is faster than
  // simply rebuilding the hash table.
  //
  // The 128 multiplier is determined as follows:
  // - C = average number of elements with the same hash index
  //     = m_active_id_count/ID_HASH_TABLE_COUNT.
  // - D = cost of destroying hash table = time to memset the table
  //       to zero.
  // - H = cost of calculating the hash table index of an id.
  // - A = The cost of adding an element to the hash table is
  //       H + two pointer assignments with is essentially H.
  // - F = The average cost of finding an element in the hash
  //       table is H plus going through half the linked list
  //       of elements at that hash index (C/2 pointer dereferences).
  // - B = number of elements in a block.
  // - I = (number times a valid hash table is destroyed)/
  //       (number of times the hash table is rebuilt).
  // - R = rebuild cost = A*m_active_id_count.
  // - Keeping the the has table up to date only makes sense if
  //   R > I*(A+F)*active_id_count
  //   it is needed often enough to justify ...
  bool rc = false;
  if ( m_bHashTableIsValid && m_active_id_count > 128*blk->m_count )
  {
    const SN_ELEMENT* e;
    SN_ELEMENT* h;
    SN_ELEMENT* prev;
    size_t i, hash_i;
    rc = true;
    for (e = blk->m_sn, i = blk->m_count; i--; e++)
    {
      if ( e->m_id_active )
      {
        prev = 0;
        hash_i = HashIndex(&e->m_id);
        for (h = m_hash_table[hash_i];h;h=h->m_next)
        {
          if (h == e)
          {
            // Do not change value of SN_ELEMENT's m_id_active.
            // This value is needed by AddBlockToHashTableHelper()
            // when it add the element back in.
            m_active_id_count--;
            if (prev)
              prev->m_next = h->m_next;
            else
              m_hash_table[hash_i] = h->m_next;
            break;
          }
          prev = h;
        }
      }
    }
  }
  return rc;
}

void ON_SerialNumberMap::AddBlockToHashTableHelper(struct ON_SerialNumberMap::SN_BLOCK* blk)
{
  if ( m_bHashTableIsValid )
  {
    SN_ELEMENT* e;
    size_t i, hash_i;
    for (e = blk->m_sn, i = blk->m_count; i--; e++)
    {
      if ( e->m_id_active )
      {
        hash_i = HashIndex(&e->m_id);
        e->m_next = m_hash_table[hash_i];
        m_hash_table[hash_i] = e;
      }
    }
  }
}


void ON_SerialNumberMap::BuildHashTableHelper()
{
  // This function is called when code requires
  // m_hash_table[] to be valid so an id can be looked up.
  if ( !m_bHashTableIsValid )
  {
    // The hash table is dirty because elements
    // have moved in memory.  Rebuild it putting
    // the newest elements first.
    m_bHashTableIsValid = true;
    if (m_active_id_count > 0)
    {
      struct SN_BLOCK* blk;
      struct SN_ELEMENT* e;
      size_t snblk_i, j, hash_i;
      for ( snblk_i = 0; snblk_i < m_snblk_list_count; snblk_i++ )
      {
        blk = m_snblk_list[snblk_i];
        if ( blk->m_purged < blk->m_count )
        {
          e = &blk->m_sn[0];
          for(j = blk->m_count; j--; e++ )
          {
            if ( e->m_id_active )
            {
              hash_i = HashIndex(&e->m_id);
              e->m_next = m_hash_table[hash_i];
              m_hash_table[hash_i] = e;
            }
            else
            {
              e->m_next = 0;
            }
          }
        }
      }

      blk = &m_sn_block0;
      if ( blk->m_purged < blk->m_count )
      {
        e = &blk->m_sn[0];
        for(j = blk->m_count; j--; e++ )
        {
          if ( e->m_id_active )
          {
            hash_i = HashIndex(&e->m_id);
            e->m_next = m_hash_table[hash_i];
            m_hash_table[hash_i] = e;
          }
          else
          {
            e->m_next = 0;
          }
        }
      }
    }
  }
}


void ON_SerialNumberMap::SN_ELEMENT::Dump(ON_TextLog& text_log) const
{
  text_log.Print("m_id = ");
  text_log.Print(m_id);
  text_log.Print("\nm_sn = %d\n",m_sn);
  text_log.Print("m_sn_active = %d\n",m_sn_active);
  text_log.Print("m_id_active = %d\n",m_id_active);
}


void ON_SerialNumberMap::SN_BLOCK::Dump(ON_TextLog& text_log) const
{
  text_log.Print("m_count = %d\n",m_count);
  text_log.Print("m_purged = %d\n",m_purged);
  text_log.Print("m_sorted = %d\n",m_sorted);
  text_log.Print("m_sn0 = %d\n",m_sn0);
  text_log.Print("m_sn1 = %d\n",m_sn1);
  if ( m_count > 0 )
  {
    text_log.Print("m_sn[0]\n");
    text_log.PushIndent();
    m_sn[0].Dump(text_log);
    text_log.PopIndent();
    if ( m_count > 1 )
    {
      text_log.Print("m_sn[%d]\n",m_count-1);
      text_log.PushIndent();
      m_sn[m_count-1].Dump(text_log);
      text_log.PopIndent();
    }
  }
}

void ON_SerialNumberMap::Dump(ON_TextLog& text_log) const
{
  text_log.Print("m_maxsn = %d\n",m_maxsn);
  text_log.Print("m_sn_count = %d\n",m_sn_count);
  text_log.Print("m_sn_purged = %d\n",m_sn_purged);
  text_log.Print("m_active_id_count = %d\n",m_sn_purged);
  text_log.Print("m_bHashTableIsValid = %d\n",m_bHashTableIsValid);
  text_log.Print("m_snblk_list_capacity = %d\n",m_snblk_list_capacity);
  text_log.Print("m_snblk_list_count = %d\n",m_snblk_list_count);
  
  text_log.Print("m_sn_block0\n");
  text_log.PushIndent();
  m_sn_block0.Dump(text_log);
  text_log.PopIndent();

  for ( size_t i = 0; i < m_snblk_list_count; i++ )
  {
    text_log.Print("m_snblk_list[%d]\n",i);
    text_log.PushIndent();
    m_snblk_list[i]->Dump(text_log);
    text_log.PopIndent();
  }
}
