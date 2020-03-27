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

#if !defined(OPENNURBS_MAP_INC_)
#define OPENNURBS_MAP_INC_

/*
Description:
  ON_SerialNumberMap provides a way to map set of unique 
  serial number - uuid pairs to application defined values
  so that adding, finding and removing serial numbers is 
  fast and efficient.  The class is designed to handle
  several millions of unique serial numbers.  There are no
  restrictions on what order numbers are added and removed.
  The minimum memory footprint is less than 150KB and doesn't
  increase until you have more than 8000 serial numbers.
  It is possible to have an active serial number and an
  inactive id.
*/
class ON_CLASS ON_SerialNumberMap
{
public:
  ON_SerialNumberMap( ON_MEMORY_POOL* pool = 0 );
  ~ON_SerialNumberMap();

  struct MAP_VALUE
  {
    ON__UINT32 m_u_type;
    union
    {
      void* ptr;
      unsigned int ui;
      int i;
    } m_u;
  };

  struct SN_ELEMENT
  {
    ////////////////////////////////////////////////////////////
    //
    // ID
    //
    ON_UUID m_id;
    struct SN_ELEMENT* m_next; // id hash table linked list

    ////////////////////////////////////////////////////////////
    //
    // Serial number:
    //
    unsigned int m_sn;

    ////////////////////////////////////////////////////////////
    //
    // Status flags:
    //
    // If m_id_active is 1, then m_sn_active must be 1.
    // If m_sn_active = 1, then m_id_active can be 0 or 1.
    //
    unsigned char m_sn_active; // 1 = serial number is active
    unsigned char m_id_active; // 1 = id is active
    unsigned char m_reserved1;
    unsigned char m_reserved2;

    ////////////////////////////////////////////////////////////
    //
    // User information:
    //
    //   ON_SerialNumberMap does not use the m_value field.
    //   When a new element is added, m_value is memset to
    //   zero.  Other than that, m_value is not changed by
    //   this class.  The location of m_value in memory,
    //   (&m_value) may change at any time.
    struct MAP_VALUE m_value;

    void Dump(ON_TextLog&) const;
  };

  /*
  Returns:
    Number of active serial numbers in the list.
  */
  std::size_t ActiveSerialNumberCount() const;

  /*
  Returns:
    Number of active ids in the list.  This number
    is less than or equal to ActiveSerialNumberCount().
  */
  std::size_t ActiveIdCount() const;

  /*
  Returns:
    The active element with the smallest serial number, 
    or null if the list is empty.
  Restrictions:
    The returned pointer may become invalid after any
    subsequent calls to any function in this class.  
    If you need to save information in the returned
    SN_ELEMENT for future use, you must copy the 
    information into storage you are managing.

    You may change the value of the SN_ELEMENT's m_value
    field.  You must NEVER change any other SN_ELEMENT
    fields or you will break searching and possibly cause
    crashes.
  */
  struct SN_ELEMENT* FirstElement() const;

  /*
  Returns:
    The active element with the biggest serial number,
    or null if the list is empty.
  Restrictions:
    The returned pointer may become invalid after any
    subsequent calls to any function in this class.  
    If you need to save information in the returned
    SN_ELEMENT for future use, you must copy the 
    information into storage you are managing.

    You may change the value of the SN_ELEMENT's m_value
    field.  You must NEVER change any other SN_ELEMENT
    fields or you will break searching and possibly cause
    crashes.
  */
  struct SN_ELEMENT* LastElement() const;

  /*
  Parameters:
    sn - [in] serial number to search for.
  Returns:
    If the serial number is active, a pointer to
    its element is returned.
  Restrictions:
    The returned pointer may become invalid after any
    subsequent calls to any function in this class.  
    If you need to save information in the returned
    SN_ELEMENT for future use, you must copy the 
    information into storage you are managing.

    You may change the value of the SN_ELEMENT's m_value
    field.  You must NEVER change any other SN_ELEMENT
    fields or you will break searching and possibly cause
    crashes.
  */
  struct SN_ELEMENT* FindSerialNumber(unsigned int sn) const;

  /*
  Parameters:
    id - [in] id number to search for.
  Returns:
    If the id is active, a pointer to
    its element is returned.
  Restrictions:
    The returned pointer may become invalid after any
    subsequent calls to any function in this class.  
    If you need to save information in the returned
    SN_ELEMENT for future use, you must copy the 
    information into storage you are managing.

    You may change the value of the SN_ELEMENT's m_value
    field.  You must NEVER change any other SN_ELEMENT
    fields or you will break searching and possibly cause
    crashes.
  */
  struct SN_ELEMENT* FindId(ON_UUID) const;

  /*
  Description:
    Add a serial number to the map.
  Parameters:
    sn - [in] serial number to add.
  Returns:
    If the serial number is valid (>0), a pointer to its
    element is returned.  When a new element is added, 
    every byte of the m_value field is set to 0.
    If the serial number was already active, its element is
    also returned.  If you need to distinguish between new
    and previously existing elements, then change  
    m_value.m_u_type to something besides 0 after you add
    a new serial number.  The id associated with this
    serial number will be zero and cannot be found using
    FindId().
  Restrictions:
    The returned pointer may become invalid after any
    subsequent calls to any function in this class.  
    If you need to save information in the returned
    SN_ELEMENT for future use, you must copy the 
    information into storage you are managing.

    You may change the value of the SN_ELEMENT's m_value
    field.  You must NEVER change any other SN_ELEMENT
    fields or you will break searching and possibly cause
    crashes.
  */
  struct SN_ELEMENT* AddSerialNumber(unsigned int sn);

  /*
  Parameters:
    sn - [in] serial number to add.
    id - [in] suggested id to add. If id is zero or
              already in use, another id will be assigned
              to the element.
  Returns:
    If the serial number is valid (>0), a pointer to its
    element is returned.  When a new element is added, 
    every byte of the m_value field is set to 0.
    If the serial number was already active, its element is
    also returned.  If you need to distinguish between new
    and previously existing elements, then change  
    m_value.m_u_type to something besides 0 after you add
    a new serial number. 
    If the id parameter is zero, then a new uuid is created
    and added. If the id parameter is non zero but is active
    on another element, a new uuid is created and added.
    You can inspect the value of m_id on the returned element
    to determine the id AddSerialNumberAndId() assigned to
    the element.
  Restrictions:
    The returned pointer may become invalid after any
    subsequent calls to any function in this class.  
    If you need to save information in the returned
    SN_ELEMENT for future use, you must copy the 
    information into storage you are managing.

    You may change the value of the SN_ELEMENT's m_value
    field.  You must NEVER change any other SN_ELEMENT
    fields or you will break searching and possibly cause
    crashes.
  */
  struct SN_ELEMENT* AddSerialNumberAndId(unsigned int sn, ON_UUID id);

  /*
  Parameters:
    sn - [in] serial number of the element to remove.
  Returns:
    If the serial number was active, it is removed
    and a pointer to its element is returned.  If
    the element's id was active, the id is also removed.
  Restrictions:
    The returned pointer may become invalid after any
    subsequent calls to any function in this class.  
    If you need to save information in the returned
    SN_ELEMENT for future use, you must copy the 
    information into storage you are managing.

    You may change the value of the SN_ELEMENT's m_value
    field.  You must NEVER change any other SN_ELEMENT
    fields or you will break searching and possibly cause
    crashes.
  */
  struct SN_ELEMENT* RemoveSerialNumberAndId(unsigned int sn);

  /*
  Parameters:
    sn - [in] If > 0, this is the serial number
              of the element with the id. If 0, the
              field is ignored.
    id - [in] id to search for.
  Returns:
    If the id was active, it is removed and a pointer
    to its element is returned.  The element's serial
    remains active. To remove both the id and serial number,
    use RemoveSerialNumberAndId().
  Restrictions:
    The returned pointer may become invalid after any
    subsequent calls to any function in this class.  
    If you need to save information in the returned
    SN_ELEMENT for future use, you must copy the 
    information into storage you are managing.

    You may change the value of the SN_ELEMENT's m_value
    field.  You must NEVER change any other SN_ELEMENT
    fields or you will break searching and possibly cause
    crashes.
  */
  struct SN_ELEMENT* RemoveId(unsigned int sn, ON_UUID id);

  /*
  Description:
    Finds all the elements whose serial numbers are
    in the range sn0 <= sn <= sn1 and appends them
    to the elements[] array.  If max_count > 0, it
    specifies the maximum number of elements to append.
  Parameters:
    sn0 - [in]
      Minimum serial number.
    sn1 - [in]
      Maximum serial number
    max_count - [in]
      If max_count > 0, this parameter specifies the
      maximum number of elements to append.
    elements - [out]
      Elements are appended to this array
  Returns:
    Number of elements appended to elements[] array.
  Remarks:
    When many elements are returned, GetElements() can be
    substantially faster than repeated calls to FindElement().
  */
  std::size_t GetElements(
          unsigned int sn0,
          unsigned int sn1, 
          std::size_t max_count,
          ON_SimpleArray<SN_ELEMENT>& elements
          ) const;

  /*
  Description:
    Empties the list.
  */
  void EmptyList();

  /*
  Description:
    Returns true if the map is valid.  Returns false if the
    map is not valid.  If an error is found and textlog
    is not null, then a description of the problem is sent
    to textlog.
  Returns:
    true if the list if valid.
  */
  bool IsValid(ON_TextLog* textlog) const;

  void Dump(ON_TextLog& text_log) const;

private:
  // prohibit copy construction and operator=
  // no implementation
  ON_SerialNumberMap(const ON_SerialNumberMap&);
  ON_SerialNumberMap& operator=(const ON_SerialNumberMap&);

  enum
  {
    // These numbers are chosen so the ON_SerialNumberMap
    // will be computationally efficient for up to
    // 10 million entries.
    SN_BLOCK_CAPACITY = 8192,
    SN_PURGE_RATIO = 16,
    ID_HASH_TABLE_COUNT = 8192
  };

  struct SN_BLOCK
  {
    std::size_t m_count;  // used elements in m_sn[]
    std::size_t m_purged; // number of purged elements in m_sn[]
    unsigned int m_sorted; // 0 = no, 1 = yes
    unsigned int m_sn0; // minimum sn in m_sn[]
    unsigned int m_sn1; // maximum sn in m_sn[]
    struct SN_ELEMENT m_sn[SN_BLOCK_CAPACITY];
    void EmptyBlock();
    void CullBlockHelper();
    void SortBlockHelper();
    bool IsValidBlock(ON_TextLog* textlog,struct SN_ELEMENT*const* hash_table,std::size_t* active_id_count) const;
    struct SN_ELEMENT* BinarySearchBlockHelper(unsigned int sn);
    static int CompareMaxSN(const void*,const void*);
    std::size_t ActiveElementEstimate(unsigned int sn0, unsigned int sn1) const;
    void Dump(ON_TextLog&) const;
  };

  unsigned int m_maxsn; // largest sn stored anywhere
  unsigned int m_reserved;

  // All heap used in this class is allocated from this pool.
  ON_MEMORY_POOL* m_pool;

  // Serial Number list counts
  std::size_t m_sn_count;   // total number of elements                       
  std::size_t m_sn_purged;  // total number of purged elements

  // ID hash table counts (all ids in the hash table are active)
  bool m_bHashTableIsValid; // true if m_hash_table[] is valid
  std::size_t m_active_id_count; // number of active ids in the hash table
  ON_UUID m_inactive_id;    // frequently and id is removed and
                            // then added back.  m_inactive_id
                            // records the most recently removed
                            // id so we don't have to waste time
                            // searching the hash table for
                            // an id that is not there.
                            

  // The blocks in m_sn_list[] are alwasy sorted, disjoint,
  // and in increasing order.  m_sn_list is used when
  // m_sn_block0.m_sn[] is not large enough.
  // The sn list is partitioned into blocks to avoid
  // requiring large amounts of contiguous memory for
  // situations with millions of serial numbers.
  struct SN_BLOCK** m_snblk_list;
  std::size_t m_snblk_list_capacity; // capacity of m_blk_list[]
  std::size_t m_snblk_list_count;    // used elements in m_snblk_list[]

  // If FindElementHelper() returns a non-null pointer
  // to an element, then m_e_blk points to the SN_BLOCK
  // that contains the returned element.  In all other
  // situations the value in m_e_blk is undefined and
  // m_e_blk must not be dereferenced.
  struct SN_BLOCK* m_e_blk;

  // m_sn_block0 is where the new additions are added.
  // When serial numbers are not added in increasing
  // order, m_sn_block0.m_sn[] may not be sorted.
  SN_BLOCK m_sn_block0;

  struct SN_ELEMENT* FindElementHelper(unsigned int sn);
  void UpdateMaxSNHelper();
  void GarbageCollectHelper();
  std::size_t GarbageCollectMoveHelper(SN_BLOCK* dst,SN_BLOCK* src);

  // When m_bHashTableIsValid is true, then m_hash_table[i] is 
  // a linked list of elements whose id satisfies 
  // i = HashIndex(&e->m_id).  When m_bHashTableIsValid is false,
  // m_hash_table[] is identically zero.
  struct SN_ELEMENT* m_hash_table[ID_HASH_TABLE_COUNT];
  std::size_t HashIndex(const ON_UUID*) const;
  void InvalidateHashTableHelper(); // marks table as dirty
  bool RemoveBlockFromHashTableHelper(const struct SN_BLOCK* blk);
  void AddBlockToHashTableHelper(struct SN_BLOCK* blk);
  void BuildHashTableHelper();      // prepares table for use
};


#endif
