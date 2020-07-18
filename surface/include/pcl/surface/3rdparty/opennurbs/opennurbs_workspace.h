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

#if !defined(OPENNURBS_WORKSPACE_INC_)
#define OPENNURBS_WORKSPACE_INC_

/*
Description:
  Use ON_Workspace classes on the stack to efficiently get 
  and automatically clean up workspace memory and scratch 
  files.
*/
class ON_CLASS ON_Workspace
{
public:
  /*
  Description:
    ON_Workspace classes should be on the stack
    or as members on classes that are never copied.
    The destructor frees memory that was allocated by
    ON_Workspace::GetMemory and closes files that were 
    opened with ON_Workspace::OpenFile.
  */
  ON_Workspace();

  /*
  Description:
    The destructor frees memory that was allocated by
    ON_Workspace::GetMemory and closes files that were 
    opened with ON_Workspace::OpenFile.
  */
  ~ON_Workspace();


  /*
  Description:
    The destructor frees memory that was allocated by
    ON_Workspace::GetMemory and closes files that were 
    opened with ON_Workspace::OpenFile.  The workspace
    can be used again after calling destroy.
  */
  void Destroy();

  /*
  Description:
    Gets a block of heap memory that will be freed by 
    ~ON_Workspace. The intent of ON_Workspace::GetMemory
    is to provide an easy way to get blocks of scratch 
    memory without having to worry about cleaning up 
    before returning.
  Parameters:
    sz - [in] (>0) size of memory block in bytes. 
              If sz <= 0, then NULL is returned.
  Returns:
    A pointer to the memory block.
  Remarks.
    onmalloc() is used to get the block of memory.
    Do NOT free the pointer returned by GetMemory().
    ~ON_Workspace() will free the memory.  If you decide
    you want to keep the memory block, pass the pointer
    to KeepMemory before ~ON_Workspace is called.
  See Also:
    ON_Workspace::::~ON_Workspace
    ON_Workspace::KeepMemory
    ON_Workspace::GrowMemory
    ON_Workspace::GetIntMemory
    ON_Workspace::GetDoubleMemory
    ON_Workspace::GetPointMemory
    ON_Workspace::GetVectorMemory
  */
  void* GetMemory( std::size_t sz );

  /*
  Description:
    Gets an array of integers that will be freed by ~ON_Workspace.
    The intent of ON_Workspace::GetIntMemory is to provide
    an easy way to get scratch integer arrays without
    having to worry about cleaning up before returning.
  Parameters:
    count - [in] (>0) number of integers in memory block.
              If count <= 0, then NULL is returned.
  Returns:
    A pointer to the array of integers.
  Remarks.
    This is a simple helper function so you don't have to
    mess around with (int*) casts and sizeof(int)s in a call
    to GetMemory().  It is exactly like calling
    (int*)GetMemory(count*sizeof(int));
  See Also:
    ON_Workspace::GetMemory
    ON_Workspace::KeepMemory
    ON_Workspace::GrowIntMemory
  */
  int* GetIntMemory( std::size_t count );

  /*
  Description:
    Gets an matrix of integers
  Parameters:
    row_count - [in] (>0) number of  rows
    col_count - [in] (>0) number of columns
  Returns:
    A pointer p so that p[i][j] is an integer when
    0 <= i < row_count and 0 <= j < col_count.
  Remarks.
    This is a simple helper function so you don't have to
    mess around building the 2d array.
  See Also:
    ON_Workspace::KeepMemory
  */
  int** GetIntMemory( std::size_t row_count, std::size_t col_count );

  /*
  Description:
    Gets an array of doubles that will be freed by ~ON_Workspace.
    The intent of ON_Workspace::GetDoubleMemory is to provide
    an easy way to get scratch double arrays without
    having to worry about cleaning up before returning.
  Parameters:
    count - [in] (>0) number of doubles in memory block.
              If count <= 0, then NULL is returned.
  Returns:
    A pointer to the array of doubles.
  Remarks.
    This is a simple helper function so you don't have to
    mess around with (double*) casts and sizeof(double)s 
    in a call to GetMemory().  It is exactly like calling
    (double*)GetMemory(count*sizeof(double));
  See Also:
    ON_Workspace::GetMemory
    ON_Workspace::KeepMemory
    ON_Workspace::GrowIntMemory
  */
  double* GetDoubleMemory( std::size_t count );

  /*
  Description:
    Gets an matrix of doubles
  Parameters:
    row_count - [in] (>0) number of  rows
    col_count - [in] (>0) number of columns
  Returns:
    A pointer p so that p[i][j] is an double when
    0 <= i < row_count and 0 <= j < col_count.
  Remarks.
    This is a simple helper function so you don't have to
    mess around building the 2d array.
  See Also:
    ON_Workspace::KeepMemory
  */
  double** GetDoubleMemory( std::size_t row_count, std::size_t col_count );

  /*
  Description:
    Gets an array of ON_3dPoints that will be freed by ~ON_Workspace.
    The intent of ON_Workspace::GetPointMemory is to 
    provide an easy way to get scratch point arrays without
    having to worry about cleaning up before returning.
  Parameters:
    count - [in] (>0) number of points in memory block.
              If count <= 0, then NULL is returned.
  Returns:
    A pointer to the memory block.
  Remarks.
    This is a simple helper function so you don't have to
    mess around with (ON_3dPoint*) casts and sizeof(ON_3dPoint)s
    in a call to GetMemory().  It is exactly like calling
    (ON_3dPoint*)GetMemory(count*sizeof(ON_3dPoint));
  See Also:
    ON_Workspace::GetMemory
    ON_Workspace::KeepMemory
    ON_Workspace::GrowIntMemory
  */
  ON_3dPoint* GetPointMemory( std::size_t count );

  /*
  Description:
    Gets an array of ON_3dVectors that will be freed by ~ON_Workspace.
    The intent of ON_Workspace::GetVectorMemory is to 
    provide an easy way to get scratch Vector arrays without
    having to worry about cleaning up before returning.
  Parameters:
    count - [in] (>0) number of Vectors in memory block.
              If count <= 0, then NULL is returned.
  Returns:
    A pointer to the memory block.
  Remarks.
    This is a simple helper function so you don't have to
    mess around with (ON_3dVector*) casts and sizeof(ON_3dVector)s
    in a call to GetMemory().  It is exactly like calling
    (ON_3dVector*)GetMemory(count*sizeof(ON_3dVector));
  See Also:
    ON_Workspace::GetMemory
    ON_Workspace::KeepMemory
    ON_Workspace::GrowIntMemory
  */
  ON_3dVector* GetVectorMemory( std::size_t count );

  /*
  Description:
    Grows a block of heap memory that was allocated by
    ON_Workspace::GetMemory.
  Parameters:
    ptr - [in] pointer returned by an earlier call to
               GetMemory or GrowMemory.
    sz - [in] (>0) size of memory block in bytes. 
              If sz <= 0, then NULL is returned.
              If ptr is not NULL and was not allocated by an 
              earlier call to GetMemory or GrowMemory, then
              NULL is returned.
  Returns:
    A pointer to the memory block.
  Remarks.
    onrealloc() is used to grow the block of memory.
    Do NOT free the pointer returned by GrowMemory().
    ~ON_Workspace() will free the memory.  If you decide
    you want to keep the memory block, pass the pointer
    to KeepMemory before ~ON_Workspace is called.
  See Also:
    ON_Workspace::GetMemory
    ON_Workspace::KeepMemory
    ON_Workspace::GrowIntMemory
    ON_Workspace::GrowDoubleMemory
    ON_Workspace::GrowPointMemory
    ON_Workspace::GrowVectorMemory
  */
  void* GrowMemory( void* ptr, std::size_t sz );

  /*
  Description:
    Grows the array of integers that was allocated by
    GetIntMemory or GrowIntMemory.
  Parameters:
    ptr - [in] pointer returned by an earlier call to
               GetIntMemory or GrowIntMemory.
    count - [in] (>0) number of integers in memory block.
              If count <= 0, then NULL is returned.
              If ptr was not allocated by this ON_Workspace
              class, then NULL is returned.
  Returns:
    A pointer to the integer array.
  Remarks.
    onrealloc() is used to grow the block of memory.
    Do NOT free the pointer returned by GrowIntMemory().
    ~ON_Workspace() will free the memory.  If you decide
    you want to keep the memory block, pass the pointer
    to KeepMemory before ~ON_Workspace is called.
  See Also:
    ON_Workspace::GetIntMemory
    ON_Workspace::KeepMemory
  */
  int* GrowIntMemory( int* ptr, std::size_t count );

  /*
  Description:
    Grows the array of doubles that was allocated by
    GetDoubleMemory or GrowDoubleMemory.
  Parameters:
    ptr - [in] pointer returned by an earlier call to
               GetDoubleMemory or GrowDoubleMemory.
    count - [in] (>0) number of doubles in memory block.
              If count <= 0, then NULL is returned.
              If ptr was not allocated by this ON_Workspace
              class, then NULL is returned.
  Returns:
    A pointer to the double array.
  Remarks.
    onrealloc() is used to grow the block of memory.
    Do NOT free the pointer returned by GrowDoubleMemory().
    ~ON_Workspace() will free the memory.  If you decide
    you want to keep the memory block, pass the pointer
    to KeepMemory before ~ON_Workspace is called.
  See Also:
    ON_Workspace::GetDoubleMemory
    ON_Workspace::KeepMemory
  */
  double* GrowDoubleMemory( double* ptr, std::size_t count );

  /*
  Description:
    Grows the array of points that was allocated by
    GetPointMemory or GrowPointMemory.
  Parameters:
    ptr - [in] pointer returned by an earlier call to
               GetPointMemory or GrowPointMemory.
    count - [in] (>0) number of points in memory block.
              If count <= 0, then NULL is returned.
              If ptr was not allocated by this ON_Workspace
              class, then NULL is returned.
  Returns:
    A pointer to the point array.
  Remarks.
    onrealloc() is used to grow the block of memory.
    Do NOT free the pointer returned by GrowMemory().
    ~ON_Workspace() will free the memory.  If you decide
    you want to keep the memory block, pass the pointer
    to KeepMemory before ~ON_Workspace is called.
  See Also:
    ON_Workspace::GetPointMemory
    ON_Workspace::KeepMemory
  */
  ON_3dPoint* GrowPointMemory( ON_3dPoint* ptr, std::size_t count );

  /*
  Description:
    Grows the array of vectors that was allocated by
    GetVectorMemory or GrowVectorMemory.
  Parameters:
    ptr - [in] pointer returned by an earlier call to
               GetVectorMemory or GrowVectorMemory.
    count - [in] (>0) number of vectors in memory block.
              If count <= 0, then NULL is returned.
              If ptr was not allocated by this ON_Workspace
              class, then NULL is returned.
  Returns:
    A pointer to the vector array.
  Remarks.
    onrealloc() is used to grow the block of memory.
    Do NOT free the pointer returned by GrowMemory().
    ~ON_Workspace() will free the memory.  If you decide
    you want to keep the memory block, pass the pointer
    to KeepMemory before ~ON_Workspace is called.
  See Also:
    ON_Workspace::GetVectorMemory
    ON_Workspace::KeepMemory
  */
  ON_3dVector* GrowVectorMemory( ON_3dVector* ptr, std::size_t count );

  /*
  Description:
    Calling the KeepMemory() function with a pointer 
    returned from one of the Get...() or Grow...() calls 
    keeps the workspace destructor from freeing the memory.
    After calling KeepMemory(), you can no longer use
    Grow...() on the pointer.  The caller is responsible 
    for using onfree() to release the memory when it is no
    longer needed.
  Parameters:
    ptr - [in] pointer returned by a Get...() or Grow()
               call to this ON_Workspace.
  Returns:
    True if the pointer was successfully found and removed
    from this ON_Workspace.
  See Also:
    ON_Workspace::~ON_Workspace
    ON_Workspace::GetMemory
    ON_Workspace::KeepAllMemory
  */
  ON_BOOL32 KeepMemory( void* ptr );

  /*
  Description:
    Calling KeepAllMemory() has the same effect as calling
    KeepMemory(p) for every active allocation in the workspace.
    After calling KeepAllMemory(), you can no longer use
    Grow...() on the pointers and you are responsible 
    for using onfree() to release the memory when it is no
    longer needed.
  See Also:
    ON_Workspace::~ON_Workspace
    ON_Workspace::GetMemory
    ON_Workspace::KeepMemory
  */
  void KeepAllMemory();

  /*
  Description:
    Uses ON::OpenFile to open a file.  ~ON_Workspace will
    close the file.
  Parameters:
    filename - [in] name of file
    filemode - [in] open mode (just like second argument to fopen).
  Returns:
    Pointer to opened file.
  Remarks:
    ~ON_Workspace will close the file.
  See Also:
    ON_Workspace::~ON_Workspace
    ON_Workspace::KeepFile
    ON::OpenFile
  */
  FILE* OpenFile(
          const char* filename, 
          const char* filemode
          );

  /*
  Description:
    Uses ON::OpenFile to open a file.  ~ON_Workspace will
    close the file.
  Parameters:
    filename - [in] name of file
    filemode - [in] open mode (just like second argument to _wfopen).
  Returns:
    Pointer to opened file.
  Remarks:
    ~ON_Workspace will close the file.
  See Also:
    ON_Workspace::~ON_Workspace
    ON_Workspace::KeepFile
    ON::OpenFile
  */
  FILE* OpenFile(
          const wchar_t* filename, 
          const wchar_t* filemode
          );

  /*
  Description:
    If you want to prevent ~ON_Workspace from closing a file
    that was opened with ON_Workspace::OpenFile, then pass
    the returned FILE pointer to KeepFile.  After calling
    KeepFile, the caller is responsible for calling
    ON::CloseFile to close the file.
  Parameters:
    fileptr - [in] pointer returned by OpenFile.
  Returns:
    True if file was successfully closed.
  See Also:
    ON_Workspace::~ON_Workspace
    ON_Workspace::OpenFile
    ON::OpenFile
    ON::CloseFile
  */
  int KeepFile(FILE* fileptr);

private:
  struct ON_Workspace_FBLK * m_pFileBlk;
  struct ON_Workspace_MBLK * m_pMemBlk;

private:
  // There is no implementation of the following to prevent use.
  // ON_Workspaces should never be copied, or you will get
  // multiple attempts to free the same pointer.
  ON_Workspace( const ON_Workspace& );
  ON_Workspace& operator=( const ON_Workspace& );
};


#endif
