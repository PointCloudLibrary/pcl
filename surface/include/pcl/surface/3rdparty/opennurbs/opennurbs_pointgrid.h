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

#if !defined(OPENNURBS_POINT_GRID_INC_)
#define OPENNURBS_POINT_GRID_INC_

class ON_CLASS ON_PointGrid : public ON_Geometry
{
public:
  ON_PointGrid();
  ON_PointGrid(const ON_PointGrid&);
  ON_PointGrid(
          int,  // point count0 (>=1)
          int   // point count1 (>=1)
          );

  void Initialize(void);  // zeros all fields

  ON_BOOL32 Create( 
          int,  // point count0 (>=1)
          int   // point count1 (>=1)
          );

  void Destroy();

  virtual ~ON_PointGrid();
  void EmergencyDestroy(); // call if memory used by point grid becomes invalid

	ON_PointGrid& operator=(const ON_PointGrid&);

  // point_grid[i][j] returns GetPoint(i,j)
  ON_3dPoint* operator[](int);             // 0 <= index < PointCount(0)
  const ON_3dPoint* operator[](int) const; // 0 <= index < PointCount(0)
  
  /////////////////////////////////////////////////////////////////
  // ON_Object overrides

  /*
  Description:
    Tests an object to see if its data members are correctly
    initialized.
  Parameters:
    text_log - [in] if the object is not valid and text_log
        is not NULL, then a brief englis description of the
        reason the object is not valid is appened to the log.
        The information appended to text_log is suitable for 
        low-level debugging purposes by programmers and is 
        not intended to be useful as a high level user 
        interface tool.
  Returns:
    @untitled table
    true     object is valid
    false    object is invalid, uninitialized, etc.
  Remarks:
    Overrides virtual ON_Object::IsValid
  */
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  void Dump( ON_TextLog& ) const; // for debugging

  ON_BOOL32 Write(
         ON_BinaryArchive&  // open binary file
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive&  // open binary file
       );

  ON::object_type ObjectType() const;

  /////////////////////////////////////////////////////////////////
  // ON_Geometry overrides

  int Dimension() const;

  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // minimum
         double*,    // maximum
         ON_BOOL32 = false  // true means grow box
         ) const;

  /*
	Description:
    Get tight bounding box of the point grid.
	Parameters:
		tight_bbox - [in/out] tight bounding box
		bGrowBox -[in]	(default=false)			
      If true and the input tight_bbox is valid, then returned
      tight_bbox is the union of the input tight_bbox and the 
      tight bounding box of the point grid.
		xform -[in] (default=NULL)
      If not NULL, the tight bounding box of the transformed
      point grid is calculated.  The point grid is not modified.
	Returns:
    True if the returned tight_bbox is set to a valid 
    bounding box.
  */
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  ON_BOOL32 Transform( 
         const ON_Xform&
         );

  // virtual ON_Geometry::IsDeformable() override
  bool IsDeformable() const;

  // virtual ON_Geometry::MakeDeformable() override
  bool MakeDeformable();

  ON_BOOL32 SwapCoordinates(
        int, int        // indices of coords to swap
        );

  /////////////////////////////////////////////////////////////////
  // Interface

  ON_BOOL32 IsClosed( 
        int // dir
        ) const;

  int PointCount(   // number of points in grid direction
        int         // dir 0 = "s", 1 = "t"
        ) const;

  int PointCount(   // total number of points in grid
        void
        ) const;

  ON_3dPoint& Point(
        int, int // point index ( 0 <= i <= PointCount(0), 0 <= j <= PointCount(1)
        );

  ON_3dPoint Point(
        int, int // point index ( 0 <= i <= PointCount(0), 0 <= j <= PointCount(1)
        ) const;

  double* PointArray();

  const double* PointArray() const;

  int PointArrayStride(  // point stride in grid direction
        int         // dir 0 = "s", 1 = "t"
        ) const;

  ON_BOOL32 SetPoint(      // set a single point
        int, int, // point index ( 0 <= i <= PointCount(0), 0 <= j <= PointCount(1)
        const ON_3dPoint& // value of point
        );

  ON_BOOL32 GetPoint(              // get a single control vertex
        int, int,   // CV index ( 0 <= i <= CVCount(0), 0 <= j <= CVCount(1)
        ON_3dPoint&      // gets euclidean cv when NURBS is rational
        ) const;

  ON_BOOL32 Reverse(  // reverse grid order
    int // dir  0 = "s", 1 = "t"
    );

  ON_BOOL32 Transpose(); // transpose grid points

  /////////////////////////////////////////////////////////////////
  // Implementation
protected:

  int m_point_count[2];   // number of points (>=1)
  int m_point_stride0;    // >= m_point_count[1]
  ON_3dPointArray m_point;
  // point[i][j] = m_point[i*m_point_stride0+j]

private:
  static ON_3dPoint m_no_point; // prevent crashes when sizes are 0

  ON_OBJECT_DECLARE(ON_PointGrid);
};


#endif
