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

#if !defined(OPENNURBS_POINTCLOUD_INC_)
#define OPENNURBS_POINTCLOUD_INC_

///////////////////////////////////////////////////////////////////////////////
//
// Class  ON_PointCloud  - unordered set of points
//          ON_PointField  - point height field
//

class ON_CLASS ON_PointCloud : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_PointCloud);

public:
  ON_PointCloud();
  ON_PointCloud(
    int  // initial point array capacity
    );
  ON_PointCloud( const ON_PointCloud& );
  ~ON_PointCloud();
  ON_PointCloud& operator=( const ON_PointCloud& );

  ON_3dPoint& operator[](int);
  const ON_3dPoint& operator[](int) const;

  /*
  Description:
    Get a point cloud point from an ON_COMPONENT_INDEX.
  Parameters:
    ci - [in] a component index with m_typ set to ON_COMPONENT_INDEX::pointcloud_point
              and 0 <= m_index and m_index < m_P.Count().
  Returns:
    Point at [ci.m_index] or ON_UNSET_POINT if ci is not valid.
  */
  ON_3dPoint Point( ON_COMPONENT_INDEX ci ) const;

  void Destroy();

  /*
  Description:
    Call when the memory pool used the point cloud's arrays is
    no longer in existence.
  */
  void EmergencyDestroy();

  // virtual ON_Object override
  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  // virtual ON_Object override
  void Dump( ON_TextLog& ) const; // for debugging

  // virtual ON_Object override
  ON_BOOL32 Write( ON_BinaryArchive& ) const;

  // virtual ON_Object override
  ON_BOOL32 Read( ON_BinaryArchive& );

  // virtual ON_Object override
  ON::object_type ObjectType() const;

  // virtual ON_Geometry override
  int Dimension() const;

  // virtual ON_Geometry override
  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // minimum
         double*,    // maximum
         ON_BOOL32 = false  // true means grow box
         ) const;

  // virtual ON_Geometry override
	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  // virtual ON_Geometry override
  ON_BOOL32 Transform( 
         const ON_Xform&
         );

  // virtual ON_Geometry override
  bool IsDeformable() const;

  // virtual ON_Geometry override
  bool MakeDeformable();

  // virtual ON_Geometry override
  ON_BOOL32 SwapCoordinates(
        int, int        // indices of coords to swap
        );

  /*
  Description:
    Get the index of the point in the point cloud that is closest
    to P.
  Parameters:
    P - [in]
    closest_point_index - [out]
    maximum_distance - [in] optional distance constraint.
        If maximum_distance > 0, then only points Q with
        |P-Q| <= maximum_distance are tested.
  Returns:
    True if a point is found; in which case *closest_point_index
    is the index of the point.  False if no point is found
    or the input is not valid.
  See Also:
    ON_GetClosestPointInPointList
  */
  bool GetClosestPoint( 
          ON_3dPoint P,
          int* closest_point_index,
          double maximum_distance = 0.0
          ) const;


  /////////////////////////////////////////////////////////////////
  // Interface
  // 
  int PointCount() const;
  void AppendPoint( const ON_3dPoint& );
  void InvalidateBoundingBox(); // call if you change values of points

  // for ordered streams
  void SetOrdered(bool bOrdered); // true if set is ordered stream
  bool IsOrdered() const; // true if set is ordered stream

  // for height fields
  bool HasPlane() const; // true if set is height field above a plane
  void SetPlane( const ON_Plane& );
  const ON_Plane& Plane();
  double Height(int);

  /*
  Returns:
    True if m_N.Count() == m_P.Count().
  */
  bool HasPointNormals() const;

  /*
  Returns:
    True if m_C.Count() == m_P.Count().
  */
  bool HasPointColors() const;


  /*
  Returns:
    Number of points that are hidden.
  */
  int HiddenPointCount() const;

  /*
  Description:
    Destroys the m_H[] array and sets m_hidden_count=0.
  */
  void DestroyHiddenPointArray();

  /*
  Returns:
    If the point cloud has some hidden points, then an array
    of length PointCount() is returned and the i-th
    element is true if the i-th vertex is hidden.
    If no ponts are hidden, NULL is returned.
  */
  const bool* HiddenPointArray() const;

  /*
  Description:
    Set the runtime hidden point flag.
  Parameters:
    point_index - [in] point vertex index
    bHidden - [in] true to hide vertex
  */
  void SetHiddenPointFlag( int point_index, bool bHidden );

  /*
  Description:
    Returns true if the point is hidden.  This is a runtime
    setting that is not saved in 3dm files.
  Parameters:
    point_index - [in]
  Returns:
    True if the point is hidden.
  */
  bool PointIsHidden( int point_index ) const;

  /////////////////////////////////////////////////////////////////
  // Implementation
  ON_3dPointArray m_P;

  /////////////////////////////////////////////////////////////////
  // Implementation - OPTIONAL point normal
  //    Either m_N[] has zero count or it has the same
  //    count as m_P[], in which case m_N[j] reports
  //    the color assigned to m_P[j].
  ON_SimpleArray<ON_3dVector> m_N;

  /////////////////////////////////////////////////////////////////
  // Implementation - OPTIONAL point color
  //    Either m_C[] has zero count or it has the same
  //    count as m_P[], in which case m_P[j] reports
  //    the color assigned to m_P[j].
  ON_SimpleArray<ON_Color> m_C;

  /////////////////////////////////////////////////////////////////
  // Implementation - RUNTIME point visibility - not saved in 3dm files.
  //    If m_H.Count() = m_P.Count(), then
  //    m_H[j] is true if the point m_P[j] 
  //    is hidden.  Otherwise, all points are visible.
  //    m_hidden_count = number of true values in the m_H[] array.
  ON_SimpleArray<bool> m_H;
  int m_hidden_count;

  ON_Plane m_plane;
  ON_BoundingBox m_bbox;
  unsigned int m_flags; // bit 1 is set if ordered
                        // bit 2 is set if plane is set

};

#endif
