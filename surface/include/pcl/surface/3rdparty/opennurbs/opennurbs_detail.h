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

#if !defined(ON_DETAIL_OBJECTY_INC_)
#define ON_DETAIL_OBJECTY_INC_

class ON_CLASS ON_DetailView : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_DetailView);

public:
  ON_DetailView();
  ~ON_DetailView();

  // C++ defaults for copy constructor and
  // operator= work fine.

  //////////////////////////////////////////////////////
  //
  // virtual ON_Object overrides
  //
  void MemoryRelocate();

  ON_BOOL32 IsValid( ON_TextLog* text_log = NULL ) const;

  void Dump( ON_TextLog& ) const;

  unsigned int SizeOf() const;

  ON_BOOL32 Write(
         ON_BinaryArchive& binary_archive
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive& binary_archive
       );

  ON::object_type ObjectType() const; // returns ON::detail_object

  //////////////////////////////////////////////////////
  //
  // virtual ON_Geometry overrides
  // The m_boundary determines all bounding boxes 
  //
  int Dimension() const;

  ON_BOOL32 GetBBox(
         double* boxmin,
         double* boxmax,
         int bGrowBox = false
         ) const;

	bool GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox = false,
			const ON_Xform* xform = 0
      ) const;

  ON_BOOL32 Transform( const ON_Xform& xform );

  // m_page_per_model_ratio is the ratio of page length / model length
  // where both lengths are in the same unit system
  // (ex. 1/4" on page = 1' in model = 0.25/12 = 0.02083)
  // (    1mm on page  = 1m in model = 1/1000  = 0.001)
  // If m_page_per_model_ratio > 0.0, then the detail
  // is drawn using the specified scale.
  double m_page_per_model_ratio;

  // A view with ON_3dmView::m_view_type = ON::nested_view_type
  // This field is used for IO purposes only.  Runtime detail
  // view projection information is on CRhDetailViewObject.
  ON_3dmView m_view;

  // 2d curve in page layout coordinates in mm
  // (0,0) = lower left corner of page
  ON_NurbsCurve m_boundary;
};



#endif

