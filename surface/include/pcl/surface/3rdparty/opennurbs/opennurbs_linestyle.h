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

#if !defined(OPENNURBS_LINESTYLE_INC_)
#define OPENNURBS_LINESTYLE_INC_


///////////////////////////////////////////////////////////////////////////////
//
// Class ON_DisplayMaterialRef
// 

/*
Description:
  Objects can have per viewport display properties
  that override a viewport's default display
  properties.  These overrides are stored on 
  ON_3dmObjectAttributes as a list of 
  ON_DisplayMaterialRefs.
  
Example:
  For example, by default a viewport
  might display objects using a wireframe, but
  one special object may need to be shaded.
  In this case the special object would have
  a display material ref with the "wireframe"
  viewport's id and the id of a display material
  that specified shading.
*/
class ON_CLASS ON_DisplayMaterialRef
{
public:
  /*
  Description:
    Default constructor sets both ids to nil.
  */
  ON_DisplayMaterialRef();
  int Compare(const ON_DisplayMaterialRef& other) const;
  bool operator==(const ON_DisplayMaterialRef& other) const;
  bool operator!=(const ON_DisplayMaterialRef& other) const;
  bool operator<(const ON_DisplayMaterialRef& other) const;
  bool operator<=(const ON_DisplayMaterialRef& other) const;
  bool operator>(const ON_DisplayMaterialRef& other) const;
  bool operator>=(const ON_DisplayMaterialRef& other) const;

  // C++ default destructor, copy constructor and operator=
  // work fine.

  ON_UUID m_viewport_id;          // identifies the ON_Viewport
                                  //   If nil, then the display material
                                  //   will be used in all viewports
                                  //   that are not explictly referenced
                                  //   in other ON_DisplayMaterialRefs.

  ON_UUID m_display_material_id;  // id used to find display attributes

  // For Rhino V4 the per detail visibility attribute is implemented
  // through a display material reference on an object. This is ONLY
  // for for detail viewports and only for V4. Keep this uuid around
  // so the per detail attributes in future versions of Rhino can be
  // implemented a different way.
  // {1403A7E4-E7AD-4a01-A2AA-41DAE6BE7ECB}
  static const ON_UUID m_invisible_in_detail_id;
};


//////////////////////////////////////////////////////////////////////
// class ON_LinetypeSegment

class ON_CLASS ON_LinetypeSegment
{
public:
  ON_LinetypeSegment();

  bool operator==( const ON_LinetypeSegment& src) const;
  bool operator!=( const ON_LinetypeSegment& src) const;

  // For a curve to be drawn starting at the start point
  // and ending at the endpoint, the first segment
  // in the pattern must be a stLine type
  enum eSegType
  {
    stLine,
    stSpace,
  };

  void Dump( class ON_TextLog& ) const;

  // do not add read/write functions to this class

  double m_length; // length in millimeters on printed output
  eSegType m_seg_type;
};

#endif
