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

#if !defined(OPENNURBS_ANNOTATION_INC_)
#define OPENNURBS_ANNOTATION_INC_


class ON_CLASS ON_AnnotationTextDot : public ON_Point
{
  // 3d annotation dot with text
  ON_OBJECT_DECLARE(ON_AnnotationTextDot);
public:
  ON_AnnotationTextDot();
  ~ON_AnnotationTextDot();
  ON_AnnotationTextDot(const ON_AnnotationTextDot&);
  ON_AnnotationTextDot& operator=(const ON_AnnotationTextDot&);

  /////////////////////////////////////////////////////////////////
  //
  // ON_Object overrides
  //

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
         ON_BinaryArchive&  // serialize definition to binary archive
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive&  // restore definition from binary archive
       );

  ON_wString m_text;
};

class ON_CLASS ON_AnnotationArrow : public ON_Geometry
{
  // 3d annotation arrow
  ON_OBJECT_DECLARE(ON_AnnotationArrow);
public:
  ON_AnnotationArrow();
  ~ON_AnnotationArrow();
  ON_AnnotationArrow(const ON_AnnotationArrow&);
  ON_AnnotationArrow& operator=(const ON_AnnotationArrow&);

  /////////////////////////////////////////////////////////////////
  //
  // ON_Object overrides
  //

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
         ON_BinaryArchive&  // serialize definition to binary archive
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive&  // restore definition from binary archive
       );

  ON::object_type ObjectType() const;

  /////////////////////////////////////////////////////////////////
  //
  // ON_Geometry overrides
  //

  int Dimension() const;

  // work horse bounding box getter
  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // boxmin[dim]
         double*,    // boxmax[dim]
         ON_BOOL32 = false  // true means grow box
         ) const;

  ON_BOOL32 Transform( 
         const ON_Xform&
         );

  /////////////////////////////////////////////////////////////////
  //
  // Interface
  //
  ON_3dVector Vector() const;
  ON_3dPoint Head() const;
  ON_3dPoint Tail() const;

  ON_3dPoint m_tail;
  ON_3dPoint m_head;
};

////////////////////////////////////////////////////////////////
//
//   ON_Annotation - used to serialize definitions of annotation
//                   objects (dimensions, text blocks, etc.).
//

class ON_CLASS ON_Annotation : public ON_Geometry
{
  ON_OBJECT_DECLARE(ON_Annotation);

  enum SYMBOLS
  {
    degreesym = 176,
    radiussym = 'R',
    diametersym = 216,
    plusminussym = 177,
  };



public:

  virtual ON_BOOL32 IsRealObject() const = 0;

  ON_Annotation();
  ON_Annotation(const ON_Annotation&);
  ~ON_Annotation();
  ON_Annotation& operator=(const ON_Annotation&);

  void Create();  // initialize class's fields assuming
                  // memory is uninitialized
  void Destroy();
  void EmergencyDestroy();

  /////////////////////////////////////////////////////////////////
  //
  // ON_Object overrides
  //

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
         ON_BinaryArchive&  // serialize definition to binary archive
       ) const;

  ON_BOOL32 Read(
         ON_BinaryArchive&  // restore definition from binary archive
       );

  ON::object_type ObjectType() const;

  /////////////////////////////////////////////////////////////////
  //
  // ON_Geometry overrides
  //

  int Dimension() const;

  ON_BOOL32 GetBBox( // returns true if successful
         double*,    // boxmin[dim]
         double*,    // boxmax[dim]
         ON_BOOL32 = false  // true means grow box
         ) const;

  ON_BOOL32 Transform( 
         const ON_Xform&
         );

  /////////////////////////////////////////////////////////////////
  //
  // ON_Annotation interface
  //

  // use these to get/set the current annotation settings
  static const ON_3dmAnnotationSettings& AnnotationSettings();
  static void SetAnnotationSettings( const ON_3dmAnnotationSettings* );

  bool IsText() const;
  bool IsLeader() const;
  bool IsDimension() const;

  virtual double NumericValue() const;
  virtual void SetTextToDefault();

  void SetType( ON::eAnnotationType type );
  ON::eAnnotationType Type() const;
  void SetTextDisplayMode( ON::eTextDisplayMode mode);
  ON::eTextDisplayMode TextDisplayMode() const;

  void SetPlane( const ON_Plane& plane );
  ON_Plane Plane() const;
  int PointCount() const;
  void SetPoints( const ON_SimpleArray<ON_2dPoint>& points );
  const ON_SimpleArray<ON_2dPoint>& Points() const;
  void SetPoint( int idx, ON_3dPoint point );
  ON_2dPoint Point( int idx ) const;
  void SetUserText( const wchar_t* string );
  const ON_wString& UserText() const;
  void SetDefaultText( const wchar_t* string );
  const ON_wString& DefaultText() const;
  void SetUserPositionedText( int bUserPositionedText );
  bool UserPositionedText() const;

  // to convert world 3d points to and from annotation 2d points
  bool GetECStoWCSXform( ON_Xform& xform ) const;
  bool GeWCStoECSXform( ON_Xform& xform ) const;

  ON::eAnnotationType m_type;          // enum for type of annotation
                                       // DimLinear, DimRadius, etc.

  ON::eTextDisplayMode m_textdisplaymode; // how the text is displayed
                                       // Horizontal, InLine, AboveLine

  ON_Plane m_plane;                    // ECS reference plane in WCS coordinates
  ON_SimpleArray<ON_2dPoint> m_points; // Definition points for the dimension

  ON_wString m_usertext;               // "<>", or user override
  ON_wString m_defaulttext;            // The displayed text string

  bool m_userpositionedtext;           // true: User has positioned text 
                                       // false: use default location
};


class ON_CLASS ON_LinearDimension : public ON_Annotation
{
  ON_OBJECT_DECLARE(ON_LinearDimension);

public:
  ON_BOOL32 IsRealObject() const;
  ON_LinearDimension();
  ON_LinearDimension(const ON_LinearDimension&);
  ~ON_LinearDimension();
  ON_LinearDimension& operator=(const ON_LinearDimension&);

  double NumericValue();
  void SetTextToDefault();
  void EmergencyDestroy();
};

class ON_CLASS ON_RadialDimension : public ON_Annotation
{
  ON_OBJECT_DECLARE(ON_RadialDimension);

public:
  ON_BOOL32 IsRealObject() const;
  ON_RadialDimension();
  ON_RadialDimension(const ON_RadialDimension&);
  ~ON_RadialDimension();
  ON_RadialDimension& operator=(const ON_RadialDimension&);

  double NumericValue();
  void SetTextToDefault();

  void EmergencyDestroy();
};

class ON_CLASS ON_AngularDimension : public ON_Annotation
{
  ON_OBJECT_DECLARE(ON_AngularDimension);

public:
  ON_BOOL32 IsRealObject() const;

  ON_AngularDimension();
  ON_AngularDimension(const ON_AngularDimension&);
  ~ON_AngularDimension();
  ON_AngularDimension& operator=(const ON_AngularDimension&);

  void EmergencyDestroy();

  ON_BOOL32 Write( ON_BinaryArchive& file ) const;
  ON_BOOL32 Read( ON_BinaryArchive& file );

  void SetAngle( double angle ) { m_angle = angle; }
  double Angle() const { return m_angle; }
  void SetRadius( double radius ) { m_radius = radius; }
  double Radius() const { return m_radius; }

  double NumericValue();
  void SetTextToDefault();


private:
  double m_angle;      // angle being dimensioned
  double m_radius;     // radius for dimension arc
};

class ON_CLASS ON_TextEntity : public ON_Annotation
{
  ON_OBJECT_DECLARE(ON_TextEntity);

public:
  ON_BOOL32 IsRealObject() const;
  ON_TextEntity();
  ON_TextEntity(const ON_TextEntity&);
  ~ON_TextEntity();
  ON_TextEntity& operator=(const ON_TextEntity&);

  void EmergencyDestroy();

  ON_BOOL32 Write( ON_BinaryArchive& file ) const;
  ON_BOOL32 Read( ON_BinaryArchive& file );

  void SetFaceName( ON_wString string ) { m_facename = string; }
  ON_wString FaceName() const { return m_facename; }
  void SetFontWeight( int weight ) { m_fontweight = weight; }
  int FontWeight() const { return m_fontweight; }
  void SetHeight( double height ) { m_height = height; }
  double Height() const { return m_height; }

private:
  ON_wString m_facename;
  int m_fontweight;  // windows - 400 = NORMAL )
  double m_height;   // gets multiplied by dimscale
};

class ON_CLASS ON_Leader : public ON_Annotation
{
  ON_OBJECT_DECLARE(ON_Leader);

public:
  ON_BOOL32 IsRealObject() const;
  ON_Leader();
  ON_Leader(const ON_Leader&);
  ~ON_Leader();
  ON_Leader& operator=(const ON_Leader&);

  void EmergencyDestroy();
};




#endif

