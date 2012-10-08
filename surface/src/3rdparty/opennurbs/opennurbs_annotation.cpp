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

ON_VIRTUAL_OBJECT_IMPLEMENT( ON_Annotation,       ON_Geometry,   "ABAF5873-4145-11d4-800F-0010830122F0" );
ON_OBJECT_IMPLEMENT( ON_LinearDimension,  ON_Annotation, "5DE6B20D-486B-11d4-8014-0010830122F0" );
ON_OBJECT_IMPLEMENT( ON_RadialDimension,  ON_Annotation, "5DE6B20E-486B-11d4-8014-0010830122F0" );
ON_OBJECT_IMPLEMENT( ON_AngularDimension, ON_Annotation, "5DE6B20F-486B-11d4-8014-0010830122F0" );
ON_OBJECT_IMPLEMENT( ON_TextEntity,       ON_Annotation, "5DE6B210-486B-11d4-8014-0010830122F0" );
ON_OBJECT_IMPLEMENT( ON_Leader,           ON_Annotation, "5DE6B211-486B-11d4-8014-0010830122F0" );

ON_BOOL32 ON_LinearDimension::IsRealObject() const {return true;}
ON_BOOL32 ON_RadialDimension::IsRealObject() const {return true;}
ON_BOOL32 ON_AngularDimension::IsRealObject() const {return true;}
ON_BOOL32 ON_TextEntity::IsRealObject() const {return true;}
ON_BOOL32 ON_Leader::IsRealObject() const {return true;}

#define REALLY_BIG_NUMBER 1e150

static const ON_3dmAnnotationSettings* sglb_asets = 0;

void ON_Annotation::SetAnnotationSettings( const ON_3dmAnnotationSettings* p )
{
  sglb_asets = p;
}

const ON_3dmAnnotationSettings& ON_Annotation::AnnotationSettings()
{
  static ON_3dmAnnotationSettings defaults;
  return sglb_asets ? *sglb_asets : defaults;
}

void ON_Annotation::Create()
{
  // TODO: initialize class members assuming any member that is not a class
  // is not initialized.
  m_type = ON::dtNothing;
  m_plane = ON_xy_plane;
  m_points.EmergencyDestroy();
  m_usertext.EmergencyDestroy();
  m_defaulttext.EmergencyDestroy();
  m_userpositionedtext = false;
}

void ON_Annotation::Destroy()
{
  m_points.Destroy();
  m_usertext.Destroy();
  m_defaulttext.Destroy();
  m_type = ON::dtNothing;
  m_plane = ON_xy_plane;
  m_userpositionedtext = false;
}

void ON_Annotation::EmergencyDestroy()
{
  m_points.EmergencyDestroy();
  m_usertext.EmergencyDestroy();
  m_defaulttext.EmergencyDestroy();
  m_type = ON::dtNothing;
  m_plane = ON_xy_plane;
  m_userpositionedtext = false;
}

ON_Annotation::ON_Annotation()
{
  Create();
}

ON_Annotation::ON_Annotation(const ON_Annotation& src)
{
  Create();
  *this = src;
}

ON_Annotation::~ON_Annotation()
{
  Destroy();
}


bool ON_Annotation::IsText() const { return Type() == ON::dtTextBlock; }
bool ON_Annotation::IsLeader() const { return Type() == ON::dtLeader; }
bool ON_Annotation::IsDimension() const { if( IsText() || IsLeader()) return false; return true; }

//virtual 
double ON_Annotation::NumericValue() const { return 0.0; }
//virtual 
void ON_Annotation::SetTextToDefault() { SetDefaultText( L""); }

void ON_Annotation::SetType( ON::eAnnotationType type ) { m_type = type; }
ON::eAnnotationType ON_Annotation::Type() const { return m_type; }
void ON_Annotation::SetTextDisplayMode( ON::eTextDisplayMode mode) { m_textdisplaymode = mode; }
ON::eTextDisplayMode ON_Annotation::TextDisplayMode() const { return m_textdisplaymode; }

void ON_Annotation::SetPlane( const ON_Plane& plane ) { m_plane = plane; }
ON_Plane ON_Annotation::Plane() const { return m_plane; }
int ON_Annotation::PointCount() const { return m_points.Count(); }
void ON_Annotation::SetPoints( const ON_SimpleArray<ON_2dPoint>& points ) { m_points = points; }
const ON_SimpleArray<ON_2dPoint>& ON_Annotation::Points() const { return m_points; }
void ON_Annotation::SetUserText( const wchar_t* string ) {m_usertext = string; }
const ON_wString& ON_Annotation::UserText() const { return m_usertext; }
void ON_Annotation::SetDefaultText( const wchar_t* string ) { m_defaulttext = string; }
const ON_wString& ON_Annotation::DefaultText() const { return m_defaulttext; }
void ON_Annotation::SetUserPositionedText( int bUserPositionedText ) { m_userpositionedtext = (bUserPositionedText?true:false); }
bool ON_Annotation::UserPositionedText() const { return m_userpositionedtext; }


ON_Annotation& ON_Annotation::operator=(const ON_Annotation& src)
{
  if ( this != &src ) {
    // get a clean and empty "this"
    Destroy();
    Create();
    ON_Geometry::operator=(src);

    // TODO: copy fields
    m_type = src.m_type;
    m_plane = src.m_plane;
    m_points = src.m_points;
    m_usertext = src.m_usertext;
    m_userpositionedtext = src.m_userpositionedtext;
  }
  return *this;
}

ON_BOOL32 ON_Annotation::IsValid( ON_TextLog* text_log ) const
{
  // TODO: quickly inspect object and return true/false
  bool rc = true;
  if ( ON::dtNothing == m_type  )
  {
    if ( 0 != text_log )
      text_log->Print("ON_Annotation has m_type = ON::dtNothing.\n");
    rc = false;
  }
  return rc;
}

void ON_Annotation::Dump( ON_TextLog& dump ) const
{
  // for debugging
  dump.Print("ON_Annotation: ....\n");
}

ON_BOOL32 ON_Annotation::Write( ON_BinaryArchive& file ) const
{
  int i;
  ON_BOOL32 rc = file.Write3dmChunkVersion( 1, 0 );
    // TODO: use 
    //    if (rc) rc = file.WritePoint(....);
    //    if (rc) rc = file.WriteString(....);
    //    if (rc) rc = file.WriteDouble(....);
    // to write object.
  i = m_type;
  if (rc) 
    rc = file.WriteInt( i );
  if (rc) 
    rc = file.WritePlane( m_plane );
  if (rc) 
    rc = file.WriteArray( m_points );
  if (rc) 
    rc = file.WriteString( m_usertext );
  if (rc) 
    rc = file.WriteString( m_defaulttext );
  if( rc )
    rc = file.WriteInt( m_userpositionedtext );
  return rc;
}

ON_BOOL32 ON_Annotation::Read( ON_BinaryArchive& file )
{
  Destroy();
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( rc && major_version == 1 ) 
  {
    int i;
    if (rc)
    {
      rc = file.ReadInt( &i );
      if (rc)
        m_type = ON::AnnotationType(i);
    }
    if (rc)
      rc = file.ReadPlane( m_plane );
    if (rc)
      rc = file.ReadArray( m_points );
    if (rc)
      rc = file.ReadString( m_usertext );
    if (rc)
      rc = file.ReadString( m_defaulttext );
    if( rc )
    {
      rc = file.ReadInt( &i );
      if (rc) m_userpositionedtext = i ? true : false;
    }
  }

  if( fabs( m_plane.origin.x) > REALLY_BIG_NUMBER || fabs( m_plane.origin.y) > REALLY_BIG_NUMBER || fabs( m_plane.origin.z) > REALLY_BIG_NUMBER)
    return false;

  for( int i = 0; i < m_points.Count(); i++)
  {
    if( fabs( m_points[i].x) > REALLY_BIG_NUMBER || fabs( m_points[i].y) > REALLY_BIG_NUMBER)
      return false;
  }


  return rc;
}

ON::object_type ON_Annotation::ObjectType() const
{
  return ON::annotation_object;
}



int ON_Annotation::Dimension() const
{
  return 3; 
}

ON_BOOL32 ON_Annotation::GetBBox( // returns true if successful
       double* boxmin,
       double* boxmax,
       ON_BOOL32 bGrowBox // default = false
       ) const
{
  // TODO:
  //   If the class is not valid, return false.
  //
  //   If the class is valid and bGrowBox is false, 
  //   return the 3d bounding box of the annotation.
  //
  //   If the class is valid and bGrowBox is true, 
  //   return the union of the input box and the 3d bounding 
  //   box of the annotation.
  if( !bGrowBox )
  {
    boxmin[0] = boxmin[1] = boxmin[2] =  1e300;
    boxmax[0] = boxmax[1] = boxmax[2] = -1e300;
  }

  ON_3dPoint wpt;
  ON_Xform xform;
  GetECStoWCSXform( xform );
  for( int i = 0; i < m_points.Count(); i++ )
  {
    wpt = m_points[i];
    
    if( wpt.y < boxmin[1] )
      boxmin[1] = wpt.y;
    if( wpt.z < boxmin[2] )
      boxmin[2] = wpt.z;
    if( wpt.x > boxmax[0] )
      boxmax[0] = wpt.x;
    if( wpt.y > boxmax[1] )
      boxmax[1] = wpt.y;
    if( wpt.z > boxmax[2] )
      boxmax[2] = wpt.z;
  }
  return true;
}

ON_BOOL32 ON_Annotation::Transform( const ON_Xform& xform )
{
  // TODO: Return false if class is invalid or xform cannot be applied.
  //       Otherwise, apply xform to geometry and return true.
  TransformUserData(xform);
  return m_plane.Transform( xform );
}

// Converts 2d points in annotation to 3d WCS points 
bool ON_Annotation::GetECStoWCSXform( ON_Xform& xform ) const
{
  ON_3dVector z = ON_CrossProduct( m_plane.xaxis, m_plane.yaxis );
  return xform.ChangeBasis( m_plane.origin, m_plane.xaxis, m_plane.yaxis, z, 
                            ON_origin, ON_xaxis, ON_yaxis, ON_zaxis );
}

// Converts from WCS 3d points to 2d points in annotation
bool ON_Annotation::GeWCStoECSXform( ON_Xform& xform ) const
{
  ON_3dVector z = ON_CrossProduct( m_plane.xaxis, m_plane.yaxis );
  return xform.ChangeBasis( ON_origin, ON_xaxis, ON_yaxis, ON_zaxis,
                            m_plane.origin, m_plane.xaxis, m_plane.yaxis, z );
}

void ON_Annotation::SetPoint( int idx, ON_3dPoint point )
{
  if( idx >= 0 && idx < m_points.Count() )
    m_points[idx] = point;
}
  
ON_2dPoint ON_Annotation::Point( int idx ) const
{
  if( idx >= 0 && idx < m_points.Count() )
    return m_points[idx];

  return ON_2dPoint( 0.0, 0.0 );
}



//----- ON_LinearDimension ------------------------------------------
ON_LinearDimension::ON_LinearDimension()
{
}

ON_LinearDimension::ON_LinearDimension(const ON_LinearDimension& src) : ON_Annotation(src)
{
}

ON_LinearDimension::~ON_LinearDimension()
{
}

ON_LinearDimension& ON_LinearDimension::operator=(const ON_LinearDimension& src)
{
  if ( this != &src ) {
    ON_Annotation::operator=(src);
  }
  return *this;
}

void ON_LinearDimension::EmergencyDestroy()
{
  ON_Annotation::EmergencyDestroy();
}

double ON_LinearDimension::NumericValue()
{
  return (Point( 1) - Point( 3)).Length();
}
void ON_LinearDimension::SetTextToDefault() 
{ 
  SetUserText( L"<>"); 
}


//----- ON_RadialDimension ------------------------------------------
ON_RadialDimension::ON_RadialDimension()
{
}

ON_RadialDimension::ON_RadialDimension(const ON_RadialDimension& src) : ON_Annotation(src)
{
}

ON_RadialDimension::~ON_RadialDimension()
{
}

ON_RadialDimension& ON_RadialDimension::operator=(const ON_RadialDimension& src)
{
  if ( this != &src ) {
    ON_Annotation::operator=(src);
  }
  return *this;
}

void ON_RadialDimension::EmergencyDestroy()
{
  ON_Annotation::EmergencyDestroy();
}

double ON_RadialDimension::NumericValue()
{
  double d = (Point( 0) - Point( 1)).Length();
  if( Type() == ON::dtDimDiameter)
    d *= 2.0;
  return d;
}

void ON_RadialDimension::SetTextToDefault() 
{ 
  ON_wString s; 
  if( Type() == ON::dtDimDiameter) 
    s.Format( L"%c<>", ON_Annotation::diametersym); 
  else 
    s.Format( L"%c<>", ON_Annotation::radiussym); 
  SetUserText( s); 
}

//----- ON_AngularDimension -----------------------------------------
ON_AngularDimension::ON_AngularDimension() : m_angle(0.0), m_radius(0.0)
{
}

ON_AngularDimension::ON_AngularDimension(const ON_AngularDimension& src) : ON_Annotation(src)
{
  m_angle = src.m_angle;
  m_radius = src.m_radius;
}

ON_AngularDimension::~ON_AngularDimension()
{
}

ON_AngularDimension& ON_AngularDimension::operator=(const ON_AngularDimension& src)
{
  if ( this != &src ) {
    ON_Annotation::operator=(src);
    m_angle = src.m_angle;
    m_radius = src.m_radius;
  }
  return *this;
}

void ON_AngularDimension::EmergencyDestroy()
{
  ON_Annotation::EmergencyDestroy();
}

ON_BOOL32 ON_AngularDimension::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = ON_Annotation::Write( file );
  if( rc )
    rc = file.WriteDouble( m_angle );
  if( rc )
    rc = file.WriteDouble( m_radius );
  return rc;
}

ON_BOOL32 ON_AngularDimension::Read( ON_BinaryArchive& file )
{
  ON_BOOL32 rc = ON_Annotation::Read( file );
  if( rc )
    rc = file.ReadDouble( &m_angle );
  if( rc )
    rc = file.ReadDouble( &m_radius );

  if( m_angle <= 0.0 || m_angle > REALLY_BIG_NUMBER)
    return false;

  if( m_radius <= 0.0 || m_radius > REALLY_BIG_NUMBER)
    return false;

  return rc;
}

double ON_AngularDimension::NumericValue()
{
  return Angle() * 180.0 / ON_PI;
}

void ON_AngularDimension::SetTextToDefault() 
{ 
  ON_wString s; 
  s.Format( L"<>%c", ON_Annotation::degreesym); 
  SetUserText( s); 
}

//----- ON_TextEntity -----------------------------------------------
ON_TextEntity::ON_TextEntity() : m_fontweight(400), m_height(20.0)
{
}

ON_TextEntity::ON_TextEntity(const ON_TextEntity& src) : ON_Annotation(src)
{
  m_facename = src.m_facename;
  m_fontweight = src.m_fontweight;
  m_height = src.m_height;
}

ON_TextEntity::~ON_TextEntity()
{
  m_facename.Destroy();
}

ON_TextEntity& ON_TextEntity::operator=(const ON_TextEntity& src)
{
  if ( this != &src ) {
    m_facename = src.m_facename;
    m_fontweight = src.m_fontweight;
    m_height = src.m_height;
    ON_Annotation::operator=(src);
  }
  return *this;
}

void ON_TextEntity::EmergencyDestroy()
{
  ON_Annotation::EmergencyDestroy();
  m_facename.EmergencyDestroy();
}

ON_BOOL32 ON_TextEntity::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = ON_Annotation::Write( file );
  if( rc )
    rc = file.WriteString( m_facename );
  if( rc )
    rc = file.WriteInt( m_fontweight );
  if( rc )
    rc = file.WriteDouble( m_height );
  return rc;
}

ON_BOOL32 ON_TextEntity::Read( ON_BinaryArchive& file )
{
  ON_BOOL32 rc = ON_Annotation::Read( file );
  if( rc )
    rc = file.ReadString( m_facename );
  if( rc )
    rc = file.ReadInt( &m_fontweight );
  if( rc )
    rc = file.ReadDouble( &m_height );

  if( fabs( m_height) > REALLY_BIG_NUMBER)
    return false;


  return rc;
}

//----- ON_Leader ------------------------------------------
ON_Leader::ON_Leader()
{
}

ON_Leader::ON_Leader(const ON_Leader& src) : ON_Annotation(src)
{
}

ON_Leader::~ON_Leader()
{
}

ON_Leader& ON_Leader::operator=(const ON_Leader& src)
{
  if ( this != &src ) {
    ON_Annotation::operator=(src);
  }
  return *this;
}

void ON_Leader::EmergencyDestroy()
{
  ON_Annotation::EmergencyDestroy();
}


ON_OBJECT_IMPLEMENT(ON_AnnotationTextDot,ON_Point,"8BD94E19-59E1-11d4-8018-0010830122F0");

ON_AnnotationTextDot::ON_AnnotationTextDot()
{}

ON_AnnotationTextDot::~ON_AnnotationTextDot()
{
  m_text.Destroy();
}

ON_AnnotationTextDot::ON_AnnotationTextDot(const ON_AnnotationTextDot& src) : ON_Point(src), m_text(src.m_text)
{}

ON_AnnotationTextDot& ON_AnnotationTextDot::operator=(const ON_AnnotationTextDot& src)
{
  if ( this != &src ) {
    ON_Point::operator=(src);
    m_text = src.m_text;
  }
  return *this;
}

ON_BOOL32 ON_AnnotationTextDot::IsValid( ON_TextLog* text_log ) const
{
  bool rc = true;
  if ( m_text.IsEmpty() )
  {
    if ( 0 != text_log )
      text_log->Print("ON_AnnotationTextDot.m_text is empty\n");
    rc = false;
  }
  return rc;
}

void ON_AnnotationTextDot::Dump( ON_TextLog& log ) const
{
  log.Print("ON_AnnotationTextDot \"%ls\" at ",m_text.Array());
  log.Print(point);
  log.Print("\n");
}

ON_BOOL32 ON_AnnotationTextDot::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,0);
  if (rc) rc = file.WritePoint( point );
  if (rc) rc = file.WriteString( m_text );
  return rc;
}

ON_BOOL32 ON_AnnotationTextDot::Read( ON_BinaryArchive& file )
{
  m_text.Destroy();
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( major_version == 1 ) {
    if (rc) rc = file.ReadPoint( point );
    if (rc) rc = file.ReadString( m_text );
  }
  else {
    rc = false;
  }
  return rc;
}

ON_OBJECT_IMPLEMENT(ON_AnnotationArrow,ON_Geometry,"8BD94E1A-59E1-11d4-8018-0010830122F0");

ON_AnnotationArrow::ON_AnnotationArrow() : m_tail(0.0,0.0,0.0), m_head(0.0,0.0,0.0)
{}

ON_AnnotationArrow::~ON_AnnotationArrow()
{}

ON_AnnotationArrow::ON_AnnotationArrow(const ON_AnnotationArrow& src) : ON_Geometry(src), m_tail(src.m_tail), m_head(src.m_head)
{}

ON_AnnotationArrow& ON_AnnotationArrow::operator=(const ON_AnnotationArrow& src)
{
  if ( this != &src ) {
    ON_Geometry::operator=(src);
    m_tail = src.m_tail;
    m_head = src.m_head;
  }
  return *this;
}

ON_BOOL32 ON_AnnotationArrow::IsValid( ON_TextLog* text_log ) const
{
  bool rc = true;
  if (m_tail == m_head)
  {
    if ( 0 != text_log )
      text_log->Print("ON_AnnotationArrow has m_head=m_tail.\n");
    rc = false;
  }
  return rc;
}

void ON_AnnotationArrow::Dump( ON_TextLog& log ) const
{
  log.Print("ON_AnnotationArrow: ");
  log.Print(m_tail);
  log.Print(" to ");
  log.Print(m_head);
  log.Print("\n");
}

ON_BOOL32 ON_AnnotationArrow::Write( ON_BinaryArchive& file ) const
{
  ON_BOOL32 rc = file.Write3dmChunkVersion(1,0);
  if (rc) rc = file.WritePoint( m_tail );
  if (rc) rc = file.WritePoint( m_head );
  return rc;
}

ON_BOOL32 ON_AnnotationArrow::Read(ON_BinaryArchive& file)
{
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if ( major_version == 1 ) {
    if (rc) rc = file.ReadPoint( m_tail );
    if (rc) rc = file.ReadPoint( m_head );

  }
  else {
    rc = false;
  }
  return rc;
}

ON::object_type ON_AnnotationArrow::ObjectType() const
{
  return ON::annotation_object;
}

int ON_AnnotationArrow::Dimension() const
{
  return 3;
}

ON_BOOL32 ON_AnnotationArrow::GetBBox( double* boxmin, double* boxmax, ON_BOOL32 bGrowBox ) const
{
  ON_BOOL32 rc = ON_GetPointListBoundingBox( 3, false, 1, 3, m_tail, boxmin, boxmax, bGrowBox?true:false );
  if (rc)
    rc = ON_GetPointListBoundingBox( 3, false, 1, 3, m_head, boxmin, boxmax, true );
  return rc;
}

ON_BOOL32 ON_AnnotationArrow::Transform( const ON_Xform& xform )
{
  TransformUserData(xform);
  m_tail = xform*m_tail;
  m_head = xform*m_head;
  return true;
}

ON_3dVector ON_AnnotationArrow::Vector() const
{
  return (m_head-m_tail);
}

ON_3dPoint ON_AnnotationArrow::Head() const
{
  return m_head;
}

ON_3dPoint ON_AnnotationArrow::Tail() const
{
  return m_tail;
}

