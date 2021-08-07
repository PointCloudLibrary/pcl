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

//March 23, 2008 - LW
//Adding ON_HatchExtra class to support movable base point for hatches
//This should be combined with the ON_Hatch class next time that is possible
// Don't put this extension class in a header file or export it.

class ON_HatchExtra : public ON_UserData
{
  ON_OBJECT_DECLARE(ON_HatchExtra);
public:
  static ON_HatchExtra* HatchExtension(ON_Hatch* pHatch, bool bCreate);
  static const ON_HatchExtra* HatchExtension(const ON_Hatch* pHatch, bool bCreate);

  ON_HatchExtra();
  ~ON_HatchExtra();

  void SetDefaults();

  // override virtual ON_Object::Dump function
  void Dump( ON_TextLog& text_log ) const;

  // override virtual ON_Object::SizeOf function
  unsigned int SizeOf() const;

  // override virtual ON_Object::Write function
  ON_BOOL32 Write(ON_BinaryArchive& binary_archive) const;

  // override virtual ON_Object::Read function
  ON_BOOL32 Read(ON_BinaryArchive& binary_archive);

  // override virtual ON_UserData::GetDescription function
  ON_BOOL32 GetDescription( ON_wString& description );

  // override virtual ON_UserData::Archive function
  ON_BOOL32 Archive() const; 

  // Get and set a 2d point in the hatch's ECS coordinates
  void SetBasePoint(ON_2dPoint& basepoint);
  ON_2dPoint BasePoint() const;

  ON_UUID    m_parent_hatch; // Hatch this extends or ON_nil_uuid
  ON_2dPoint m_basepoint;    // Base point in hatch's ECS
  
};

ON_OBJECT_IMPLEMENT(ON_HatchExtra,ON_UserData,"3FF7007C-3D04-463f-84E3-132ACEB91062");

ON_HatchExtra* ON_HatchExtra::HatchExtension(ON_Hatch* pHatch, bool bCreate)
{
  ON_HatchExtra* pExtra = 0;
  if(pHatch)
  {
    pExtra = ON_HatchExtra::Cast(pHatch->GetUserData(ON_HatchExtra::m_ON_HatchExtra_class_id.Uuid()));
    if(pExtra == 0 && bCreate)
    {
      pExtra = new ON_HatchExtra;
      if(pExtra)
      {
        if(!pHatch->AttachUserData(pExtra))
        {
          delete pExtra;
          pExtra = 0;
        }
      }  
    }
  }
  return pExtra;
}

const ON_HatchExtra* ON_HatchExtra::HatchExtension(const ON_Hatch* pHatch, bool bCreate)
{
  return HatchExtension((ON_Hatch*)pHatch, bCreate);
}

ON_HatchExtra::ON_HatchExtra()
{
  m_userdata_uuid = ON_HatchExtra::m_ON_HatchExtra_class_id.Uuid();
  m_application_uuid = ON_opennurbs5_id; // opennurbs.dll reads/writes this userdata
                                         // The id must be the version 5 id because
                                         // V6 SaveAs V5 needs to work, but SaveAs
                                         // V4 should not write this userdata.   
  m_userdata_copycount = 1;
  SetDefaults();
}

ON_HatchExtra::~ON_HatchExtra()
{
}

void ON_HatchExtra::SetDefaults()
{
  m_parent_hatch = ON_nil_uuid;
  m_basepoint.Set(0.0,0.0);
}

void ON_HatchExtra::Dump(ON_TextLog&) const
{
}

unsigned int ON_HatchExtra::SizeOf() const
{
  unsigned int sz = ON_UserData::SizeOf();
  sz += sizeof(*this)-sizeof(ON_UserData);
  return sz;
}

ON_BOOL32 ON_HatchExtra::Write(ON_BinaryArchive& archive) const
{
  bool rc = archive.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,0);

  if(rc) rc = archive.WriteUuid( m_parent_hatch);
  if(rc) rc = archive.WritePoint(m_basepoint);

  if(!archive.EndWrite3dmChunk())
    rc = false;

  return rc;
}

ON_BOOL32 ON_HatchExtra::Read(ON_BinaryArchive& archive)
{
  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk(TCODE_ANONYMOUS_CHUNK,&major_version,&minor_version);

  if(major_version != 1)
      rc = false;

  m_basepoint.Set(0.0,0.0);
  if(rc) rc = archive.ReadUuid(m_parent_hatch);
  if(rc) rc = archive.ReadPoint(m_basepoint);

  if(!archive.EndRead3dmChunk())
    rc = false;

  return rc;
}

ON_BOOL32 ON_HatchExtra::GetDescription( ON_wString& description)
{
  description.Format( "Userdata extension of ON_Hatch (contains basepoint)");
  return true;
}

ON_BOOL32 ON_HatchExtra::Archive() const
{
  return true;
}

void ON_HatchExtra::SetBasePoint(ON_2dPoint& point)
{
  if(point.IsValid())
    m_basepoint = point;
}

ON_2dPoint ON_HatchExtra::BasePoint() const
{
  return m_basepoint;
}

/////////////////////////////////////////////////////////////////
//  class ON_HatchLine
/////////////////////////////////////////////////////////////////

ON_HatchLine::ON_HatchLine()
: m_angle( 0.0), m_base( 0.0,0.0), m_offset( 0.0, 1.0)
{
}

ON_HatchLine::ON_HatchLine(double angle, 
                           const ON_2dPoint& base, 
                           const ON_2dVector& offset,
                           const ON_SimpleArray<double> dashes)
: m_angle( angle), m_base( base), m_offset( offset), m_dashes( dashes)
{
}

bool ON_HatchLine::operator==(const ON_HatchLine& src) const
{
  return( m_angle == src.m_angle && 
          m_base == src.m_base &&
          m_offset == src.m_offset && 
          m_dashes == src.m_dashes);
}

bool ON_HatchLine::operator!=(const ON_HatchLine& src) const
{
  return !operator==( src);
}

ON_BOOL32 ON_HatchLine::IsValid( ON_TextLog* text_log) const
{
  bool rc = m_angle >= 0.0;
  if( !rc)
  {
    if( text_log)
      text_log->Print( "Angle ( %lf) must be >= 0.0\n", m_angle);
    return false;
  }
  rc = m_angle < ON_PI * 2.0;
  if( !rc)
  {
    if( text_log)
      text_log->Print( "Angle ( %lf) must be < 2*Pi.\n", m_angle);
    return false;
  }
  rc = m_base != ON_2dPoint( ON_UNSET_VALUE, ON_UNSET_VALUE);
  if( !rc)
  {
    if( text_log)
      text_log->Print( "Base is not a valid point.\n");
    return false;
  }
  rc = m_offset.x != ON_UNSET_VALUE;
  if( !rc)
  {
    if( text_log)
      text_log->Print( "Offset is not a valid vector.\n");
    return false;
  }
  rc = m_offset.y > ON_SQRT_EPSILON;
  if( !rc)
  {
    if( text_log)
      text_log->Print( "Offset.y ( %lf) must be > 0.0", m_offset.y);
    return false;
  }
  return true;
}

void ON_HatchLine::Dump( ON_TextLog& dump) const
{
  dump.Print( "ON_HatchLine: angle = %lf radians ( %lf degrees) ", 
    Angle(), ON_RADIANS_TO_DEGREES * Angle());
  dump.Print( " base = ");
  dump.Print( m_base);
  dump.Print( " offset = ");
  dump.Print( m_offset);
  int count = m_dashes.Count();
  dump.Print( "\nDash count = %d: ", count);
  for( int i = 0; i < count; i++)
  {
    dump.Print( "%lf", Dash( i));
    if( i < count-1)
      dump.Print( ", ");
  }
  dump.Print( "\n");
}

ON_BOOL32 ON_HatchLine::Write( ON_BinaryArchive& ar) const
{
  ON_BOOL32 rc = ar.Write3dmChunkVersion(1,1);

  if (rc) rc = ar.WriteDouble( m_angle);
  if (rc) rc = ar.WritePoint( m_base);
  if (rc) rc = ar.WriteVector( m_offset);
  if (rc) rc = ar.WriteArray( m_dashes);

  return rc;
}

ON_BOOL32 ON_HatchLine::Read( ON_BinaryArchive& ar)
{
  m_angle = 0.0;
  m_base.Set( 0.0, 0.0);
  m_offset.Set( 0.0, 1.0);
  m_dashes.Empty();
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = ar.Read3dmChunkVersion( &major_version, &minor_version);
  if ( major_version == 1 ) 
  {
    if ( rc) rc = ar.ReadDouble( &m_angle);
    if ( rc) rc = ar.ReadPoint( m_base);
    if ( rc) rc = ar.ReadVector( m_offset);
    if ( rc) rc = ar.ReadArray( m_dashes);
  }
  return rc;
}

// ON_HatchLine Interface
double ON_HatchLine::Angle() const
{
  return m_angle;
}

void ON_HatchLine::SetAngle( double angle)
{
  m_angle = angle;
  double twopi = ON_PI * 2.0;

  // clamp between [0  2pi)
  while( m_angle < 0.0)
    m_angle += twopi;
  while( m_angle > twopi)
    m_angle -= twopi;
}

ON_2dPoint ON_HatchLine::Base() const
{
  return m_base;
}

void ON_HatchLine::SetBase( const ON_2dPoint& base)
{
  m_base = base;
}

ON_2dVector ON_HatchLine::Offset() const
{
  return m_offset;
}

void ON_HatchLine::SetOffset( const ON_2dVector& offset)
{
  m_offset = offset;
}

int ON_HatchLine::DashCount() const
{
  return m_dashes.Count();
}

double ON_HatchLine::Dash( int index) const
{
  if( index >= 0 && index < m_dashes.Count())
    return m_dashes[index];
  return 0.0;
}

void ON_HatchLine::AppendDash( double dash)
{
//  if( fabs( dash) > ON_SQRT_EPSILON)
    m_dashes.Append( dash);
}

void ON_HatchLine::SetPattern( const ON_SimpleArray<double>& dashes)
{
  m_dashes = dashes;
}

void ON_HatchLine::GetLineData( double& angle, 
                                ON_2dPoint& base, 
                                ON_2dVector& offset, 
                                ON_SimpleArray<double>& dashes) const
{
  angle = m_angle;
  base = m_base;
  offset = m_offset;  dashes = m_dashes;
}

double ON_HatchLine::GetPatternLength() const
{
  int i;
  double length = 0.0;
  for( i = 0; i < m_dashes.Count(); i++)
    length += fabs( m_dashes[i]);

  return length;
}


//  class ON_HatchPattern
/////////////////////////////////////////////////////////////////
ON_OBJECT_IMPLEMENT( ON_HatchPattern, ON_Object, "064E7C91-35F6-4734-A446-79FF7CD659E1" );

ON_HatchPattern::ON_HatchPattern()
: m_hatchpattern_index(-1)
, m_hatchpattern_id(ON_nil_uuid)
, m_type(ON_HatchPattern::ftSolid)
{
}

ON_HatchPattern::~ON_HatchPattern()
{
}

ON_BOOL32 ON_HatchPattern::IsValid( ON_TextLog* text_log) const
{
  eFillType type = FillType();
  ON_BOOL32 rc = true;
  if( type != ftSolid && type != ftLines && type != ftGradient)
  {
    if( text_log)
      text_log->Print( "Type field not set correctly.\n");
    rc = false;
  }
  if( type == ftLines)
  {
    int count = m_lines.Count();
    if( count < 1)
    {
      if( text_log)
        text_log->Print( "Line type patetern with no lines.\n");
      return false;
    }
    for( int i = 0; i < count; i++)
    {
      if( !m_lines[i].IsValid())
      {
        if( text_log)
          text_log->Print( "Line[%d] is not valid.\n", i);
        return false;
      }
    }
    return true;
  }
  return rc;
}

void ON_HatchPattern::Dump( ON_TextLog& dump) const
{
  dump.Print( "Hatch pattern - ");
  switch( m_type)
  {
  case ftSolid:
    dump.Print( "fill type: Solid");
    break;
  case ftLines:
    dump.Print( "fill type: Lines");
    break;
  case ftGradient:
    dump.Print( "fill type: Gradient");
    break;
  case ftLast:
    // no action, but this keeps gcc happy
    break;
  }
  dump.Print( "\n");

  const wchar_t* wsHatchPatternName = m_hatchpattern_name;
  if ( 0 == wsHatchPatternName )
    wsHatchPatternName = L"";
  dump.Print( "Name: %ls\n", wsHatchPatternName);

  const wchar_t* wsDescription =  m_description;
  if ( 0 == wsDescription )
    wsDescription = L"";
  dump.Print( "Description: %ls\n", wsDescription);

  if( m_type == ftLines)
  {
    int count = m_lines.Count();
    dump.Print( "Line count = %d\n", count);
    for( int i = 0; i < count; i++)
    {
      m_lines[i].Dump( dump);
    }
    dump.Print( "\n");
  }
}

ON_BOOL32 ON_HatchPattern::Write( ON_BinaryArchive& ar) const
{
  ON_BOOL32 rc = ar.Write3dmChunkVersion(1,2);

  if (rc) rc = ar.WriteInt( m_hatchpattern_index);
  if (rc) rc = ar.WriteInt( m_type);
  if (rc) rc = ar.WriteString( m_hatchpattern_name);
  if (rc) rc = ar.WriteString( m_description);
  if( rc)
  {
    if( m_type == ftLines)
    {
      int i, count = m_lines.Count();
      if ( count < 0 )
        count = 0;
      rc = ar.WriteInt( count );
      for( i = 0; i < count && rc; i++)
        rc = m_lines[i].Write( ar);
    }
  }
  // version 1.2 field
  if (rc) rc = ar.WriteUuid(m_hatchpattern_id);

  return rc;
}

ON_BOOL32 ON_HatchPattern::Read( ON_BinaryArchive& ar)
{
  m_hatchpattern_index = -1;
  memset(&m_hatchpattern_id,0,sizeof(m_hatchpattern_id));
  m_type = ftSolid;
  m_hatchpattern_name.Empty();
  m_description.Empty();
  m_lines.Empty();
  int i;

  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = ar.Read3dmChunkVersion( &major_version, &minor_version);
  if ( major_version == 1 ) 
  {
    if( rc) rc = ar.ReadInt( &m_hatchpattern_index);
    i = 0;
    if( rc) rc = ar.ReadInt( &i);
    if( rc) 
    {
      switch( i)
      {
      case 0:  m_type = ftSolid;    break;
      case 1:  m_type = ftLines;    break;
      case 2:  m_type = ftGradient; break;
      default: rc = false;          break;
      }
    }
    if( rc) rc = ar.ReadString( m_hatchpattern_name);
    if( rc) rc = ar.ReadString( m_description);
    if( rc)
    {
      if( m_type == ftLines)
      {
        m_lines.Empty();
        int count = 0;
        rc = ar.ReadInt( &count);
        if( rc && count > 0 ) 
        {
          m_lines.SetCapacity( count);
          int i;
          for( i = 0; rc && i < count; i++)
          {
            ON_HatchLine& line = m_lines.AppendNew();
            rc = line.Read( ar);
          }
        }
      }
    }
    if ( minor_version >= 2 )
    {
      rc = ar.ReadUuid(m_hatchpattern_id);
    }
  }
  return rc;
}

ON_HatchPattern::eFillType ON_HatchPattern::FillType() const
{
  if( m_type >= ftSolid && m_type < ftLast)
    return m_type;

  return ftLast;
}

void ON_HatchPattern::SetFillType( eFillType type)
{
  m_type = type;
}

void ON_HatchPattern::SetName( const wchar_t* pName)
{
  m_hatchpattern_name = pName;
  m_hatchpattern_name.TrimLeftAndRight();
}

void ON_HatchPattern::SetName( const char* pName)
{
  m_hatchpattern_name = pName;
  m_hatchpattern_name.TrimLeftAndRight();
}

void ON_HatchPattern::GetName( ON_wString& string) const
{
  string = m_hatchpattern_name;
}

const wchar_t* ON_HatchPattern::Name() const
{
  return m_hatchpattern_name;
}


void ON_HatchPattern::SetDescription( const wchar_t* pDescription)
{
  m_description = pDescription;
}

void ON_HatchPattern::SetDescription( const char* pDescription)
{
  m_description = pDescription;
}

void ON_HatchPattern::GetDescription( ON_wString& string) const
{
  string = m_description;
}

const wchar_t* ON_HatchPattern::Description() const
{
  return m_description;
}


void ON_HatchPattern::SetIndex( int i)
{
  m_hatchpattern_index = i;
}

int ON_HatchPattern::Index() const
{
  return m_hatchpattern_index;
}


//  Line HatchPattern functions

int ON_HatchPattern::HatchLineCount() const
{
  return m_lines.Count();
}

int ON_HatchPattern::AddHatchLine( const ON_HatchLine& line)
{
  m_lines.Append( line);
  return m_lines.Count()-1;
}

const ON_HatchLine* ON_HatchPattern::HatchLine( int index) const
{
  if( index >= 0 && index < m_lines.Count())
    return &m_lines[index];
  else
    return NULL;
}

bool ON_HatchPattern::RemoveHatchLine( int index)
{
  if( index >= 0 && index < m_lines.Count())
  {
    m_lines.Remove( index);
    return true;
  }
  return false;
}

void ON_HatchPattern::RemoveAllHatchLines()
{
  m_lines.Empty();
}

int ON_HatchPattern::SetHatchLines( const ON_ClassArray<ON_HatchLine> lines)
{
  m_lines = lines;
  return m_lines.Count();
}




//  class ON_HatchLoop
/////////////////////////////////////////////////////////////////

#if defined(ON_DLL_EXPORTS)

// When the Microsoft CRT(s) is/are used, this is the best
// way to prevent crashes that happen when a hatch loop is
// allocated with new in one DLL and deallocated with
// delete in another DLL.

void* ON_HatchLoop::operator new(std::size_t sz)
{
  // ON_HatchLoop new
  return onmalloc(sz);
}

void ON_HatchLoop::operator delete(void* p)
{
  // ON_HatchLoop delete
  onfree(p);
}

void* ON_HatchLoop::operator new[] (std::size_t sz)
{
  // ON_HatchLoop array new
  return onmalloc(sz);
}

void ON_HatchLoop::operator delete[] (void* p)
{
  // ON_HatchLoop array delete
  onfree(p);
}

void* ON_HatchLoop::operator new(std::size_t, void* p)
{
  // ON_HatchLoop placement new
  return p;
}

void ON_HatchLoop::operator delete(void*, void*)
{
  // ON_HatchLoop placement delete
  return;
}

#endif


ON_HatchLoop::ON_HatchLoop()
: m_type( ON_HatchLoop::ltOuter), m_p2dCurve( NULL)
{
}

ON_HatchLoop::ON_HatchLoop( ON_Curve* pCurve2d, eLoopType type)
: m_type( type), m_p2dCurve( pCurve2d)
{
}

ON_HatchLoop::ON_HatchLoop( const ON_HatchLoop& src)
: m_type( src.m_type), m_p2dCurve( NULL)
{ 
  if( src.m_p2dCurve)
    m_p2dCurve = src.m_p2dCurve->DuplicateCurve();
}

ON_HatchLoop::~ON_HatchLoop()
{
  delete m_p2dCurve;
}

ON_HatchLoop& ON_HatchLoop::operator=( const ON_HatchLoop& src)
{
  if( this != &src)
  {
    if( m_p2dCurve)
      delete m_p2dCurve;
    m_p2dCurve = src.m_p2dCurve->DuplicateCurve();

    m_type = src.m_type;
  }
  return *this;
}

ON_BOOL32 ON_HatchLoop::IsValid( ON_TextLog* text_log) const
{
  ON_BOOL32 rc = m_p2dCurve != NULL;
  if( !rc)
  {
    if( text_log)
      text_log->Print( "2d loop curve is NULL\n");
  }
  if( rc)
  {
    rc = m_p2dCurve->IsValid( text_log);
    if( !rc)
    {
      if( text_log)
        text_log->Print( "Loop curve is not valid\n");
    }
  }

  if( rc)
  {
    ON_BoundingBox box;
    m_p2dCurve->GetBoundingBox( box);
    rc = ( box.Max().z == box.Min().z && box.Max().z == 0.0);
    if( !rc)
    {
      if( text_log)
        text_log->Print( "2d loop curve has non-zero z coordinates\n");
    }
  }

  if( rc && m_type != ltOuter && m_type != ltInner)
  {
    if( text_log)
      text_log->Print( "Loop type is invalid.\n");
    rc = false;
  }

  return rc;
}

void ON_HatchLoop::Dump( ON_TextLog& dump) const
{
  if( m_type == ltOuter)
    dump.Print( "Outer hatch loop\n");
  if( m_type == ltInner)
    dump.Print( "Inner hatch loop\n");

  if ( 0 == m_p2dCurve )
  {
    dump.Print( "2d curve: null pointer\n");
  }
  else
  {
    dump.Print( "2d curve:\n");
    m_p2dCurve->Dump(dump);
  }

}

ON_BOOL32 ON_HatchLoop::Write( ON_BinaryArchive& ar) const
{
  ON_BOOL32 rc = ar.Write3dmChunkVersion(1,1);
  if( rc) rc = ar.WriteInt( m_type);
  if( rc) rc = ar.WriteObject( m_p2dCurve);
  return rc;
}

ON_BOOL32 ON_HatchLoop::Read( ON_BinaryArchive& ar)
{
  m_type = ltOuter;
  delete m_p2dCurve;
  m_p2dCurve = NULL;
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = ar.Read3dmChunkVersion( &major_version, &minor_version);
  if ( major_version == 1 ) 
  {
    int type = 0;
    if( rc) rc = ar.ReadInt( &type);
    if( rc) 
    {
      switch( type)
      {
      case ltOuter:  m_type = ltOuter; break;
      case ltInner:  m_type = ltInner; break;
      default: rc = false; break;
      }
    }
    if( rc)
    {
      ON_Object* pObj = NULL;
      rc = ar.ReadObject( &pObj);
      if( pObj)
      {
        m_p2dCurve = ON_Curve::Cast( pObj);
        if( !m_p2dCurve) // read something, but it wasn't right
        {
          rc = false;
          delete pObj;
        }
      }
    }
  }
  return rc;
}

const ON_Curve* ON_HatchLoop::Curve() const
{
  return m_p2dCurve;
}

bool ON_HatchLoop::SetCurve( const ON_Curve& curve)
{
  ON_Curve* pC = curve.DuplicateCurve();
  if( pC)
  {
    if(pC->Dimension() == 3 && !pC->ChangeDimension(2))
      return false;

    if( m_p2dCurve)
      delete m_p2dCurve;
    m_p2dCurve = pC;
  }
  return true;
}
ON_HatchLoop::eLoopType ON_HatchLoop::Type() const
{
  return m_type;
}

void ON_HatchLoop::SetType( eLoopType type)
{
  m_type = type;
}

//  class ON_Hatch
/////////////////////////////////////////////////////////////////
ON_OBJECT_IMPLEMENT( ON_Hatch, ON_Geometry, "0559733B-5332-49d1-A936-0532AC76ADE5");


ON_Hatch::ON_Hatch()
: m_pattern_scale( 1.0),
  m_pattern_rotation( 0.0),
  m_pattern_index( -1)
{
}

ON_Hatch::ON_Hatch( const ON_Hatch& src)
:  ON_Geometry(src),
   m_plane( src.m_plane), 
   m_pattern_scale( src.m_pattern_scale),
   m_pattern_rotation( src.m_pattern_rotation),
   m_pattern_index( src.m_pattern_index)
{
  m_loops.Reserve( src.m_loops.Count());
  for( int i = 0; i < src.m_loops.Count(); i++)
  {
    ON_HatchLoop* pL = new ON_HatchLoop( *src.m_loops[i]);
    m_loops.Append( pL);
  }
}

ON_Hatch& ON_Hatch::operator=( const ON_Hatch& src)
{
  if( this != &src)
  {
    // Nov 3 2004 Dale Lear:
    //   Delete existing loops so we don't leak the memory;
    int i;
    for ( i = 0; i < m_loops.Count(); i++ )
    {
      ON_HatchLoop* pL = m_loops[i];
      if ( pL )
      {
        m_loops[i] = 0;
        delete pL;
      }
    }
    m_loops.SetCount(0);

    ON_Geometry::operator =(src);

    m_plane = src.m_plane;
    m_pattern_index = src.m_pattern_index;
    m_pattern_scale = src.m_pattern_scale;
    m_pattern_rotation = src.m_pattern_rotation;
    m_loops.Reserve( src.m_loops.Count());
    for( i = 0; i < src.m_loops.Count(); i++)
    {
      ON_HatchLoop* pL = new ON_HatchLoop( *src.m_loops[i]);
      m_loops.Append( pL);
    }
  }
  return *this;
}

ON_Hatch::~ON_Hatch()
{
  int i;
  for ( i = 0; i < m_loops.Count(); i++ )
  {
    ON_HatchLoop* pL = m_loops[i];
    if ( pL )
    {
      m_loops[i] = 0;
      delete pL;
    }
  }
}


ON_Hatch* ON_Hatch::DuplicateHatch() const
{
  return Duplicate();
}

ON_BOOL32 ON_Hatch::IsValid( ON_TextLog* text_log) const
{
  ON_BOOL32 rc = m_plane.IsValid();
  if( !rc)
  {
    if( text_log)
      text_log->Print( "Plane is not valid\n");
    return false;
  }
  // 18 June 2012 - Lowell - Added loop self-intersection and 
  // intersecting other loops tests
  int count = m_loops.Count();
  for(int i = 0; i < count; i++)
  {
    if(m_loops[i] == 0)
    {
      if( text_log)
        text_log->Print( "Loop[%d] is NULL\n", i);
      return false;
    }
    if(rc)
      rc = m_loops[i]->IsValid( text_log);
    if( !rc)
    {
      if( text_log)
        text_log->Print( "Loop[%d] is not valid\n", i);
      return false;
    }
  }
  
  return true;
}

void ON_Hatch::Dump( ON_TextLog& dump) const
{
  dump.Print("Hatch: Pattern index: %d\n", PatternIndex());
  dump.Print("Pattern rotation: %g\n", PatternRotation());
  dump.Print("Pattern scale: %g\n", PatternScale());
  ON_3dPoint p = this->BasePoint();
  dump.Print("Base point: %g, %g, %g\n", p.x, p.y, p.z);
  dump.Print("Plane origin: %g, %g, %g\n", m_plane.origin.x, m_plane.origin.y, m_plane.origin.z);
  dump.Print("Plane x axis: %g, %g, %g\n", m_plane.xaxis.x, m_plane.xaxis.y, m_plane.xaxis.z);
  dump.Print("Plane y axis: %g, %g, %g\n", m_plane.yaxis.x, m_plane.yaxis.y, m_plane.yaxis.z);
  dump.Print("Plane z axis: %g, %g, %g\n", m_plane.zaxis.x, m_plane.zaxis.y, m_plane.zaxis.z);
  int count = m_loops.Count();
  dump.Print("Loop count = %d\n", count);
  for( int i = 0; i < count; i++)
    m_loops[i]->Dump( dump);
}

ON_BOOL32 ON_Hatch::Write( ON_BinaryArchive& ar) const
{
  ON_BOOL32 rc = ar.Write3dmChunkVersion(1,1);
  if (rc) rc = ar.WritePlane( m_plane);
  if (rc) rc = ar.WriteDouble( m_pattern_scale);
  if (rc) rc = ar.WriteDouble( m_pattern_rotation);
  if (rc) rc = ar.WriteInt( m_pattern_index);
  if (rc)
  {
    int i, count = m_loops.Count();
    if( count < 0 )
      count = 0;
    ON_BOOL32 rc = ar.WriteInt( count);
    for( i = 0; i < count && rc; i++)
      rc = m_loops[i]->Write( ar);
  }
  return rc;
}

ON_BOOL32 ON_Hatch::Read( ON_BinaryArchive& ar)
{
  m_plane.CreateFromNormal( ON_origin, ON_zaxis);
  m_pattern_scale = 1.0;
  m_pattern_rotation = 0.0;
  m_pattern_index = -1;
  m_loops.Empty();
  int major_version = 0;
  int minor_version = 0;
  ON_BOOL32 rc = ar.Read3dmChunkVersion( &major_version, &minor_version);
  if ( major_version == 1 ) 
  {
    if( rc) rc = ar.ReadPlane( m_plane);
    if( rc) rc = ar.ReadDouble( &m_pattern_scale);
    if( rc) rc = ar.ReadDouble( &m_pattern_rotation);
    if( rc) rc = ar.ReadInt( &m_pattern_index);
    if( rc)
    {
      m_loops.Empty();
      int i, count = 0;
      rc = ar.ReadInt( &count);
      if( rc && count > 0)
      {
        m_loops.SetCapacity( count );
        for( i = 0; rc && i < count; i++)
        {
          ON_HatchLoop*& pLoop = m_loops.AppendNew();
          pLoop = new ON_HatchLoop;
          if( pLoop)
            rc = pLoop->Read( ar);
          else
            rc = false;
        }
      }
    }
  }
  return rc;
}

ON::object_type ON_Hatch::ObjectType() const
{
  return ON::hatch_object;
}

int ON_Hatch::Dimension() const
{
  return 3;
}

// Copy the 2d curve, make it 3d, and transform it 
// to the 3d plane position
ON_Curve* ON_Hatch::LoopCurve3d( int index) const
{
  int count = m_loops.Count();
  ON_Curve* pC = NULL;

  if( index >= 0 && index < count)
  {
    if( m_loops[index]->Curve())
    {
      pC = m_loops[index]->Curve()->DuplicateCurve();
      if( pC)
      {
        pC->ChangeDimension( 3);

        ON_Xform xf;
        xf.Rotation( ON_xy_plane, m_plane);

        pC->Transform( xf);
      }
    }
  }
  return pC;
}


int ON_Hatch::PatternIndex() const
{
  return m_pattern_index;
}

void ON_Hatch::SetPatternIndex( int index)
{
  m_pattern_index = index;
}


ON_BOOL32 ON_Hatch::GetBBox( double* bmin, double* bmax, ON_BOOL32 bGrowBox) const
{
  int i;
  int count = m_loops.Count();
  ON_BOOL32 rc = true;
  ON_Curve* pC;
  for( i = 0; rc && i < count; i++)
  {
    pC = LoopCurve3d( i);
    if( pC)
    {
      rc = pC->GetBBox( bmin, bmax, i?true:bGrowBox);
      delete pC;
    }
  }
  return rc;
}

bool ON_Hatch::GetTightBoundingBox( ON_BoundingBox& tight_bbox, int bGrowBox, const ON_Xform* xform) const
{
  int i;
  int count = m_loops.Count();
  ON_CurveArray curves(count);
  for( i = 0; i < count; i++)
  {
    curves.Append( LoopCurve3d(i) );
  }
  return curves.GetTightBoundingBox(tight_bbox,bGrowBox,xform);
}

static double Angle3d(const ON_3dVector& axis, ON_3dVector& from, const ON_3dVector& to)
{
  ON_3dVector x = from, a = to;
  x.Unitize();
  a.Unitize();

  ON_3dVector y = ON_CrossProduct(axis, from);
  y.Unitize();

  double cosa = x * a;

  if(cosa > 1.0 - ON_SQRT_EPSILON)
    return 0.0;
  if(cosa < ON_SQRT_EPSILON - 1.0)
    return ON_PI;

  double sina = a * y;

  return atan2(sina, cosa);
}


#define ARBBOUND  0.015625
void arbaxis(const ON_3dVector& givenaxis, ON_3dVector& newaxis)
{
  if(fabs(givenaxis[0]) < ARBBOUND && fabs(givenaxis[1]) < ARBBOUND) // near world z
    newaxis = ON_CrossProduct(ON_yaxis, givenaxis);
  else
    newaxis = ON_CrossProduct(ON_zaxis, givenaxis);

  newaxis.Unitize();
}

double arbaxisRotation(const ON_Plane& plane)
{
  // get arbaxis frame and angle of rotation from it
  ON_3dVector arbXaxis;
  arbaxis(plane.zaxis, arbXaxis);
  return Angle3d(plane.zaxis, arbXaxis, plane.xaxis);
}

// 20 June 2012 - Lowell - rr44706, 68320
// This will find A, the arbaxis direction for the hatch plane
// and rotate the hatch plane by -A and rotate the hatch boundaries
// by A and add A to the hatch rotation.
// The picture will be the same after that, but the way the
// angle is represented will match the way AutoCAD does it
// so hatches can be round-tripped with acad files.
// In addition, after several hatches are rotated by different amounts
// the hatch angles can be set to look the same by setting them all
// to the same pattern rotation
 
static void UnrotateHatch(ON_Hatch* hatch)
{
  double a = arbaxisRotation(hatch->Plane());
  ON_Plane& plane = *(ON_Plane*)(&hatch->Plane());
  if(fabs(a) > ON_ZERO_TOLERANCE)
  {
    plane.Rotate(-a, plane.zaxis);
    for(int i = 0; i < hatch->LoopCount(); i++)
    {
      ON_Curve* pC = (ON_Curve*)hatch->Loop(i)->Curve();
      pC->Rotate(a, ON_zaxis, ON_origin);
    }
    hatch->SetPatternRotation(hatch->PatternRotation()+a);
  }
  ON_3dPoint P;
  plane.ClosestPointTo(ON_origin, &P.x, &P.y);

  if(fabs(P.x) > ON_ZERO_TOLERANCE ||fabs(P.y) > ON_ZERO_TOLERANCE ||fabs(P.z) > ON_ZERO_TOLERANCE)
  {
    ON_2dVector V(-P.x, -P.y);
    for(int i = 0; i < hatch->LoopCount(); i++)
    {
      ON_Curve* pC = (ON_Curve*)hatch->Loop(i)->Curve();
      pC->Translate(V);
    }
    P = plane.PointAt(P.x, P.y);
    plane.origin = P;
  }
}

ON_BOOL32 ON_Hatch::Transform( const ON_Xform& xform)
{
  if( fabs( fabs( xform.Determinant()) - 1.0) > 1.0e-4)
  {
    // xform has a scale component
    ON_Plane tmp( m_plane);
    tmp.Transform( xform);
    ON_Xform A, B, T;
    A.Rotation( ON_xy_plane, m_plane);
    B.Rotation( tmp, ON_xy_plane);
    T = B * xform * A;

    // kill translation and z-scaling
    T[0][2] = T[0][3] = 0.0;
    T[1][2] = T[1][3] = 0.0;
    T[2][0] = T[2][1] = 0.0; T[2][2] = 1.0; T[2][3] = 0.0; 
    T[3][0] = T[3][1] = T[3][2] = 0.0; T[3][3] = 1.0;

    for( int i = 0; i < LoopCount(); i++)
      m_loops[i]->m_p2dCurve->Transform( T);
  }
  int rc = m_plane.Transform( xform);

  UnrotateHatch(this);

  TransformUserData(xform);




  return rc;
}

bool ON_Hatch::Create( const ON_Plane& plane,
                       const ON_SimpleArray<const ON_Curve*> loops, 
                       int pattern_index, 
                       double pattern_rotation, 
                       double pattern_scale)
{
  if( loops.Count() < 1)
    return false;
  if( pattern_index < 0)
    return false;
  SetPlane( plane);
  for( int i = 0; i < loops.Count(); i++)
  {
    ON_HatchLoop* pLoop = new ON_HatchLoop;
    pLoop->SetCurve( *loops[i]);
    pLoop->SetType( i?ON_HatchLoop::ltInner:ON_HatchLoop::ltOuter);
    AddLoop( pLoop);
  }
  SetPatternIndex( pattern_index);
  SetPatternRotation( pattern_rotation);
  SetPatternScale( pattern_scale);
  return true;
}

const ON_Plane& ON_Hatch::Plane() const
{
  return m_plane;
}

void ON_Hatch::SetPlane( const ON_Plane& plane)
{
  m_plane = plane;
}

double ON_Hatch::PatternRotation() const
{
  return m_pattern_rotation;
}

void ON_Hatch::SetPatternRotation( double rotation)
{
  m_pattern_rotation = rotation;
}

double ON_Hatch::PatternScale() const
{
  return m_pattern_scale;
}

void ON_Hatch::SetPatternScale( double scale)
{
  if( scale > 0.001) // Changed May 13, 2009 - Lowell - rr39185
    m_pattern_scale = scale;
}

int ON_Hatch::LoopCount() const
{
  return m_loops.Count();
}

void ON_Hatch::AddLoop( ON_HatchLoop* pLoop)
{
  m_loops.Append( pLoop);
}

bool ON_Hatch::InsertLoop( int index, ON_HatchLoop* loop)
{
  if( index >= 0 && index <= m_loops.Count()) // 26 June 2012 - Lowell - Changed ndex < to ndex <= 
  {
    m_loops.Insert(index, loop);
	return true;
  }

  return false;
}

bool ON_Hatch::RemoveLoop( int index)
{
  if( index >= 0 && index < m_loops.Count())
  {
    delete m_loops[index];
    m_loops.Remove(index);
    return true;
  }
  
  return false;
}


bool ON_Hatch::ReplaceLoops(ON_SimpleArray<const ON_Curve*> loop_curves)
{
  if(loop_curves.Count() < 1)
    return false;

  bool rc = true;
  ON_Xform xf;
  bool flat = false;
  ON_SimpleArray<ON_HatchLoop*> loops;

  for(int i = 0; i < loop_curves.Count(); i++)
  {
    if(loop_curves[i] == 0)
    {
      rc = false;
      break;
    }
    ON_Curve* p2d = loop_curves[i]->Duplicate();
    if(p2d == 0)
    {
      rc = false;
      break;
    }
    if(p2d->Dimension() == 3)
    {
      if(!flat)
      {
        xf.PlanarProjection(m_plane);
        flat = true;
      }
      if(!p2d->Transform(xf) ||
         !p2d->ChangeDimension(2))
      {
        delete p2d;
        rc = false;
        break;
      }
    }
    ON_HatchLoop* loop = new ON_HatchLoop(p2d,loops.Count()?ON_HatchLoop::ltInner:ON_HatchLoop::ltOuter);
    if(loop)
      loops.Append(loop);
    else
      delete p2d;
  }
  if(!rc)
  {
    for(int i = 0; i < loops.Count(); i++)
      delete loops[i];

    loops.Empty();
  }

  if(loops.Count() < 1)
    return false;

  for(int i = 0; i < m_loops.Count(); i++)
    delete m_loops[i];
  m_loops.Empty();
  for(int i = 0; i < loops.Count(); i++)
    m_loops.Append(loops[i]);
  return true;
}

const ON_HatchLoop* ON_Hatch::Loop( int index) const
{
  if( index >= 0 && index < m_loops.Count())
    return m_loops[index];
  
  return NULL;
}

// Basepoint functions added March 23, 2008 -LW
void ON_Hatch::SetBasePoint(ON_2dPoint basepoint)
{
  ON_HatchExtra* pE = ON_HatchExtra::HatchExtension(this,true);
  if(pE)
  {
    pE->SetBasePoint(basepoint);
  }
}

void ON_Hatch::SetBasePoint(ON_3dPoint point)
{
  ON_HatchExtra* pE = ON_HatchExtra::HatchExtension(this,true);
  if(pE)
  {
    ON_2dPoint base;
    if(m_plane.ClosestPointTo(point, &base.x, &base.y))
      pE->SetBasePoint(base);
  }
}

ON_3dPoint ON_Hatch::BasePoint() const
{
  ON_3dPoint point(ON_origin);
  const ON_HatchExtra* pE = ON_HatchExtra::HatchExtension(this,false);
  if(pE)
  {
    ON_2dPoint base = pE->BasePoint();
    point = m_plane.PointAt(base.x, base.y);
  }
  return point;
}

ON_2dPoint ON_Hatch::BasePoint2d() const
{
  ON_2dPoint basepoint(0.0,0.0);
  const ON_HatchExtra* pE = ON_HatchExtra::HatchExtension(this,false);
  if(pE)
    basepoint = pE->BasePoint();

  return basepoint;
}

// Added March 23, 2008 -LW
// This function is temporary and will be removed next time the SDK can be modified.
class ON_HatchExtra* ON_Hatch::HatchExtension()
{
  ON_HatchExtra* pExtra = ON_HatchExtra::Cast( GetUserData( ON_HatchExtra::m_ON_HatchExtra_class_id.Uuid()));
  return pExtra;
}
