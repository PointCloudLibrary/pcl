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

bool ON_IsHairlinePrintWidth(double width_mm)
{
  if(width_mm > 0.0 && width_mm < 0.001)
    return true;

  return false;
}

double ON_HairlinePrintWidth()
{
  return 0.0001;
}


//////////////////////////////////////////////////////////////////////
// class ON_LinetypeSegment
bool ON_LinetypeSegment::operator==( const ON_LinetypeSegment& src) const
{
  return ( m_length == src.m_length && m_seg_type == src.m_seg_type);
}

bool ON_LinetypeSegment::operator!=( const ON_LinetypeSegment& src) const
{
  return ( m_length != src.m_length || m_seg_type != src.m_seg_type);
}


void ON_LinetypeSegment::Dump( ON_TextLog& dump) const
{
  switch( m_seg_type)
  {
  case stLine:
    dump.Print( "Segment type = Line: %g\n", m_length);
    break;
  case stSpace:
    dump.Print( "Segment type = Space: %g\n", m_length);
    break;
  }
}

ON_LinetypeSegment::ON_LinetypeSegment()
{
  memset(this,0,sizeof(*this));
  m_length = 1.0;
  m_seg_type = stLine;
}

//////////////////////////////////////////////////////////////////////
// class ON_Linetype

ON_OBJECT_IMPLEMENT( ON_Linetype, ON_Object, "26F10A24-7D13-4f05-8FDA-8E364DAF8EA6" );

ON_Linetype::ON_Linetype() : m_linetype_index(-1)
{
  memset(&m_linetype_id,0,sizeof(m_linetype_id));
}

ON_Linetype::~ON_Linetype()
{
  m_linetype_name.Destroy();
}

void ON_Linetype::Default()
{
  m_linetype_index = -1;
  memset(&m_linetype_id,0,sizeof(m_linetype_id));
  m_linetype_name.Destroy();
  m_segments.Destroy();
}

ON_BOOL32 ON_Linetype::IsValid( ON_TextLog* text_log ) const
{
  int i, count = m_segments.Count();

  // An ON_Linetype with an empty name is valid.

  if ( count < 1 )
  {
    if ( text_log )
      text_log->Print("ON_Linetype m_segments.Count() = 0\n");
    return false;
  }

  if ( 1 == count )
  {
    if ( m_segments[0].m_length <= 0.0  )
    {
      if ( text_log )
        text_log->Print("ON_Linetype bogus single segment linetype - length <= 0.0 (it must be > 0)\n");
      return false;
    }

    if ( ON_LinetypeSegment::stLine != m_segments[0].m_seg_type )
    {
      if ( text_log )
        text_log->Print("ON_Linetype bogus single segment linetype - type != stLine\n");
      return false;
    }
  }
  else
  {
    for (i = 0; i < count; i++ )
    {
      if ( m_segments[i].m_length < 0.0 )
      {
        if ( text_log )
          text_log->Print("ON_Linetype segment has negative length.\n");
        return false;
      }

      if ( ON_LinetypeSegment::stLine != m_segments[i].m_seg_type && ON_LinetypeSegment::stSpace != m_segments[i].m_seg_type )
      {
        if ( text_log )
          text_log->Print("ON_Linetype segment has invalid m_seg_type.\n");
        return false;
      }

      if ( i )
      {
        if ( m_segments[i].m_seg_type == m_segments[i-1].m_seg_type )
        {
          if ( text_log )
            text_log->Print("ON_Linetype consecutive segments have same type.\n");
          return false;
        }

        if ( 0.0 == m_segments[i].m_length && 0.0 == m_segments[i-1].m_length )
        {
          if ( text_log )
            text_log->Print("ON_Linetype consecutive segments have length zero.\n");
          return false;
        }
      }
    }
  }

  return true;
}

void ON_Linetype::Dump( ON_TextLog& dump ) const
{
  const wchar_t* sName = LinetypeName();
  if ( !sName )
    sName = L"";
  dump.Print( "Segment count = %d\n", m_segments.Count());
  dump.Print( "Pattern length = %g\n", PatternLength());
  dump.Print( "Pattern = (" );
  for( int i = 0; i < m_segments.Count(); i++)
  {
    const ON_LinetypeSegment& seg = m_segments[i];
    if ( i )
      dump.Print(",");
    switch( seg.m_seg_type)
    {
    case ON_LinetypeSegment::stLine:
      dump.Print( "line");
      break;
    case ON_LinetypeSegment::stSpace:
      dump.Print( "space");
      break;
    default:
      dump.Print( "invalid");
      break;
    }
  }
  dump.Print(")\n");
}

ON_BOOL32 ON_Linetype::Write( ON_BinaryArchive& file) const
{
  bool rc = file.BeginWrite3dmChunk(TCODE_ANONYMOUS_CHUNK,1,1);
  if (rc)
  {
    for(;;)
    {
      // chunk version 1.0 fields
      rc = file.WriteInt( LinetypeIndex());
      if(!rc) break;

      rc = file.WriteString( m_linetype_name );
      if (!rc) break;

      rc = file.WriteArray( m_segments );
      if(!rc) break;

      // chunk version 1.1 fields
      rc = file.WriteUuid( m_linetype_id );
      if (!rc) break;

      break;
    }

    if ( !file.EndWrite3dmChunk() )
      rc = false;
  }
  return rc;
}

ON_BOOL32 ON_Linetype::Read( ON_BinaryArchive& file)
{
  Default();
  m_linetype_index = -1;

  int major_version=0;
  int minor_version=0;
  bool rc = file.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );

  if (rc)
  {
    if( 1 == major_version ) 
    {
      // chunk version 1.0 fields
      if( rc) 
        rc = file.ReadInt( &m_linetype_index );
      if( rc) 
        rc = file.ReadString( m_linetype_name );
      if( rc) 
        rc = file.ReadArray( m_segments );

      if ( minor_version >= 1 )
      {
        if (rc)
          rc = file.ReadUuid( m_linetype_id );
      }
    }
    else
    {
      rc = false;
    }

    if ( !file.EndRead3dmChunk() )
      rc = false;
  }

  return rc;
}

bool ON_Linetype::SetLinetypeName( const char* s)
{
  m_linetype_name = s;
  return IsValid()?true:false;
}

bool ON_Linetype::SetLinetypeName( const wchar_t* s)
{
  m_linetype_name = s;
  return IsValid()?true:false;
}

const wchar_t* ON_Linetype::LinetypeName() const
{
  const wchar_t* s = m_linetype_name;
  return s;
}

bool ON_Linetype::SetLinetypeIndex( int i)
{
  m_linetype_index = i;
  return true;
}

int ON_Linetype::LinetypeIndex() const
{
  return m_linetype_index;
}

double ON_Linetype::PatternLength() const
{
  double length = 0.0;
  int seg_count = m_segments.Count();
  for( int i = 0; i < seg_count; i++)
  {
    length += m_segments[i].m_length;
  }
  return length;
}

ON_SimpleArray<ON_LinetypeSegment>& ON_Linetype::Segments()
{
  return m_segments;
}

const ON_SimpleArray<ON_LinetypeSegment>& ON_Linetype::Segments() const
{
  return m_segments;
}

int ON_Linetype::SegmentCount() const
{
  return m_segments.Count();
}

int ON_Linetype::AppendSegment( const ON_LinetypeSegment& segment)
{
  m_segments.Append( segment);
  return( m_segments.Count()-1);
}

bool ON_Linetype::RemoveSegment( int index )
{
  bool rc = ( index >= 0 && index < m_segments.Count());
  if (rc)
    m_segments.Remove(index);
  return rc;
}

bool ON_Linetype::SetSegment( int index, const ON_LinetypeSegment& segment)
{
  if( index >= 0 && index < m_segments.Count())
  {
    m_segments[index] = segment;
    return true;
  }
  else
    return false;
}

bool ON_Linetype::SetSegment( int index, double length, ON_LinetypeSegment::eSegType type)
{
  if( index >= 0 && index < m_segments.Count())
  {
    m_segments[index].m_length = length;
    m_segments[index].m_seg_type = type;
    return true;
  }
  else
    return false;
}

ON_LinetypeSegment ON_Linetype::Segment( int index) const
{
  if( index >= 0 && index < m_segments.Count())
    return m_segments[index];
  else
    return ON_LinetypeSegment();
}







