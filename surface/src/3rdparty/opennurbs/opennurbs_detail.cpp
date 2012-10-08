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

ON_OBJECT_IMPLEMENT(ON_DetailView,ON_Geometry,"C8C66EFA-B3CB-4e00-9440-2AD66203379E");

ON_DetailView::ON_DetailView()
{
  m_page_per_model_ratio = 0.0;
}

ON_DetailView::~ON_DetailView()
{
}

void ON_DetailView::MemoryRelocate()
{
  m_boundary.MemoryRelocate();
}

ON_BOOL32 ON_DetailView::IsValid( ON_TextLog* text_log ) const
{
  // Don't bother checking m_view - during runtime it's
  // not filled in. It is only used for IO.  See
  // CRhDetailViewObject::PrepareToWrite() for details.
  return m_boundary.IsValid(text_log);
}

void ON_DetailView::Dump( ON_TextLog& text_log ) const
{
  m_view.Dump(text_log);
  m_boundary.Dump(text_log);
}

unsigned int ON_DetailView::SizeOf() const
{
  unsigned int sz = ON_Geometry::SizeOf();
  sz += sizeof(*this) - sizeof(ON_Geometry);
  sz += m_boundary.SizeOf();
  return sz;
}

ON_BOOL32 ON_DetailView::Write( ON_BinaryArchive& archive ) const
{
  bool rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 1 );
  if (!rc)
    return false;

  for(;;)
  {
    // m_view is wrapped in a subchunk so ON_3dmView can be expanded 
    // without breaking the file format.
    rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 0 );
    if (rc) 
    {
      rc = m_view.Write(archive);
      if (!archive.EndWrite3dmChunk())
        rc = false;
    }
    if(!rc)
      break;

    // m_boundary is wrapped in a subchunk so ON_NurbsCurve can be expanded 
    // without breaking the file format.
    rc = archive.BeginWrite3dmChunk( TCODE_ANONYMOUS_CHUNK, 1, 0 );
    if (rc) 
    {
      rc = m_boundary.Write(archive)?true:false;
      if (!archive.EndWrite3dmChunk())
        rc = false;
    }
    if(!rc)
      break;

    // 28 Feb 2006  1.1 fields added
    rc = archive.WriteDouble(m_page_per_model_ratio);
    if ( !rc )
      break;

    break;
  }

  if ( !archive.EndWrite3dmChunk() )
    rc = false;

  return rc;
}

ON_BOOL32 ON_DetailView::Read(ON_BinaryArchive& archive)
{
  m_page_per_model_ratio = 0.0;
  m_view.Default();
  m_boundary.Destroy();

  int major_version = 0;
  int minor_version = 0;
  bool rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &major_version, &minor_version );
  if (!rc)
    return false;

  for(;;)
  {
    rc = (1 == major_version );
    if (!rc) break;

    // m_view is wrapped in a subchunk so ON_3dmView can be expanded 
    // without breaking the file format.
    int mj = 0, mn = 0;
    rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &mj, &mn );
    if (rc) 
    {
      rc = m_view.Read(archive);
      if (!archive.EndRead3dmChunk())
        rc = false;
    }
    if (!rc) break;


    // m_boundary is wrapped in a subchunk so ON_NurbsCurve can be expanded 
    // without breaking the file format.
    mj = mn = 0;
    rc = archive.BeginRead3dmChunk( TCODE_ANONYMOUS_CHUNK, &mj, &mn );
    if (rc) 
    {
      rc = m_boundary.Read(archive)?true:false;
      if (!archive.EndRead3dmChunk())
        rc = false;
    }
    if (!rc) break;

    if ( minor_version >= 1 )
    {
      rc = archive.ReadDouble(&m_page_per_model_ratio);
    }


    break;
  }

  if ( !archive.EndRead3dmChunk() )
    rc = false;

  return rc;
}

ON::object_type ON_DetailView::ObjectType() const
{
  return ON::detail_object;
}

int ON_DetailView::Dimension() const
{
  return m_boundary.Dimension();
}

ON_BOOL32 ON_DetailView::GetBBox(
    double* boxmin,
    double* boxmax,
    int bGrowBox
    ) const
{
  return m_boundary.GetBBox(boxmin,boxmax,bGrowBox);
}

bool ON_DetailView::GetTightBoundingBox( 
      ON_BoundingBox& tight_bbox, int bGrowBox, const ON_Xform* xform
      ) const
{
  return m_boundary.GetTightBoundingBox(tight_bbox,bGrowBox,xform);
}

ON_BOOL32 ON_DetailView::Transform( const ON_Xform& xform )
{
  return m_boundary.Transform(xform);
}

