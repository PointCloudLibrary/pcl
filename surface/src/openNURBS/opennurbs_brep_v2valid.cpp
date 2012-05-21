/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2011 Robert McNeel & Associates. All rights reserved.
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

#include <pcl/surface/openNURBS/opennurbs.h>

bool ON_Brep::IsDeformable() const
{
  int ei, edge_count = m_E.Count();
  for ( ei = 0; ei < edge_count; ei++ )
  {
    const ON_BrepEdge& edge = m_E[ei];
    if ( edge.m_edge_index != ei )
      continue;
    const ON_Curve* crv = edge.EdgeCurveOf();
    if ( !crv )
      continue;
    if ( !crv->IsDeformable() )
      return false;
  }

  int fi, face_count = m_F.Count();
  for ( fi = 0; fi < face_count; fi++ )
  {
    const ON_BrepFace& face = m_F[fi];
    if ( face.m_face_index != fi )
      continue;
    const ON_Surface* srf = face.SurfaceOf();
    if ( !srf )
      continue;
    if ( !srf->IsDeformable() )
      return false;
  }

  return true;
}


