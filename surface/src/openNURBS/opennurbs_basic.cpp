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

#if !defined(OPENNURBS_PLUS_INC_)

////////////////////////////////////////////////////////////////
//
// Basic ON_PlaneEquation functions
//

double ON_PlaneEquation::MinimumValueAt(const ON_SurfaceLeafBox& srfleafbox) const
{
  // The working function is part of the Rhino SDK.
  return 0.0;
}

double ON_PlaneEquation::MaximumValueAt(const ON_SurfaceLeafBox& srfleafbox) const
{
  // The working function is part of the Rhino SDK.
  return 0.0;
}

double ON_PlaneEquation::MinimumValueAt(const class ON_CurveLeafBox& crvleafbox) const
{
  // The working function is part of the Rhino SDK.
  return 0.0;
}

double ON_PlaneEquation::MaximumValueAt(const class ON_CurveLeafBox& crvleafbox) const
{
  // The working function is part of the Rhino SDK.
  return 0.0;
}

bool ON_Curve::GetTightBoundingBox( 
		ON_BoundingBox& tight_bbox, 
    int bGrowBox,
		const ON_Xform* xform
    ) const
{
  if ( bGrowBox && !tight_bbox.IsValid() )
  {
    bGrowBox = false;
  }

  if ( !bGrowBox )
  {
    tight_bbox.Destroy();
  }

  // In general, putting start and end point in the box lets me avoid
  // testing lots of nodes.
  ON_3dPoint P = PointAtStart();
  if ( xform )
    P = (*xform)*P;
  tight_bbox.Set( P, bGrowBox );
  bGrowBox = true;

  P = PointAtEnd();
  if ( xform )
    P = (*xform)*P;
  tight_bbox.Set( P, bGrowBox );

  ON_BoundingBox curve_bbox = BoundingBox();
  if ( ON_WorldBBoxIsInTightBBox( tight_bbox, curve_bbox, xform ) )
  {
    // Curve is inside tight_bbox
    return true;
  }

  // free opennurbs does not provide tight bounding boxes
  ON_NurbsCurve N;
  if ( 0 == GetNurbForm(N) )
    return false;
  if ( N.m_order < 2 || N.m_cv_count < N.m_order )
    return false;

  ON_BezierCurve B;
  for ( int span_index = 0; span_index <= N.m_cv_count - N.m_order; span_index++ )
  {
    if ( !(N.m_knot[span_index + N.m_order-2] < N.m_knot[span_index + N.m_order-1]) )
      continue;
    if ( !N.ConvertSpanToBezier( span_index, B ) )
      continue;
    if ( !B.GetTightBoundingBox(tight_bbox,bGrowBox,xform) )
      continue;
    bGrowBox = true;
  }

  return (0!=bGrowBox);
}

ON_PolylineCurve* ON_Curve::MeshCurve(
    ON_MeshCurveParameters&,
    ON_PolylineCurve*,
    bool,
    const ON_Interval*
    )
{
  // The working function is part of the Rhino SDK.
  return 0;
}


#endif
