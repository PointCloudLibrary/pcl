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

ON_VIRTUAL_OBJECT_IMPLEMENT(ON_Geometry,ON_Object,"4ED7D4DA-E947-11d3-BFE5-0010830122F0");

ON_Geometry::ON_Geometry()
{}

ON_Geometry::ON_Geometry(const ON_Geometry& src) : ON_Object(src)
{}

ON_Geometry& ON_Geometry::operator=(const ON_Geometry& src)
{
  ON_Object::operator=(src);
  return *this;
}

ON_Geometry::~ON_Geometry()
{}

ON_BoundingBox ON_Geometry::BoundingBox() const
{
  ON_BoundingBox bbox;
  if ( !GetBoundingBox( bbox.m_min, bbox.m_max, false ) )
    bbox.Destroy();
  return bbox;
}

ON_BOOL32 
ON_Geometry::GetBoundingBox( // returns true if successful
       ON_BoundingBox& bbox,
       ON_BOOL32 bGrowBox
       ) const
{
  return GetBoundingBox( bbox.m_min, bbox.m_max, bGrowBox );
}

ON_BOOL32 
ON_Geometry::GetBoundingBox( // returns true if successful
       ON_3dPoint& boxmin,
       ON_3dPoint& boxmax,
       ON_BOOL32 bGrowBox
       ) const
{
  ON_Workspace ws;
  const int dim = Dimension();
  double *bmin, *bmax;
  if ( dim <= 3 ) {
    bmin = &boxmin.x;
    bmax = &boxmax.x;
  }
  else {
    bmin = ws.GetDoubleMemory(dim*2);
    bmax = bmin+dim;
    memset( bmin, 0, 2*dim*sizeof(*bmin) );
    if ( bGrowBox ) {
      bmin[0] = boxmin.x; bmin[1] = boxmin.y; bmin[1] = boxmin.z;
      bmax[0] = boxmax.x; bmax[1] = boxmax.y; bmax[1] = boxmax.z;
    }
  }
	// Treat invalid box on input as empty
	bool invalid=false;	//input box invalid=empty
	if(bGrowBox)
		invalid =  boxmin.x>boxmax.x || boxmin.y>boxmax.y|| boxmin.z>boxmax.z;
	if(bGrowBox && invalid)
		bGrowBox=false;

  const ON_BOOL32 rc = GetBBox( bmin, bmax, bGrowBox );
  if ( dim > 3 ) {
    boxmin.x = bmin[0]; boxmin.y = bmin[1]; boxmin.z = bmin[2];
    boxmax.x = bmax[0]; boxmax.y = bmax[1]; boxmax.z = bmax[2];
  }
  else if ( dim <= 2 ) {
    boxmin.z = 0.0;
    boxmax.z = 0.0;
    if ( dim <= 1 ) {
      boxmin.y = 0.0;
      boxmax.y = 0.0;
    }
  }
  return rc;
}

bool ON_Geometry::GetTightBoundingBox( 
			ON_BoundingBox& tight_bbox, 
      int bGrowBox,
			const ON_Xform* xform
      ) const
{
  //	This implementation should be overridden by classes devived
  //  from ON_Geometry
  if ( bGrowBox && !tight_bbox.IsValid() )
  {
    bGrowBox = false;
  }
  if ( !bGrowBox )
  {
    tight_bbox.Destroy();
  }

  if ( xform && !xform->IsIdentity() )
  {
    ON_3dPointArray corners(8);
    ON_BoundingBox world_bbox;
    if ( GetBoundingBox(world_bbox,false) )
    {
      world_bbox.GetCorners(corners);
      if ( corners.GetTightBoundingBox(tight_bbox,bGrowBox,xform) )
        bGrowBox = true;
    }
  }
  else
  {
    if ( GetBoundingBox(tight_bbox,bGrowBox) )
      bGrowBox = true;
  }

  return bGrowBox?true:false;
}

ON_BOOL32 ON_Geometry::SwapCoordinates(
      int i, int j        // indices of coords to swap
      )
{
  ON_BOOL32 rc = false;
  const int dim = Dimension();
  if ( dim > 0 && dim <= 3 && i >= 0 && i < 3 && j >= 0 && j < 3 ) {
    if ( i == j ) {
      rc = true;
    }
    else {
      int k;
      ON_Xform swapij(0.0);
      for ( k = 0; k < 4; k++ ) {
        if ( i == k )
          swapij[k][j] = 1.0;
        else if ( j == k )
          swapij[k][i] = 1.0;
        else
          swapij[k][k] = 1.0;
      }
      rc = Transform( swapij );
    }
  }
  return rc;
}

ON_BOOL32 ON_Geometry::Rotate(
      double sin_angle,          // sin(angle)
      double cos_angle,          // cos(angle)
      const ON_3dVector& axis, // axis of rotation
      const ON_3dPoint& center // center of rotation
      )
{
  if ( sin_angle == 0.0 && cos_angle == 1.0 )
    return true;
  ON_Xform rot;
  rot.Rotation( sin_angle, cos_angle, axis, center );
  return Transform( rot );
}

ON_BOOL32 ON_Geometry::Rotate(
      double angle,              // angle in radians
      const ON_3dVector& axis, // axis of rotation
      const ON_3dPoint& center // center of rotation
      )
{
  if ( angle == 0.0 )
    return true;
  return Rotate( sin(angle), cos(angle), axis, center );
}

ON_BOOL32 ON_Geometry::Translate( const ON_3dVector& delta )
{
  if ( delta.IsZero() )
    return true;
  ON_Xform tr;
  tr.Translation( delta );
  return Transform( tr );
}

ON_BOOL32 ON_Geometry::Scale( double x )
{
  if ( x == 1.0 )
    return true;
  ON_Xform s;
  s.Scale( x, x, x );
  return Transform( s );
}

bool ON_Geometry::IsDeformable() const
{
  return false;
}

bool ON_Geometry::MakeDeformable()
{
  return false;
}

void ON_Geometry::ClearBoundingBox()
{
  // default implementation does nothing
}

ON_BOOL32 ON_Geometry::Transform( const ON_Xform& xform )
{
  TransformUserData(xform);
  return true;
}

ON_BOOL32 ON_Geometry::HasBrepForm() const
{
  // override if specific geoemtry has brep form
  return false;
}

ON_Brep* ON_Geometry::BrepForm( ON_Brep* ) const
{
  // override if specific geoemtry has brep form
  return NULL;
}


ON_COMPONENT_INDEX ON_Geometry::ComponentIndex() const
{
  // default constructor sets
  // m_type = ON_COMPONENT_INDEX::invalid_type and m_index = -1.
  ON_COMPONENT_INDEX ci;
  return ci;  
}

bool ON_Geometry::EvaluatePoint( const class ON_ObjRef&, ON_3dPoint& P ) const
{
  // virtual function default
  P = ON_UNSET_POINT;
  return false;
}

