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

static 
void ON_BrepExtrudeHelper_ReserveSpace( 
          ON_Brep& brep, 
          int extruded_trim_count, 
          int cap_count 
          )
{
  if ( extruded_trim_count >= 0 && cap_count >= 0 )
  {
    const int vertex_count0 = brep.m_V.Count();
    const int trim_count0 = brep.m_T.Count();
    const int loop_count0 = brep.m_L.Count();
    const int edge_count0 = brep.m_E.Count();
    const int face_count0 = brep.m_F.Count();
    const int srf_count0 = brep.m_S.Count();
    const int c2_count0 = brep.m_C2.Count();
    const int c3_count0 = brep.m_C3.Count();

    // the +1's are for open loops

    brep.m_V.Reserve( vertex_count0 + extruded_trim_count + 1 );
    brep.m_T.Reserve( trim_count0 + (4+cap_count)*extruded_trim_count );
    brep.m_F.Reserve( face_count0 + extruded_trim_count + cap_count );
    brep.m_E.Reserve( edge_count0 + 2*extruded_trim_count + 1 );
    brep.m_L.Reserve( loop_count0 + extruded_trim_count + cap_count );
    brep.m_S.Reserve( srf_count0 + extruded_trim_count + cap_count );
    brep.m_C2.Reserve( c2_count0 + (4+cap_count)*extruded_trim_count );
    brep.m_C3.Reserve( c3_count0 + 2*extruded_trim_count + 1 );
  }
}

static
ON_SumSurface* ON_BrepExtrudeHelper_MakeSumSrf( const ON_Curve& path_curve,
                                                 const ON_BrepEdge& base_edge, ON_BOOL32 bRev )
{
  ON_SumSurface* sum_srf = 0;
  // create side surface
  if ( base_edge.ProxyCurve() )
  {
    ON_Curve* srf_path_curve = path_curve.DuplicateCurve();
    ON_Curve* srf_base_curve = base_edge.DuplicateCurve();
    if ( !bRev )
      srf_base_curve->Reverse();
    ON_3dPoint sum_basepoint = -ON_3dVector(srf_path_curve->PointAtStart());
    sum_srf = new ON_SumSurface();
    sum_srf->m_curve[0] = srf_base_curve;
    sum_srf->m_curve[1] = srf_path_curve;
    sum_srf->m_basepoint = sum_basepoint;
    sum_srf->BoundingBox(); // fills in sum_srf->m_bbox
  }
  return sum_srf;
}

static
ON_NurbsSurface* ON_BrepExtrudeHelper_MakeConeSrf( const ON_3dPoint& apex_point,
                                                 const ON_BrepEdge& edge, ON_BOOL32 bRev )
{
  // The "s" parameter runs along the edge.
  // The "t" parameter is the ruling parameter;
  // t=0 is at the base_edge and t=max is at the apex.
  //   surface side    location
  //     south           base_edge
  //     east            line from bRev?START:END of edge to apex
  //     north           singular side at apex
  //     west            line from bRev?END:START of edge to apex.
  ON_NurbsSurface* cone_srf = new ON_NurbsSurface();
  if ( cone_srf->CreateConeSurface( apex_point, edge ) )
  {
    if ( bRev )
      cone_srf->Reverse(0);
    // get a decent interval for the ruling parameter
    double d = 0.0;
    ON_Interval edom = edge.Domain();
    ON_3dPoint pt;
    int i, hint=0;
    for ( i = 0; i <= 16; i++ )
    {
      if ( !edge.EvPoint( edom.ParameterAt(i/16.0), pt, 0, &hint ) )
        continue;
      if ( pt.DistanceTo(apex_point) > d )
        d = pt.DistanceTo(apex_point);
    }
    if ( d > ON_SQRT_EPSILON )
      cone_srf->SetDomain(1,0.0,d);
  }
  else
  {
    delete cone_srf;
    cone_srf = 0;
  }
  return cone_srf;
}

static
ON_BOOL32 ON_BrepExtrudeHelper_MakeSides(
          ON_Brep& brep,
          int loop_index,
          const ON_Curve& path_curve,
          ON_BOOL32 bCap,
          ON_SimpleArray<int>& side_face_index
          )
{
  int lti, ti, i, vid[4], eid[4], bRev3d[4];

  // indices of new faces appended to the side_face_index[] array 
  // (1 face index for each trim, -1 is used for singular trims)

  // count number of new objects so we can grow arrays
  // efficiently and use refs to dynamic array elements.
  const int loop_trim_count = brep.m_L[loop_index].m_ti.Count();
  if ( loop_trim_count == 0 )
    return false;

  // save input trim and edge counts for use below
  const int trim_count0 = brep.m_T.Count();
  const int edge_count0 = brep.m_E.Count();

  ON_BrepExtrudeHelper_ReserveSpace( brep, loop_trim_count, bCap?1:0 );

  side_face_index.Reserve( side_face_index.Count() + loop_trim_count); // index of new face above brep.m_L[loop_index].m_ti[lti]
  int prev_face_index = -1;
  int first_face_east_trim_index = -1;

  for ( lti = 0; lti < loop_trim_count; lti++ )
  {
    ON_SumSurface* sum_srf = 0;
    side_face_index.Append(-1);
    ti = brep.m_L[loop_index].m_ti[lti];
    if ( ti < 0 || ti >= trim_count0 )
      continue;

    for ( i = 0; i < 4; i++ )
    {
      vid[i] = -1;
      eid[i] = -1;
    }
    bRev3d[0] = false;
    bRev3d[1] = false;
    bRev3d[2] = false;
    bRev3d[3] = false;

    // get side surface for new face
    {
      ON_BrepTrim& trim = brep.m_T[ti];
      if ( trim.m_ei >= 0 &&  trim.m_ei < edge_count0 )
      {
        const ON_BrepEdge& base_edge = brep.m_E[trim.m_ei];

        // 5 September, 2003 Dale Lear
        //   do not extrude seams - fixes rectangle slabe bug
        if ( trim.m_type == ON_BrepTrim::seam )
        {
          prev_face_index = -1;
          continue;
        }

        // connect new face to existing topology on trim
        vid[0] = trim.m_vi[1];
        vid[1] = trim.m_vi[0];
        eid[0] = base_edge.m_edge_index;
        bRev3d[0] = (trim.m_bRev3d?false:true);
        sum_srf = ON_BrepExtrudeHelper_MakeSumSrf( path_curve, base_edge, trim.m_bRev3d );
      }
    }
    if ( !sum_srf )
      continue;

    if ( prev_face_index >= 0 )
    {
      const ON_BrepTrim& prev_west_trim = brep.m_T[ brep.m_L[ brep.m_F[prev_face_index].m_li[0]].m_ti[3] ];
      vid[2] = prev_west_trim.m_vi[0];
      eid[1] = prev_west_trim.m_ei;
      bRev3d[1] = (prev_west_trim.m_bRev3d?false:true);
    }
    if ( first_face_east_trim_index >= 0 && brep.m_T[first_face_east_trim_index].m_vi[0] == vid[0] )
    {
      const ON_BrepTrim& first_face_east_trim = brep.m_T[first_face_east_trim_index];
      vid[3] = first_face_east_trim.m_vi[1];
      eid[3] = first_face_east_trim.m_ei;
      bRev3d[3] = (first_face_east_trim.m_bRev3d?false:true);
    }
    const ON_BrepFace* side_face = brep.NewFace(sum_srf,vid,eid,bRev3d);
    if ( side_face )
    {
      *side_face_index.Last() = side_face->m_face_index;
      prev_face_index = side_face->m_face_index;
      if ( first_face_east_trim_index < 0 )
        first_face_east_trim_index = brep.m_L[ side_face->m_li[0] ].m_ti[1];
    }
  }

  return true;
}

static
bool ON_BrepExtrudeHelper_CheckPathCurve( const ON_Curve& path_curve, ON_3dVector& path_vector )
{
  ON_Line path_line;
  path_line.from = path_curve.PointAtStart();
  path_line.to = path_curve.PointAtEnd();
  path_vector = path_line.Direction();
  return ( path_vector.IsZero() ? false : true );
}

static 
bool ON_BrepExtrudeHelper_MakeTopLoop( 
          ON_Brep& brep, 
          ON_BrepFace& top_face,
          int bottom_loop_index,
          const ON_3dVector path_vector,
          const int* side_face_index // array of brep.m_L[bottom_loop_index].m_ti.Count() face indices
          )
{
  bool rc = true;

  int lti, top_trim_index, i;
  if ( bottom_loop_index < 0 || bottom_loop_index >= brep.m_L.Count() )
    return false;
  ON_BrepLoop::TYPE loop_type = brep.m_L[bottom_loop_index].m_type;
  if ( loop_type != ON_BrepLoop::inner )
    loop_type = ON_BrepLoop::outer;
  ON_BrepLoop& top_loop = brep.NewLoop( loop_type, top_face );
  const ON_BrepLoop& bottom_loop = brep.m_L[bottom_loop_index];
  const int loop_trim_count = bottom_loop.m_ti.Count();
  brep.m_T.Reserve( brep.m_T.Count() + loop_trim_count );

  // Set top_vertex_index[lti] = index of vertex above 
  // vertex brep.m_V[brep.m_T[bottom_loop.m_ti[lti]].m_vi[0]].
  // Set top_vertex_index[lti] = index of edge above 
  // edge of brep.m_T[bottom_loop.m_ti[lti]].
  // This informtion is needed for singular and seam trims.
  ON_SimpleArray<int> top_vertex_index(loop_trim_count);
  ON_SimpleArray<int> top_edge_index(loop_trim_count);
  ON_SimpleArray<bool> top_trim_bRev3d(loop_trim_count);
  for ( lti = 0; lti < loop_trim_count; lti++ )
  {
    top_vertex_index.Append(-1);
    top_edge_index.Append(-1);
    top_trim_bRev3d.Append(false);
  }

  // some (often all of) of the "top" vertices are already on
  // the side faces
  for ( lti = 0; lti < loop_trim_count; lti++ )
  {
    if ( side_face_index[lti] >= 0 )
    {
      const ON_BrepFace& side_face = brep.m_F[side_face_index[lti]];
      const ON_BrepLoop& side_loop = brep.m_L[side_face.m_li[0]];
      const ON_BrepTrim& side_north_trim = brep.m_T[side_loop.m_ti[2]];
      top_vertex_index[lti] = side_north_trim.m_vi[0];
      top_vertex_index[(lti+1)%loop_trim_count] = side_north_trim.m_vi[1];
      top_edge_index[lti] = side_north_trim.m_ei;
    }
    else 
    {
      // fix for RR 20423
      int lti_prev = (lti+loop_trim_count-1)%loop_trim_count;
      int lti_next = (lti+1)%loop_trim_count;
      if (   side_face_index[lti_prev] < 0 
           && side_face_index[lti_next] < 0 
           && top_vertex_index[lti] < 0
           && top_vertex_index[lti_next] < 0
           )
      {
        int bottom_ti_prev = bottom_loop.m_ti[lti_prev];
        int bottom_ti      = bottom_loop.m_ti[lti];
        int bottom_ti_next = bottom_loop.m_ti[lti_next];
        if (    bottom_ti >= 0      && bottom_ti < brep.m_T.Count() 
             && bottom_ti_prev >= 0 && bottom_ti_prev < brep.m_T.Count() 
             && bottom_ti_next >= 0 && bottom_ti_next < brep.m_T.Count() 
           )
        {
          const ON_BrepTrim& bottom_trim_prev = brep.m_T[bottom_ti_prev];
          const ON_BrepTrim& bottom_trim = brep.m_T[bottom_ti];
          const ON_BrepTrim& bottom_trim_next = brep.m_T[bottom_ti_next];
          if (    ON_BrepTrim::seam == bottom_trim_prev.m_type
               && ON_BrepTrim::singular == bottom_trim.m_type
               && ON_BrepTrim::seam == bottom_trim_next.m_type 
               && bottom_trim.m_vi[0] == bottom_trim.m_vi[1]
               )
          {
            int vi = bottom_trim.m_vi[0];
            if ( vi >= 0 && vi < brep.m_V.Count() )
            {
              ON_BrepVertex& top_vertex = brep.NewVertex(brep.m_V[vi].point+path_vector,0.0);
              top_vertex_index[lti] = top_vertex.m_vertex_index;
              top_vertex_index[lti_next] = top_vertex_index[lti];
            }
          }
        }
      }
    }
  }

  // Fill in the missing "top" vertices that
  // are associated with singular and trim edges by looking
  // at their neighbors.
  {
    bool bKeepChecking = true;
    while( bKeepChecking )
    {
      // set back to true if we make a change.  This handles the
      // (very rare) cases of multiple adjacent singular trims.
      bKeepChecking = false; 

      for ( lti = 0; lti < loop_trim_count; lti++ )
      {
        if ( top_vertex_index[lti] == -1 )
        {
          for ( i = lti+1; i < loop_trim_count; i++ )
          {
            if ( ON_BrepTrim::singular != brep.m_T[bottom_loop.m_ti[i-1]].m_type )
              break;
            if ( top_vertex_index[i] >= 0 )
            {
              top_vertex_index[lti] = top_vertex_index[i];
              bKeepChecking = true; 
              break;
            }
          }
        }

        if ( top_vertex_index[lti] == -1 )
        {
          for ( i = lti-1; i >= 0; i-- )
          {
            if ( ON_BrepTrim::singular != brep.m_T[bottom_loop.m_ti[i+1]].m_type )
              break;
            if ( top_vertex_index[i] >= 0 )
            {
              top_vertex_index[lti] = top_vertex_index[i];
              bKeepChecking = true; 
              break;
            }
          }
        }
      }
    }
  }

  // Fill in missing edges of "seam" trims.
  for ( lti = 0; lti < loop_trim_count; lti++ )
  {
    if ( -1 != top_edge_index[lti] )
      continue;
    int bottom_ti = bottom_loop.m_ti[lti];
    if ( bottom_ti < 0 || bottom_ti >= brep.m_T.Count() )
      continue;
    const ON_BrepTrim& bottom_trim = brep.m_T[bottom_ti];
    if ( ON_BrepTrim::seam != bottom_trim.m_type )
      continue;
    if ( bottom_trim.m_ei < 0 )
      continue;
    if ( bottom_trim.m_ei >= brep.m_E.Count() )
      continue;

    // duplicate bottom edge curve
    const ON_BrepEdge& bottom_edge = brep.m_E[bottom_trim.m_ei];
    ON_Curve* top_c3 = bottom_edge.DuplicateCurve();
    if ( 0 == top_c3 )
      continue;
    // move new edge curve to top location
    top_c3->Translate(path_vector);
    ON_3dPoint P0 = top_c3->PointAtStart();
    ON_3dPoint P1 = top_c3->PointAtEnd();
    int top_c3i = brep.AddEdgeCurve(top_c3);
    top_c3 = 0;
    // get vertices at start/end of the new edge
    int e_vi0 = top_vertex_index[lti];
    int e_vi1 = top_vertex_index[(lti+1)%loop_trim_count];
    if ( bottom_trim.m_bRev3d )
    {
      // put points in trim order
      ON_3dPoint tmp_P = P0; P0 = P1; P1 = tmp_P;
    }
    if ( e_vi0 < 0 )
    {
      e_vi0 = brep.NewVertex(P0).m_vertex_index;
      top_vertex_index[lti] = e_vi0;
    }
    if ( e_vi1 < 0 )
    {
      e_vi1 = brep.NewVertex(P1).m_vertex_index;
      top_vertex_index[(lti+1)%loop_trim_count] = e_vi1;
    }
    if ( bottom_trim.m_bRev3d )
    {
      // put edge vertex indices in edge order
      int tmp_i = e_vi0; e_vi0 = e_vi1; e_vi1 = tmp_i;
    }
    ON_BrepEdge& top_edge = brep.NewEdge(brep.m_V[e_vi0],brep.m_V[e_vi1],top_c3i);
    top_edge.m_tolerance = bottom_edge.m_tolerance;
    top_edge_index[lti] = top_edge.m_edge_index;
    top_trim_bRev3d[lti] = bottom_trim.m_bRev3d?true:false;

    // find seam mate and set it's 
    // top_edge_index[] to top_edge.m_edge_index.
    int mate_lti;
    for( mate_lti = lti+1; mate_lti < loop_trim_count; mate_lti++ )
    {
      if ( top_edge_index[mate_lti] != -1  )
        continue;
      int bottom_mate_ti = bottom_loop.m_ti[mate_lti];
      if ( bottom_mate_ti < 0 || bottom_mate_ti >= brep.m_T.Count() )
        continue;
      const ON_BrepTrim& bottom_mate_trim = brep.m_T[bottom_mate_ti];
      if ( bottom_mate_trim.m_type != ON_BrepTrim::seam )
        continue;
      if ( bottom_mate_trim.m_ei != bottom_trim.m_ei )
        continue;
      top_edge_index[mate_lti] = top_edge.m_edge_index;
      top_trim_bRev3d[mate_lti] = bottom_mate_trim.m_bRev3d?true:false;
      break;
    }
  }


  for ( lti = 0; lti < loop_trim_count; lti++ )
  {
    const ON_BrepTrim& bottom_trim = brep.m_T[ bottom_loop.m_ti[lti] ];
    ON_Curve* top_c2 = bottom_trim.DuplicateCurve();
    int top_c2i = (0!=top_c2) ? brep.AddTrimCurve(top_c2) : bottom_trim.m_c2i;
    top_trim_index = -1;
    if ( bottom_trim.m_type == ON_BrepTrim::singular && top_vertex_index[lti] >= 0 )
    {
      top_trim_index = brep.NewSingularTrim(brep.m_V[top_vertex_index[lti]], top_loop, bottom_trim.m_iso, top_c2i ).m_trim_index;
    }
    else if ( bottom_trim.m_type != ON_BrepTrim::singular && top_edge_index[lti] >= 0 && top_edge_index[lti] < brep.m_E.Count() )
    {
      ON_BrepEdge& top_edge = brep.m_E[top_edge_index[lti]];
      top_trim_index = brep.NewTrim( top_edge, top_trim_bRev3d[lti], top_loop, top_c2i ).m_trim_index;
    }
    else
    {
      ON_ERROR("ON_BrepExtrudeHelper_MakeTopLoop ran into capping trouble.");
      rc = false;
      break;
    }
    ON_BrepTrim& top_trim = brep.m_T[top_trim_index];
    top_trim.m_pline = bottom_trim.m_pline;
    top_trim.m_pbox = bottom_trim.m_pbox;
    top_trim.m_iso = bottom_trim.m_iso;
    top_trim.m_type = bottom_trim.m_type;
    top_trim.m_tolerance[0] = bottom_trim.m_tolerance[0];
    top_trim.m_tolerance[1] = bottom_trim.m_tolerance[1];
    top_trim.m__legacy_2d_tol = bottom_trim.m__legacy_2d_tol;
    top_trim.m__legacy_3d_tol = bottom_trim.m__legacy_2d_tol;
    top_trim.m__legacy_flags = bottom_trim.m__legacy_flags;
  }
  if (rc)
  {
    top_loop.m_pbox = bottom_loop.m_pbox;
  }
  return rc;
}

static          
bool ON_BrepExtrudeHelper_CheckLoop( const ON_Brep& brep, int loop_index )
{
  bool rc = false;
  if ( loop_index >= 0 )
  {
    ON_BrepLoop::TYPE loop_type = brep.m_L[loop_index].m_type;
    if ( loop_type == ON_BrepLoop::inner || loop_type == ON_BrepLoop::outer )
      rc = true;
  }
  return rc;
}

static
bool ON_BrepExtrudeHelper_MakeCap(
          ON_Brep& brep,
          int bottom_loop_index,
          const ON_3dVector path_vector,
          const int* side_face_index
          )
{
  bool bCap = true;
  // make cap
  if ( !ON_BrepExtrudeHelper_CheckLoop( brep, bottom_loop_index ) )
    return false;
  brep.m_F.Reserve(brep.m_F.Count() + 1);
  brep.m_L.Reserve(brep.m_L.Count() + 1);
  const ON_BrepLoop& bottom_loop = brep.m_L[bottom_loop_index];
  const ON_BrepFace& bottom_face = brep.m_F[bottom_loop.m_fi];
  const ON_Surface* bottom_surface = bottom_face.SurfaceOf();
  ON_Surface* top_surface = bottom_surface->Duplicate();
  top_surface->Translate( path_vector );
  int top_surface_index = brep.AddSurface( top_surface );
  ON_BrepFace& top_face = brep.NewFace( top_surface_index );

  bCap = ON_BrepExtrudeHelper_MakeTopLoop( brep, top_face, bottom_loop_index, path_vector, side_face_index );
  if ( bCap )
  {
    ON_BrepLoop& top_loop = brep.m_L[brep.m_L.Count()-1];
    if ( bottom_loop.m_type == ON_BrepLoop::inner )
    {
      // we capped an inner boundary
      // top_loop.m_type = ON_BrepLoop::outer; // done in ON_BrepExtrudeHelper_MakeTopLoop
      brep.FlipLoop(top_loop);
    }
    else if ( bottom_loop.m_type == ON_BrepLoop::outer )
    {
      // we capped an outer boundary
      // top_loop.m_type = ON_BrepLoop::outer; // done in ON_BrepExtrudeHelper_MakeTopLoop
      brep.FlipFace(top_face);
    }
  }
  else
  {
    // delete partially made cap face
    brep.DeleteFace( top_face, false );
    delete brep.m_S[top_surface_index];
    brep.m_S[top_surface_index] = 0;
  }
  return bCap;
}



int ON_BrepExtrudeFace( 
          ON_Brep& brep,
          int face_index,
          const ON_Curve& path_curve,
          bool bCap
          )
{
  int rc = 0; // returns 1 for success with no cap, 2 for success with a cap

  if ( face_index < 0 || face_index >= brep.m_F.Count() )
    return false;

  const int face_loop_count = brep.m_F[face_index].m_li.Count();
  if ( face_loop_count < 1 )
    return false;


  if ( brep.m_F[face_index].m_li.Count() == 1 )
  {
    rc = ON_BrepExtrudeLoop( brep, brep.m_F[face_index].m_li[0], path_curve, bCap );
  }
  else
  {
    ON_3dVector path_vector;
    ON_SimpleArray<int> side_face_index;
    ON_SimpleArray<int> side_face_index_loop_mark;
    int li, fli;

    if ( !ON_BrepExtrudeHelper_CheckPathCurve( path_curve, path_vector ) )
      return 0;

    //const int trim_count0 = brep.m_T.Count();
    const int loop_count0 = brep.m_L.Count();
    const int face_count0 = brep.m_F.Count();

    // count number of new objects so we can grow arrays
    // efficiently and use refs to dynamic array elements.
    int new_side_trim_count = 0;
    for ( fli = 0; fli < face_loop_count; fli++ )
    {
      li = brep.m_F[face_index].m_li[fli];
      if ( li < 0 || li >= loop_count0 )
        return false;
      if ( !ON_BrepExtrudeHelper_CheckLoop( brep, li ) )
        continue;
      new_side_trim_count += brep.m_L[li].m_ti.Count();
    }
    if ( new_side_trim_count == 0 )
      return false;
    ON_BrepExtrudeHelper_ReserveSpace( brep, new_side_trim_count, bCap?1:0 );

    side_face_index.Reserve(new_side_trim_count);
    side_face_index_loop_mark.Reserve(face_loop_count);

    const ON_BrepFace& face = brep.m_F[face_index];
  
    rc = true;
    int outer_loop_index = -1;
    int outer_fli = -1;
    for ( fli = 0; fli < face_loop_count && rc; fli++ )
    {
      side_face_index_loop_mark.Append( side_face_index.Count() );
      li = face.m_li[fli];
      if ( !ON_BrepExtrudeHelper_CheckLoop( brep, li ) )
        continue;
      ON_BrepLoop& loop = brep.m_L[li];
      if ( bCap && loop.m_type == ON_BrepLoop::outer )
      {
        if ( outer_loop_index >= 0 )
          bCap = false;
        else 
        {
          outer_loop_index = li;
          outer_fli = fli;
        }
      }
      rc = ON_BrepExtrudeHelper_MakeSides( brep, li, path_curve, bCap, side_face_index );
    }

    if ( bCap && rc && outer_loop_index >= 0 )
    {
      const int face_count1 = brep.m_F.Count();
      bCap = ON_BrepExtrudeHelper_MakeCap( 
                  brep, 
                  outer_loop_index, 
                  path_vector, 
                  side_face_index.Array() + side_face_index_loop_mark[outer_fli] );
      if ( bCap && brep.m_F.Count() > face_count1)
      {
        // put inner bondaries on the cap
        rc = 2;

        ON_BrepFace& cap_face = brep.m_F[brep.m_F.Count()-1];
        for ( fli = 0; fli < face_loop_count && rc; fli++ )
        {
          li = face.m_li[fli];
          if ( li == outer_loop_index )
            continue;
          if ( !ON_BrepExtrudeHelper_CheckLoop( brep, li ) )
            continue;
          if ( ON_BrepExtrudeHelper_MakeTopLoop( 
                      brep, 
                      cap_face, 
                      li, 
                      path_vector,
                      side_face_index.Array() + side_face_index_loop_mark[fli] ) )
          {
            ON_BrepLoop& top_loop = brep.m_L[brep.m_L.Count()-1];
            top_loop.m_type = brep.m_L[li].m_type;
          }
        }
      }
    }

    if ( brep.m_F[face_index].m_bRev )
    {
      for ( int fi = face_count0; fi < brep.m_F.Count(); fi++ )
      {
        brep.FlipFace(brep.m_F[fi]);
      }
    }
  }

  return rc;
}


int ON_BrepExtrudeLoop( 
          ON_Brep& brep,
          int loop_index,
          const ON_Curve& path_curve,
          bool bCap
          )
{
  ON_SimpleArray<int> side_face_index; // index of new face above brep.m_L[loop_index].m_ti[lti]
  ON_3dVector path_vector;

  const int face_count0 = brep.m_F.Count();

  if ( loop_index < 0 || loop_index >= brep.m_L.Count() )
    return false;

  if ( !ON_BrepExtrudeHelper_CheckPathCurve(path_curve,path_vector) )
    return false;

  // can only cap closed loops ( for now, just test for inner and outer loops).
  if ( brep.m_L[loop_index].m_type != ON_BrepLoop::outer && brep.m_L[loop_index].m_type != ON_BrepLoop::inner )
    bCap = false;

  // make sides
  if ( !ON_BrepExtrudeHelper_MakeSides( brep, loop_index, path_curve, bCap, side_face_index ) )
    return false;

  // make cap
  if ( bCap )
    bCap = ON_BrepExtrudeHelper_MakeCap( brep, loop_index, path_vector, side_face_index.Array() );

  const ON_BrepLoop& loop = brep.m_L[loop_index];
  if ( loop.m_fi >= 0 && loop.m_fi < brep.m_F.Count() && brep.m_F[loop.m_fi].m_bRev )
  {
    for ( int fi = face_count0; fi < brep.m_F.Count(); fi++ )
    {
      brep.FlipFace( brep.m_F[fi] );
    }
  }

  return (bCap?2:1);
}



int ON_BrepExtrudeEdge( 
          ON_Brep& brep,
          int edge_index,
          const ON_Curve& path_curve
          )
{
  ON_3dVector path_vector;

  if ( edge_index < 0 && edge_index >= brep.m_E.Count() )
    return false;

  if ( !ON_BrepExtrudeHelper_CheckPathCurve(path_curve,path_vector) )
    return false;


  // make sides
  bool bRev = false;
  ON_SumSurface* sum_srf = ON_BrepExtrudeHelper_MakeSumSrf( 
                              path_curve, brep.m_E[edge_index], bRev );

  if ( !sum_srf )
    return false;

  int vid[4], eid[4], bRev3d[4];

  vid[0] = brep.m_E[edge_index].m_vi[bRev?0:1];
  vid[1] = brep.m_E[edge_index].m_vi[bRev?1:0];
  vid[2] = -1;
  vid[3] = -1;

  eid[0] = edge_index; // "south side edge"
  eid[1] = -1;
  eid[2] = -1;
  eid[3] = -1;

  bRev3d[0] = bRev?0:1;
  bRev3d[1] = 0;
  bRev3d[2] = 0;
  bRev3d[3] = 0;

  return brep.NewFace( sum_srf, vid, eid, bRev3d ) ? true : false;
}


bool ON_BrepExtrude( 
          ON_Brep& brep,
          const ON_Curve& path_curve,
          bool bCap
          )
{
  ON_Workspace ws;
  const int vcount0 = brep.m_V.Count();
  const int tcount0 = brep.m_T.Count();
  const int lcount0 = brep.m_L.Count();
  const int ecount0 = brep.m_E.Count();
  const int fcount0 = brep.m_F.Count();

  const ON_3dPoint PathStart = path_curve.PointAtStart();
  ON_3dPoint P = path_curve.PointAtEnd();
  if ( !PathStart.IsValid() || !P.IsValid() )
    return false;
  const ON_3dVector height = P - PathStart;
  if ( !height.IsValid() || height.Length() <= ON_ZERO_TOLERANCE )
    return false;

  ON_Xform tr;
  tr.Translation(height);

  // count number of new sides
  int side_count = 0;
  int i, vi, ei, fi;
  bool* bSideEdge = (bool*)ws.GetIntMemory(ecount0*sizeof(bSideEdge[0]));
  for ( ei = 0; ei < ecount0; ei++ )
  {
    const ON_BrepEdge& e = brep.m_E[ei];
    if ( 1 == e.m_ti.Count() )
    {
      side_count++;
      bSideEdge[ei] = true;
    }
    else
    {
      bSideEdge[ei] = false;
    }
  }

  brep.m_V.Reserve( 2*vcount0 );
  i = 4*side_count + (bCap?tcount0:0);
  brep.m_T.Reserve( tcount0 + i );
  brep.m_C2.Reserve( brep.m_C2.Count() + i );
  brep.m_L.Reserve( lcount0 + side_count + (bCap?lcount0:0) );
  i = side_count + (bCap?ecount0:side_count);
  brep.m_E.Reserve( ecount0 + i );
  brep.m_C3.Reserve( brep.m_C3.Count() + i );
  i = side_count + (bCap?fcount0:0);
  brep.m_F.Reserve( fcount0 + i );
  brep.m_S.Reserve( brep.m_S.Count() + i );

  bool bOK = true;

  // build top vertices
  int* topvimap = ws.GetIntMemory(vcount0);
  memset(topvimap,0,vcount0*sizeof(topvimap[0]));
  if ( bCap )
  {
    for ( vi = 0; vi < vcount0; vi++ )
    {
      const ON_BrepVertex& bottomv = brep.m_V[vi];
      ON_BrepVertex& topv = brep.NewVertex(bottomv.point+height,bottomv.m_tolerance);
      topvimap[vi] = topv.m_vertex_index;
    }
  }
  else
  {
    for ( ei = 0; ei < ecount0; ei++ )
    {
      if ( bSideEdge[ei] )
      {
        const ON_BrepEdge& bottome = brep.m_E[ei];
        int bottomvi0 = bottome.m_vi[0];
        if ( bottomvi0 < 0 || bottomvi0 >= vcount0 )
        {
          bOK = false;
          break;
        }
        int bottomvi1 = bottome.m_vi[1];
        if ( bottomvi1 < 0 || bottomvi1 >= vcount0 )
        {
          bOK = false;
          break;
        }
        if ( !topvimap[bottomvi0] )
        {
          const ON_BrepVertex& bottomv = brep.m_V[bottomvi0];
          ON_BrepVertex& topv = brep.NewVertex(bottomv.point+height,bottomv.m_tolerance);
          topvimap[bottomvi0] = topv.m_vertex_index;
        }
        if ( !topvimap[bottomvi1] )
        {
          const ON_BrepVertex& bottomv = brep.m_V[bottomvi1];
          ON_BrepVertex& topv = brep.NewVertex(bottomv.point+height,bottomv.m_tolerance);
          topvimap[bottomvi1] = topv.m_vertex_index;
        }
      }
    }
  }

  // build top edges
  int* topeimap = ws.GetIntMemory(ecount0);
  memset(topeimap,0,ecount0*sizeof(topeimap[0]));
  if ( bOK ) for ( ei = 0; ei < ecount0; ei++ )
  {
    if ( bCap || bSideEdge[ei] )
    {
      const ON_BrepEdge& bottome = brep.m_E[ei];
      ON_BrepVertex& topv0 = brep.m_V[topvimap[bottome.m_vi[0]]];
      ON_BrepVertex& topv1 = brep.m_V[topvimap[bottome.m_vi[1]]];
      ON_Curve* c3 = bottome.DuplicateCurve();
      if ( !c3 )
      {
        bOK = false;
        break;
      }
      c3->Transform(tr);
      int c3i = brep.AddEdgeCurve(c3);
      ON_BrepEdge& tope = brep.NewEdge(topv0,topv1,c3i,0,bottome.m_tolerance);
      topeimap[ei] = tope.m_edge_index;
    }
  }

  // build side edges
  int* sideveimap = ws.GetIntMemory(vcount0);
  memset(sideveimap,0,vcount0*sizeof(sideveimap[0]));
  if ( bOK ) for ( vi = 0; vi < vcount0; vi++ )
  {
    ON_BrepVertex& bottomv = brep.m_V[vi];
    for ( int vei = 0; vei < bottomv.m_ei.Count(); vei++ )
    {
      if ( bSideEdge[bottomv.m_ei[vei]] && topvimap[vi] )
      {
        ON_BrepVertex& topv = brep.m_V[topvimap[vi]];
        ON_Curve* c3 = path_curve.DuplicateCurve();
        if ( !c3 )
        {
          bOK = false;
        }
        else
        {
          ON_3dVector D = bottomv.point - PathStart;
          c3->Translate(D);
          int c3i = brep.AddEdgeCurve(c3);
          const ON_BrepEdge& e = brep.NewEdge(bottomv,topv,c3i,0,0.0);
          sideveimap[vi] = e.m_edge_index;
        }
        break;
      }
    }
  }

  if ( bOK && bCap )
  {
    // build top faces
    for (fi = 0; fi < fcount0; fi++ )
    {
      const ON_BrepFace& bottomf = brep.m_F[fi];
      ON_Surface* srf = bottomf.DuplicateSurface();
      if ( !srf )
      {
        bOK = false;
        break;
      }
      srf->Transform(tr);
      int si = brep.AddSurface(srf);
      ON_BrepFace& topf = brep.NewFace(si);
      topf.m_bRev = !bottomf.m_bRev;
      const int loop_count = bottomf.m_li.Count();
      topf.m_li.Reserve(loop_count);
      for ( int fli = 0; fli < loop_count; fli++ )
      {
        const ON_BrepLoop& bottoml = brep.m_L[bottomf.m_li[fli]];
        ON_BrepLoop& topl = brep.NewLoop(bottoml.m_type,topf);
        const int loop_trim_count = bottoml.m_ti.Count();
        topl.m_ti.Reserve(loop_trim_count);
        for ( int lti = 0; lti < loop_trim_count; lti++ )
        {
          const ON_BrepTrim& bottomt = brep.m_T[bottoml.m_ti[lti]];
          ON_NurbsCurve* c2 = ON_NurbsCurve::New();
          if ( !bottomt.GetNurbForm(*c2) )
          {
            delete c2;
            bOK = false;
            break;
          }
          int c2i = brep.AddTrimCurve(c2);
          ON_BrepTrim* topt = 0;
          if ( bottomt.m_ei >= 0 )
          {
            ON_BrepEdge& tope = brep.m_E[topeimap[bottomt.m_ei]];
            topt = &brep.NewTrim(tope,bottomt.m_bRev3d,topl,c2i);
          }
          else
          {
            // singular trim
            ON_BrepVertex& topv = brep.m_V[topvimap[bottomt.m_vi[0]]];
            topt = &brep.NewSingularTrim(topv,topl,bottomt.m_iso,c2i);
          }
          topt->m_tolerance[0] = bottomt.m_tolerance[0];
          topt->m_tolerance[1] = bottomt.m_tolerance[1];
          topt->m_pbox = bottomt.m_pbox;
          topt->m_type = bottomt.m_type;
          topt->m_iso = bottomt.m_iso;
        }
        topl.m_pbox = bottoml.m_pbox;
      }
    }
  }

  // build sides
  int bRev3d[4] = {0,0,1,1};
  int vid[4], eid[4];
  if( bOK ) for ( ei = 0; ei < ecount0; ei++ )
  {
    if ( bSideEdge[ei] && topeimap[ei] )
    {
      ON_BrepEdge& bottome = brep.m_E[ei];
      ON_BrepEdge& tope = brep.m_E[topeimap[ei]];
      vid[0] = bottome.m_vi[0];
      vid[1] = bottome.m_vi[1];
      vid[2] = topvimap[vid[1]];
      vid[3] = topvimap[vid[0]];
      if ( sideveimap[vid[0]] && sideveimap[vid[1]] )
      {
        ON_BrepEdge& leftedge = brep.m_E[sideveimap[vid[0]]];
        ON_BrepEdge& rightedge = brep.m_E[sideveimap[vid[1]]];
        ON_Curve* cx = bottome.DuplicateCurve();
        if ( !cx )
        {
          bOK = false;
          break;
        }
        ON_Curve* cy = leftedge.DuplicateCurve();
        if ( !cy )
        {
          delete cx;
          bOK = false;
          break;
        }
        ON_SumSurface* srf = new ON_SumSurface();
        srf->m_curve[0] = cx;
        srf->m_curve[1] = cy;
        srf->m_basepoint = srf->m_curve[1]->PointAtStart();
        srf->m_basepoint.x = -srf->m_basepoint.x;
        srf->m_basepoint.y = -srf->m_basepoint.y;
        srf->m_basepoint.z = -srf->m_basepoint.z;
        eid[0] = bottome.m_edge_index;
        eid[1] = rightedge.m_edge_index;
        eid[2] = tope.m_edge_index;
        eid[3] = leftedge.m_edge_index;
        ON_BrepFace* face = brep.NewFace(srf,vid,eid,bRev3d);
        if ( !face )
        {
          bOK = false;
          break;
        }
        else if ( bottome.m_ti.Count() == 2 )
        {
          const ON_BrepTrim& trim0 = brep.m_T[bottome.m_ti[0]];
          const ON_BrepTrim& trim1 = brep.m_T[bottome.m_ti[1]];
          const ON_BrepLoop& loop0 = brep.m_L[trim0.m_li];
          const ON_BrepLoop& loop1 = brep.m_L[trim1.m_li];
          bool bBottomFaceRev = brep.m_F[(loop0.m_fi != face->m_face_index) ? loop0.m_fi : loop1.m_fi].m_bRev;
          bool bSideFaceRev = ( trim0.m_bRev3d != trim1.m_bRev3d ) 
                            ? bBottomFaceRev 
                            : !bBottomFaceRev;
          face->m_bRev = bSideFaceRev;
        }
      }
    }
  }

  if ( !bOK )
  {
    for ( vi = brep.m_V.Count(); vi >= vcount0; vi-- )
    {
      brep.DeleteVertex(brep.m_V[vi]);
    }
  }

  return bOK;
}



int ON_BrepExtrudeVertex( 
          ON_Brep& brep,
          int vertex_index,
          const ON_Curve& path_curve
          )
{
  ON_3dVector path_vector;
  if ( vertex_index < 0 && vertex_index >= brep.m_V.Count() )
    return false;
  if ( !ON_BrepExtrudeHelper_CheckPathCurve(path_curve,path_vector) )
    return false;
  ON_Curve* c3 = path_curve.Duplicate();
  brep.m_V.Reserve( brep.m_V.Count() + 1 );
  ON_BrepVertex& v0 = brep.m_V[vertex_index];
  ON_BrepVertex& v1 = brep.NewVertex( v0.point + path_vector, 0.0 );
  c3->Translate( v0.point - c3->PointAtStart() );
  int c3i = brep.AddEdgeCurve( c3 );
  ON_BrepEdge& edge = brep.NewEdge( v0, v1, c3i );
  edge.m_tolerance = 0.0;
  return true;
}


int ON_BrepConeFace( 
          ON_Brep& brep,
          int face_index,
          ON_3dPoint apex_point
          )
{
  int rc = 0; // returns 1 for success with no cap, 2 for success with a cap

  if ( face_index < 0 || face_index >= brep.m_F.Count() )
    return false;

  const int face_loop_count = brep.m_F[face_index].m_li.Count();
  if ( face_loop_count < 1 )
    return false;

  if ( brep.m_F[face_index].m_li.Count() == 1 )
  {
    rc = ON_BrepConeLoop( brep, brep.m_F[face_index].m_li[0], apex_point );
  }
  else
  {
    int li, fli;

    //const int trim_count0 = brep.m_T.Count();
    const int loop_count0 = brep.m_L.Count();
    //const int face_count0 = brep.m_F.Count();

    // count number of new objects so we can grow arrays
    // efficiently and use refs to dynamic array elements.
    int new_side_trim_count = 0;
    for ( fli = 0; fli < face_loop_count; fli++ )
    {
      li = brep.m_F[face_index].m_li[fli];
      if ( li < 0 || li >= loop_count0 )
        return false;
      if ( !ON_BrepExtrudeHelper_CheckLoop( brep, li ) )
        continue;
      new_side_trim_count += brep.m_L[li].m_ti.Count();
    }
    if ( new_side_trim_count == 0 )
      return false;
    ON_BrepExtrudeHelper_ReserveSpace( brep, new_side_trim_count, 0 );

    const ON_BrepFace& face = brep.m_F[face_index];

    //ON_BrepVertex& apex_vertex = 
    brep.NewVertex( apex_point, 0.0 );
  
    rc = true;
    for ( fli = 0; fli < face_loop_count && rc; fli++ )
    {
      li = face.m_li[fli];
      if ( !ON_BrepExtrudeHelper_CheckLoop( brep, li ) )
        continue;

      rc = ON_BrepConeLoop( brep, li, apex_point );
    }
  }

  return rc;
}


bool ON_BrepConeLoop( 
          ON_Brep& brep,
          int loop_index,
          ON_3dPoint apex_point
          )
{
  if ( loop_index < 0 && loop_index >= brep.m_L.Count() )
    return false;

  int lti, ti, i, vid[4], eid[4], bRev3d[4];

  // indices of new faces appended to the side_face_index[] array 
  // (1 face index for each trim, -1 is used for singular trims)

  // count number of new objects so we can grow arrays
  // efficiently and use refs to dynamic array elements.
  const int loop_trim_count = brep.m_L[loop_index].m_ti.Count();
  if ( loop_trim_count == 0 )
    return false;

  // save input trim and edge counts for use below
  const int trim_count0 = brep.m_T.Count();
  const int edge_count0 = brep.m_E.Count();

  ON_BrepExtrudeHelper_ReserveSpace( brep, loop_trim_count, 0 );

  int prev_face_index = -1;
  int first_face_east_trim_index = -1;

  ON_BrepVertex& apex_vertex = brep.NewVertex( apex_point, 0.0 );

  for ( lti = 0; lti < loop_trim_count; lti++ )
  {
    ON_NurbsSurface* cone_srf = 0;
    ti = brep.m_L[loop_index].m_ti[lti];
    if ( ti < 0 || ti >= trim_count0 )
      continue;

    for ( i = 0; i < 4; i++ )
    {
      vid[i] = -1;
      eid[i] = -1;
    }
    bRev3d[0] = false;
    bRev3d[1] = false;
    bRev3d[2] = false;
    bRev3d[3] = false;

    // get side surface for new face
    // get side surface for new face
    {
      ON_BrepTrim& trim = brep.m_T[ti];
      if ( trim.m_ei >= 0 &&  trim.m_ei < edge_count0 )
      {
        const ON_BrepEdge& base_edge = brep.m_E[trim.m_ei];

        // connect new face to existing topology on trim
        vid[0] = trim.m_vi[1];
        vid[1] = trim.m_vi[0];
        eid[0] = base_edge.m_edge_index;
        bRev3d[0] = (trim.m_bRev3d?false:true);
        cone_srf = ON_BrepExtrudeHelper_MakeConeSrf( apex_point, base_edge, bRev3d[0] );
      }
    }
    if ( !cone_srf )
      continue;
    vid[2] = apex_vertex.m_vertex_index;
    vid[3] = apex_vertex.m_vertex_index;

    if ( prev_face_index >= 0 )
    {
      const ON_BrepTrim& prev_west_trim = brep.m_T[ brep.m_L[ brep.m_F[prev_face_index].m_li[0]].m_ti[3] ];
      vid[2] = prev_west_trim.m_vi[0];
      eid[1] = prev_west_trim.m_ei;
      bRev3d[1] = (prev_west_trim.m_bRev3d?false:true);
    }
    if ( first_face_east_trim_index >= 0 && brep.m_T[first_face_east_trim_index].m_vi[0] == vid[0] )
    {
      const ON_BrepTrim& first_face_east_trim = brep.m_T[first_face_east_trim_index];
      vid[3] = first_face_east_trim.m_vi[1];
      eid[3] = first_face_east_trim.m_ei;
      bRev3d[3] = (first_face_east_trim.m_bRev3d?false:true);
    }
    const ON_BrepFace* side_face = brep.NewFace(cone_srf,vid,eid,bRev3d);
    if ( side_face )
    {
      prev_face_index = side_face->m_face_index;
      if ( first_face_east_trim_index < 0 )
        first_face_east_trim_index = brep.m_L[ side_face->m_li[0] ].m_ti[1];
    }
  }

  return true;
}



int ON_BrepConeEdge( 
          ON_Brep& brep,
          int edge_index,
          ON_3dPoint apex_point
          )
{
  //ON_3dVector path_vector;

  if ( edge_index < 0 && edge_index >= brep.m_E.Count() )
    return false;

  // make sides
  ON_NurbsSurface* cone_srf = ON_BrepExtrudeHelper_MakeConeSrf( 
                              apex_point, brep.m_E[edge_index], false );

  if ( !cone_srf )
    return false;

  int vid[4], eid[4], bRev3d[4];

  vid[0] = brep.m_E[edge_index].m_vi[0];
  vid[1] = brep.m_E[edge_index].m_vi[1];
  vid[2] = -1;
  vid[3] = -1;

  eid[0] = edge_index;
  eid[1] = -1;
  eid[2] = -1;
  eid[3] = -1;

  bRev3d[0] = 0;
  bRev3d[1] = 0;
  bRev3d[2] = 0;
  bRev3d[3] = 0;

  return brep.NewFace( cone_srf, vid, eid, bRev3d ) ? true : false;
}

