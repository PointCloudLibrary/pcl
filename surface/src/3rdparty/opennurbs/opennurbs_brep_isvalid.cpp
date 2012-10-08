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

bool
ON_Brep::IsValidVertexTopology( int vertex_index, ON_TextLog* text_log ) const
{
  if ( vertex_index < 0 || vertex_index >= m_V.Count() )
  {
    if ( text_log )
      text_log->Print("brep vertex_index = %d (should be >=0 and <%d=brep.m_V.Count() ).\n",
                      vertex_index, m_V.Count());
    return false;
  }
  const ON_BrepVertex& vertex = m_V[vertex_index];
  if ( vertex.m_vertex_index != vertex_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_V[%d] vertex is not valid.\n",vertex_index);
      text_log->PushIndent();
      text_log->Print("vertex.m_vertex_index = %d (should be %d).\n",
                       vertex.m_vertex_index, vertex_index );
      text_log->PopIndent();
    }
    return false;
  }

  const int vertex_edge_count = vertex.m_ei.Count();
  int i, j, vei, ei;
  for ( vei = 0; vei < vertex_edge_count; vei++ ) 
  {
    ei = vertex.m_ei[vei];

    if ( ei < 0 || ei >= m_E.Count() )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_V[%d] vertex is not valid.\n",vertex_index);
        text_log->PushIndent();
        text_log->Print("vertex.m_ei[%d] = %d (should be >=0 and <%d).\n", vei, ei, m_E.Count());
        text_log->PopIndent();
      }
      return false;
    }

    const ON_BrepEdge& edge = m_E[ei];

    if ( ei != edge.m_edge_index )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_V[%d] vertex is not valid.\n",vertex_index);
        text_log->PushIndent();
        text_log->Print("vertex.m_ei[%d] = %d is a deleted edge.\n", vei, ei);
        text_log->PopIndent();
      }
      return false;
    }

    if ( edge.m_vi[0] != vertex_index && edge.m_vi[1] != vertex_index )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_V[%d] vertex or brep.m_E[%d] edge is not valid.\n",vertex_index,ei);
        text_log->PushIndent();
        text_log->Print("vertex.m_ei[%d] = %d but brep.m_E[%d].m_vi[] = [%d,%d]. "
                        "At least one edge m_vi[] value should be %d.\n",
                        vei,ei,ei,edge.m_vi[0],edge.m_vi[1],vertex_index);
        text_log->PopIndent();
      }
      return false;
    }

    for ( i = 0; i < vei; i++ ) 
    {
      if ( vertex.m_ei[i] == ei ) 
      {
        // edge should be closed
        if ( edge.m_vi[0] != vertex_index || edge.m_vi[1] != vertex_index )
        {
          if ( text_log )
          {
            text_log->Print("brep.m_V[%d] vertex is not valid.\n",vertex_index);
            text_log->PushIndent();
            text_log->Print("vertex.m_ei[%d] and vertex.m_ei[%d] = %d but brep.m_E[%d].m_vi[0] = %d",
                             i,vei,ei,ei,edge.m_vi[0]);
            text_log->Print("and ON_Brep.m_E[%d].m_vi[1] = %d (both m_vi[] values should be %d).\n",
                            ei,edge.m_vi[1],vertex_index);
            text_log->PopIndent();
          }
          return false;
        }
        for (j = i+1; j < vei; j++ ) 
        {
          if ( vertex.m_ei[j] == ei )
          {
            if ( text_log )
            {
              text_log->Print("brep.m_V[%d] vertex is not valid.\n",vertex_index);
              text_log->PushIndent();
              text_log->Print("vertex.m_ei[%d,%d,%d] = %d. An open edge index should appear once\n",i,vei,j,ei);
              text_log->Print("in vertex.m_ei[] and a closed edge index should appear twice.\n");
              text_log->PopIndent();
            }
            return false;
          }
        }
        break;
      }
    }

  }

  return true;
}

bool
ON_Brep::IsValidFaceTopology( int face_index, ON_TextLog* text_log  ) const
{
  if ( face_index < 0 || face_index >= m_F.Count() )
  {
    if ( text_log )
    {
      text_log->Print("brep face_index = %d (should be >=0 and <%d=brep.m_F.Count()).\n",
                      face_index, m_F.Count());
    }
    return false;
  }
  const ON_BrepFace& face = m_F[face_index];
  if ( face.m_face_index != face_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("face.m_face_index = %d (should be %d).\n",
                       face.m_face_index, face_index );
      text_log->PopIndent();
    }
    return false;
  }
  if ( face.m_brep != this )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("face.m_brep does not point to parent brep.\n");
      text_log->PopIndent();
    }
    return false;
  }

  const int face_loop_count = face.m_li.Count();
  if ( face_loop_count <= 0 )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("face.m_li.Count() <= 0 (should be >= 1)\n");
      text_log->PopIndent();
    }
    return false;
  }

  int i, fli, li;
  for ( fli = 0; fli < face_loop_count; fli++ ) 
  {
    li = face.m_li[fli];
    for ( i = 0; i < fli; i++ ) 
    {
      if ( face.m_li[i] == li )
      {
        if ( text_log )
        {
          text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
          text_log->PushIndent();
          text_log->Print("face.m_li[%d]=face.m_li[%d]=%d (a loop index should appear once in face.m_li[])\n",
                          fli,i,li);
          text_log->PopIndent();
        }
        return false;
      }
    }
    if ( !IsValidLoop( li, text_log ) )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
        text_log->PushIndent();
        text_log->Print("brep.m_L[face.m_li[%d]=%d] is not valid.\n",fli,li);
        text_log->PopIndent();
      }
      return false;
    }
    const ON_BrepLoop& loop = m_L[li];
    if ( loop.m_loop_index != li )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
        text_log->PushIndent();
        text_log->Print("face.m_li[%d]=%d is a deleted loop\n",
                        fli,li);
        text_log->PopIndent();
      }
      return false;
    }
    if ( loop.m_fi != face_index )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
        text_log->PushIndent();
        text_log->Print("face.m_li[%d]=%d but brep.m_L[%d].m_fi=%d (m_fi should be %d)\n",
                        fli,li,li,loop.m_fi,face_index);
        text_log->PopIndent();
      }
      return false;
    }
    if ( fli == 0 ) {
      if ( loop.m_type != ON_BrepLoop::outer )
      {
        if ( text_log )
        {
          text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
          text_log->PushIndent();
          text_log->Print("brep.m_L[face.m_li[0]=%d].m_type is not outer.\n",li);
          text_log->PopIndent();
        }
        return false;
      }
    }
    else {
      if ( loop.m_type != ON_BrepLoop::inner )
      {
        if ( text_log )
        {
          text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
          text_log->PushIndent();
          text_log->Print("brep.m_L[face.m_li[%d]=%d].m_type is not inner.\n",fli,li);
          text_log->PopIndent();
        }
        return false;
      }
    }
  }

  const int si = face.m_si;
  if ( si < 0 || si >= m_S.Count() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("face.m_si=%d (should be >=0 and <%d=m_S.Count())\n",
                      face.m_si,m_S.Count());                      
      text_log->PopIndent();
    }
    return false;
  }

  if ( 0 == m_S[si] )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("brep.m_S[face.m_si=%d] is NULL\n",face.m_si);
      text_log->PopIndent();
    }
    return false;
  }

  if ( 0 == face.ProxySurface() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("face.ProxySurface() is NULL\n");
      text_log->PopIndent();
    }
    return false;
  }

  if ( m_S[si] != face.ProxySurface() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("brep.m_S[face.m_si=%d] != face.ProxySurface().\n",si);
      text_log->PopIndent();
    }
    return false;
  }

  if ( face.ProxySurfaceIsTransposed() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("face.ProxySurfaceIsTransposed() is true.\n");
      text_log->PopIndent();
    }
    return false;
  }

  return true; 
}

bool
ON_Brep::IsValidEdgeTopology( int edge_index, ON_TextLog* text_log ) const
{
  if ( edge_index < 0 || edge_index >= m_E.Count() )
  {
    if ( text_log )
      text_log->Print("brep edge_index = %d (should be >=0 and <%d=brep.m_E.Count() ).\n",
                      edge_index, m_E.Count());
    return false;
  }
  const ON_BrepEdge& edge = m_E[edge_index];
  if ( edge.m_edge_index != edge_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_edge_index = %d (should be %d).\n",
                       edge.m_edge_index, edge_index );
      text_log->PopIndent();
    }
    return false;
  }
  if ( edge.m_brep != this )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_brep does not point to parent brep\n");
      text_log->PopIndent();
    }
    return false;
  }

  if ( !edge.IsValid(text_log) )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge is not a valid.\n");
      text_log->PopIndent();
    }
    return false;
  }

  // This checks to make sure that m_C3[] and the edge's proxy information is valid.
  // This isn't exactly "topology", but if this stuff isn't set right, then the
  // "geometry" checks might crash.

  const int c3i = edge.m_c3i;
  if ( c3i < 0 || c3i >= m_C3.Count() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_c3i = %d (should be >=0 and <%d=m_C3.Count()\n",
                      edge.m_c3i,m_C3.Count() );
      text_log->PopIndent();
    }
    return false;
  }
  
  if ( 0 == m_C3[c3i] )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_E[%d].m_c3i = %d, but m_C3[%d] is NULL.\n",edge_index,c3i,c3i);
    return false;
  }

  if ( 0 == edge.ProxyCurve() )
  {
    if ( text_log )
      text_log->Print("brep.m_E[%d].m_c3i = %d, but edge.ProxyCurve() is NULL.\n",edge_index,c3i);
    return false;
  }

  if ( m_C3[c3i] != edge.ProxyCurve() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("m_E[%d].m_c3i=%d, m_C3[%d] != m_E[%d].ProxyCurve()\n",edge_index,c3i,c3i,edge_index);
      text_log->PopIndent();
    }
    return false;
  }

  ON_Interval proxy_sub_dom = edge.ProxyCurveDomain();
  if ( !proxy_sub_dom.IsIncreasing() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("m_E[%d].ProxyCurveDomain() = (%g,%g) is not increasing\n",edge_index,proxy_sub_dom[0],proxy_sub_dom[1]);
      text_log->PopIndent();
    }
    return false;
  }

  ON_Interval c3_dom = m_C3[c3i]->Domain();
  if ( !c3_dom.Includes(proxy_sub_dom) )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("m_C3[%d].Domain() = (%g,%g) does not inlclude m_E[%d].ProxyCurveDomain() = (%g,%g) is not increasing\n",
                      c3i,c3_dom[0],c3_dom[1],edge_index,proxy_sub_dom[0],proxy_sub_dom[1]);
      text_log->PopIndent();
    }
    return false;
  }

  ON_Interval edge_dom = edge.Domain();
  if ( !edge_dom.IsIncreasing() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] trim is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("m_E[%d].Domain() = (%g,%g) is not increasing\n",edge_index,edge_dom[0],edge_dom[1]);
      text_log->PopIndent();
    }
    return false;
  }



  const int vi0 = edge.m_vi[0];
  const int vi1 = edge.m_vi[1];
  if ( vi0 < 0 || vi0 >= m_V.Count() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_vi[0]=%d (should be >=0 and <%d=m_V.Count()\n",
                       vi0, m_V.Count() );
      text_log->PopIndent();
    }
    return false;
  }
  if ( vi1 < 0 || vi1 >= m_V.Count() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_vi[1]=%d (should be >=0 and <%d=m_V.Count()\n",
                       vi1, m_V.Count() );
      text_log->PopIndent();
    }
    return false;
  }
  int evi;
  for ( evi = 0; evi < 2; evi++ ) 
  {
    const ON_BrepVertex& vertex = m_V[edge.m_vi[evi]];

    if ( edge.m_vi[evi] != vertex.m_vertex_index )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
        text_log->PushIndent();
        text_log->Print("edge.m_vi[%d]=%d is a deleted vertex\n",
                         evi,edge.m_vi[evi] );
        text_log->PopIndent();
      }
      return false;
    }


    const int vertex_edge_count = vertex.m_ei.Count();
    ON_BOOL32 bFoundIt = false;
    int vei;
    for ( vei = 0; vei < vertex_edge_count && !bFoundIt; vei++ ) 
    {
      bFoundIt = (vertex.m_ei[vei] == edge_index);
    }
    if ( !bFoundIt )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
        text_log->PushIndent();
        text_log->Print("edge.m_vi[%d]=%d but edge is not referenced in m_V[%d].m_ei[]\n",
                         evi,edge.m_vi[evi],edge.m_vi[evi] );
        text_log->PopIndent();
      }
      return false;
    }
  }

  const int edge_trim_count = edge.m_ti.Count();
  if ( edge_trim_count < 0 )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_ti.Count() < 0\n");
      text_log->PopIndent();
    }
    return false;
  }
  int i, eti, ti;
  for (eti = 0; eti < edge_trim_count; eti++ )
  {
    ti = edge.m_ti[eti];
    if ( ti < 0 || ti >= m_T.Count() )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
        text_log->PushIndent();
        text_log->Print("edge.m_ti[%d]=%d (should be >=0 and <%d=m_T.Count())\n",eti,ti);
        text_log->PopIndent();
      }
      return false;
    }
    if ( m_T[ti].m_trim_index != ti )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
        text_log->PushIndent();
        text_log->Print("edge.m_ti[%d]=%d is a deleted trim\n",eti,ti);
        text_log->PopIndent();
      }
      return false;
    }
    for ( i = 0; i < eti; i++ ) 
    {
      if ( edge.m_ti[i] == ti )
      {
        if ( text_log )
        {
          text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
          text_log->PushIndent();
          text_log->Print("edge.m_ti[%d]=edge.m_ti[%d]=%d (a trim should be referenced once).\n",i,eti,ti);
          text_log->PopIndent();
        }
        return false;
      }
    }
    const ON_BrepTrim& trim = m_T[ti];
    if ( trim.m_ei != edge_index )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
        text_log->PushIndent();
        text_log->Print("edge.m_ti[%d]=%d but brep.m_T[%d].m_ei=%d\n",eti,ti,ti,trim.m_ei);
        text_log->PopIndent();
      }
      return false;
    }
  }

  return true;
}

bool
ON_Brep::IsValidLoopTopology( int loop_index, ON_TextLog* text_log ) const
{
  int lti, ti;

  if ( loop_index < 0 || loop_index >= m_L.Count() )
  {
    if ( text_log )
      text_log->Print("brep loop_index = %d (should be >=0 and <%d=brep.m_L.Count() ).\n",
                      loop_index, m_L.Count());
    return false;
  }
  const ON_BrepLoop& loop = m_L[loop_index];
  if ( loop.m_loop_index != loop_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_L[%d] loop is not valid.\n",loop_index);
      text_log->PushIndent();
      text_log->Print("loop.m_loop_index = %d (should be %d).\n",
                       loop.m_loop_index, loop_index );
      text_log->PopIndent();
    }
    return false;
  }
  if ( loop.m_brep != this )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_L[%d] loop is not valid.\n",loop_index);
      text_log->PushIndent();
      text_log->Print("loop.m_brep does not point to parent brep\n");
      text_log->PopIndent();
    }
    return false;
  }

  if ( loop.m_fi < 0 || loop.m_fi >= m_F.Count() )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_L[%d].m_fi = %d is not invalid.\n",loop_index,loop.m_fi);
    return false;
  }
  if ( m_F[loop.m_fi].m_face_index != loop.m_fi )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_L[%d].m_fi = %d is a deleted face.\n",loop_index,loop.m_fi);
    return false;
  }
  if ( loop.m_ti.Count() < 1 )
  {
      if ( text_log )
        text_log->Print("ON_Brep.m_L[%d].m_ti.Count() = %d  (should be > 0 )\n",loop_index,loop.m_ti.Count());
      return false;
  }

  for ( lti = 0; lti < loop.m_ti.Count(); lti++ )
  {
    ti = loop.m_ti[lti];
    if ( ti < 0 || ti >= m_T.Count() )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_L[%d].m_ti[%d] = %d is not invalid.\n",loop_index,lti,ti);
      return false;
    }
    const ON_BrepTrim& trim = m_T[ti];
    if ( trim.m_trim_index != ti )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_L[%d].m_ti[%d] = %d is a deleted trim.\n",loop_index,lti,ti);
      return false;
    }
    if ( trim.m_li != loop_index )
    {
      if ( text_log )
      {
        text_log->Print("brep loop m_L[%d] or trim m_T[%d] is not valid.\n",loop_index,ti);
        text_log->PushIndent();
        text_log->Print("loop.m_ti[%d] = %d != %d =trim.m_li\n",lti,ti,trim.m_li);
        text_log->PopIndent();
      }
      return false;
    }
  }


  int first_trim_ti = -4;
  int first_trim_vi0 = -3;
  int prev_trim_vi1 = -2;
  int prev_trim_ti=-9;
  for ( lti = 0; lti < loop.m_ti.Count(); lti++ )
  {
    const ON_BrepTrim& trim = m_T[loop.m_ti[lti]];
    if ( 0 == lti )
    {
      first_trim_ti = loop.m_ti[lti];
      first_trim_vi0 = trim.m_vi[0];
    }
    else if ( prev_trim_vi1 != trim.m_vi[0] )
    {
      // 23 May 2003 Dale Lear
      //     Added this test to make sure adjacent trims
      //     in a loop shared vertices.
      if ( text_log )
      {
         text_log->Print("brep loop m_L[%d] is not valid.\n",loop_index);
         text_log->PushIndent();
         text_log->Print("m_T[loop.m_ti[%d]=%d].m_vi[1] = %d != m_T[loop.m_ti[%d]=%d].m_vi[0]=%d.\n",
                          lti-1,prev_trim_ti,prev_trim_vi1,lti,loop.m_ti[lti],trim.m_vi[0]);
         text_log->PopIndent();
      }
      return false;
    }
    prev_trim_ti = loop.m_ti[lti];
    prev_trim_vi1 = trim.m_vi[1];
  }

  if ( first_trim_ti >= 0 && first_trim_vi0 != prev_trim_vi1 )
  {
    // 23 May 2003 Dale Lear
    //     Added this test to make sure adjacent trims
    //     in a loop shared vertices.
    if ( text_log )
    {
       text_log->Print("brep loop m_L[%d] is not valid.\n",loop_index);
       text_log->PushIndent();
       text_log->Print("m_T[loop.m_ti[%d]=%d].m_vi[1] = %d != m_T[loop.m_ti[0]=%d].m_vi[0]=%d.\n",
                        loop.m_ti.Count()-1,prev_trim_ti,prev_trim_vi1,first_trim_ti,first_trim_vi0);
       text_log->PopIndent();
    }
    return false;
  }


  return true;
}

bool
ON_Brep::IsValidTrimTopology( int trim_index, ON_TextLog* text_log ) const
{
  if ( trim_index < 0 || trim_index >= m_T.Count() )
  {
    if ( text_log )
      text_log->Print("brep trim_index = %d (should be >=0 and <%d=brep.m_T.Count() ).\n",
                      trim_index, m_T.Count());
    return false;
  }
  const ON_BrepTrim& trim = m_T[trim_index];
  if ( trim.m_trim_index != trim_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim is not valid.\n",trim_index);
      text_log->PushIndent();
      text_log->Print("trim.m_trim_index = %d (should be %d).\n",
                       trim.m_trim_index, trim_index );
      text_log->PopIndent();
    }
    return false;
  }
  if ( trim.m_brep != this )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim is not valid.\n",trim_index);
      text_log->PushIndent();
      text_log->Print("trim.m_brep does not point to parent brep\n");
      text_log->PopIndent();
    }
    return false;
  }

  if ( trim.m_vi[0] < 0 || trim.m_vi[0] >= m_V.Count() )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_T[%d].m_vi[0] = %d is not invalid.\n",trim_index,trim.m_vi[0]);
    return false;
  }
  if ( trim.m_vi[1] < 0 || trim.m_vi[1] >= m_V.Count() )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_T[%d].m_vi[1] = %d is not invalid.\n",trim_index,trim.m_vi[1]);
    return false;
  }

  if ( m_V[trim.m_vi[0]].m_vertex_index != trim.m_vi[0] )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_T[%d].m_vi[0] is deleted.\n",trim_index);
    return false;
  }
  if ( m_V[trim.m_vi[1]].m_vertex_index != trim.m_vi[1] )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_T[%d].m_vi[1] is deleted.\n",trim_index);
    return false;
  }

  if ( trim.m_c2i < 0 || trim.m_c2i >= m_C2.Count() )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_T[%d].m_c2i = %d is not valid.\n",trim_index,trim.m_c2i);
    return false;
  }

  if ( 0 == m_C2[trim.m_c2i] )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_T[%d].m_c2i = %d, but m_C2[%d] is NULL.\n",trim_index,trim.m_c2i,trim.m_c2i);
    return false;
  }

  if ( 0 == trim.ProxyCurve() )
  {
    if ( text_log )
      text_log->Print("brep.m_T[%d].m_c2i = %d, but trim.ProxyCurve() is NULL.\n",trim_index,trim.m_c2i);
    return false;
  }

  if ( m_C2[trim.m_c2i] != trim.ProxyCurve() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim is not valid.\n",trim_index);
      text_log->PushIndent();
      text_log->Print("m_T[%d].m_c2i=%d, m_C2[%d] != m_T[%d].ProxyCurve()\n",
                      trim_index, trim.m_c2i, trim.m_c2i, trim_index);
      text_log->PopIndent();
    }
    return false;
  }

  ON_Interval proxy_sub_dom = trim.ProxyCurveDomain();
  if ( !proxy_sub_dom.IsIncreasing() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim is not valid.\n",trim_index);
      text_log->PushIndent();
      text_log->Print("m_T[%d].ProxyCurveDomain() = (%g,%g) is not increasing\n",trim_index,proxy_sub_dom[0],proxy_sub_dom[1]);
      text_log->PopIndent();
    }
    return false;
  }

  ON_Interval c2_dom = m_C2[trim.m_c2i]->Domain();
  if ( !c2_dom.Includes(proxy_sub_dom) )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim is not valid.\n",trim_index);
      text_log->PushIndent();
      text_log->Print("m_C2[%d].Domain() = (%g,%g) does not include m_T[%d].ProxyCurveDomain() = (%g,%g) is not increasing\n",
                      trim.m_c2i,c2_dom[0],c2_dom[1],
                      trim_index,proxy_sub_dom[0],proxy_sub_dom[1]);
      text_log->PopIndent();
    }
    return false;
  }

  ON_Interval trim_dom = trim.Domain();
  if ( !trim_dom.IsIncreasing() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim is not valid.\n",trim_index);
      text_log->PushIndent();
      text_log->Print("m_T[%d].Domain() = (%g,%g) is not increasing\n",trim_index,trim_dom[0],trim_dom[1]);
      text_log->PopIndent();
    }
    return false;
  }

  if ( trim.m_li < 0 || trim.m_li >= m_L.Count() )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_T[%d].m_li = %d is not valid.\n",trim_index,trim.m_li);
    return false;
  }

  const ON_BrepLoop& loop = m_L[trim.m_li];

  if ( loop.m_loop_index != trim.m_li )
  {
    if ( text_log )
      text_log->Print("ON_Brep.m_T[%d].m_li = %d is a deleted loop.\n",trim_index,trim.m_li);
    return false;
  }

  bool bFoundTrim = false;
  int lti, loop_trim_count = loop.m_ti.Count();
  for ( lti = 0; lti < loop_trim_count && !bFoundTrim; lti++ )
  {
    if ( loop.m_ti[lti] == trim_index )
    {
      bFoundTrim = true;
      break;
    }
  }
  if ( !bFoundTrim )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim or brep.m_L[%d] loop is not valid.\n",trim_index,trim.m_li);
      text_log->PushIndent();
      text_log->Print("trim.m_li = %d but loop.m_ti[] does not contain %d (should appear once in).\n",
                      trim.m_li,trim_index);
      text_log->PopIndent();
    }
    return false;
  }

  if ( trim.m_type == ON_BrepTrim::singular )
  {
    // trim has no 3d edge
    if ( trim.m_ei != -1 )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d].m_type = singular, but m_ei = %d (should be -1).\n",trim_index,trim.m_ei);
      return false;
    }
    if ( trim.m_bRev3d )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d].m_type = singular, but m_bRev3d = %d (should be 0).\n",trim_index,trim.m_bRev3d);
      return false;
    }
    if ( trim.m_vi[0] != trim.m_vi[1] )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d].m_type = singular, but m_vi = (%d,%d) (should be same vertex index).\n",
                        trim_index,trim.m_vi[0],trim.m_vi[1]);
      return false;
    }
  }
  else
  {
    // trim should be connected to a 3d edge
    if ( trim.m_ei < 0 || trim.m_ei >= m_E.Count() )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d].m_ei = %d is not invalid.\n",trim_index,trim.m_ei);
      return false;
    }
  
    const ON_BrepEdge& edge = m_E[trim.m_ei];
    if ( edge.m_edge_index != trim.m_ei )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d].m_ei is deleted.\n",trim_index);
      return false;
    }

    const int evi0 = trim.m_bRev3d ? 1 : 0;
    const int evi1 = trim.m_bRev3d ? 0 : 1;
    if ( trim.m_vi[0] != edge.m_vi[evi0] )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d].m_bRev3d = %d, but m_vi[0] != m_E[m_ei].m_vi[%d].\n",trim_index,trim.m_bRev3d,evi0);
      return false;
    }
    if ( trim.m_vi[1] != edge.m_vi[evi1] )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d].m_bRev3d = %d, but m_vi[0] != m_E[m_ei].m_vi[%d].\n",trim_index,trim.m_bRev3d,evi1);
      return false;
    }
  }

  return true;
}

bool
ON_Brep::IsValidTopology( ON_TextLog* text_log ) const
{
  const int curve2d_count = m_C2.Count();
  const int curve3d_count = m_C3.Count();
  const int surface_count = m_S.Count();
  const int vertex_count  = m_V.Count();
  const int edge_count    = m_E.Count();
  const int trim_count    = m_T.Count();
  const int loop_count    = m_L.Count();
  const int face_count    = m_F.Count();

  int vi, ei, fi, ti, li;

  if ( 0 == face_count && 0 == edge_count && 0 == vertex_count )
  {
    if ( text_log )
      text_log->Print( "ON_Brep has no faces, edges, or vertices\n");
    return false;
  }

  if ( 0 != face_count )
  {
    if ( 0 == edge_count )
    {
      if ( text_log )
        text_log->Print( "ON_Brep has no edges.\n");
      return false;
    }
    if ( 0 == loop_count )
    {
      if ( text_log )
        text_log->Print( "ON_Brep has no loops.\n");
      return false;
    }
    if ( 0 == surface_count )
    {
      if ( text_log )
        text_log->Print( "ON_Brep has no surfaces.\n");
      return false;
    }
    if ( 0 == trim_count )
    {
      if ( text_log )
        text_log->Print( "ON_Brep has no trims.\n");
      return false;
    }
    if ( 0 == curve2d_count )
    {
      if ( text_log )
        text_log->Print( "ON_Brep has no 2d curves.\n");
      return false;
    }
  }

  if ( 0 != edge_count )
  {
    if ( 0 == curve3d_count )
    {
      if ( text_log )
        text_log->Print( "ON_Brep has no 3d curves.\n");
      return false;
    }
    if ( 0 == vertex_count )
    {
      if ( text_log )
        text_log->Print( "ON_Brep has no vertices.\n");
      return false;
    }
  }

  // check element indices match array positions
  for ( vi = 0; vi < vertex_count; vi++ ) 
  {
    if ( m_V[vi].m_vertex_index == -1 )
    {
      const ON_BrepVertex& vertex = m_V[vi];
      if ( vertex.m_ei.Count() > 0 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_V[%d] is deleted (m_vertex_index = -1) but vertex.m_ei.Count() = %d.\n",
                           vi, vertex.m_ei.Count() );
        return false;
      }
    }
    else if ( m_V[vi].m_vertex_index != vi )
    {
      if ( text_log )
        text_log->Print( "ON_Brep.m_V[%d].m_vertex_index = %d (should be %d)\n",
                         vi, m_V[vi].m_vertex_index, vi );
      return false;
    }
  }

  for ( ei = 0; ei < edge_count; ei++ ) 
  {
    if ( m_E[ei].m_edge_index == -1 )
    {
      const ON_BrepEdge& edge = m_E[ei];
      if ( edge.m_ti.Count() > 0 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_E[%d] is deleted (m_edge_index = -1) but edge.m_ei.Count() = %d.\n",
                           ei, edge.m_ti.Count() );
        return false;
      }
      if ( edge.m_c3i != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_E[%d] is deleted (m_edge_index = -1) but edge.m_c3i=%d (should be -1).\n",
                           ei, edge.m_c3i );
        return false;
      }
      if ( edge.ProxyCurve() )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_E[%d] is deleted (m_edge_index = -1) but edge.m_curve is not NULL.\n",
                           ei );
        return false;
      }
      if ( edge.m_vi[0] != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_E[%d] is deleted (m_edge_index = -1) but edge.m_vi[0]=%d (should be -1).\n",
                           ei, edge.m_vi[0] );
        return false;
      }
      if ( edge.m_vi[1] != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_E[%d] is deleted (m_edge_index = -1) but edge.m_vi[1]=%d (should be -1).\n",
                           ei, edge.m_vi[1] );
        return false;
      }
    }
    else if ( m_E[ei].m_edge_index != ei )
    {
      if ( text_log )
        text_log->Print( "ON_Brep.m_E[%d].m_edge_index = %d (should be %d)\n",
                         ei, m_E[ei].m_edge_index, ei );
      return false;
    }
  }

  for ( ti = 0; ti < trim_count; ti++ ) 
  {
    if ( m_T[ti].m_trim_index == -1 )
    {
      const ON_BrepTrim& trim = m_T[ti];
      if ( trim.m_ei != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_T[%d] is deleted (m_trim_index = -1) but trim.m_ei=%d (should be -1).\n",
                           ti, trim.m_ei );
        return false;
      }
      if ( trim.m_li != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_T[%d] is deleted (m_trim_index = -1) but trim.m_li=%d (should be -1).\n",
                           ti, trim.m_li );
        return false;
      }
      if ( trim.m_c2i != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_T[%d] is deleted (m_trim_index = -1) but trim.m_c2i=%d (should be -1).\n",
                           ti, trim.m_c2i );
        return false;
      }
      if ( trim.m_vi[0] != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_T[%d] is deleted (m_trim_index = -1) but trim.m_vi[0]=%d (should be -1).\n",
                           ti, trim.m_vi[0] );
        return false;
      }
      if ( trim.m_vi[1] != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_T[%d] is deleted (m_trim_index = -1) but trim.m_vi[1]=%d (should be -1).\n",
                           ti, trim.m_vi[1] );
        return false;
      }
    }
    else if ( m_T[ti].m_trim_index != ti  )
    {
      if ( text_log )
        text_log->Print( "ON_Brep.m_T[%d].m_trim_index = %d (should be %d)\n",
                         ti, m_T[ti].m_trim_index, ti );
      return false;
    }
    else if ( !m_T[ti].IsValid( text_log ) )
    {
      if ( text_log )
        text_log->Print( "ON_Brep.m_T[%d] is not valid\n",ti );
      return false;
    }
  }

  for ( li = 0; li < loop_count; li++ ) 
  {
    if ( m_L[li].m_loop_index == -1 )
    {
      const ON_BrepLoop& loop = m_L[li];
      if ( loop.m_fi != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_L[%d] is deleted (m_loop_index = -1) but loop.m_fi=%d (should be -1).\n",
                           li, loop.m_fi );
        return false;
      }
      if ( loop.m_ti.Count() > 0 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_L[%d] is deleted (m_loop_index = -1) but loop.m_ti.Count()=%d.\n",
                           li, loop.m_ti.Count() );
        return false;
      }
    }
    else if ( m_L[li].m_loop_index != li )
    {
      if ( text_log )
        text_log->Print( "ON_Brep.m_L[%d].m_loop_index = %d (should be %d)\n",
                         li, m_L[li].m_loop_index, li );
      return false;
    }
  }

  for ( fi = 0; fi < face_count; fi++ ) 
  {
    if ( m_F[fi].m_face_index == -1 )
    {
      const ON_BrepFace& face = m_F[fi];
      if ( face.m_si != -1 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_F[%d] is deleted (m_face_index = -1) but face.m_si=%d (should be -1).\n",
                           fi, face.m_si );
        return false;
      }
      if ( face.ProxySurface() )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_F[%d] is deleted (m_face_index = -1) but face.ProxySurface() is not NULL.\n",
                           fi );
        return false;
      }
      if ( face.m_li.Count() > 0 )
      {
        if ( text_log )
          text_log->Print( "ON_Brep.m_F[%d] is deleted (m_face_index = -1) but face.m_li.Count()=%d.\n",
                           fi, face.m_li.Count() );
        return false;
      }
    }
    else if ( m_F[fi].m_face_index != fi )
    {
      if ( text_log )
        text_log->Print( "ON_Brep.m_F[%d].m_face_index = %d (should be %d)\n",
                         fi, m_F[fi].m_face_index, fi );
      return false;
    }
  }

  // check vertices
  for ( vi = 0; vi < vertex_count; vi++ ) {
    if ( m_V[vi].m_vertex_index == -1 )
      continue;
    if ( !IsValidVertexTopology( vi, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_V[%d] is invalid.\n",vi);
      return false;
    }
  }

  // check edges
  for ( ei = 0; ei < edge_count; ei++ ) 
  {
    if ( m_E[ei].m_edge_index == -1 )
      continue;
    if ( !IsValidEdgeTopology( ei, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_E[%d] is invalid.\n",ei);
      return false;
    }
  }

  // check faces
  for ( fi = 0; fi < face_count; fi++ ) 
  {
    if ( m_F[fi].m_face_index == -1 )
      continue;
    if ( !IsValidFaceTopology( fi, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_F[%d] is invalid.\n",fi);
      return false;
    }
  }

  // check trims
  for ( ti = 0; ti < trim_count; ti++ )
  {
    const ON_BrepTrim& trim = m_T[ti];
    if ( trim.m_trim_index == -1 )
      continue;
    if ( !IsValidTrimTopology( ti, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d] is invalid.\n",ti);
      return false;
    }
  }

  // check loops
  for ( li = 0; li < loop_count; li++ )
  {
    const ON_BrepLoop& loop = m_L[li];
    if ( loop.m_loop_index == -1 )
      continue;
    if ( !IsValidLoopTopology( li, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_L[%d] is invalid.\n",li);
      return false;
    }
  }

  return true;
}


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

bool
ON_Brep::IsValidVertexGeometry( int vertex_index, ON_TextLog* text_log ) const
{
  if ( vertex_index < 0 || vertex_index >= m_V.Count() )
  {
    if ( text_log )
      text_log->Print("brep vertex_index = %d (should be >=0 and <%d=brep.m_V.Count() ).\n",
                      vertex_index, m_V.Count());
    return false;
  }
  const ON_BrepVertex& vertex = m_V[vertex_index];
  if ( vertex.m_vertex_index != vertex_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_V[%d] vertex is not valid.\n",vertex_index);
      text_log->PushIndent();
      text_log->Print("vertex.m_vertex_index = %d (should be %d).\n",
                       vertex.m_vertex_index, vertex_index );
      text_log->PopIndent();
    }
    return false;
  }

  if ( !vertex.point.IsValid() )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_V[%d] vertex geometry is not valid.\n",vertex_index);
      text_log->PushIndent();
      text_log->Print("vertex.point = (%g,%g,%g) is not valid.\n",vertex.point.x,vertex.point.y,vertex.point.z );
      text_log->PopIndent();
    }
    return false;
  }
  return true;
}

bool
ON_Brep::IsValidEdgeGeometry( int edge_index, ON_TextLog* text_log ) const
{
  if ( edge_index < 0 || edge_index >= m_E.Count() )
  {
    if ( text_log )
      text_log->Print("brep edge_index = %d (should be >=0 and <%d=brep.m_E.Count() ).\n",
                      edge_index, m_E.Count());
    return false;
  }
  const ON_BrepEdge& edge = m_E[edge_index];
  if ( edge.m_edge_index != edge_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_edge_index = %d (should be %d).\n",
                       edge.m_edge_index, edge_index );
      text_log->PopIndent();
    }
    return false;
  }

  int vi0 = edge.m_vi[0];
  int vi1 = edge.m_vi[1];
  if ( edge.IsClosed() ) 
  {
    if ( vi0 != vi1 )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
        text_log->PushIndent();
        text_log->Print("edge.m_vi[]=(%d,%d) but edge.IsClosed() is true\n",
                         vi0,vi1);
        text_log->PopIndent();
      }
      return false;
    }
  }
  else {
    if ( vi0 == vi1 )
    {
      if ( text_log )
      {
        text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
        text_log->PushIndent();
        text_log->Print("edge.m_vi[0]=edge.m_vi[1]=%d but edge.IsClosed() is false.\n",
                         vi0);
        text_log->PopIndent();
      }
      return false;
    }
  }

  return true;
}

bool
ON_Brep::IsValidFaceGeometry( int face_index, ON_TextLog* text_log ) const
{
  if ( face_index < 0 || face_index >= m_F.Count() )
  {
    if ( text_log )
      text_log->Print("brep face_index = %d (should be >=0 and <%d=brep.m_F.Count() ).\n",
                      face_index, m_F.Count());
    return false;
  }
  const ON_BrepFace& face = m_F[face_index];
  if ( face.m_face_index != face_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("face.m_face_index = %d (should be %d).\n",
                       face.m_face_index, face_index );
      text_log->PopIndent();
    }
    return false;
  }
  return true;
}

bool
ON_Brep::IsValidLoopGeometry( int loop_index, ON_TextLog* text_log ) const
{
  if ( loop_index < 0 || loop_index >= m_L.Count() )
  {
    if ( text_log )
      text_log->Print("brep loop_index = %d (should be >=0 and <%d=brep.m_L.Count() ).\n",
                      loop_index, m_L.Count());
    return false;
  }
  const ON_BrepLoop& loop = m_L[loop_index];
  if ( loop.m_loop_index != loop_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_L[%d] loop is not valid.\n",loop_index);
      text_log->PushIndent();
      text_log->Print("loop.m_loop_index = %d (should be %d).\n",
                       loop.m_loop_index, loop_index );
      text_log->PopIndent();
    }
    return false;
  }
  return true;
}

bool
ON_Brep::IsValidTrimGeometry( int trim_index, ON_TextLog* text_log ) const
{
  if ( trim_index < 0 || trim_index >= m_T.Count() )
  {
    if ( text_log )
      text_log->Print("brep trim_index = %d (should be >=0 and <%d=brep.m_T.Count() ).\n",
                      trim_index, m_T.Count());
    return false;
  }
  const ON_BrepTrim& trim = m_T[trim_index];
  if ( trim.m_trim_index != trim_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim is not valid.\n",trim_index);
      text_log->PushIndent();
      text_log->Print("trim.m_trim_index = %d (should be %d).\n",
                       trim.m_trim_index, trim_index );
      text_log->PopIndent();
    }
    return false;
  }
  return true;
}

bool
ON_Brep::IsValidGeometry( ON_TextLog* text_log ) const
{
  const int curve2d_count = m_C2.Count();
  const int curve3d_count = m_C3.Count();
  const int surface_count = m_S.Count();
  const int vertex_count  = m_V.Count();
  const int edge_count    = m_E.Count();
  const int trim_count    = m_T.Count();
  const int loop_count    = m_L.Count();
  const int face_count    = m_F.Count();

  int c2i, c3i, si, vi, ei, fi, ti, li;

  // check 2d curve geometry
  for ( c2i = 0; c2i < curve2d_count; c2i++ ) {
    if ( !m_C2[c2i] )
    {
      continue;
      // NULL 2d curves are ok if they are not referenced
    }
    if ( !m_C2[c2i]->IsValid(text_log) )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_C2[%d] is invalid.\n",c2i);
      return false;
    }
    int c2_dim = m_C2[c2i]->Dimension();
    if ( c2_dim != 2 )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_C2[%d]->Dimension() = %d (should be 2).\n", c2i, c2_dim );
      return false;
    }
  }

  // check 3d curve geometry
  for ( c3i = 0; c3i < curve3d_count; c3i++ ) {
    if ( !m_C3[c3i] )
    {
      continue;
      // NULL 3d curves are ok if they are not referenced
    }
    if ( !m_C3[c3i]->IsValid(text_log) )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_C3[%d] is invalid.\n",c3i);
      return false;
    }
    int c3_dim = m_C3[c3i]->Dimension();
    if ( c3_dim != 3 )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_C3[%d]->Dimension() = %d (should be 3).\n", c3i, c3_dim );
      return false;
    }
  }

  // check 3d surface geometry
  for ( si = 0; si < surface_count; si++ ) {
    if ( !m_S[si] )
    {
      continue;
      // NULL 3d surfaces are ok if they are not referenced
    }
    if ( !m_S[si]->IsValid(text_log) )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_S[%d] is invalid.\n",si);
      return false;
    }
    int dim = m_S[si]->Dimension();
    if ( dim != 3 )
    {
      if ( text_log )
        text_log->Print("ON_Brep.m_S[%d]->Dimension() = %d (should be 3).\n", si, dim );
      return false;
    }
  }

  // check vertices
  for ( vi = 0; vi < vertex_count; vi++ ) {
    if ( m_V[vi].m_vertex_index == -1 )
      continue;
    if ( !IsValidVertexGeometry( vi, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_V[%d] is invalid.\n",vi);
      return false;
    }
  }

  // check edges
  for ( ei = 0; ei < edge_count; ei++ ) 
  {
    if ( m_E[ei].m_edge_index == -1 )
      continue;
    if ( !IsValidEdgeGeometry( ei, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_E[%d] is invalid.\n",ei);
      return false;
    }
  }

  // check faces
  for ( fi = 0; fi < face_count; fi++ ) 
  {
    if ( m_F[fi].m_face_index == -1 )
      continue;
    if ( !IsValidFaceGeometry( fi, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_F[%d] is invalid.\n",fi);
      return false;
    }
  }

  // check trims
  for ( ti = 0; ti < trim_count; ti++ )
  {
    if ( m_T[ti].m_trim_index == -1 )
      continue;
    if ( !IsValidTrimGeometry( ti, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d] is invalid.\n",ti);
      return false;
    }
  }

  // check loops
  for ( li = 0; li < loop_count; li++ )
  {
    if ( m_L[li].m_loop_index == -1 )
      continue;
    if ( !IsValidLoopGeometry( li, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_L[%d] is invalid.\n",li);
      return false;
    }
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

bool
ON_Brep::IsValidVertexTolerancesAndFlags( int vertex_index, ON_TextLog* text_log ) const
{
  if ( vertex_index < 0 || vertex_index >= m_V.Count() )
  {
    if ( text_log )
      text_log->Print("brep vertex_index = %d (should be >=0 and <%d=brep.m_V.Count() ).\n",
                      vertex_index, m_V.Count());
    return false;
  }
  const ON_BrepVertex& vertex = m_V[vertex_index];
  if ( vertex.m_vertex_index != vertex_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_V[%d] vertex is not valid.\n",vertex_index);
      text_log->PushIndent();
      text_log->Print("vertex.m_vertex_index = %d (should be %d).\n",
                       vertex.m_vertex_index, vertex_index );
      text_log->PopIndent();
    }
    return false;
  }

  if ( vertex.m_tolerance < 0.0 )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_V[%d] vertex is not valid.\n",vertex_index);
      text_log->PushIndent();
      text_log->Print("vertex.m_tolerace = %g (should be >= 0.0)\n",vertex.m_tolerance);
      text_log->PopIndent();
    }
    return false;
  }

  return true;
}

bool
ON_Brep::IsValidEdgeTolerancesAndFlags( int edge_index, ON_TextLog* text_log ) const
{
  if ( edge_index < 0 || edge_index >= m_E.Count() )
  {
    if ( text_log )
      text_log->Print("brep edge_index = %d (should be >=0 and <%d=brep.m_E.Count() ).\n",
                      edge_index, m_E.Count());
    return false;
  }
  const ON_BrepEdge& edge = m_E[edge_index];
  if ( edge.m_edge_index != edge_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_edge_index = %d (should be %d).\n",
                       edge.m_edge_index, edge_index );
      text_log->PopIndent();
    }
    return false;
  }

  if ( edge.m_tolerance < 0.0 )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_E[%d] edge is not valid.\n",edge_index);
      text_log->PushIndent();
      text_log->Print("edge.m_tolerance=%g (should be >= 0.0)\n",edge.m_tolerance);
      text_log->PopIndent();
    }
    return false;
  }

  return true;
}

bool
ON_Brep::IsValidFaceTolerancesAndFlags( int face_index, ON_TextLog* text_log ) const
{
  if ( face_index < 0 || face_index >= m_F.Count() )
  {
    if ( text_log )
      text_log->Print("brep face_index = %d (should be >=0 and <%d=brep.m_F.Count() ).\n",
                      face_index, m_F.Count());
    return false;
  }
  const ON_BrepFace& face = m_F[face_index];
  if ( face.m_face_index != face_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_F[%d] face is not valid.\n",face_index);
      text_log->PushIndent();
      text_log->Print("face.m_face_index = %d (should be %d).\n",
                       face.m_face_index, face_index );
      text_log->PopIndent();
    }
    return false;
  }
  return true;
}

bool
ON_Brep::IsValidLoopTolerancesAndFlags( int loop_index, ON_TextLog* text_log ) const
{
  if ( loop_index < 0 || loop_index >= m_L.Count() )
  {
    if ( text_log )
      text_log->Print("brep loop_index = %d (should be >=0 and <%d=brep.m_L.Count() ).\n",
                      loop_index, m_L.Count());
    return false;
  }
  const ON_BrepLoop& loop = m_L[loop_index];
  if ( loop.m_loop_index != loop_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_L[%d] loop is not valid.\n",loop_index);
      text_log->PushIndent();
      text_log->Print("loop.m_loop_index = %d (should be %d).\n",
                       loop.m_loop_index, loop_index );
      text_log->PopIndent();
    }
    return false;
  }
  return true;
}

bool
ON_Brep::IsValidTrimTolerancesAndFlags( int trim_index, ON_TextLog* text_log ) const
{
  if ( trim_index < 0 || trim_index >= m_T.Count() )
  {
    if ( text_log )
      text_log->Print("brep trim_index = %d (should be >=0 and <%d=brep.m_T.Count() ).\n",
                      trim_index, m_T.Count());
    return false;
  }
  const ON_BrepTrim& trim = m_T[trim_index];
  if ( trim.m_trim_index != trim_index )
  {
    if ( text_log )
    {
      text_log->Print("brep.m_T[%d] trim is not valid.\n",trim_index);
      text_log->PushIndent();
      text_log->Print("trim.m_trim_index = %d (should be %d).\n",
                       trim.m_trim_index, trim_index );
      text_log->PopIndent();
    }
    return false;
  }
  return true;
}

bool
ON_Brep::IsValidTolerancesAndFlags( ON_TextLog* text_log ) const
{
  const int vertex_count  = m_V.Count();
  const int edge_count    = m_E.Count();
  const int trim_count    = m_T.Count();
  const int loop_count    = m_L.Count();
  const int face_count    = m_F.Count();

  int vi, ei, fi, ti, li;

  // check vertices
  for ( vi = 0; vi < vertex_count; vi++ ) {
    if ( m_V[vi].m_vertex_index == -1 )
      continue;
    if ( !IsValidVertexTolerancesAndFlags( vi, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_V[%d] is invalid.\n",vi);
      return false;
    }
  }

  // check edges
  for ( ei = 0; ei < edge_count; ei++ ) 
  {
    if ( m_E[ei].m_edge_index == -1 )
      continue;
    if ( !IsValidEdgeTolerancesAndFlags( ei, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_E[%d] is invalid.\n",ei);
      return false;
    }
  }

  // check faces
  for ( fi = 0; fi < face_count; fi++ ) 
  {
    if ( m_F[fi].m_face_index == -1 )
      continue;
    if ( !IsValidFaceTolerancesAndFlags( fi, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_F[%d] is invalid.\n",fi);
      return false;
    }
  }

  // check trims
  for ( ti = 0; ti < trim_count; ti++ )
  {
    if ( m_T[ti].m_trim_index == -1 )
      continue;
    if ( !IsValidTrimTolerancesAndFlags( ti, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_T[%d] is invalid.\n",ti);
      return false;
    }
  }

  // check loops
  for ( li = 0; li < loop_count; li++ )
  {
    if ( m_L[li].m_loop_index == -1 )
      continue;
    if ( !IsValidLoopTolerancesAndFlags( li, text_log ) ) {
      if ( text_log )
        text_log->Print("ON_Brep.m_L[%d] is invalid.\n",li);
      return false;
    }
  }

  return true;
}

