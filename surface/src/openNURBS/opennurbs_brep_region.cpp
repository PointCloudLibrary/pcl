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
void ON_Brep::MemoryRelocate()
{
  int i, count;

  // The call to the base class MemoryRelocate() takes care of 
  // updating user data back-pointers.
  ON_Geometry::MemoryRelocate();

  // When the memory location of an ON_Brep changes,
  // the m_brep backpointers on its pieces need to be updated.

  count = m_E.Count();
  for ( i = 0; i < count; i++ )
  {
    m_E[i].m_brep = this;
  }

  count = m_T.Count();
  for ( i = 0; i < count; i++ )
  {
    m_T[i].m_brep = this;
  }

  count = m_L.Count();
  for ( i = 0; i < count; i++ )
  {
    m_L[i].m_brep = this;
  }

  count = m_F.Count();
  for ( i = 0; i < count; i++ )
  {
    m_F[i].m_brep = this;
  }
}

ON_Brep* ON_Brep::SubBrep( 
          int subfi_count, 
          const int* subfi, 
          ON_Brep* sub_brep
          ) const
{
  class LeakStopper : public ON_Workspace
  {
    // If an error occures during construction,
    // this class cleans up sub_brep in an
    // appropriate fashion.
  public:
    LeakStopper() {m_p=0;m_sub_brep=0;}
    ~LeakStopper() {if (m_p) delete m_p; else if (m_sub_brep) m_sub_brep->Destroy();}
    ON_Brep* m_p;        // ON_Brep::SubBrep allocated sub_brep
    ON_Brep* m_sub_brep; // user's sub_brep argument
  };
  LeakStopper leak_stopper;

  if ( sub_brep )
    sub_brep->Destroy();

  if ( subfi_count <= 0 || 0 == subfi )
    return 0;

  if ( subfi_count > m_F.Count() )
    return 0;

  // validate indices in extract_fi[] and
  // make sure there are no duplicates.
  int fi, fli, lti, i, j;
  int Lcount = 0;
  int Tcount = 0;
  int Ecount = 0;
  int Vcount = 0;
  int maxfi = -1;
  int minfi = m_F.Count();
  int* Emap = leak_stopper.GetIntMemory(m_E.Count());
  memset(Emap,0,m_E.Count()*sizeof(Emap[0]));
  int* Vmap = leak_stopper.GetIntMemory(m_V.Count());
  memset(Vmap,0,m_V.Count()*sizeof(Vmap[0]));
  for ( i = 0; i < subfi_count; i++ )
  {
    fi = subfi[i];
    if ( fi < 0 || fi >= m_F.Count() )
    {
      ON_ERROR("ON_Brep::SubBrep sub_fi[] has invalid indices");
      return false;
    }
    if ( fi > maxfi )
      maxfi = fi;
    else if ( fi < minfi )
      minfi = fi;
    else
    {
      for ( j = 0; j < i; j++ )
      {
        if ( subfi[j] == fi )
        {
          ON_ERROR("ON_Brep::SubBrep sub_fi[] has duplicate indices");
          return false;
        }
      }
    }

    const ON_BrepFace& face = m_F[fi];
    for ( fli = 0; fli < face.m_li.Count(); fli++ )
    {
      const ON_BrepLoop* loop = face.Loop(fli);
      if ( !loop || this != loop->Brep() )
        return false;
      Lcount++;
      for ( lti = 0; lti < loop->m_ti.Count(); lti++ )
      {
        const ON_BrepTrim* trim = loop->Trim(lti);
        if ( !trim || this != trim->Brep() )
          return 0;
        Tcount++;
        if ( trim->m_vi[0] < 0 || trim->m_vi[0] >= m_V.Count() )
          return 0;
        if ( trim->m_vi[1] < 0 || trim->m_vi[1] >= m_V.Count() )
          return 0;
        if ( 0 == Vmap[trim->m_vi[0]] )
        {
          Vmap[trim->m_vi[0]] = 1;
          Vcount++;
        }
        if ( 0 == Vmap[trim->m_vi[1]] )
        {
          Vmap[trim->m_vi[1]] = 1;
          Vcount++;
        }
        if ( ON_BrepTrim::singular == trim->m_type ||
             ON_BrepTrim::ptonsrf == trim->m_type)   // March 29, 2010 Lowell - Allow ptonsrf
        {
          if ( trim->m_ei >= 0 || trim->m_vi[0] != trim->m_vi[1] )
            return 0;
        }
        else if ( trim->m_ei >= 0 )
        {
          const ON_BrepEdge* edge = trim->Edge();
          if ( 0 == edge || this != edge->Brep() )
            return 0;
          if ( 0 == Emap[trim->m_ei] )
          {
            Emap[trim->m_ei] = 1;
            Ecount++;
            // edge's vertices should already be mapped.
            if ( 0 == Vmap[edge->m_vi[0]] )
              return 0;
            if ( 0 == Vmap[edge->m_vi[1]] )
              return 0;
          }          
        }
        else
        {
          return 0;
        }
      }
    }
  }

  if ( !sub_brep )
  {
    sub_brep = ON_Brep::New();
    leak_stopper.m_p = sub_brep;
  }
  else
  {
    leak_stopper.m_sub_brep = sub_brep;
  }

  sub_brep->m_F.Reserve(subfi_count);
  sub_brep->m_L.Reserve(Lcount);
  sub_brep->m_T.Reserve(Tcount);
  sub_brep->m_E.Reserve(Ecount);
  sub_brep->m_V.Reserve(Vcount);
  sub_brep->m_S.Reserve(subfi_count);
  sub_brep->m_C2.Reserve(Tcount);
  sub_brep->m_C3.Reserve(Ecount);

  // build sub_brep vertices
  for ( i = 0; i < m_V.Count(); i++ )
  {
    if ( Vmap[i] )
    {
      const ON_BrepVertex& vertex = m_V[i];
      ON_BrepVertex& sub_vertex = sub_brep->NewVertex(vertex.point,vertex.m_tolerance);
      Vmap[i] = sub_vertex.m_vertex_index;
      sub_vertex.CopyUserData(vertex);
      // March 29, 2010 Lowell - Copy user fields
      memcpy(&sub_vertex.m_vertex_user, &vertex.m_vertex_user, sizeof(sub_vertex.m_vertex_user));
    }
    else
      Vmap[i] = -1;
  }

  // build sub_brep edges
  for ( i = 0; i < m_E.Count(); i++ )
  {
    if ( Emap[i] )
    {
      const ON_BrepEdge& edge = m_E[i];
      if ( Vmap[edge.m_vi[0]] < 0 )
        return 0;
      if ( Vmap[edge.m_vi[1]] < 0 )
        return 0;
      ON_Curve* c3 = edge.DuplicateCurve();
      if ( 0 == c3 )
        return 0;
      sub_brep->m_C3.Append(c3);
      ON_BrepVertex& sub_v0 = sub_brep->m_V[Vmap[edge.m_vi[0]]];
      ON_BrepVertex& sub_v1 = sub_brep->m_V[Vmap[edge.m_vi[1]]];
      ON_BrepEdge& sub_edge = sub_brep->NewEdge(sub_v0,sub_v1,sub_brep->m_C3.Count()-1,0,edge.m_tolerance);
      Emap[i] = sub_edge.m_edge_index;
      sub_edge.CopyUserData(edge);
      // March 29, 2010 Lowell - Copy user fields
      memcpy(&sub_edge.m_edge_user, &edge.m_edge_user, sizeof(sub_edge.m_edge_user));
    }
    else
      Emap[i] = -1;
  }

  bool bHaveBBox = m_bbox.IsValid();
  ON_BoundingBox sub_bbox;

  for ( i = 0; i < subfi_count; i++ )
  {
    const ON_BrepFace& face = m_F[subfi[i]];
    ON_Surface* srf = face.DuplicateSurface();
    if (!srf)
      return 0;
    sub_brep->m_S.Append(srf);
    ON_BrepFace& sub_face = sub_brep->NewFace(sub_brep->m_S.Count()-1);
    sub_face.CopyUserData(face);
    sub_face.m_bRev = face.m_bRev;
    sub_face.m_face_material_channel = face.m_face_material_channel;
    sub_face.m_face_uuid = face.m_face_uuid;
    sub_face.m_bbox = face.m_bbox;
    sub_face.m_domain[0] = face.m_domain[0];
    sub_face.m_domain[1] = face.m_domain[1];
    // March 29, 2010 Lowell - Copy user fields
    memcpy(&sub_face.m_face_user, &face.m_face_user, sizeof(sub_face.m_face_user));

    if ( bHaveBBox )
    {
      if ( sub_face.m_bbox.IsValid() )
        sub_bbox.Union(sub_face.m_bbox);
      else
      {
        bHaveBBox = false;
        sub_bbox.Destroy();
      }
    }


    for ( fli = 0; fli < face.m_li.Count(); fli++ )
    {
      const ON_BrepLoop& loop = m_L[face.m_li[fli]];
      ON_BrepLoop& sub_loop = sub_brep->NewLoop(loop.m_type,sub_face);
      sub_loop.CopyUserData(loop);
      sub_loop.m_pbox = loop.m_pbox;
      // April 19, 2010 Lowell - Copy user fields
      memcpy(&sub_loop.m_loop_user, &loop.m_loop_user, sizeof(sub_loop.m_loop_user));

      for ( lti = 0; lti < loop.m_ti.Count(); lti++ )
      {
        const ON_BrepTrim& trim = m_T[loop.m_ti[lti]];
        if ( Vmap[trim.m_vi[0]] < 0 || Vmap[trim.m_vi[1]] < 0 )
          return 0;
        if ( trim.m_ei >= 0 && Emap[trim.m_ei] < 0 )
          return 0;
        if(trim.m_c2i >= 0)
        {
          ON_Curve* c2 = trim.DuplicateCurve();
          if ( !c2 )
            return 0;
          sub_brep->m_C2.Append(c2);
        }
        else if(trim.m_type != ON_BrepTrim::ptonsrf)
          return 0;
        if ( trim.m_ei >= 0 )
        {
          ON_BrepEdge& sub_edge = sub_brep->m_E[Emap[trim.m_ei]];
          sub_brep->NewTrim(sub_edge,trim.m_bRev3d,sub_loop,sub_brep->m_C2.Count()-1);
        }
        else if ( ON_BrepTrim::singular == trim.m_type )
        {
          ON_BrepVertex& sub_vertex = sub_brep->m_V[Vmap[trim.m_vi[0]]];
          sub_brep->NewSingularTrim(sub_vertex,sub_loop,trim.m_iso,sub_brep->m_C2.Count()-1);
        }
        // March 29, 2010 Lowell - copy ptonsrf type
        else if ( ON_BrepTrim::ptonsrf == trim.m_type)
        {
          ON_BrepTrim& sub_trim = sub_brep->NewTrim(false, sub_loop, -1);
          sub_trim.m_type = ON_BrepTrim::ptonsrf;
          ON_BrepVertex& sub_vertex = sub_brep->m_V[Vmap[trim.m_vi[0]]];
          sub_trim.m_vi[0] = sub_trim.m_vi[1] = sub_vertex.m_vertex_index;
        }
        else
        {
          return 0;
        }
        ON_BrepTrim& sub_trim = sub_brep->m_T[sub_brep->m_T.Count()-1];
        sub_trim.CopyUserData(trim);
        sub_trim.m__legacy_2d_tol = trim.m__legacy_2d_tol;
        sub_trim.m__legacy_3d_tol = trim.m__legacy_3d_tol;
        sub_trim.m__legacy_flags = trim.m__legacy_flags;
        sub_trim.m_tolerance[0] = trim.m_tolerance[0];
        sub_trim.m_tolerance[1] = trim.m_tolerance[1];
        sub_trim.m_pbox = trim.m_pbox;
        sub_trim.m_iso = trim.m_iso;
        // April 19, 2010 Lowell - Copy user fields
        memcpy(&sub_trim.m_trim_user, &trim.m_trim_user, sizeof(sub_trim.m_trim_user));

        // Since we are extracting a subset of the original brep,
        // some mated edges could turn into boundary edges. The
        // call to NewTrim() above will correctly handle setting
        // and updating sub_trims that came from mated trims.
        if ( ON_BrepTrim::mated != trim.m_type )
          sub_trim.m_type = trim.m_type;
      }
    }
  }

  if ( !bHaveBBox || !sub_bbox.IsValid() )
    sub_brep->BoundingBox();
  else
    sub_brep->m_bbox = sub_bbox;

  // return subbrep after disabling the leak stopper
  leak_stopper.m_p = 0;
  leak_stopper.m_sub_brep = 0;
  return sub_brep;
}

