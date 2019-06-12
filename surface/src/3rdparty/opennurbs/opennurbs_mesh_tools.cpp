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


/////////////////////////////////////////////////////////////////////////////
// SwapMeshEdge()
//

bool ON_Mesh::SwapEdge_Helper( int topei, bool bTestOnly )
{
  ON_Mesh& mesh = *this;
  const ON_MeshTopology& top = mesh.Topology();  
  const int F_count = mesh.m_F.Count();
  const int V_count = mesh.m_V.Count();
  const int topv_count = top.m_topv.Count();
  //const int tope_count = top.m_tope.Count();

  if ( topei < 0 || topei >= top.m_tope.Count() )
  {
    return false;
  }

  if ( top.m_topf.Count() != F_count )
    return false;

  const ON_MeshTopologyEdge& tope = top.m_tope[topei];

  if (    tope.m_topf_count != 2 
       || tope.m_topvi[0] == tope.m_topvi[1] 
       || tope.m_topvi[0] < 0
       || tope.m_topvi[1] < 0
       || tope.m_topvi[0] >= topv_count
       || tope.m_topvi[1] >= topv_count )
  {
    return false;
  }

  int fi0 = tope.m_topfi[0];
  int fi1 = tope.m_topfi[1];
  if ( fi0 < 0 || fi0 >= F_count || fi1 < 0 || fi1 >= F_count || fi0 == fi1 )
    return false;

  const ON_MeshFace& f0 = mesh.m_F[fi0];
  const ON_MeshFace& f1 = mesh.m_F[fi1];
  if ( !f0.IsValid(V_count) )
    return false;
  if ( !f1.IsValid(V_count) )
    return false;
  if ( !f0.IsTriangle() )
    return false;
  if ( !f1.IsTriangle() )
    return false;

  const ON_MeshTopologyFace& topf0 = top.m_topf[fi0];
  const ON_MeshTopologyFace& topf1 = top.m_topf[fi1];

  int fei0;
  for ( fei0 = 0; fei0 < 3; fei0++ )
  {
    if ( topf0.m_topei[fei0] == topei )
      break;
  }
  if ( fei0 >= 3 )
    return false;

  int f0_vi0 = f0.vi[(fei0+3)%4];
  int f0_vi1 = f0.vi[fei0];
  int f0_vi2 = f0.vi[(fei0+1)%3];

  int fei1;
  for ( fei1 = 0; fei1 < 3; fei1++ )
  {
    if ( topf1.m_topei[fei1] == topei )
      break;
  }
  if ( fei1 >= 3 )
    return false;

  int f1_vi0 = f1.vi[(fei1+3)%4];
  int f1_vi1 = f1.vi[fei1];
  int f1_vi2 = f1.vi[(fei1+1)%3];

  if ( topf0.m_reve[fei0] == topf1.m_reve[fei1] )
    return false;
  if ( f0_vi0 != f1_vi1 )
    return false;
  if ( f0_vi1 != f1_vi0 )
    return false;

  const ON_MeshTopologyVertex& topv0 = top.m_topv[tope.m_topvi[0]];
  const ON_MeshTopologyVertex& topv1 = top.m_topv[tope.m_topvi[1]];
  if ( topv0.m_v_count < 1 || topv1.m_v_count < 1 )
  {
    return false;
  }
  if ( topv0.m_vi[0] < 0 || topv0.m_vi[0] >= V_count )
  {
    return false;
  }
  if ( topv1.m_vi[0] < 0 || topv1.m_vi[0] >= V_count )
  {
    return false;
  }

  if ( bTestOnly )
    return true;


  ON_MeshFace newf0;
  ON_MeshFace newf1;
  newf0.vi[0] = f0_vi1;
  newf0.vi[1] = f0_vi2;
  newf0.vi[2] = f1_vi2;
  newf0.vi[3] = newf0.vi[2];

  newf1.vi[0] = f1_vi1;
  newf1.vi[1] = f1_vi2;
  newf1.vi[2] = f0_vi2;
  newf1.vi[3] = newf1.vi[2];

  mesh.m_F[fi0] = newf0;
  mesh.m_F[fi1] = newf1;

  mesh.DestroyTopology();
  mesh.DestroyPartition();

  return true;
}

bool ON_Mesh::IsSwappableEdge(int topei )
{
  return const_cast<ON_Mesh*>(this)->SwapEdge_Helper( topei, true );
}

bool ON_Mesh::SwapEdge( int topei )
{
  return SwapEdge_Helper( topei, false );
}

/////////////////////////////////////////////////////////////////////////////
// CollapseMeshEdge()
//

// DO NOT PUT THIS CLASS IN A HEADER FILE
class ON__MESHEDGE
{
public:
  // ON_Mesh m_V[] indices
  int vi0;
  int vi1; // always > vi0
  
  // ON_MeshTopology m_topvi[] indices
  int topvi0;
  int topvi1;

  // ON_Mesh m_V[] index of new vertex
  int newvi;

  // location of vertex that will replace the edge
  ON_3fPoint  newV;
  ON_3fVector newN;
  ON_2fPoint  newT;
};

// DO NOT PUT THIS CLASS IN A HEADER FILE
class ON__NEWVI
{
public:
  // ON_Mesh m_V[] indices
  int oldvi;
  int newvi;
};

using QSORTCMPFUNC = int (*)(const void *, const void *);

static int CompareMESHEDGE( const ON__MESHEDGE* a, const ON__MESHEDGE* b )
{
  // sort based on "vi0" and vi1" values (vi0 is always < vi1)
  int i = (a->vi0 - b->vi0);
  if ( 0 == i )
  {
    i = a->vi1 - b->vi1;
  }
  return i;
}

static int CompareNEWVI( const ON__NEWVI* a, const ON__NEWVI* b )
{
  // sort based on "oldvi" value
  return a->oldvi - b->oldvi;
}

static int CompareInt( const void* a, const void* b )
{
  return ( *((int*)a) - *((int*)b) );
}

bool ON_Mesh::CollapseEdge( int topei )
{
  ON_Mesh& mesh = *this;

  ON__MESHEDGE me;
  memset(&me,0,sizeof(me));
  const ON_MeshTopology& top = mesh.Topology();  
  const int F_count = mesh.m_F.Count();
  const int V_count = mesh.m_V.Count();
  const int topv_count = top.m_topv.Count();
  //const int tope_count = top.m_tope.Count();

  if ( topei < 0 || topei >= top.m_tope.Count() )
  {
    return false;
  }

  const ON_MeshTopologyEdge& tope = top.m_tope[topei];

  if (    tope.m_topf_count < 1 
       || tope.m_topvi[0] == tope.m_topvi[1] 
       || tope.m_topvi[0] < 0
       || tope.m_topvi[1] < 0
       || tope.m_topvi[0] >= topv_count
       || tope.m_topvi[1] >= topv_count )
  {
    return false;
  }

  const ON_MeshTopologyVertex& topv0 = top.m_topv[tope.m_topvi[0]];
  const ON_MeshTopologyVertex& topv1 = top.m_topv[tope.m_topvi[1]];
  if ( topv0.m_v_count < 1 || topv1.m_v_count < 1 )
  {
    return false;
  }
  if ( topv0.m_vi[0] < 0 || topv0.m_vi[0] >= V_count )
  {
    return false;
  }
  if ( topv1.m_vi[0] < 0 || topv1.m_vi[0] >= V_count )
  {
    return false;
  }
  
  // create a ON__MESHEDGE for each face (usually one or two) that uses the edge
  ON__MESHEDGE* me_list = (ON__MESHEDGE*)alloca(tope.m_topf_count*sizeof(me_list[0]));
  int me_list_count = 0;
  int efi;
  for ( efi = 0; efi < tope.m_topf_count; efi++ )
  {
    int fi = tope.m_topfi[efi];
    if ( fi < 0 || fi >= F_count )
      continue;
    const ON_MeshFace& f = mesh.m_F[fi];
    if ( !f.IsValid(V_count) )
      continue;
    me.vi1 = f.vi[3];
    me.topvi1 = top.m_topv_map[me.vi1];
    int fvi;
    for ( fvi = 0; fvi < 4; fvi++ )
    {
      me.vi0 = me.vi1;
      me.topvi0 = me.topvi1;
      me.vi1 = f.vi[fvi];
      me.topvi1 = top.m_topv_map[me.vi1];
      if ( me.vi0 != me.vi1 )
      {
        if (    (me.topvi0 == tope.m_topvi[0] && me.topvi1 == tope.m_topvi[1])
             || (me.topvi0 == tope.m_topvi[1] && me.topvi1 == tope.m_topvi[0]) )
        {
          if ( me.vi0 > me.vi1 )
          {
            int i = me.vi0; me.vi0 = me.vi1; me.vi1 = i;
            i = me.topvi0; me.topvi0 = me.topvi1; me.topvi1 = i;
          }
          me_list[me_list_count++] = me;
          break;
        }
      }
    }
  }

  if (me_list_count<1)
  {
    return false;
  }

  // Sort me_list[] so edges using same vertices are adjacent
  // to each other in the list.  This is needed so that non-manifold
  // crease edges will be properly collapsed.
  ON_qsort(me_list,me_list_count,sizeof(me_list[0]),(QSORTCMPFUNC)CompareMESHEDGE);

  // create new vertex or vertices that edge will be
  // collapsed to.
  mesh.m_C.Destroy();
  mesh.m_K.Destroy();
  
  int mei;
  bool bHasVertexNormals = mesh.HasVertexNormals();
  bool bHasTextureCoordinates = mesh.HasTextureCoordinates();
  bool bHasFaceNormals = mesh.HasFaceNormals();
  if ( topv0.m_v_count == 1 || topv1.m_v_count == 1 )
  {
    // a single new vertex
    ON_Line Vline(ON_origin,ON_origin);
    ON_Line Tline(ON_origin,ON_origin);
    ON_3dVector N0(0,0,0);
    ON_3dVector N1(0,0,0);
    ON_3dPoint P;
    int vi, tvi, cnt;

    int newvi = topv0.m_vi[0];

    cnt = 0;
    for ( tvi = 0; tvi < topv0.m_v_count; tvi++ )
    {
      vi = topv0.m_vi[tvi];
      if ( vi < 0 || vi > V_count )
        continue;
      if ( vi < newvi )
        newvi = vi;
      cnt++;
      P = mesh.m_V[vi];
      Vline.from += P;
      if ( bHasVertexNormals )
      {
        N0 += ON_3dVector(mesh.m_N[vi]);
      }
      if ( bHasTextureCoordinates )
      {
        P = mesh.m_T[vi];
        Tline.from += P;
      }
    }

    if (cnt > 1)
    {
      double s = 1.0/((double)cnt);
      Vline.from.x *= s;
      Vline.from.y *= s;
      Vline.from.z *= s;
      Tline.from.x *= s;
      Tline.from.y *= s;
      Tline.from.z *= s;
      N0 = s*N0;
    }

    cnt = 0;
    for ( tvi = 0; tvi < topv1.m_v_count; tvi++ )
    {
      vi = topv1.m_vi[tvi];
      if ( vi < 0 || vi > V_count )
        continue;
      if ( vi < newvi )
        newvi = vi;
      cnt++;
      P = mesh.m_V[vi];
      Vline.to += P;
      if ( bHasVertexNormals )
      {
        N1 += ON_3dVector(mesh.m_N[vi]);
      }
      if ( bHasTextureCoordinates )
      {
        P = mesh.m_T[vi];
        Tline.to += P;
      }
    }

    if (cnt > 1)
    {
      double s = 1.0/((double)cnt);
      Vline.to.x *= s;
      Vline.to.y *= s;
      Vline.to.z *= s;
      Tline.to.x *= s;
      Tline.to.y *= s;
      Tline.to.z *= s;
      N1 = s*N1;
    }

    ON_3fPoint newV(Vline.PointAt(0.5));
    ON_3fVector newN;
    ON_2fPoint newT;
    if ( bHasVertexNormals )
    {
      N0.Unitize();
      N1.Unitize();
      ON_3dVector N = N0+N1;
      if ( !N.Unitize() )
      {
        N = (topv0.m_v_count == 1) ? mesh.m_N[topv0.m_vi[0]] :mesh.m_N[topv1.m_vi[0]];
      }
      newN = N;
    }
    if ( bHasTextureCoordinates )
    {
      newT = Tline.PointAt(0.5);
    }
    for ( mei = 0; mei < me_list_count; mei++ )
    {
      me_list[mei].newvi = newvi;
      me_list[mei].newV = newV;
      me_list[mei].newN = newN;
      me_list[mei].newT = newT;
    }
  }
  else
  {
    // collapsing a "crease" edge - attempt to preserve
    // the crease.
    memset(&me,0,sizeof(me));
    me.vi0 = -1;
    me.vi1 = -1;
    for ( mei = 0; mei < me_list_count; mei++ )
    {
      if ( 0 == mei && CompareMESHEDGE(&me,me_list+mei) )
      {
        // cook up new vertex
        me_list[mei].newvi = mesh.m_V.Count();
        me = me_list[mei];
        ON_Line line;
        line.from = mesh.m_V[me.vi0];
        line.to   = mesh.m_V[me.vi1];
        me.newV = line.PointAt(0.5);
        if ( bHasVertexNormals )
        {
          ON_3dVector N0(mesh.m_N[me.vi0]);
          ON_3dVector N1(mesh.m_N[me.vi1]);
          ON_3dVector N = N0 + N1;
          if ( !N.Unitize() )
            N = N0;
          me.newN = N;
        }
        if ( bHasTextureCoordinates )
        {
          line.from = mesh.m_T[me.vi0];
          line.to   = mesh.m_T[me.vi1];
          me.newT = line.PointAt(0.5);
        }
        me.newvi = (me.vi0 < me.vi1) ? me.vi0 : me.vi1;
      }
      else
      {
        me_list[mei].newvi = me.newvi;
        me_list[mei].newV = me.newV;
        me_list[mei].newN = me.newN;
        me_list[mei].newT = me.newT;
      }
    }
  }

  // We are done averaging old mesh values.
  // Change values in mesh m_V[], m_N[] and m_T[] arrays.
  for ( mei = 0; mei < me_list_count; mei++ )
  {
    mesh.m_V[me_list[mei].vi0] = me_list[mei].newV;
    mesh.m_V[me_list[mei].vi1] = me_list[mei].newV;
    if ( bHasVertexNormals )
    {
      mesh.m_N[me_list[mei].vi0] = me_list[mei].newN;
      mesh.m_N[me_list[mei].vi1] = me_list[mei].newN;
    }
    if ( bHasTextureCoordinates )
    {
      mesh.m_T[me_list[mei].vi0] = me_list[mei].newT;
      mesh.m_T[me_list[mei].vi1] = me_list[mei].newT;
    }
  }

  // make a map of old to new
  int old2new_map_count = 0;
  ON__NEWVI* old2new_map = (ON__NEWVI*)alloca(2*me_list_count*sizeof(old2new_map[0]));

  for ( mei = 0; mei < me_list_count; mei++ )
  {
    old2new_map[old2new_map_count].oldvi = me_list[mei].vi0;
    old2new_map[old2new_map_count].newvi = me_list[mei].newvi;
    old2new_map_count++;
    old2new_map[old2new_map_count].oldvi = me_list[mei].vi1;
    old2new_map[old2new_map_count].newvi = me_list[mei].newvi;
    old2new_map_count++;
  }

  // sort old2new_map[] so we can use a fast bsearch() call
  // to update faces.
  ON_qsort(old2new_map,old2new_map_count,sizeof(old2new_map[0]),(QSORTCMPFUNC)CompareNEWVI);

  // count faces that use the vertices that are being changed
  int bad_fi_count = 0;
  int topv_end, vei, fi, fvi23, fvi;
  ON__NEWVI nvi;

  for ( topv_end = 0; topv_end < 2; topv_end++ )
  {
    const ON_MeshTopologyVertex& topv = (topv_end) ? topv1 : topv0;
    for ( vei = 0; vei < topv.m_tope_count; vei++ )
    {
      topei = topv.m_topei[vei];
      if ( topei < 0 && topei >= top.m_tope.Count() )
        continue;
      bad_fi_count += top.m_tope[topei].m_topf_count;
    }
  }
  int* bad_fi = (int*)alloca(bad_fi_count*sizeof(*bad_fi));
  bad_fi_count = 0;

  // Go through all the faces that use the vertices at the
  // ends of the edge and update the vi[] values to use the
  // new vertices.
  for ( topv_end = 0; topv_end < 2; topv_end++ )
  {
    const ON_MeshTopologyVertex& topv = (topv_end) ? topv1 : topv0;
    for ( vei = 0; vei < topv.m_tope_count; vei++ )
    {
      topei = topv.m_topei[vei];
      if ( topei < 0 && topei >= top.m_tope.Count() )
        continue;
      const ON_MeshTopologyEdge& e = top.m_tope[topei];
      for ( efi = 0; efi < e.m_topf_count; efi++ )
      {
        fi = e.m_topfi[efi];
        if ( fi < 0 || fi >= F_count )
          continue;
        bool bChangedFace = false;
        ON_MeshFace& f = mesh.m_F[fi];
        for ( fvi = 0; fvi < 4; fvi++ )
        {
          nvi.oldvi = f.vi[fvi];
          ON__NEWVI* p = (ON__NEWVI*)bsearch(&nvi,old2new_map,old2new_map_count,sizeof(old2new_map[0]),(QSORTCMPFUNC)CompareNEWVI);
          if ( 0 != p && p->oldvi != p->newvi)
          {
            f.vi[fvi] = p->newvi;
            bChangedFace = true;
          }
        }
        if ( bChangedFace )
        {
          if ( !f.IsValid(V_count) )
          {
            if ( f.vi[3] == f.vi[0] )
            {
              f.vi[0] = f.vi[1];
              f.vi[1] = f.vi[2];
              f.vi[2] = f.vi[3];
            }
            else if ( f.vi[0] == f.vi[1] )
            {
              fvi23 = f.vi[0];
              f.vi[0] = f.vi[2];
              f.vi[1] = f.vi[3];
              f.vi[2] = fvi23;
              f.vi[3] = fvi23;
            }
            else if ( f.vi[1] == f.vi[2] )
            {
              fvi23 = f.vi[1];
              f.vi[1] = f.vi[0];
              f.vi[0] = f.vi[3];
              f.vi[2] = fvi23;
              f.vi[3] = fvi23;
            }
            if ( f.vi[0] == f.vi[1] || f.vi[1] == f.vi[2] || f.vi[2] == f.vi[0] || f.vi[2] != f.vi[3] )
            {
              bad_fi[bad_fi_count++] = fi;
            }
          }
          if ( bHasFaceNormals )
          {
            // invalid faces are removed below
            ON_3fVector a, b, n;
            a = mesh.m_V[f.vi[2]] - mesh.m_V[f.vi[0]];
            b = mesh.m_V[f.vi[3]] - mesh.m_V[f.vi[1]];
            n = ON_CrossProduct( a, b );
            n.Unitize();
            mesh.m_FN[fi] = n;
          }
        }
      }
    }
  }

  if ( bad_fi_count > 0 )
  {
    // remove collapsed faces
    ON_qsort(bad_fi,bad_fi_count,sizeof(bad_fi[0]),CompareInt);
    int bfi = 1;
    int dest_fi = bad_fi[0];
    for ( fi = dest_fi+1; fi < F_count && bfi < bad_fi_count; fi++ )
    {
      if ( fi == bad_fi[bfi] )
      {
        bfi++;
      }
      else
      {
        mesh.m_F[dest_fi++] = mesh.m_F[fi];
      }
    }
    while (fi<F_count)
    {
      mesh.m_F[dest_fi++] = mesh.m_F[fi++];
    }
    mesh.m_F.SetCount(dest_fi);

    if ( bHasFaceNormals )
    {
      bfi = 1;
      dest_fi = bad_fi[0];
      for ( fi = dest_fi+1; fi < F_count && bfi < bad_fi_count; fi++ )
      {
        if ( fi == bad_fi[bfi] )
        {
          bfi++;
        }
        else
        {
          mesh.m_FN[dest_fi++] = mesh.m_FN[fi];
        }
      }
      while (fi<F_count)
      {
        mesh.m_FN[dest_fi++] = mesh.m_FN[fi++];
      }
      mesh.m_FN.SetCount(dest_fi);
    }
  }

  mesh.Compact();
  mesh.DestroyTopology();
  mesh.DestroyPartition();

  return true;
}


//static int NewIndex( int old_index, int missing_index_count, const int* missing_index_list )
//{
//  // tool to calculate new indices when some elements are deleted from a list
//  int delta;
//  for ( delta = 0; delta < missing_index_count; delta++ )
//  {
//    if ( old_index < missing_index_list[delta] )
//      break;
//  }
//  return old_index - delta;
//}

bool ON_Mesh::DeleteFace( int meshfi )
{
  // Do NOT add a call Compact() in this function.
  // Compact() is slow and this function may be called
  // many times in sequence.  
  // It is the callers responsibility to call Compact()
  // when it is needed.
  bool rc = false;

  if ( meshfi >= 0 && meshfi < m_F.Count() )
  {
    if ( m_top.m_topf.Count() > 0 )
    {
      DestroyTopology();
    }
    DestroyPartition();
    DestroyTree();
    if ( m_FN.Count() == m_F.Count() )
    {
      m_FN.Remove(meshfi);
    }
    m_F.Remove(meshfi);

    // 6 Mar 2010 S. Baer
    // Invalidate the cached IsClosed flag. This forces the mesh to
    // recompute IsClosed the next time it is called
    SetClosed(-1);

    rc = true;
  }
  return rc;
}

ON_Mesh* ON_ControlPolygonMesh( 
          const ON_NurbsSurface& nurbs_surface, 
          bool bCleanMesh,
          ON_Mesh* input_mesh
          )
{
  int u0 = 0;
  int u1 = nurbs_surface.CVCount(0);

  int v0 = 0;
  int v1 = nurbs_surface.CVCount(1);

  if ( 0 == nurbs_surface.m_cv || !nurbs_surface.IsValid() )
  {
    ON_ERROR("ON_ControlPolygonMesh - surface is not valid");
    return NULL;
  }

  ON_SimpleArray<double> gu(u1);
  ON_SimpleArray<double> gv(v1);
  gu.SetCount(u1);
  gv.SetCount(v1);
  nurbs_surface.GetGrevilleAbcissae(0,gu.Array());
  nurbs_surface.GetGrevilleAbcissae(1,gv.Array());

  ON_Interval d0 = nurbs_surface.Domain(0);
  ON_Interval d1 = nurbs_surface.Domain(1);

  bool bPeriodic0 = nurbs_surface.IsPeriodic(0)?true:false;
  bool bIsClosed0 = bPeriodic0 ? true : (nurbs_surface.IsClosed(0)?true:false);
  bool bPeriodic1 = nurbs_surface.IsPeriodic(1)?true:false;
  bool bIsClosed1 = bPeriodic1 ? true : (nurbs_surface.IsClosed(1)?true:false);

  if ( bPeriodic0 )
  {
    u1 -= (nurbs_surface.Degree(0) - 1);
    while ( u1 < nurbs_surface.CVCount(0) && gu[u0] < d0[0] && gu[u1-1] <= d0[1] )
    {
      u0++;
      u1++;
    }
    d0.Set(gu[u0],gu[u1-1]);
  }

  if ( bPeriodic1 )
  {
    v1 -= (nurbs_surface.Degree(1) - 1);
    while ( v1 < nurbs_surface.CVCount(1) && gv[v0] < d1[0] && gv[v1-1] <= d1[1] )
    {
      v0++;
      v1++;
    }
    d1.Set(gv[v0],gv[v1-1]);
  }

  ON_Mesh* mesh = (0 == input_mesh) ? new ON_Mesh() : input_mesh;

  int vertex_count = (u1-u0)*(v1-v0);
  int face_count = (u1-u0-1)*(v1-v0-1);

  mesh->m_V.Reserve(vertex_count);
  mesh->m_N.Reserve(vertex_count);
  mesh->m_T.Reserve(vertex_count);
  mesh->m_F.Reserve(face_count);

  ON_3dPoint V;
  ON_3dVector N;
  ON_2dPoint T;

  int hint[2] = {0,0};
  int i, j;
  int k = -1;
  for ( j = v0; j < v1; j++)
  {
    T.y = d1.NormalizedParameterAt(gv[j]);
    for ( i = u0; i < u1; i++)
    {
      nurbs_surface.GetCV( i, j, V);
      T.x = d0.NormalizedParameterAt(gu[i]);
      nurbs_surface.EvNormal(gu[i],gv[j],N,0,hint);
      mesh->m_V.AppendNew() = V;
      mesh->m_N.AppendNew() = N;
      mesh->m_T.AppendNew() = T;
      if ( i > u0 && j > v0 )
      {
        ON_MeshFace& f = mesh->m_F.AppendNew();
        f.vi[0] = k++;
        f.vi[1] = k;
        f.vi[2] = mesh->m_V.Count()-1;
        f.vi[3] = f.vi[2]-1;
      }
    }
    k++;
  }
  
  u1 -= u0;
  v1 -= v0;

  // make sure closed seams are spot on
  if ( bIsClosed0 )
  {
    i = 0;
    for ( j = 0; j < v1; j++ )
    {
      k = i + (u1-1);
      mesh->m_V[k] = mesh->m_V[i];
      if ( bPeriodic0 )
      {
        mesh->m_N[k] = mesh->m_N[i];
      }
      i = k+1;
      // do NOT synch texture coordinates
    }
  }

  if ( bIsClosed1 )
  {
    for ( i = 0, k = u1*(v1-1); i < u1; i++, k++ )
    {
      mesh->m_V[k] = mesh->m_V[i];
      if ( bPeriodic1 )
      {
        mesh->m_N[k] = mesh->m_N[i];
      }
      // do NOT synch texture coordinates
    }
  }

  // make sure singular ends are spot on
  i=0;
  for ( k = 0; k < 4; k++ )
  {
    if ( nurbs_surface.IsSingular(k) )
    {
      switch(k)
      {
      case 0: // 0 = south
        i = 0;
        j = 1;
        k = u1;
        break;

      case 1: // 1 = east
        i = u1-1;
        j = u1;
        k = u1*v1;
        break;

      case 2: // 2 = north
        i = u1*(v1-1);
        j = 1;
        k = u1*v1;
        break;

      case 3: // 3 = west
        i = 0;
        j = u1;
        k = u1*(v1-1)+1;
        break;
      }
      V = mesh->m_V[i];
      for ( i = i+j; i < k; i += j )
      {
        mesh->m_V[i] = V;
      }
    }
  }

  if ( bCleanMesh )
  {
    // Clean up triangles etc.
    ON_3dPoint P[4];
    ON_SimpleArray<int> badfi(32);
    for ( i = 0; i < mesh->m_F.Count(); i++ )
    {
      ON_MeshFace& f = mesh->m_F[i];
      P[0] = mesh->m_V[f.vi[0]];
      P[1] = mesh->m_V[f.vi[1]];
      P[2] = mesh->m_V[f.vi[2]];
      P[3] = mesh->m_V[f.vi[3]];
      if ( P[0] == P[1] )
      {
        f.vi[1] = f.vi[2];
        f.vi[2] = f.vi[3];
        P[1] = P[2];
        P[2] = P[3];
      }
      if ( P[1] == P[2] )
      {
        f.vi[2] = f.vi[3];
        P[2] = P[3];
      }
      if ( P[2] == P[3] )
      {
        f.vi[2] = f.vi[3];
        P[2] = P[3];
      }
      if ( P[3] == P[0] )
      {
        f.vi[0] = f.vi[1];
        f.vi[1] = f.vi[2];
        f.vi[2] = f.vi[3];
        P[0] = P[1];
        P[1] = P[2];
        P[2] = P[3];
      }
      if (    f.vi[0] == f.vi[1] 
           || f.vi[1] == f.vi[2] 
           || f.vi[3] == f.vi[0] 
           || P[0] == P[2] || P[1] == P[3] )
      {
        badfi.Append(i);
      }
    }
    
    if ( badfi.Count() > 0 )
    {
      if ( badfi.Count() == mesh->m_F.Count() )
      {
        if ( input_mesh )
        {
          mesh->Destroy();
        }
        else
        {
          delete mesh;
        }
        mesh = 0;
      }
      else
      {
        // remove bad faces
        i = badfi[0];
        j = i+1;
        k = 1;
        for ( j = i+1; j < mesh->m_F.Count(); j++ )
        {
          if ( k < badfi.Count() && j == badfi[k] )
          {
            k++;
          }
          else
          {
            mesh->m_F[i++] = mesh->m_F[j];
          }
        }
        mesh->m_F.SetCount(i);
      }

      // 29 May 2008: Mikko, TRR 34687:
      // Added crash protection. At this point mesh is NULL if it contained all bad faces.
      if ( mesh)
        mesh->CullUnusedVertices();
    }
  }

  return mesh;
}

///////////////////////////////////////////////////////////////////////
//
// mesh components
static bool IsUnweldedEdge(int edgeidx, const ON_MeshTopology& Top)
{

  const ON_MeshTopologyEdge& edge = Top.m_tope[edgeidx];
  if (1 == edge.m_topf_count)
    return true;

  if (1 == Top.m_topv[edge.m_topvi[0]].m_v_count || 1 == Top.m_topv[edge.m_topvi[1]].m_v_count)
  {
    //Both ends of the edge have to have more than one mesh vertex or they are for sure welded.
    //However having more than 1 vertex at both ends does not necessarily mean it is unwelded.
    return false; 
  }

  ON_3fPoint ptA = Top.m_mesh->m_V[Top.m_topv[edge.m_topvi[0]].m_vi[0]];
  ON_3fPoint ptB = Top.m_mesh->m_V[Top.m_topv[edge.m_topvi[1]].m_vi[0]];
  ON_SimpleArray<int> ptAindexes(Top.m_topv[edge.m_topvi[0]].m_v_count);
  ON_SimpleArray<int> ptBindexes(Top.m_topv[edge.m_topvi[1]].m_v_count);

  int i, ict = edge.m_topf_count;
  int j, jct;
  int k, kct;
  for (i=0; ict>i; i++)
  {
    const ON_MeshFace& face = Top.m_mesh->m_F[edge.m_topfi[i]];
    jct = face.IsQuad()?4:3;
    for (j=0; jct>j; j++)
    {
      if (ptA == Top.m_mesh->m_V[face.vi[j]])
      {
        if (0 == ptAindexes.Count())
        {
          ptAindexes.Append(face.vi[j]);
          continue;
        }
        else
        {
          kct = ptAindexes.Count();
          for (k=0; kct>k; k++)
          {
            if (ptAindexes[k] == face.vi[j])
              return false;
          }
          ptAindexes.Append(face.vi[j]);
        }
      }
      else if (ptB == Top.m_mesh->m_V[face.vi[j]])
      {
        if (0 == ptBindexes.Count())
        {
          ptBindexes.Append(face.vi[j]);
          continue;
        }
        else
        {
          kct = ptBindexes.Count();
          for (k=0; kct>k; k++)
          {
            if (ptBindexes[k] == face.vi[j])
              return false;
          }
          ptBindexes.Append(face.vi[j]);
        }
      }
    }
  }

  return true;
}

static void FindAdjacentFaces(const ON_MeshTopology& Top, 
                              ON_SimpleArray<int>& FacesToCheck, 
                              const ON_SimpleArray<int>& SortedFaceArray,
                              ON_SimpleArray<int>& DupFaceArray,
                              bool bUseVertexConnections, 
                              bool bTopologicalConnections)
{
  int fi, vi, ei, facecount = FacesToCheck.Count(), totalcount = SortedFaceArray.Count();
  DupFaceArray.Zero();
  ON_SimpleArray<int> OldFacesToCheck = FacesToCheck;

  FacesToCheck.Empty();

  for (fi=0;fi<facecount;fi++)
  {
    if (totalcount > OldFacesToCheck[fi])
    {
      if (0 == SortedFaceArray[OldFacesToCheck[fi]])
      {
        FacesToCheck.Append(OldFacesToCheck[fi]);
        DupFaceArray[OldFacesToCheck[fi]] = 1;
      }

      if (false == bUseVertexConnections)
      {
        int j;
        const ON_MeshTopologyFace& face = Top.m_topf[OldFacesToCheck[fi]];
        for(ei=0;ei<(face.IsQuad()?4:3);ei++)
        {
          const ON_MeshTopologyEdge& edge = Top.m_tope[face.m_topei[ei]];

          if (1 == edge.m_topf_count || (false == bTopologicalConnections && true == IsUnweldedEdge(face.m_topei[ei], Top)))
            continue;

          for(j=0;j<edge.m_topf_count;j++)
          {  
            if (0 == SortedFaceArray[edge.m_topfi[j]] && 1 != DupFaceArray[edge.m_topfi[j]])
            {
              FacesToCheck.Append(edge.m_topfi[j]);
              DupFaceArray[edge.m_topfi[j]] = 1;
            }
          }
        }
      }
      else
      {
        int j, k, m;
        ON_3fPoint Pt;
        const ON_MeshFace& face = Top.m_mesh->m_F[OldFacesToCheck[fi]];
        for(vi=0;vi<(face.IsQuad()?4:3);vi++)
        {
          const ON_MeshTopologyVertex& vertex = Top.m_topv[Top.m_topv_map[face.vi[vi]]];
          for (j=0; vertex.m_tope_count>j; j++)
          {
            const ON_MeshTopologyEdge& edge = Top.m_tope[vertex.m_topei[j]];
            for (k=0; edge.m_topf_count>k; k++)
            {
              if (true == bTopologicalConnections)
              {
                if (0 == SortedFaceArray[edge.m_topfi[k]] && 1 != DupFaceArray[edge.m_topfi[k]])
                {
                  FacesToCheck.Append(edge.m_topfi[k]);
                  DupFaceArray[edge.m_topfi[k]] = 1;
                }
              }
              else
              {
                Pt = Top.m_mesh->m_V[vertex.m_vi[0]];
                const ON_MeshFace& thisface = Top.m_mesh->m_F[edge.m_topfi[k]];
                for (m=0; m<(thisface.IsQuad()?4:3);m++)
                {
                  if (Pt != Top.m_mesh->m_V[thisface.vi[m]])
                    continue;

                  if (face.vi[vi] == thisface.vi[m] && 0 == SortedFaceArray[edge.m_topfi[k]] && 1 != DupFaceArray[edge.m_topfi[k]])
                  {
                    //Faces share vertex 
                    FacesToCheck.Append(edge.m_topfi[k]);
                    DupFaceArray[edge.m_topfi[k]] = 1;
                    break;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}


int ON_Mesh::GetConnectedComponents( bool bUseVertexConnections, 
                                     bool bTopologicalConnections, 
                                     ON_SimpleArray<int>& facet_component_labels
                                   ) const
{
  int i, facecount = m_F.Count(), meshidx = 0;

  //This array will act as an associative array to m_F since ON_MeshFace do not have something
  //like m_trim_user_i on a ON_BrepTrim.  It will have the indice of the final mesh the face 
  //belongs to.
  if (facecount != facet_component_labels.Count())
  {
    facet_component_labels.Reserve(facecount);
    facet_component_labels.SetCount(facecount);
  }

  //initialize to 0
  facet_component_labels.MemSet(0);

  ON_SimpleArray<int> DupFaceArray(facecount);
  DupFaceArray.SetCount(facecount);

  const ON_MeshTopology& Top = Topology();
  if (!Top.IsValid())
    return 0;

  ON_SimpleArray<int> FacesToCheck;
  FacesToCheck.Reserve(64);
  i = 0;
  while (i < facecount)
  {
    meshidx++;

    FacesToCheck.Append(i);

    while(0 != FacesToCheck.Count())
    {
      //Figure out which faces are connected to each other
      FindAdjacentFaces(Top, FacesToCheck, facet_component_labels, DupFaceArray, bUseVertexConnections, bTopologicalConnections);
      int j;
      for (j=0;j<FacesToCheck.Count();j++)
        facet_component_labels[FacesToCheck[j]] = meshidx;
    }

    for(;i<facecount;i++)
    {
      if (0 == facet_component_labels[i])
        break;
    }
  }
  
  return meshidx;
}

int ON_Mesh::GetConnectedComponents( bool bUseVertexConnections, 
                                     bool bTopologicalConnections, 
                                     ON_SimpleArray<ON_Mesh*>* components
                                   ) const
{ 
  int i, j, k, kct, facecount = m_F.Count();
  ON_SimpleArray<int> SortedFaceArray(facecount);
  SortedFaceArray.SetCount(facecount);

  int compct = GetConnectedComponents(bUseVertexConnections, bTopologicalConnections, SortedFaceArray);
  if (0 == compct || 0 == components)
    return compct;

  bool bHasFaceNormals = HasFaceNormals();
  bool bHasPrincipalCurvatures = HasPrincipalCurvatures();
  bool bHasSurfaceParameters = HasSurfaceParameters();
  bool bHasTextureCoordinates = HasTextureCoordinates();
  bool bHasVertexColors = HasVertexColors();
  bool bHasVertexNormals = HasVertexNormals();

  ON_MeshFace newface;
  newface.vi[0] = -1; newface.vi[1] = -1; newface.vi[2] = -1; newface.vi[3] = -1; 

  ON_SimpleArray<int> vertidxarray(m_V.Count());
  vertidxarray.SetCount(m_V.Count());

  const ON_3dPoint* pMesh_D = 0;
  if ( HasDoublePrecisionVertices() )
  {
    if ( m_V.Count() > 0 && HasSynchronizedDoubleAndSinglePrecisionVertices() )
    {
      pMesh_D = DoublePrecisionVertices().Array();
    }
  }

  ON_3dPointArray bogus_D;

  for (i=1;compct>=i;i++)
  {
    kct = vertidxarray.Count();
    for (k=0; kct>k; k++)
      vertidxarray[k] = -1;

    ON_Mesh* pNewMesh = new ON_Mesh();
    if (0 == pNewMesh)
      continue;

    ON_3dPointArray& pNewMesh_D = ( 0 != pMesh_D )
                       ? pNewMesh->DoublePrecisionVertices()
                       : bogus_D;

	  for (j=0;j<facecount;j++)
    {
      if ( i == SortedFaceArray[j] )
      {
        const ON_MeshFace& face = m_F[j];
        kct = face.IsTriangle()?3:4; 
        for (k=0; k<kct; k++)
        {
          if (-1 != vertidxarray[face.vi[k]])
          {
            newface.vi[k] = vertidxarray[face.vi[k]];
            continue;
          }

          int newvi;
          if ( 0 != pMesh_D )
          {
            newvi = pNewMesh_D.Count();
            pNewMesh_D.Append(pMesh_D[face.vi[k]]);
          }
          else
          {
            newvi = pNewMesh->m_V.Count();
            pNewMesh->m_V.Append(m_V[face.vi[k]]);
          }

          newface.vi[k] = vertidxarray[face.vi[k]] = newvi;

          if (true == bHasPrincipalCurvatures)
            pNewMesh->m_K.Append(m_K[face.vi[k]]);

          if (true == bHasSurfaceParameters)
            pNewMesh->m_S.Append(m_S[face.vi[k]]);

          if (true == bHasTextureCoordinates)
            pNewMesh->m_T.Append(m_T[face.vi[k]]);

          if (true == bHasVertexColors)
            pNewMesh->m_C.Append(m_C[face.vi[k]]);

          if (true == bHasVertexNormals)
            pNewMesh->m_N.Append(m_N[face.vi[k]]);
        }

        if (3 == kct)
          newface.vi[3] = newface.vi[2];

        pNewMesh->m_F.Append(newface);
        if (true == bHasFaceNormals)
          pNewMesh->m_FN.Append(m_FN[j]);
      }
    }

    if ( 0 != pMesh_D )
      pNewMesh->UpdateSinglePrecisionVertices();

    pNewMesh->Compact();

    if (0 < pNewMesh->m_F.Count())
      components->Append(pNewMesh);
    else
    {
      delete pNewMesh;
      pNewMesh = 0;
    }
  }

  return compct;
}
