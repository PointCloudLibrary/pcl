/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2011 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Assoicates.
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
#include "pcl/surface/3rdparty/opennurbs/opennurbs_gl.h" // ON_GL() function declarations

void ON_GL( const int order,     // ON_NurbsCurve order
              const int cv_count,  // ON_NurbsCurve cv count
              const double* knot,  // ON_NurbsCurve knot vector
              GLfloat* glknot,     // GL knot vector
              int bPermitScaling,  // true if knot scaling is allowed
              double* scale        // If not NULL and knot scaling is
                                   // allowed, then the scaling
                                   // parameters are returned here.
                                   // glknot = (knot - scale[0])*scale[1]
            )
{
  // Because GL uses floats instead of doubles for knot vectors and
  // because some GLs are intolerant of closely spaced knots,
  // the returned glknots[] may be re-scaled when bPermitScaling
  // is true.  When the knots belong to a trimmed surface, any rescaling
  // done to the surface's knots must be applied to the trimming geometry.

  const int knot_count = order + cv_count - 2;
  const int nknots = knot_count+2;
  
  // GL knot vectors have old-fashioned extra knot at start and end
  const double k0 = ON_SuperfluousKnot( order, cv_count, knot,0);
  const double k1 = ON_SuperfluousKnot( order, cv_count, knot,1);

  if ( scale ) {
    scale[0] = 0.0;
    scale[1] = 1.0;
  }
  
  int i, j;
  if ( bPermitScaling ) {
    double d0 = knot[order-2];
    double dk = 1.0;
    if ( bPermitScaling ) {
      double dmin = 1.0;
      double dmax = 1.0;
      double d;
      for ( i = 1; i < knot_count; i++ ) {
        d = knot[i] - knot[i-1];
        if ( d <= 0.0 )
          continue; // multiple knot
        if ( d < dmin )
          dmin = d;
        else if ( d > dmax )
          dmax = d;
      }
      if ( dmin > 0.0 && dmax >= dmin ) {
        if ( dmin < 1.0e-2 )
          dk = 1.0e-2/dmin;
        else if ( dmax > 1.0e4 ) {
          if ( 1.0e4*dmin >= 1.0e-2*dmax )
            dk = 1.0e4/dmax;        
        }
      }
    }
    if ( scale ) {
      scale[0] = d0;
      scale[1] = dk;
    }
    glknot[0] = (GLfloat)((k0-d0)*dk);
    for( i = 1, j = 0; j < knot_count; i++, j++ )
      glknot[i] = (GLfloat)((knot[j]-d0)*dk);
    glknot[nknots-1] = (GLfloat)((k1-d0)*dk);
  }
  else {
    glknot[0] = (GLfloat)k0;
    for( i = 1, j = 0; j < knot_count; i++, j++ )
      glknot[i] = (GLfloat)knot[j];
    glknot[nknots-1] = (GLfloat)k1;
  }
}

static void GetGLCV( const int dim, const int is_rat, const double* cv,
                     double xform[4][4], 
                     GLfloat* glcv )
{
  if ( xform ) {
    const double x = cv[0];
    const double y = cv[1];
    const double z = (dim == 3) ? cv[2] : 0.0;
    const double w = (is_rat) ? cv[dim] : 1.0;
    glcv[0] = (GLfloat)(xform[0][0]*x + xform[0][1]*y + xform[0][2]*z + xform[0][3]*w);
    glcv[1] = (GLfloat)(xform[1][0]*x + xform[1][1]*y + xform[1][2]*z + xform[1][3]*w);
    if ( dim == 3 )
      glcv[2] = (GLfloat)(xform[2][0]*x + xform[2][1]*y + xform[2][2]*z + xform[2][3]*w);
    if ( is_rat )
      glcv[dim] = (GLfloat)(xform[3][0]*x + xform[3][1]*y + xform[3][2]*z + xform[3][3]*w);
  }
  else {
    glcv[0] = (GLfloat)cv[0];
    glcv[1] = (GLfloat)cv[1];
    if ( dim == 3)
      glcv[2] = (GLfloat)cv[2];
    if ( is_rat )
      glcv[dim] = (GLfloat)cv[dim];
  }
}

void ON_GL( const ON_NurbsCurve& nurbs_curve,
              GLUnurbsObj* nobj, // created with gluNewNurbsRenderer )
              GLenum type, // = 0 (and type is automatically set)
              int bPermitKnotScaling,
              double* knot_scale,
              double xform[][4]
            )
{
  ON_GL( nurbs_curve.Dimension(), 
         nurbs_curve.IsRational(),
         nurbs_curve.Order(),
         nurbs_curve.CVCount(),
         nurbs_curve.Knot(),
         nurbs_curve.m_cv_stride,
         nurbs_curve.m_cv,
         nobj,
         type,
         bPermitKnotScaling,
         knot_scale,
         xform
         );
}

void ON_GL( const ON_Curve& curve,
              GLUnurbsObj* nobj, // created with gluNewNurbsRenderer )
              GLenum type, // = 0 (and type is automatically set)
              double xform[][4]
            )
{
  const ON_PolyCurve* poly_curve = ON_PolyCurve::Cast(&curve);
  if ( poly_curve ) 
  {
    ON_Curve* pSegmentCurve = 0;
    int segment_count = poly_curve->Count();
    int i;
    for ( i = 0; i < segment_count; i++ ) {
      pSegmentCurve = poly_curve->SegmentCurve(i);
      if ( pSegmentCurve )
        ON_GL( *pSegmentCurve, nobj, type, xform );
    }
    return;
  }

  const ON_CurveProxy* curve_proxy = ON_CurveProxy::Cast(&curve);
  if ( curve_proxy && !curve_proxy->ProxyCurveIsReversed() ) 
  {
    const ON_Curve* real_curve = curve_proxy->ProxyCurve();
    if ( 0 == real_curve )
      return;
    if ( curve_proxy == real_curve )
      return;
    if ( curve_proxy->ProxyCurveDomain() == real_curve->Domain() )
    {
      ON_GL( *real_curve, nobj, type, xform );
      return;
    }
  }

  {
    ON_NurbsCurve tmp;
    const ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(&curve);
    if ( !nurbs_curve ) 
    {
      if ( curve.GetNurbForm(tmp) )
        nurbs_curve = &tmp;
    }
    ON_GL( *nurbs_curve, nobj, type, true, NULL, xform );
  }
}

void ON_GL( int dim, int is_rat, int nurb_order, int cv_count,
            const double* knot_vector, 
            int cv_stride, const double* cv,
            GLUnurbsObj* nobj,
            GLenum type,
            int bPermitKnotScaling,
            double* knot_scale,
            double xform[][4]
            )
{
  ON_BOOL32 bCallgluBeginEndCurve = false;
  int i;

  GLint nknots = nurb_order + cv_count; // GL knot count = TL knot count + 2
  GLfloat* knot = (GLfloat*)onmalloc( nknots*sizeof(*knot) );
  ON_GL( nurb_order, cv_count, knot_vector, knot, bPermitKnotScaling, knot_scale );

  // control vertices
  //const int cv_size = (is_rat) ? dim+1: dim;
  GLint stride = cv_stride;
  GLfloat* ctlarray = (GLfloat*)onmalloc( stride*cv_count*sizeof(*ctlarray) );
  for ( i = 0; i < cv_count; i++ ) {
    GetGLCV( dim, is_rat, cv + i*cv_stride, xform, ctlarray + stride*i );
  }

  GLint order = nurb_order;
  switch(type)
  {
  case 0:
    {
      switch ( dim ) {
      case 2: // must be a GLU_MAP1_TRIM_2/3
        type = ( is_rat ) 
             ? GLU_MAP1_TRIM_3   // rational 2d trim uses homogeneous coords
             : GLU_MAP1_TRIM_2;  // non-rational 2d trim uses euclidean coords
        break;
      case 3: // must be a GLU_MAP1_VERTEX_3/4
        type = ( is_rat ) 
             ? GL_MAP1_VERTEX_4  // rational 3d curve uses homogeneous coords
             : GL_MAP1_VERTEX_3; // non-rational 3d curve used euclidean coords
        bCallgluBeginEndCurve = true;
        break;
      }
    }
    break;

  case GLU_MAP1_TRIM_2:
  case GLU_MAP1_TRIM_3:
    // make sure type matches rational flag
    type = ( is_rat ) 
         ? GLU_MAP1_TRIM_3   // rational 2d trim uses homogeneous coords
         : GLU_MAP1_TRIM_2;  // non-rational 2d trim uses euclidean coords
    break;

  case GL_MAP1_VERTEX_3:
  case GL_MAP1_VERTEX_4:
    // make sure type matches rational flag
    type = ( is_rat ) 
         ? GL_MAP1_VERTEX_4  // rational 3d curve uses homogeneous coords
         : GL_MAP1_VERTEX_3; // non-rational 3d curve used euclidean coords
    bCallgluBeginEndCurve = true;
    break;
  }

  if ( bCallgluBeginEndCurve )
    gluBeginCurve(nobj);
    gluNurbsCurve(
    nobj,
    nknots,
    knot,
    stride, 	
    ctlarray, 	
    order, 	
    type	
  );	
  if ( bCallgluBeginEndCurve )
    gluEndCurve(nobj);

  onfree( ctlarray );
  onfree( knot );
}



// See comments in opennurbs_gl.h for calling instructions.

void ON_GL( const ON_NurbsSurface& s,
              GLUnurbsObj* nobj, // created with gluNewNurbsRenderer )
              GLenum type,       // = 0 (and type is automatically set)
              int bPermitKnotScaling,
              double* knot_scale0,
              double* knot_scale1
             )
{
  int i, j, k;

  // The "bPermitScaling" parameters to the ON_GL() call that
  // fills in the knot vectors is set to false because any
  // rescaling that is applied to a surface domain must also
  // be applied to parameter space trimming curve geometry.

  // GL "s" knots
  GLint sknot_count = s.KnotCount(0) + 2;
  GLfloat* sknot = (GLfloat*)onmalloc( sknot_count*sizeof(*sknot) );
  ON_GL( s.Order(0), s.CVCount(0), s.Knot(0), sknot, 
           bPermitKnotScaling, knot_scale0 );

  // GL "t" knots
  GLint tknot_count = s.KnotCount(1) + 2;
  GLfloat* tknot = (GLfloat*)onmalloc( tknot_count*sizeof(*tknot) );
  ON_GL( s.Order(1), s.CVCount(1), s.Knot(1), tknot,
           bPermitKnotScaling, knot_scale1 );

  // control vertices
  const int cv_size= s.CVSize();
  const int cv_count[2] = {s.CVCount(0), s.CVCount(1)};
  GLint s_stride = cv_size*cv_count[1];
  GLint t_stride = cv_size;
  GLfloat* ctlarray = (GLfloat*)onmalloc( s_stride*cv_count[0]*sizeof(*ctlarray) );
  for ( i = 0; i < cv_count[0]; i++ ) {
    for ( j = 0; j < cv_count[1]; j++ ) {
      const double*  cv = s.CV(i,j);
      GLfloat* gl_cv = ctlarray + s_stride*i + t_stride*j;
      for ( k = 0; k < cv_size; k++ ) {
        gl_cv[k] = (GLfloat)cv[k];
      }
    }
  }
  
  GLint sorder = s.Order(0);
  GLint torder = s.Order(1);

  if ( type == 0 ) {
    // set GL surface type for 3d CVs in homogeneous/euclidean form.
    type = ( s.IsRational() ) ? GL_MAP2_VERTEX_4 : GL_MAP2_VERTEX_3;
  }

  gluNurbsSurface (
    nobj,
    sknot_count,
    sknot,
    tknot_count,
    tknot,
    s_stride, 	
    t_stride, 	
    ctlarray, 	
    sorder, 	
    torder, 	
    type	
  );	

  onfree( ctlarray );
  onfree( tknot );
  onfree( sknot );
}

void ON_GL( const ON_Brep& brep,
            GLUnurbsObj* nobj     // created with gluNewNurbsRenderer )
          )
{
  const int face_count = brep.m_F.Count();
  int face_index;
  for ( face_index = 0; face_index < face_count; face_index++ ) {
    const ON_BrepFace& face = brep.m_F[face_index];
    ON_GL( face, nobj );
  }
}

// See comments in opennurbs_gl.h for calling instructions.

void ON_GL( const ON_BrepFace& face,
            GLUnurbsObj* nobj     // created with gluNewNurbsRenderer )
          )
{
  bool bSkipTrims = false;

  const ON_Mesh* mesh;
  mesh = face.Mesh(ON::render_mesh);
  if ( mesh ) 
  {
    // use saved render mesh
    ON_GL(*mesh);
  }
  else 
  {
    // use (slow and buggy) glu trimmed NURBS rendering
    double knot_scale[2][2] = {{0.0,1.0},{0.0,1.0}};
    const ON_Brep* brep = face.Brep();
    if ( !brep )
      return;

    // untrimmed surface
    {
      ON_NurbsSurface tmp_nurbssrf;
      const ON_Surface* srf = brep->m_S[face.m_si];
      const ON_NurbsSurface* nurbs_srf = ON_NurbsSurface::Cast(srf);
      if ( !nurbs_srf ) 
      {
        // attempt to get NURBS form of this surface
        if ( srf->GetNurbForm( tmp_nurbssrf ) )
          nurbs_srf = &tmp_nurbssrf;
      }
      if ( !nurbs_srf )
        return;
      gluBeginSurface( nobj );
      ON_GL( *nurbs_srf,
             nobj, 
             (nurbs_srf->IsRational()) ? GL_MAP2_VERTEX_4 : GL_MAP2_VERTEX_3,
             true, knot_scale[0], knot_scale[1]
            );
    }

    if ( bSkipTrims || brep->FaceIsSurface( face.m_face_index ) ) {
      gluEndSurface( nobj );
      return; // face is trivially trimmed
    }

    int fli, li, lti, ti;

    // any knot scaling applied to the surface must also be applied to
    // the parameter space trimming geometry
    double xform[4][4] 
      = {{knot_scale[0][1], 0.0, 0.0, -knot_scale[0][0]*knot_scale[0][1] },
         {0.0, knot_scale[1][1], 0.0, -knot_scale[1][0]*knot_scale[1][1] },
         {0.0, 0.0, 1.0, 0.0},
         {0.0, 0.0, 0.0, 1.0}};

    // Add face's 2d trimming loop(s)
    const int face_loop_count = face.m_li.Count();
    for ( fli = 0; fli < face_loop_count; fli++ ) 
    {
      gluBeginTrim( nobj );

      li = face.m_li[fli];
      const ON_BrepLoop& loop = brep->m_L[li];
      const int loop_trim_count = loop.m_ti.Count();
      for ( lti = 0; lti < loop_trim_count; lti++ )
      {
        ti = loop.m_ti[lti];
        const ON_BrepTrim& trim = brep->m_T[ti];
        ON_GL( trim,
               nobj, 
               GLU_MAP1_TRIM_2,
               xform
              );
      }

      gluEndTrim( nobj );
    }
    gluEndSurface( nobj );
  }
}

void ON_GL( const ON_Mesh& mesh )
{
  int i0, i1, i2, j0, j1, j2;
  int fi;
  ON_3fPoint v[4];
  ON_3fVector n[4];
  ON_2fPoint t[4];

  const int face_count = mesh.FaceCount();
  const ON_BOOL32 bHasNormals = mesh.HasVertexNormals();
  const ON_BOOL32 bHasTCoords = mesh.HasTextureCoordinates();

  glBegin(GL_TRIANGLES);
  for ( fi = 0; fi < face_count; fi++ ) {
    const ON_MeshFace& f = mesh.m_F[fi];

    v[0] = mesh.m_V[f.vi[0]];
    v[1] = mesh.m_V[f.vi[1]];
    v[2] = mesh.m_V[f.vi[2]];


    if ( bHasNormals ) {
      n[0] = mesh.m_N[f.vi[0]];
      n[1] = mesh.m_N[f.vi[1]];
      n[2] = mesh.m_N[f.vi[2]];
    }

    if ( bHasTCoords ) {
      t[0] = mesh.m_T[f.vi[0]];
      t[1] = mesh.m_T[f.vi[1]];
      t[2] = mesh.m_T[f.vi[2]];
    }

    if ( f.IsQuad() ) {
      // quadrangle - render as two triangles
      v[3] = mesh.m_V[f.vi[3]];
      if ( bHasNormals )
        n[3] = mesh.m_N[f.vi[3]];
      if ( bHasTCoords )
        t[3] = mesh.m_T[f.vi[3]];
      if ( v[0].DistanceTo(v[2]) <= v[1].DistanceTo(v[3]) ) {
        i0 = 0; i1 = 1; i2 = 2;
        j0 = 0; j1 = 2; j2 = 3;
      }
      else {
        i0 = 1; i1 = 2; i2 = 3;
        j0 = 1; j1 = 3; j2 = 0;
      }
    }
    else {
      // single triangle
      i0 = 0; i1 = 1; i2 = 2;
      j0 = j1 = j2 = 0;
    }

    // first triangle
    if ( bHasNormals )
      glNormal3f( n[i0].x, n[i0].y, n[i0].z );
    if ( bHasTCoords )
      glTexCoord2f( t[i0].x, t[i0].y );
    glVertex3f( v[i0].x, v[i0].y, v[i0].z );

    if ( bHasNormals )
      glNormal3f( n[i1].x, n[i1].y, n[i1].z );
    if ( bHasTCoords )
      glTexCoord2f( t[i1].x, t[i1].y );
    glVertex3f( v[i1].x, v[i1].y, v[i1].z );

    if ( bHasNormals )
      glNormal3f( n[i2].x, n[i2].y, n[i2].z );
    if ( bHasTCoords )
      glTexCoord2f( t[i2].x, t[i2].y );
    glVertex3f( v[i2].x, v[i2].y, v[i2].z );

    if ( j0 != j1 ) {
      // if we have a quad, second triangle
      if ( bHasNormals )
        glNormal3f( n[j0].x, n[j0].y, n[j0].z );
      if ( bHasTCoords )
        glTexCoord2f( t[j0].x, t[j0].y );
      glVertex3f( v[j0].x, v[j0].y, v[j0].z );

      if ( bHasNormals )
        glNormal3f( n[j1].x, n[j1].y, n[j1].z );
      if ( bHasTCoords )
        glTexCoord2f( t[j1].x, t[j1].y );
      glVertex3f( v[j1].x, v[j1].y, v[j1].z );

      if ( bHasNormals )
        glNormal3f( n[j2].x, n[j2].y, n[j2].z );
      if ( bHasTCoords )
        glTexCoord2f( t[j2].x, t[j2].y );
      glVertex3f( v[j2].x, v[j2].y, v[j2].z );
    }

  }
  glEnd();
}

void ON_GL( 
      const ON_3dPoint& point
      )
{
  glVertex3d( point.x, point.y, point.z );
}

void ON_GL( 
      const ON_Point& point
      )
{
  glBegin(GL_POINTS);
  ON_GL(point.point);
  glEnd();
}

void ON_GL( const ON_PointCloud& cloud )
{
  int i;
  ON_3dPoint P;
  glBegin(GL_POINTS);
  for ( i = 0; i < cloud.PointCount(); i++ ) {
    ON_GL( cloud.m_P[i] );
  }
  glEnd();
}

void ON_GL( const ON_Material& m )
{
  ON_GL( &m );
}

void ON_GL( const ON_Color& rc, double alpha, GLfloat c[4] )
{
  c[0] = (GLfloat)rc.FractionRed();
  c[1] = (GLfloat)rc.FractionGreen();
  c[2] = (GLfloat)rc.FractionBlue();
  c[3] = (GLfloat)alpha;
}

void ON_GL( const ON_Color& rc, GLfloat c[4] )
{
  c[0] = (GLfloat)rc.FractionRed();
  c[1] = (GLfloat)rc.FractionGreen();
  c[2] = (GLfloat)rc.FractionBlue();
  c[3] = (GLfloat)1.0;
}


void ON_GL( const ON_Material* pMat )
{
  // set GL material to match Rhino material
  if ( !pMat ) {
    ON_Material default_mat;
    ON_GL( &default_mat );
  }
  else {
    GLfloat ambient[4], diffuse[4], specular[4], emission[4];
    GLfloat alpha = (GLfloat)(1.0 - pMat->Transparency());
    ON_GL( pMat->Ambient(), alpha, ambient );
    ON_GL( pMat->Diffuse(), alpha, diffuse );
    ON_GL( pMat->Specular(), alpha, specular );
    ON_GL( pMat->Emission(), alpha, emission );
    GLint shine = (GLint)(128.0*(pMat->Shine() / ON_Material::MaxShine()));
    if ( shine == 0 ) {
      specular[0]=specular[1]=specular[2]=(GLfloat)0.0;
    }
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT,   ambient  );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE,   diffuse  );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR,  specular );
    glMaterialfv( GL_FRONT_AND_BACK, GL_EMISSION,  emission );
    glMateriali(  GL_FRONT_AND_BACK, GL_SHININESS, shine );
  }
}

void ON_GL( const ON_Light* light, GLenum light_index )
{
  ON_Light default_light;
  if ( !light ) {
    default_light.Default();
    light = &default_light;
  }
  ON_GL( *light, light_index );
}

void ON_GL( const ON_Light& light, GLenum light_index )
{
  ON_BOOL32 bPopModelViewMatrix = false;
  ON_BOOL32 bPopProjectionMatrix = false;

  switch ( light.CoordinateSystem() ) 
  {
  case ON::world_cs:
    break;
  case ON::clip_cs:
    bPopProjectionMatrix = true;
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    // no break here
  case ON::camera_cs:
    bPopModelViewMatrix = true;
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    break;
  case ON::screen_cs:
    break;
  }

  GLfloat ambient[4], diffuse[4], specular[4];
  ON_GL( light.Ambient(), ambient );
  ON_GL( light.Diffuse(), diffuse );
  ON_GL( light.Specular(), specular );
  glLightfv( light_index, GL_AMBIENT, ambient );
  glLightfv( light_index, GL_DIFFUSE, diffuse );
  glLightfv( light_index, GL_SPECULAR, specular );

  ON_3dPoint loc = light.Location();
  GLfloat f[4] = {(GLfloat)loc.x,(GLfloat)loc.y,(GLfloat)loc.z,(GLfloat)1.0};
  glLightfv( light_index, GL_POSITION, f );

  ON_3dVector dir = light.Direction();
  f[0] = (GLfloat)dir.x; 
  f[1] = (GLfloat)dir.y; 
  f[2] = (GLfloat)dir.z;
  glLightfv( light_index, GL_SPOT_DIRECTION, f );

  glLightf( light_index, GL_SPOT_EXPONENT, (GLfloat)(light.SpotExponent()*128.0)  );
  glLightf( light_index, GL_SPOT_CUTOFF, (GLfloat)light.SpotAngleRadians() );

  ON_3dVector attenuation = light.Attenuation();
  glLightf( light_index, GL_CONSTANT_ATTENUATION,  (GLfloat)attenuation.x  );
  glLightf( light_index, GL_LINEAR_ATTENUATION,    (GLfloat)attenuation.y  );
  glLightf( light_index, GL_QUADRATIC_ATTENUATION, (GLfloat)attenuation.z  );

  if ( light.IsEnabled() )
    glEnable( light_index );
  else
    glDisable( light_index );
  if ( bPopProjectionMatrix ) {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
  }
  if ( bPopModelViewMatrix ) {
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }
}

void ON_GL( ON_Viewport& viewport,
            int port_left, int port_right,
            int port_bottom, int port_top
          )
{
  // Sets viewport's port to port_* values and adjusts frustum
  // so it's aspect matches the port's.
  ON_Xform projectionMatrix; // camera to clip transformation

  const int port_width  = abs(port_right - port_left);
  const int port_height = abs(port_top - port_bottom);
  if ( port_width == 0 || port_height == 0 )
    return;
  const double port_aspect = ((double)port_width)/((double)port_height);

  viewport.SetFrustumAspect( port_aspect );

  viewport.SetScreenPort( port_left, port_right, port_bottom, port_top,
                          0, 0xff );

  ON_BOOL32 bHaveCameraToClip = viewport.GetXform( 
                                       ON::camera_cs,  
                                       ON::clip_cs,
                                       projectionMatrix 
                                       );

  if ( bHaveCameraToClip ) {
    projectionMatrix.Transpose();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd( &projectionMatrix.m_xform[0][0] );
  }
}

void ON_GL( const ON_Viewport& viewport )
{
  // sets model view matrix (world to camera transformation)
  ON_Xform modelviewMatrix; // world to camera transformation
  ON_BOOL32 bHaveWorldToCamera = viewport.GetXform( 
                                       ON::world_cs,  
                                       ON::camera_cs,
                                       modelviewMatrix 
                                       );
  if ( bHaveWorldToCamera ) {
    modelviewMatrix.Transpose();
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd( &modelviewMatrix.m_xform[0][0] );
  }
}

void ON_GL( 
      const ON_Surface& surface,   // 
      GLUnurbsObj* nobj           // created with gluNewNurbsRenderer
      )
{
  ON_NurbsSurface tmp;
  const ON_NurbsSurface* nurbs_surface;
  nurbs_surface = ON_NurbsSurface::Cast(&surface);
  if ( !nurbs_surface ) {
    if ( surface.GetNurbForm(tmp) ) {
      nurbs_surface = &tmp;
    }
  }
  if ( nurbs_surface )
    ON_GL( *nurbs_surface, nobj, 0, true );
}
