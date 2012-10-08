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

ON_OBJECT_IMPLEMENT( ON_Viewport, ON_Geometry, "D66E5CCF-EA39-11d3-BFE5-0010830122F0" );

static double len2d( double x, double y )
{
  double d= 0.0;
  double fx = fabs(x);
  double fy = fabs(y);
  if ( fx > fy ) {
    d = fy/fx;
    d = fx*sqrt(1.0+d*d);
  }
  else if ( fy > fx ){
    d = fx/fy;
    d = fy*sqrt(1.0+d*d);
  }
  return d;
}

static void unitize2d( double x, double y, double* ux, double* uy )
{
  const double eps = 2.0*ON_SQRT_EPSILON;
  // carefully turn two numbers into a 2d unit vector
  double s, c, d;
  c = x;
  s = y;
  if ( s == 0.0 ) {
    c = (c < 0.0) ? -1.0 : 1.0;
  }
  else {
    if ( fabs(s) > fabs(c) ) {
      d = c/s;
      d = fabs(s)*sqrt(1.0+d*d);
    }
    else {
      d = s/c;
      d = fabs(c)*sqrt(1.0+d*d);
    }
    d = 1.0/d;
    if ( fabs(d-1.0) > eps ) {
      s *= d;
      c *= d;
    }
    if ( fabs(s) <= eps || fabs(c) >= 1.0-eps ) {
      s = 0.0;
      c = (c < 0.0) ? -1.0 : 1.0;
    }
    else if ( fabs(c) < eps || fabs(s) >= 1.0-eps) {
      c = 0.0;
      s = (s < 0.0) ? -1.0 : 1.0;
    }
  }
  if ( ux ) 
    *ux = c;
  if ( uy )
    *uy = s;
}


static
bool ON__IsCameraFrameUnitVectorHelper( const ON_3dVector& v )
{
  // looser standard than ON_3dVector::IsUnitVector() so
  // going to/from floats in OpenGL and Direct3d doesn't
  // create "invalid" views.
  return (v.x != ON_UNSET_VALUE && v.y != ON_UNSET_VALUE && v.z != ON_UNSET_VALUE && fabs(v.Length() - 1.0) <= 1.0e-6);
}

static
bool ON__IsCameraFramePerpindicular( const ON_3dVector& unit_vector0,const ON_3dVector& unit_vector1 )
{
  return ( fabs(unit_vector0.x*unit_vector1.x + unit_vector0.y*unit_vector1.y + unit_vector0.z*unit_vector1.z) <= 1.0e-6 );
}

bool 
ON_GetViewportRotationAngles( 
    const ON_3dVector& X, // X,Y,Z must be a right handed orthonormal basis
    const ON_3dVector& Y, 
    const ON_3dVector& Z,
    double* angle1, // returns rotation about world Z
    double* angle2, // returns rotation about world X ( 0 <= a2 <= pi )
    double* angle3  // returns rotation about world Z
    )
{
  // double a1 = 0.0;  // rotation about world Z
  // double a2 = 0.0;  // rotation about world X ( 0 <= a2 <= pi )
  // double a3 = 0.0;  // rotation about world Z
  bool bValidFrame = false;
  double sin_a1 = 0.0;
  double cos_a1 = 1.0;
  double sin_a2 = 0.0;
  double cos_a2 = 1.0;
  double sin_a3 = 0.0;
  double cos_a3 = 1.0;

  // If si = sin(ai) and ci = cos(ai), then the relationship between the camera
  // frame and the angles is defined by the matrix equation C = R3*R2*R1, where:
  //
  //      c1 -s1  0        1   0   0         c3 -s3  0
  // R1 = s1  c1  0  R2 =  0  c2 -s2   R3 =  s3  c3  0
  //       0   0  1        0  s2  c2          0   0  1
  //
  //     CamX[0]  CamY[0] CamZ[0]
  // C = CamX[1]  CamY[1] CamZ[1]
  //     CamX[2]  CamY[2] CamZ[2]
  //
  //              .       .     s2*s3
  //
  // R3*R2*R1 =   .       .    -s2*c3
  //
  //            s1*s2   c1*s2    c2
  //

  {
    // don't attempt to work with slop
    const double eps = 8.0*ON_SQRT_EPSILON;
    double dx,dy,dz,d;
    dx = X*X;
    dy = Y*Y;
    dz = Z*Z;
    if ( fabs(dx-1.0) <= eps && fabs(dy-1.0) <= eps && fabs(dz-1.0) <= eps ) {
      dx = X*Y;
      dy = Y*Z;
      dz = Z*X;
      if ( fabs(dx) <= eps && fabs(dy) <= eps && fabs(dz) <= eps ) {
        d = ON_TripleProduct( X, Y, Z );
        bValidFrame = (d > 0.0);
      }
    }
  }

  if ( bValidFrame ) 
  {
    // Usually "Z" = opposite of unitized camera direction.
    //         "Y" = camera up made ortho to "Z" and unitized.
    //         "X" = YxZ.
    // So, when possible, I solve for angles in terms
    // of "Z" and "Y" since "X" will generally have the most noise.
    //         
    // Use C = R3*R2*R1 to get sin(a2), cos(a2).
    cos_a2 = Z.z;
    sin_a2 = len2d(Z.x,Z.y);
    unitize2d(cos_a2,sin_a2,&cos_a2,&sin_a2); // kill noise

    if ( sin_a2 > 0.0 ) {
      // use bottom row to get angle1.
      sin_a1 = X.z;
      cos_a1 = Y.z;
      unitize2d(cos_a1,sin_a1,&cos_a1,&sin_a1); // kill noise

      // use right column to get angle3
      cos_a3 = -Z.y;
      sin_a3 =  Z.x;
      unitize2d(cos_a3,sin_a3,&cos_a3,&sin_a3); // kill noise
    }
    else if ( cos_a2 == 1.0 ) {
      // R2 = identity and C determines (angle1+angle3)
      //      arbitrarily set angle1 = 0.
      cos_a3 =  Y.y; // = cos(angle3+angle1)
      sin_a3 = -Y.x; // = sin(angle3+angle1)
    }
    else if ( cos_a2 == -1.0 ) {
      // R2 = [1 0 0 / 0 -1 0/ 0 0 -1] and C determines (angle3-angle1)
      //      arbitrarily set angle1 = 0
      cos_a3 = -Y.y; // = cos(angle3-angle1)
      sin_a3 =  Y.x; // = sin(angle3-angle1)
    }
  }


  if ( cos_a1 == -1.0 && sin_a1 == 0.0 ) {
    // when a1 = pi, juggle angles to get a1 = 0 with
    // same effective rotation to keep legacy 3d apps
    // happy.
    // a1: pi -> 0
    // a2: a2 -> 2pi - a2
    // a1: a3 ->  pi + a3
    sin_a1 = 0.0;
    cos_a1 = 0.0;
    sin_a2 = -sin_a2;
    sin_a3 = -sin_a3;
    cos_a3 = -cos_a3;
  }
  
  if ( angle1 )
    *angle1 = atan2( sin_a1, cos_a1 );
  if ( angle2 )
    *angle2 = atan2( sin_a2, cos_a2 );
  if ( angle3 )
    *angle3 = atan2( sin_a3, cos_a3 );

  return bValidFrame;
}

void ON_Viewport::SetPerspectiveClippingPlaneConstraints(
        unsigned int depth_buffer_bit_depth
        )
{
  double min_near_dist = 0.0;
  double min_near_over_far = 0.0;
  ON_Viewport::GetPerspectiveClippingPlaneConstraints(m_CamLoc,depth_buffer_bit_depth,&min_near_dist,&min_near_over_far);
  SetPerspectiveMinNearDist(min_near_dist);
  SetPerspectiveMinNearOverFar(min_near_over_far);
}


void ON_Viewport::SetPerspectiveMinNearOverFar(double min_near_over_far)
{
  if (    ON_IsValid(min_near_over_far) 
       && min_near_over_far > ON_ZERO_TOLERANCE
       && min_near_over_far < 1.0-ON_ZERO_TOLERANCE
     )
  {
    m__MIN_NEAR_OVER_FAR = min_near_over_far;
  }
}

double ON_Viewport::PerspectiveMinNearOverFar() const
{
  return m__MIN_NEAR_OVER_FAR;
}

void ON_Viewport::SetPerspectiveMinNearDist(double min_near_dist)
{
  if ( ON_IsValid(min_near_dist) && min_near_dist > ON_ZERO_TOLERANCE )
  {
    m__MIN_NEAR_DIST = min_near_dist;
  }
}

double ON_Viewport::PerspectiveMinNearDist() const
{
  return m__MIN_NEAR_DIST;
}



// Discuss any changes of these values with Dale Lear
const double ON_Viewport::DefaultNearDist = 0.005;
const double ON_Viewport::DefaultFarDist = 1000.0;
const double ON_Viewport::DefaultMinNearDist = 0.0001;
const double ON_Viewport::DefaultMinNearOverFar = 0.0001;

// For 32 bit float based OpenGL drivers, the value of
// the ON_Viewport::DefaultMinNearOverFar constant must 
// be <0.01 and >= 0.0001.  
// If you change this value, you need to retest RR 8902 on OpenGL
// drivers that (internally) use float precision transformations.
// Some OpenGL drivers, like the Microsoft software emulation
// driver for XP crash in some cases when near/far > 1e8.
//
// ON_Viewport::DefaultMinNearOverFar = 0.001    // used in Rhino 3.0 beta testing until 11 Sep 2002
// ON_Viewport::DefaultMinNearOverFar = 0.01     // used for Rhino 3.0 CD1 and CD2
// ON_Viewport::DefaultMinNearOverFar = 0.000001 // used for Rhino 3.0 CD3
// ON_Viewport::DefaultMinNearOverFar = 0.0001   // used for Rhino 4.0 Fixes RR 8902

void
ON_Viewport::Initialize()
{
  m__MIN_NEAR_DIST     = ON_Viewport::DefaultMinNearDist;
  m__MIN_NEAR_OVER_FAR = ON_Viewport::DefaultMinNearOverFar;

  m_bValidCamera = true;
  m_bValidFrustum = true;
  m_bValidPort = false;
  m_reserved1 = 0;
  m_projection = ON::parallel_view;

  m_bLockCamUp = false;
  m_bLockCamDir = false;
  m_bLockCamLoc = false;
  m_frustum_symmetry_flags = 0;

  m_CamLoc.x =   0.0;
  m_CamLoc.y =   0.0;
  m_CamLoc.z = 100.0;
  m_CamDir = -ON_zaxis;
  m_CamUp = ON_yaxis;
  m_CamX = ON_xaxis;
  m_CamY = ON_yaxis;
  m_CamZ = ON_zaxis;
  m_frus_left = -20.0;
  m_frus_right = 20.0;
  m_frus_bottom = -20.0;
  m_frus_top = 20.0;
  m_frus_near = m__MIN_NEAR_DIST;
  m_frus_far = ON_Viewport::DefaultFarDist;
  m_port_left = 0;
  m_port_right = 1000;
  m_port_bottom = 0;
  m_port_top = 1000;
  m_port_near = 0;//0xffff;
  m_port_far =  1;//0;
  m_clip_mods.Identity();
  m_clip_mods_inverse.Identity();
  m_target_point = ON_UNSET_POINT;
  m_viewport_id = ON_nil_uuid;
}

const ON_3dVector ON_Viewport::Default3dCameraDirection(-0.43301270189221932338186158537647,0.75,-0.5);

ON_Viewport::ON_Viewport()
{
  Initialize();
}

ON_Viewport::~ON_Viewport()
{}

ON_Viewport& ON_Viewport::operator=( const ON_Viewport& src )
{
  if ( this != &src ) 
  {
    ON_Object::operator=(src);

    m_bValidCamera   = src.m_bValidCamera;
    m_bValidFrustum  = src.m_bValidFrustum;
    m_bValidPort     = src.m_bValidPort;

    m_projection     = src.m_projection;

    m_bLockCamUp  = src.m_bLockCamUp;
    m_bLockCamDir = src.m_bLockCamDir;
    m_bLockCamLoc = src.m_bLockCamLoc;
    m_frustum_symmetry_flags = src.m_frustum_symmetry_flags;

    m_CamLoc = src.m_CamLoc;
    m_CamDir = src.m_CamDir;
    m_CamUp  = src.m_CamUp;
    m_CamX   = src.m_CamX;
    m_CamY   = src.m_CamY;
    m_CamZ   = src.m_CamZ;

    m_frus_left      = src.m_frus_left;
    m_frus_right     = src.m_frus_right;
    m_frus_bottom    = src.m_frus_bottom;
    m_frus_top       = src.m_frus_top;
    m_frus_near      = src.m_frus_near;
    m_frus_far       = src.m_frus_far;

    m_port_left      = src.m_port_left;
    m_port_right     = src.m_port_right;
    m_port_bottom    = src.m_port_bottom;
    m_port_top       = src.m_port_top;
    m_port_near      = src.m_port_near;
    m_port_far       = src.m_port_far;

    m_target_point = src.m_target_point;

    m_clip_mods         = src.m_clip_mods;
    m_clip_mods_inverse = src.m_clip_mods_inverse;

    m__MIN_NEAR_OVER_FAR = src.m__MIN_NEAR_OVER_FAR;
    m__MIN_NEAR_DIST     = src.m__MIN_NEAR_DIST;

    m_viewport_id = src.m_viewport_id;
  }
  return *this;
}


ON_BOOL32 ON_Viewport::Read( ON_BinaryArchive& file )
{
  Initialize();
  int major_version = 0;
  int minor_version = 1;
  bool rc = file.Read3dmChunkVersion(&major_version,&minor_version);
  if (rc && major_version==1) 
  {
    // common to all 1.x versions
    int i=0;
    if (rc) rc = file.ReadInt( &i );
    if (rc) m_bValidCamera = (i?true:false);
    if (rc) rc = file.ReadInt( &i );
    if (rc) m_bValidFrustum = (i?true:false);
    if (rc) rc = file.ReadInt( &i );
    if (rc) m_bValidPort = (i?true:false);
    if (rc) rc = file.ReadInt( &i );
    if (rc) m_projection = ON::ViewProjection(i);
    if (rc) rc = file.ReadPoint( m_CamLoc );
    if (rc) rc = file.ReadVector( m_CamDir );
    if (rc) rc = file.ReadVector( m_CamUp );
    if (rc) rc = file.ReadVector( m_CamX );
    if (rc) rc = file.ReadVector( m_CamY );
    if (rc) rc = file.ReadVector( m_CamZ );
    if (rc) rc = file.ReadDouble( &m_frus_left );
    if (rc) rc = file.ReadDouble( &m_frus_right );
    if (rc) rc = file.ReadDouble( &m_frus_bottom );
    if (rc) rc = file.ReadDouble( &m_frus_top );
    if (rc) rc = file.ReadDouble( &m_frus_near );
    if (rc) rc = file.ReadDouble( &m_frus_far );
    if (rc) rc = file.ReadInt( &m_port_left );
    if (rc) rc = file.ReadInt( &m_port_right );
    if (rc) rc = file.ReadInt( &m_port_bottom );
    if (rc) rc = file.ReadInt( &m_port_top );
    if (rc) rc = file.ReadInt( &m_port_near );
    if (rc) rc = file.ReadInt( &m_port_far );   

    if (rc && minor_version >= 1 )
    {
      // 1.1 fields
      if (rc) rc = file.ReadUuid(m_viewport_id);

      if (rc && minor_version >= 2 )
      {
        // 1.2 fields 
        bool b;
        b = false;
        if (rc) rc = file.ReadBool(&b);
        if (rc) SetCameraUpLock(b);

        b = false;
        if (rc) rc = file.ReadBool(&b);
        if (rc) SetCameraDirectionLock(b);

        b = false;
        if (rc) rc = file.ReadBool(&b);
        if (rc) SetCameraLocationLock(b);

        b = false;
        if (rc) rc = file.ReadBool(&b);
        if (rc) SetFrustumLeftRightSymmetry(b);

        b = false;
        if (rc) rc = file.ReadBool(&b);
        if (rc) SetFrustumTopBottomSymmetry(b);
      }
    }

    if ( m_bValidCamera )
    {
      if ( !m_CamLoc.IsValid() || !m_CamUp.IsValid() || !m_CamDir.IsValid() )
      {
        ON_ERROR("ON_Viewport.m_bValidCamera in file was true and it should be false.");
        m_bValidCamera = false;
      }
    }

    if( m_bValidFrustum )
    {
      if (    !ON_IsValid(m_frus_left) || !ON_IsValid(m_frus_right)
           || !ON_IsValid(m_frus_top)  || !ON_IsValid(m_frus_bottom)
           || !ON_IsValid(m_frus_near) || !ON_IsValid(m_frus_far)
           || m_frus_right <= m_frus_left
           || m_frus_top   <= m_frus_bottom
           || m_frus_near  <= 0.0
           || m_frus_far   <= m_frus_near
         )
      {
        ON_ERROR("ON_Viewport.m_bValidFrustum in file was true and it should be false.");
        m_bValidFrustum = false;
      }
    }
  }
  return rc;
}

ON_BOOL32 ON_Viewport::Write( ON_BinaryArchive& file ) const
{
  int i;
  bool rc = file.Write3dmChunkVersion(1,2);
  if (rc) 
  {
    i = m_bValidCamera?1:0;
    if (rc) rc = file.WriteInt( i );
    i = m_bValidFrustum?1:0;
    if (rc) rc = file.WriteInt( i );
    i = m_bValidPort?1:0;
    if (rc) rc = file.WriteInt( i );
    i = m_projection;
    if ( file.Archive3dmVersion() <= 4 && IsPerspectiveProjection() )
    {
      // V4 files do not support 2 point perspective projection
      i = ON::perspective_view;
    }
    if (rc) rc = file.WriteInt( i );
    if (rc) rc = file.WritePoint( m_CamLoc );
    if (rc) rc = file.WriteVector( m_CamDir );
    if (rc) rc = file.WriteVector( m_CamUp );
    if (rc) rc = file.WriteVector( m_CamX );
    if (rc) rc = file.WriteVector( m_CamY );
    if (rc) rc = file.WriteVector( m_CamZ );
    if (rc) rc = file.WriteDouble( m_frus_left );
    if (rc) rc = file.WriteDouble( m_frus_right );
    if (rc) rc = file.WriteDouble( m_frus_bottom );
    if (rc) rc = file.WriteDouble( m_frus_top );
    if (rc) rc = file.WriteDouble( m_frus_near );
    if (rc) rc = file.WriteDouble( m_frus_far );
    if (rc) rc = file.WriteInt( m_port_left );
    if (rc) rc = file.WriteInt( m_port_right );
    if (rc) rc = file.WriteInt( m_port_bottom );
    if (rc) rc = file.WriteInt( m_port_top );
    if (rc) rc = file.WriteInt( m_port_near );
    if (rc) rc = file.WriteInt( m_port_far );    

    // 1.1 fields
    if (rc) rc = file.WriteUuid(m_viewport_id);

    // 1.2 fields 
    bool b;

    b = CameraUpIsLocked();
    if (rc) rc = file.WriteBool(b);

    b = CameraDirectionIsLocked();
    if (rc) rc = file.WriteBool(b);

    b = CameraLocationIsLocked();
    if (rc) rc = file.WriteBool(b);

    b = FrustumIsLeftRightSymmetric();
    if (rc) rc = file.WriteBool(b);

    b = FrustumIsTopBottomSymmetric();
    if (rc) rc = file.WriteBool(b);
  }
  return rc;
}

bool ON_Viewport::IsValidCamera() const
{
  return ( m_bValidCamera );
}

bool ON_Viewport::IsValidFrustum() const
{
  return ( m_bValidFrustum );
}

ON_BOOL32 ON_Viewport::IsValid( ON_TextLog* text_log ) const
{
  if ( !IsValidCamera() )
  {
    if ( 0 != text_log )
    {
      text_log->Print("invalid viewport camera settings.\n");
    }
    return false;
  }
  if ( !IsValidFrustum() )
  {
    if ( 0 != text_log )
    {
      text_log->Print("invalid viewport frustum settings.\n");
    }
    return false;
  }
  if ( !m_bValidPort )
  {
    if ( 0 != text_log )
    {
      text_log->Print("invalid viewport port extents settings.\n");
    }
    return false;
  }
  return true;
}


int ON_Viewport::Dimension() const
{
  return 3;
}

bool ON_Viewport::GetNearPlane( ON_Plane& near_plane ) const
{
  bool rc = IsValidFrustum() && IsValidCamera();
  if ( rc ) 
  {
    near_plane.origin = m_CamLoc - m_frus_near*m_CamZ;
    near_plane.xaxis = m_CamX;
    near_plane.yaxis = m_CamY;
    near_plane.zaxis = m_CamZ;
    near_plane.UpdateEquation();
  }
  return rc;
}

bool ON_Viewport::GetNearPlaneEquation( 
  ON_PlaneEquation& near_plane_equation 
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {
    near_plane_equation.ON_3dVector::operator=(m_CamZ);
    near_plane_equation.d = -near_plane_equation.ON_3dVector::operator*(m_CamLoc - m_frus_near*m_CamZ);
  }
  return rc;
}


bool ON_Viewport::GetFarPlane( ON_Plane& far_plane ) const
{
  bool rc = IsValidFrustum() && IsValidCamera();
  if ( rc ) 
  {
    far_plane.origin = m_CamLoc - m_frus_far*m_CamZ;
    far_plane.xaxis = m_CamX;
    far_plane.yaxis = m_CamY;
    far_plane.zaxis = m_CamZ;
    far_plane.UpdateEquation();
  }
  return rc;
}

bool ON_Viewport::GetFarPlaneEquation( 
  ON_PlaneEquation& far_plane_equation 
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {
    far_plane_equation.ON_3dVector::operator=(m_CamZ);
    far_plane_equation.d = -far_plane_equation.ON_3dVector::operator*(m_CamLoc - m_frus_far*m_CamZ);
  }
  return rc;
}


bool ON_Viewport::GetViewPlane( 
  double view_plane_depth,
  ON_Plane& view_plane 
  ) const
{
  bool rc = IsValidFrustum() && IsValidCamera();
  if ( rc ) 
  {
    view_plane.origin = m_CamLoc - view_plane_depth*m_CamZ;
    view_plane.xaxis = m_CamX;
    view_plane.yaxis = m_CamY;
    view_plane.zaxis = m_CamZ;
    view_plane.UpdateEquation();
  }
  return rc;
}

bool ON_Viewport::GetViewPlaneEquation( 
  double view_plane_depth,
  ON_PlaneEquation& view_plane_equation 
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {
    view_plane_equation.ON_3dVector::operator=(m_CamZ);
    view_plane_equation.d = -view_plane_equation.ON_3dVector::operator*(m_CamLoc - view_plane_depth*m_CamZ);
  }
  return rc;
}


bool ON_Viewport::GetNearRect( 
       ON_3dPoint& left_bottom,
       ON_3dPoint& right_bottom,
       ON_3dPoint& left_top,
       ON_3dPoint& right_top
       ) const
{
  ON_Plane near_plane;
  bool rc = GetNearPlane( near_plane );
  if (rc ) {
    double x = 1.0, y = 1.0;
    GetViewScale(&x,&y);
    x = 1.0/x;
    y = 1.0/y;
    left_bottom  = near_plane.PointAt( x*m_frus_left,  y*m_frus_bottom );
    right_bottom = near_plane.PointAt( x*m_frus_right, y*m_frus_bottom );
    left_top     = near_plane.PointAt( x*m_frus_left,  y*m_frus_top );
    right_top    = near_plane.PointAt( x*m_frus_right, y*m_frus_top );
  }
  return rc;
}

bool ON_Viewport::GetFarRect( 
       ON_3dPoint& left_bottom,
       ON_3dPoint& right_bottom,
       ON_3dPoint& left_top,
       ON_3dPoint& right_top
       ) const
{
  ON_Plane far_plane;
  bool rc = GetFarPlane( far_plane );
  if (rc )
  {
    double s = IsPerspectiveProjection()
            ? m_frus_far/m_frus_near
            : 1.0;
    double x = 1.0, y = 1.0;
    GetViewScale(&x,&y);
    x = 1.0/x;
    y = 1.0/y;
    left_bottom  = far_plane.PointAt( s*x*m_frus_left,  s*y*m_frus_bottom );
    right_bottom = far_plane.PointAt( s*x*m_frus_right, s*y*m_frus_bottom );
    left_top     = far_plane.PointAt( s*x*m_frus_left,  s*y*m_frus_top );
    right_top    = far_plane.PointAt( s*x*m_frus_right, s*y*m_frus_top );
  }
  return rc;
}

bool ON_Viewport::GetViewPlaneRect(
        double view_plane_depth,
        ON_3dPoint& left_bottom,
        ON_3dPoint& right_bottom,
        ON_3dPoint& left_top,
        ON_3dPoint& right_top
        ) const
{
  ON_Plane view_plane;
  bool rc = GetViewPlane( view_plane_depth, view_plane );
  if (rc )
  {
    double s = IsPerspectiveProjection()
            ? view_plane_depth/m_frus_near
            : 1.0;
    double x = 1.0, y = 1.0;
    GetViewScale(&x,&y);
    x = 1.0/x;
    y = 1.0/y;
    left_bottom  = view_plane.PointAt( s*x*m_frus_left,  s*y*m_frus_bottom );
    right_bottom = view_plane.PointAt( s*x*m_frus_right, s*y*m_frus_bottom );
    left_top     = view_plane.PointAt( s*x*m_frus_left,  s*y*m_frus_top );
    right_top    = view_plane.PointAt( s*x*m_frus_right, s*y*m_frus_top );
  }
  return rc;
}


ON_BOOL32 ON_Viewport::GetBBox( 
       double* boxmin,
       double* boxmax,
       ON_BOOL32 bGrowBox
       ) const
{
  ON_3dPoint corners[9];
  bool rc = GetNearRect(corners[0],corners[1],corners[2],corners[3]);
  if (rc)
    rc = GetFarRect(corners[4],corners[5],corners[6],corners[7]);
  corners[8] = m_CamLoc;
  if (rc)
  {
    rc = ON_GetPointListBoundingBox( 
            3, 0, 9, 
            3, &corners[0].x, 
            boxmin, boxmax,  bGrowBox?true:false
            );
  }
  return rc;
}

ON_BOOL32 ON_Viewport::Transform( const ON_Xform& xform )
{
  ON_BOOL32 rc = IsValidCamera();
  if (rc) {
    // save input settings
    const ON_3dPoint c0 = m_CamLoc;
    const ON_3dPoint u0 = m_CamUp;
    const ON_3dPoint d0 = m_CamDir;
    const ON_3dPoint x0 = m_CamX;
    const ON_3dPoint y0 = m_CamY;
    const ON_3dPoint z0 = m_CamZ;

    // compute transformed settings
    ON_3dPoint c = xform*c0;
    ON_3dVector u = (xform*(c0 + u0)) - c;
    ON_3dVector d = (xform*(c0 + d0)) - c;

    if ( m_bLockCamLoc )
      c = m_CamLoc;
    if ( m_bLockCamUp )
      u = m_CamY;
    if ( m_bLockCamDir )
      d = -m_CamZ;

    if (    !u.IsValid() || !d.IsValid() 
         || u.IsTiny()   || d.IsTiny() 
         || ON_CrossProduct(u,d).IsTiny() ) 
    {
      rc = false;
    }
    else 
    {
      if ( m_bLockCamUp && !m_bLockCamDir )
      {
        d.Unitize();
        if ( fabs(d*u) <= ON_ZERO_TOLERANCE )
          d = -m_CamZ;
      }
      else if ( m_bLockCamDir && !m_bLockCamUp )
      {
        u.Unitize();
        if ( fabs(d*u) <= ON_ZERO_TOLERANCE )
          u = m_CamY;
      }

      // set new camera position
      if ( !m_bLockCamLoc )
        SetCameraLocation(c);
      if ( !m_bLockCamDir )
        SetCameraDirection(d);
      if ( !m_bLockCamUp)
        SetCameraUp(u);
      rc = SetCameraFrame();
      if ( !rc ) 
      {
        // restore input settings
        m_CamLoc = c0;
        m_CamUp  = u0;
        m_CamDir = d0;
        m_CamX = x0;
        m_CamY = y0;
        m_CamZ = z0;
      }
    }
  }
  return rc;
}

bool ON_Viewport::SetCameraLocation( const ON_3dPoint& p )
{
  if ( m_bLockCamLoc )
  {
    if ( m_CamLoc.IsValid() )
    {
      return (p == m_CamLoc);
    }
  }
  if ( p != ON_3dPoint::UnsetPoint && !p.IsValid() )
    return false;
  m_CamLoc = p;
  if ( !m_CamLoc.IsValid() )
    m_bValidCamera = false;
  return m_bValidCamera;
}

bool ON_Viewport::SetCameraDirection( const ON_3dVector& v )
{
  if ( m_bLockCamDir )
  {
    if ( m_CamDir.IsValid() && !m_CamDir.IsTiny() )
    {
      return (v == m_CamDir);
    }
  }
  if ( !v.IsValid() || v.IsTiny() )
    return false;
  m_CamDir = v;
  return SetCameraFrame();
}

bool ON_Viewport::SetCameraUp( const ON_3dVector& v )
{
  if ( m_bLockCamUp )
  {
    if ( m_CamUp.IsValid() && !m_CamUp.IsTiny() )
    {
      return (v == m_CamUp);
    }
  }
  if ( !v.IsValid() || v.IsTiny() )
    return false;
  m_CamUp = v;
  return SetCameraFrame();
}

//ON_BOOL32 ON_Viewport::SetTargetDistance( double d )
//{
//  m_target_distance = (d>0.0) ? d : 0.0;
//  return (d>=0.0) ? true : false;
//}

bool ON_Viewport::SetCameraFrame()
{
  m_bValidCamera = false;

  if ( !m_CamDir.IsValid() || !m_CamUp.IsValid() )
    return false;

  double d;
  ON_3dVector CamX, CamY, CamZ;

  if ( m_bLockCamUp && !m_bLockCamDir )
  {
    // up takes precedence over direction
    CamY = m_CamUp;
    if ( !CamY.IsValid() )
      return false;
    if ( !CamY.Unitize() )
      return false;

    d = m_CamDir*CamY;
    CamZ = -m_CamDir + d*CamY;
    if ( !CamZ.IsValid() )
      return false;
    if ( !CamZ.Unitize() )
      return false;
  }
  else
  {
    // direction takes precedence over up
    CamZ = -m_CamDir;
    if ( !CamZ.IsValid() )
      return false;
    if ( !CamZ.Unitize() )
      return false;

    d = m_CamUp*CamZ;
    CamY = m_CamUp - d*CamZ;
    if ( !CamY.IsValid() )
      return false;
    if ( !CamY.Unitize() )
      return false;
  }

  CamX = ON_CrossProduct( CamY, CamZ );
  if ( !CamX.IsValid() )
    return false;
  if ( !CamX.Unitize() )
    return false;

  // Gaurd against garbage resulting from nearly parallel 
  // and/or ultra short short dir and up.
  if ( !ON__IsCameraFrameUnitVectorHelper(CamX) )
    return false;
  if ( !ON__IsCameraFrameUnitVectorHelper(CamY) )
    return false;
  if ( !ON__IsCameraFrameUnitVectorHelper(CamZ) )
    return false;
  if ( !ON__IsCameraFramePerpindicular(CamX,CamY) )
    return false;
  if ( !ON__IsCameraFramePerpindicular(CamY,CamZ) )
    return false;
  if ( !ON__IsCameraFramePerpindicular(CamZ,CamX) )
    return false;

  m_CamX = CamX;
  m_CamY = CamY;
  m_CamZ = CamZ;

  m_bValidCamera = m_CamLoc.IsValid();
  return m_bValidCamera;
}

ON_3dPoint ON_Viewport::CameraLocation() const
{
  return m_CamLoc;
}

ON_3dVector ON_Viewport::CameraDirection() const
{
  return m_CamDir;
}

ON_3dVector ON_Viewport::CameraUp() const
{
  return m_CamUp;
}

bool ON_Viewport::CameraLocationIsLocked() const
{
  return m_bLockCamLoc;
}

bool ON_Viewport::CameraDirectionIsLocked() const
{
  return m_bLockCamDir;
}

bool ON_Viewport::CameraUpIsLocked() const
{
  return m_bLockCamUp;
}

bool ON_Viewport::FrustumIsLeftRightSymmetric() const
{
  return (0 != (0x02 & m_frustum_symmetry_flags));
}

bool ON_Viewport::FrustumIsTopBottomSymmetric() const
{
  return (0 != (0x01 & m_frustum_symmetry_flags));
}

void ON_Viewport::UnlockCamera()
{
  SetCameraLocationLock(false);
  SetCameraDirectionLock(false);
  SetCameraUpLock(false);
}

void ON_Viewport::UnlockFrustumSymmetry()
{
  SetFrustumLeftRightSymmetry(false);
  SetFrustumTopBottomSymmetry(false);
}

void ON_Viewport::SetCameraLocationLock( bool bLockCameraLocation )
{
  m_bLockCamLoc = bLockCameraLocation ? true : false;
}

void ON_Viewport::SetCameraDirectionLock( bool bLockCameraDirection ) 
{
  m_bLockCamDir = bLockCameraDirection ? true : false;
}

void ON_Viewport::SetCameraUpLock( bool bLockCameraUp )
{
  m_bLockCamUp = bLockCameraUp ? true : false;
}

void ON_Viewport::SetFrustumLeftRightSymmetry( bool bForceLeftRightSymmetry )
{
  if ( bForceLeftRightSymmetry )
    m_frustum_symmetry_flags |= 0x02; // set bit 2
  else 
    m_frustum_symmetry_flags &= 0xFD; // clear bit 2
}

void ON_Viewport::SetFrustumTopBottomSymmetry( bool bForceTopBottomSymmetry )
{
  if ( bForceTopBottomSymmetry )
    m_frustum_symmetry_flags |= 0x01; // set bit 1
  else 
    m_frustum_symmetry_flags &= 0xFE; // clear bit 1
}



bool ON_Viewport::GetDollyCameraVector(
         int x0, int y0,    // (x,y) screen coords of start point
         int x1, int y1,    // (x,y) screen coords of end point
         double distance_to_camera, // distance from camera
         ON_3dVector& dolly_vector// dolly vector returned here
         ) const
{
  int port_left, port_right, port_bottom, port_top;
  ON_Xform c2w;
  dolly_vector.Zero();
  bool rc = GetScreenPort( &port_left, &port_right, &port_bottom, &port_top );
  if ( rc )
    rc = GetXform( ON::clip_cs, ON::world_cs, c2w );
  if ( rc ) {
    double dx = 0.5*(port_right - port_left);
    double dy = 0.5*(port_top - port_bottom);
    double dz = 0.5*(FrustumFar() - FrustumNear());
    if ( dx == 0.0 || dy == 0.0 || dz == 0.0 )
      rc = false;
    else {
      double z = (distance_to_camera - FrustumNear())/dz - 1.0;
      ON_3dPoint c0( (x0-port_left)/dx-1.0, (y0-port_bottom)/dy-1.0, z );
      ON_3dPoint c1( (x1-port_left)/dx-1.0, (y1-port_bottom)/dy-1.0, z );
      ON_3dPoint w0 = c2w*c0;
      ON_3dPoint w1 = c2w*c1;
      dolly_vector = w0 - w1;
    }
  }
  return rc;
}

bool ON_Viewport::DollyCamera( const ON_3dVector& dolly )
{
  bool rc = false;
  if ( m_CamLoc.IsValid() && dolly.IsValid() )
  {
    m_CamLoc += dolly;
    rc = m_bValidCamera;
  }
  return rc;
}

bool ON_Viewport::DollyFrustum( double dollyDistance )
{
  bool rc = false;
  double new_near, new_far, scale;
  if ( m_bValidFrustum ) 
  {
    new_near = m_frus_near + dollyDistance;
    new_far  = m_frus_far + dollyDistance;
    if ( IsPerspectiveProjection() && new_near < m__MIN_NEAR_DIST ) 
    {
      new_near = m__MIN_NEAR_DIST;
    }
    scale = ( IsPerspectiveProjection() ) 
          ? new_near/m_frus_near 
          : 1.0;
    if ( new_near > 0.0 && new_far > new_near && scale > 0.0 ) 
    {
      m_frus_near = new_near;
      m_frus_far  = new_far;
      m_frus_left   *= scale;
      m_frus_right  *= scale;
      m_frus_top    *= scale;
      m_frus_bottom *= scale;
      rc = true;
    }
  }
  return rc;
}

bool ON_Viewport::GetCameraFrame(
    double* CameraLocation,
    double* CameraX,
    double* CameraY,
    double* CameraZ
    ) const
{
  if ( CameraLocation ) {
    CameraLocation[0] = m_CamLoc.x;
    CameraLocation[1] = m_CamLoc.y;
    CameraLocation[2] = m_CamLoc.z;
  }
  if ( CameraX ) {
    CameraX[0] = m_CamX.x;
    CameraX[1] = m_CamX.y;
    CameraX[2] = m_CamX.z;
  }
  if ( CameraY ) {
    CameraY[0] = m_CamY.x;
    CameraY[1] = m_CamY.y;
    CameraY[2] = m_CamY.z;
  }
  if ( CameraZ ) {
    CameraZ[0] = m_CamZ.x;
    CameraZ[1] = m_CamZ.y;
    CameraZ[2] = m_CamZ.z;
  }
  return m_bValidCamera;
}

ON_3dVector ON_Viewport::CameraX() const
{
  return m_CamX;
}

ON_3dVector ON_Viewport::CameraY() const
{
  return m_CamY;
}

ON_3dVector ON_Viewport::CameraZ() const
{
  return m_CamZ;
}

bool ON_Viewport::IsCameraFrameWorldPlan( 
      int* xindex,
      int* yindex,
      int* zindex
      )
{
  int i;
  int ix = 0;
  int iy = 0;
  int iz = 0;
  double X[3], Y[3], Z[3];
  bool rc = GetCameraFrame( NULL, X, Y, Z );
  if ( rc ) {
    for ( i = 0; i < 3; i++ ) {
      if ( X[i] == 1.0 ) {
        ix = i+1;
        break;
      }
      if ( X[i] == -1.0 ) {
        ix = -(i+1);
        break;
      }
    }
    for ( i = 0; i < 3; i++ ) {
      if ( Y[i] == 1.0 ) {
        iy = i+1;
        break;
      }
      if ( Y[i] == -1.0 ) {
        iy = -(i+1);
        break;
      }
    }
    for ( i = 0; i < 3; i++ ) {
      if ( Z[i] == 1.0 ) {
        iz = i+1;
        break;
      }
      if ( Z[i] == -1.0 ) {
        iz = -(i+1);
        break;
      }
    }
    rc = ( iz != 0 ) ? 1 : 0;
  }
  if ( xindex ) *xindex = ix;
  if ( yindex ) *yindex = iy;
  if ( zindex ) *zindex = iz;
  return rc;
}


bool ON_Viewport::GetCameraExtents( 
    // returns bounding box in camera coordinates - this is useful information
    // for setting view frustrums to include the point list
    int count,            // count = number of 3d points
    int stride,           // stride = number of doubles to skip between points (>=3)
    const double* points, // 3d points in world coordinates
    ON_BoundingBox& cbox, // bounding box in camera coordinates
    int bGrowBox         // set to true to grow existing box
    ) const
{
  ON_Xform w2c;
  bool rc = bGrowBox?true:false;
  int i;
  if ( count > 0 && stride >= 3 && points != NULL ) {
    rc = false;
    if ( GetXform( ON::world_cs, ON::camera_cs, w2c ) ) {
      rc = true;
      for ( i = 0; i < count && rc; i++, points += stride ) {
        rc = cbox.Set( w2c*ON_3dPoint(points), bGrowBox );
        bGrowBox = true;
      }
    }
  }
  return rc;
}

bool ON_Viewport::GetCameraExtents( 
    // returns bounding box in camera coordinates - this is useful information
    // for setting view frustrums to include the point list
    const ON_BoundingBox& wbox, // world coordinate bounding box
    ON_BoundingBox& cbox, // bounding box in camera coordinates
    int bGrowBox         // set to true to grow existing box
    ) const
{
  bool rc = false;
  ON_3dPointArray corners;
  if ( wbox.GetCorners( corners ) ) {
    rc = GetCameraExtents( corners.Count(), 3, &corners.Array()[0].x, cbox, bGrowBox );
  }
  return rc;
}

bool ON_Viewport::GetCameraExtents( 
    // returns bounding box in camera coordinates - this is useful information
    // for setting view frustrums to include the point list
    ON_3dPoint& worldSphereCenter,
    double worldSphereRadius,
    ON_BoundingBox& cbox, // bounding box in camera coordinates
    int bGrowBox         // set to true to grow existing box
    ) const
{
  bool rc = false;
  ON_BoundingBox sbox;
  if ( GetCameraExtents( 1, 3, &worldSphereCenter.x, sbox, false ) ) {
    const double r = fabs( worldSphereRadius );
    sbox[0][0] -= r;
    sbox[0][1] -= r;
    sbox[0][2] -= r;
    sbox[1][0] += r;
    sbox[1][1] += r;
    sbox[1][2] += r;
    if ( bGrowBox )
      cbox.Union( sbox );
    else
      cbox = sbox;
    rc = true;
  }
  return rc;
}


static void UpdateTargetPointHelper( ON_Viewport& vp, double target_distance )
{
  if ( !vp.IsValidCamera() || !vp.IsValidFrustum() )
    return;
  if ( !ON_IsValid(target_distance) || target_distance <= 0.0 )
    return;

  ON_3dPoint old_tp = vp.TargetPoint();

  // Put the target directly in front of the camera.
  // The target_tol test is here to avoid making insignificant 
  // changes that appear in the user interface and upset users
  // who find 1.00000000001 to be grossly different from 1.0.
  double target_tol = 1.0e-5*(vp.FrustumWidth()+vp.FrustumHeight())
                    + ON_ZERO_TOLERANCE;
  ON_3dPoint new_tp = vp.CameraLocation() - target_distance*vp.CameraZ();
  if ( new_tp.IsValid() 
       && (!old_tp.IsValid() || new_tp.DistanceTo(old_tp) > target_tol)
     )
  {
    vp.SetTargetPoint(new_tp); 
  }
}

bool ON_Viewport::ChangeToParallelProjection( bool bSymmetricFrustum )
{
  bool rc = (m_bValidCamera && m_bValidFrustum);

  SetCameraUpLock(false);
  SetCameraDirectionLock(false);

  if (    ON::parallel_view == m_projection 
       && (bSymmetricFrustum?true:false) == FrustumIsLeftRightSymmetric()
       && (bSymmetricFrustum?true:false) == FrustumIsTopBottomSymmetric()
     )
  {
    // no changes are required
    return rc;
  }

  // if needed, make frustum symmetric
  // If bSymmetricFrustum is true and the input frustum is not symmetric, 
  // then this will dolly the camera location.
  ChangeToSymmetricFrustum(bSymmetricFrustum,bSymmetricFrustum,ON_UNSET_VALUE);
  SetFrustumTopBottomSymmetry(bSymmetricFrustum);
  SetFrustumLeftRightSymmetry(bSymmetricFrustum);

  const ON::view_projection old_projection = m_projection;
  double target_distance = TargetDistance(true);
  if ( !ON_IsValid(target_distance) 
       || !m_bValidFrustum
       || !ON_IsValid(m_frus_near)
       || m_frus_near <= 0.0
       || target_distance <= m_frus_near 
       )
  {
    target_distance = 0.0; // makes it easier to check for valid target distance
  }

  // if needed change projection
  if ( ON::parallel_view != old_projection )
  {
    if ( !SetProjection(ON::parallel_view) )
      rc = false;
  }

  if ( rc )
  {
    if ( ON::perspective_view == old_projection )
    {
      // change from a perspective to a parallel projection
      if ( target_distance > 0.0 && 0.0 < m_frus_near && m_frus_near < m_frus_far )
      {
        // Update the frustum so that the plane through the target point
        // is the one that is parallel projected.  This is generally
        // the best choice when switching from perspective to
        // parallel projection. If needed, SetFrustum() will make the
        // frustum symmetric
        double s = target_distance/m_frus_near;
        double l = m_frus_left*s; 
        double r = m_frus_right*s; 
        double t = m_frus_top*s; 
        double b = m_frus_bottom*s;
        if ( !SetFrustum( l, r, b, t, m_frus_near, m_frus_far ))
          rc = false;
      }
    }
    if ( m_target_point.IsValid() )
      UpdateTargetPointHelper(*this,target_distance);
  }

  return rc;
}

static bool ChangeFromParallelToPerspectiveHelper( ON_Viewport& vp, double target_distance, double lens_length )
{
  // helper use by ChangeToPerspectiveProjection() and ChangeToTwoPointPerspectiveProjection()
  if ( ON::perspective_view == vp.Projection() )
    return true;

  if ( !vp.SetProjection(ON::perspective_view) )
    return false;

  // change from a parallel to a perspective  projection
  double frus_left,frus_right,frus_bottom,frus_top,frus_near,frus_far;
  if ( !vp.GetFrustum(&frus_left,&frus_right,&frus_bottom,&frus_top,&frus_near,&frus_far) )
    return false;

  // Using width because it works for both two point and ordinary perspective
  const double width = fabs(frus_right - frus_left);
  const ON_3dPoint width_point = ( ON_IsValid(target_distance) && target_distance > 0.0) 
                               ? vp.CameraLocation() - target_distance*vp.CameraZ()
                               : ON_3dPoint::UnsetPoint;

  if ( frus_near < 1.0e-8 && frus_far >= 1.0e-7)
  {
    frus_near = 1.0e-8;
    vp.SetFrustum(frus_left,frus_right,frus_bottom,frus_top,frus_near,frus_far);
    vp.GetFrustum(&frus_left,&frus_right,&frus_bottom,&frus_top,&frus_near,&frus_far);
  }

  bool rc = false;

  if ( ON_IsValid(lens_length) && lens_length > 0.0 )
  {
    rc = vp.SetCamera35mmLensLength(lens_length);
    if ( rc 
         && width_point.IsValid()
         && !vp.CameraLocationIsLocked() 
         && vp.GetFrustum(&frus_left,&frus_right,&frus_bottom,&frus_top,&frus_near,&frus_far)
         && frus_near > 0.0
       )
    {
      double d = (vp.CameraLocation() - width_point)*vp.CameraZ();
      if ( d > frus_near )
      {
        // make sure target plane is visible
        double w = fabs(frus_right - frus_left)*d/frus_near;
        if ( width > w && w > 0.0 )
        {
          // move camera back to increase "w" back up to "width"
          ON_3dPoint cam_loc0 = vp.CameraLocation();
          double dz = d*(width/w - 1.0);
          ON_3dPoint cam_loc1 = cam_loc0 + dz*vp.CameraZ();
          vp.SetCameraLocation(cam_loc1);
        }
      }
    }
  }
  return rc;
}

bool ON_Viewport::ChangeToPerspectiveProjection( 
          double target_distance,
          bool bSymmetricFrustum,
          double lens_length
        )
{
  bool rc = (m_bValidCamera && m_bValidFrustum);

  SetCameraUpLock(false);
  SetCameraDirectionLock(false);

  if (    ON::perspective_view == m_projection 
       && (bSymmetricFrustum?true:false) == FrustumIsLeftRightSymmetric()
       && (bSymmetricFrustum?true:false) == FrustumIsTopBottomSymmetric()
     )
  {
    double current_lens_length = lens_length;
    if ( ON_IsValid(lens_length) 
         && lens_length > 0.0 
         && GetCamera35mmLensLength(&current_lens_length) 
         && fabs(current_lens_length - lens_length) > 0.125
        )
    {
      SetCamera35mmLensLength(lens_length);
    }
    // no other changes are required
    return rc;
  }

  if ( !ON_IsValid(target_distance) || target_distance <= 0.0 )
    target_distance = TargetDistance(true);

  // If needed, make frustum symmetric.  This may move the 
  // camera location in a direction perpendicular to m_CamZ.
  ChangeToSymmetricFrustum(bSymmetricFrustum,bSymmetricFrustum,target_distance);
  SetFrustumTopBottomSymmetry(bSymmetricFrustum);
  SetFrustumLeftRightSymmetry(bSymmetricFrustum);

  // If needed change projection to perspective.  If
  // the input projection is parallel, this may move
  // the camera in the m_CamZ direction to preserve
  // viewing the target plane.
  if (!ChangeFromParallelToPerspectiveHelper(*this,target_distance,lens_length))
    rc = false;

  if ( rc && m_target_point.IsValid() )
    UpdateTargetPointHelper(*this,target_distance);

  return rc;
}

static
bool GetTwoPointPerspectiveUpAndDirHelper( const ON_3dVector& up,
                                           const ON_3dVector& CamDir,
                                           const ON_3dVector& CamY,
                                           const ON_3dVector& CamZ,
                                           ON_3dVector& new_up,
                                           ON_3dVector& new_dir
                                           )
{
  // get up direction
  ON_3dVector unit_up;
  ON_3dVector unit_dir;
  if ( up.IsZero() && CamY.IsValid() && CamY.IsUnitVector() )
  {
    new_up = CamY;
    if ( fabs(new_up.z) >= fabs(new_up.y) && fabs(new_up.z) >= fabs(new_up.x) )
      new_up.Set(0.0,0.0,new_up.z<0.0?-1.0:1.0);
    else if ( fabs(new_up.y) >= fabs(new_up.z) && fabs(new_up.y) >= fabs(new_up.x) )
      new_up.Set(0.0,new_up.y<0.0?-1.0:1.0,0.0);
    else
      new_up.Set(new_up.x<0.0?-1.0:1.0,0.0,0.0);
    unit_up = new_up;
  }
  else if ( up.IsValid() && !up.IsTiny() )
  {
    unit_up = up;
    if ( !unit_up.IsUnitVector() && !unit_up.Unitize() )
      return false;
    new_up = up;
  }
  else
  {
    return false;
  }

  // get camera dir
  if ( CamDir.IsValid() && !CamDir.IsTiny() )
  {
    new_dir = CamDir;
    unit_dir = new_dir;
    if ( unit_dir.Unitize() && ON__IsCameraFramePerpindicular(unit_up,unit_dir) )
      return true;
    unit_dir = unit_dir - (unit_dir*unit_up)*unit_up;
    if ( unit_dir.IsValid() && !unit_dir.IsTiny() && unit_dir.Unitize() )
    {
      new_dir = unit_dir;
      return true;
    }
  }

  if ( CamZ.IsValid() && CamZ.IsUnitVector() )
  {
    new_dir = -CamZ;
    unit_dir = new_dir;
    if ( unit_dir.Unitize() && ON__IsCameraFramePerpindicular(unit_up,unit_dir) )
      return true;
    unit_dir = unit_dir - (new_dir*unit_up)*unit_up;
    if ( unit_dir.IsValid() && !unit_dir.IsTiny() && unit_dir.Unitize() )
    {
      new_dir = unit_dir;
      return true;
    }
  }

  return false;
}

bool ON_Viewport::ChangeToTwoPointPerspectiveProjection( 
        double target_distance,
        ON_3dVector up,
        double lens_length
        )
{
  bool rc = (m_bValidCamera && m_bValidFrustum);

  SetCameraDirectionLock(false);

  if ( IsTwoPointPerspectiveProjection() )
  {
    double current_lens_length = lens_length;
    if ( ON_IsValid(lens_length) 
         && lens_length > 0.0 
         && GetCamera35mmLensLength(&current_lens_length) 
         && fabs(current_lens_length - lens_length) > 0.125
        )
    {
      SetCamera35mmLensLength(lens_length);
    }
    // no other changes are required
    return rc;
  }

  if ( !ON_IsValid(target_distance) || target_distance <= 0.0 )
    target_distance = TargetDistance(true);

  // if needed, make frustum left/right symmetric. This may move the 
  // camera location in a direction perpendicular to m_CamZ.
  ChangeToSymmetricFrustum(true,false,target_distance);
  SetFrustumLeftRightSymmetry(true);
  SetFrustumTopBottomSymmetry(false);

  // If needed change projection to perspective.  If
  // the input projection is parallel, this may move
  // the camera in the m_CamZ direction to preserve
  // viewing the target plane.
  if (!ChangeFromParallelToPerspectiveHelper(*this,target_distance,lens_length))
    rc = false;

  if ( rc )
  {
    ON_3dVector new_up = m_CamY;
    ON_3dVector new_dir = -m_CamZ;
    ON_3dPoint  new_loc = m_CamLoc;
    if ( !GetTwoPointPerspectiveUpAndDirHelper(up,m_CamDir,m_CamY,m_CamZ,new_up,new_dir) )
    {
      rc = false;
    }
    else
    {
      // move location so the stuff that is currently visible
      // tends to end up in someplace in the new frustum.
      ON_3dPoint center_point = FrustumCenterPoint(target_distance);
      if ( center_point.IsValid() && (new_loc-center_point)*m_CamZ > 0.0 )
      {
        ON_Xform rot;
        rot.Rotation(m_CamY,new_up,center_point);
        new_loc = rot*m_CamLoc;
        if ( !new_loc.IsValid() )
          new_loc = m_CamLoc;
      }

      ON_3dVector saved_up = m_CamUp;
      ON_3dVector saved_dir = m_CamDir;
      bool bSavedLockCamUp = m_bLockCamUp;
      m_CamUp = new_up;    // intentionally ignoring m_bLockCamUp
      m_CamDir = new_dir;  // intentionally ignoring m_bLockDirUp
      SetCameraUpLock(true);
      if ( !SetCameraFrame() )
      {
        rc = false;
        m_CamUp = saved_up;
        m_CamDir = saved_dir;
        m_bLockCamUp = bSavedLockCamUp;
      }
      SetCameraLocation(new_loc);

      UpdateTargetPointHelper(*this,target_distance);
    }

  }

  return rc;
}

bool ON_Viewport::SetProjection( ON::view_projection projection )
{
  // Debugging projection changes is easier if we
  // do this initial check.
  if ( projection == m_projection )
    return true;

  bool rc = false;
  if ( projection == ON::perspective_view ) 
  {
    rc = true;
    m_projection = ON::perspective_view;
  }
  else 
  {
    rc = (projection == ON::parallel_view);
    m_projection = ON::parallel_view;
  }

  return rc;
}

ON::view_projection ON_Viewport::Projection() const
{
  return m_projection;
}

bool ON_Viewport::IsParallelProjection() const
{
  return ( ON::parallel_view == m_projection );
}

bool ON_Viewport::IsPerspectiveProjection() const
{
  return ( ON::perspective_view == m_projection );
}

bool ON_Viewport::IsTwoPointPerspectiveProjection() const
{
  bool rc =    IsPerspectiveProjection() 
            && CameraUpIsLocked() 
            && FrustumIsLeftRightSymmetric() 
            && !FrustumIsTopBottomSymmetric();
  return rc;
}

bool ON_Viewport::SetFrustum(
      double frus_left,
      double frus_right,
      double frus_bottom,
      double frus_top,
      double frus_near,
      double frus_far
      )
{
  bool rc = false;
  if (  
          ON_IsValid(frus_left)
       && ON_IsValid(frus_right)
       && ON_IsValid(frus_top)
       && ON_IsValid(frus_bottom)
       && ON_IsValid(frus_near)
       && ON_IsValid(frus_far)
       && frus_left < frus_right 
       && frus_bottom < frus_top 
       && 0.0 < frus_near 
       && frus_near < frus_far 
     ) 
  {
    if ( IsPerspectiveProjection() 
         && (frus_near <= 1.0e-8 || frus_far > 1.0001e8*frus_near) 
       )
    {
      ON_ERROR("ON_Viewport::SetFrustum - Beyond float precision perspective frus_near/frus_far values - will crash MS OpenGL");
    }

    if ( FrustumIsLeftRightSymmetric() && frus_left != -frus_right )
    {
      double d = 0.5*(frus_right-frus_left);
      frus_right = d;
      frus_left = -d;
    }

    if ( FrustumIsTopBottomSymmetric() && frus_bottom != -frus_top )
    {
      double d = 0.5*(frus_top-frus_bottom);
      frus_top = d;
      frus_bottom = -d;
    }

    m_frus_left   = frus_left;
    m_frus_right  = frus_right;
    m_frus_bottom = frus_bottom;
    m_frus_top    = frus_top;
    m_frus_near   = frus_near;
    m_frus_far    = frus_far;
    m_bValidFrustum = true;
    rc = true;
  }
  else
  {
    // 17 March 2008 Dale Lear
    //   I added this to trap the bug that is creating
    //   invalid viewports.  Developers: If you ever
    //   get this error, immediately investigate it.
    ON_ERROR("ON_Viewport::SetFrustum - invalid input");
  }
  return rc;
}


bool ON_Viewport::GetFrustum(
      double* frus_left,
      double* frus_right,
      double* frus_bottom,
      double* frus_top,
      double* frus_near,   // = NULL
      double* frus_far     // = NULL
      ) const
{
  if ( frus_left )
    *frus_left = m_frus_left;
  if ( frus_right )
    *frus_right = m_frus_right;
  if ( frus_bottom )
    *frus_bottom = m_frus_bottom;
  if ( frus_top )
    *frus_top = m_frus_top;
  if ( frus_near )
    *frus_near = m_frus_near;
  if ( frus_far )
    *frus_far = m_frus_far;
  return m_bValidFrustum;
}

double ON_Viewport::FrustumLeft()   const { return m_frus_left; }
double ON_Viewport::FrustumRight()  const { return m_frus_right; }
double ON_Viewport::FrustumBottom() const { return m_frus_bottom; }
double ON_Viewport::FrustumTop()    const { return m_frus_top; }
double ON_Viewport::FrustumNear()   const { return m_frus_near; }
double ON_Viewport::FrustumFar()    const { return m_frus_far;  }
double ON_Viewport::FrustumWidth()  const { return m_frus_right-m_frus_left; }
double ON_Viewport::FrustumHeight() const { return m_frus_top-m_frus_bottom; }

double ON_Viewport::FrustumMinimumDiameter() const 
{ 
  double w = fabs(m_frus_right-m_frus_left); 
  double h = fabs(m_frus_top-m_frus_bottom);
  return (w<=h)?w:h;
}

double ON_Viewport::FrustumMaximumDiameter() const
{ 
  double w = fabs(m_frus_right-m_frus_left); 
  double h = fabs(m_frus_top-m_frus_bottom);
  return (w<=h)?w:h;
}


bool ON_Viewport::SetFrustumAspect( double frustum_aspect )
{
  // maintains camera angle
  bool rc = false;
  double w, h, d, left, right, bot, top, near_dist, far_dist;
  if ( frustum_aspect > 0.0 && GetFrustum( &left, &right, &bot, &top, &near_dist, &far_dist ) ) {
    w = right - left;
    h = top - bot;
    if ( fabs(h) > fabs(w) ) {
      d = (h>=0.0) ? fabs(w) : -fabs(w);
      d *= 0.5;
      h = 0.5*(top+bot);
      bot = h-d;
      top = h+d;
      h = top - bot;
    }
    else {
      d = (w>=0.0) ? fabs(h) : -fabs(h);
      d *= 0.5;
      w = 0.5*(left+right);
      left  = w-d;
      right = w+d;
      w = right - left;
    }
    if ( frustum_aspect > 1.0 ) {
      // increase width
      d = 0.5*w*frustum_aspect;
      w = 0.5*(left+right);
      left = w-d;
      right = w+d;
      w = right - left;
    }
    else if ( frustum_aspect < 1.0 ) {
      // increase height
      d = 0.5*h/frustum_aspect;
      h = 0.5*(bot+top);
      bot = h-d;
      top = h+d;
      h = top - bot;
    }
    rc = SetFrustum( left, right, bot, top, near_dist, far_dist );
  }
  return rc;
}



bool ON_Viewport::GetFrustumAspect( double& frustum_aspect ) const
{
  // frustum_aspect = frustum width / frustum height
  double w, h, left, right, bot, top;
  bool rc = m_bValidFrustum;
  frustum_aspect = 0.0;

  if ( GetFrustum( &left, &right, &bot, &top ) ) {
    w = right - left;
    h = top - bot;
    if ( h == 0.0 )
      rc = false;
    else
      frustum_aspect = w/h;
  }
  return rc;
}

bool ON_Viewport::GetFrustumCenter( double* frus_center ) const
{
  double camZ[3], frus_near, frus_far, d;
  if ( !frus_center )
    return false;
  if ( !GetCameraFrame( frus_center, NULL, NULL, camZ ) )
    return false;
  if ( !GetFrustum( NULL, NULL, NULL, NULL, &frus_near, &frus_far ) )
    return false;
  d = -0.5*(frus_near+frus_far);
  frus_center[0] += d*camZ[0];
  frus_center[1] += d*camZ[1];
  frus_center[2] += d*camZ[2];
  return true;
}

bool ON_Viewport::SetScreenPort( 
      int port_left, 
      int port_right,
      int port_bottom, 
      int port_top,
      int port_near, // = 0
      int port_far   // = 0
      )
{
  if ( port_left == port_right )
    return false;
  if ( port_bottom == port_top )
    return false;
  m_port_left   = port_left;
  m_port_right  = port_right;
  m_port_bottom = port_bottom;
  m_port_top    = port_top;
  if ( port_near || port_near != port_far ) 
  {
    m_port_near   = port_near;
    m_port_far    = port_far;
  }
  m_bValidPort = true;
  return m_bValidPort;
}

bool ON_Viewport::GetScreenPort( 
      int* port_left,
      int* port_right,
      int* port_bottom,
      int* port_top,
      int* port_near, // = NULL
      int* port_far   // = NULL
      ) const
{
  if ( port_left )
    *port_left = m_port_left;
  if ( port_right )
    *port_right = m_port_right;
  if ( port_bottom )
    *port_bottom = m_port_bottom;
  if ( port_top )
    *port_top = m_port_top;
  if ( port_near )
    *port_near = m_port_near;
  if ( port_far )
    *port_far = m_port_far;
  return m_bValidPort;
}

int ON_Viewport::ScreenPortWidth() const
{
  int width = m_port_right - m_port_left;
  return width >= 0 ? width : -width;
}

int ON_Viewport::ScreenPortHeight() const
{
  int height = m_port_top - m_port_bottom;
  return height >= 0 ? height : -height;
}

bool ON_Viewport::GetScreenPortAspect(double& aspect) const
{
  const double width = m_port_right - m_port_left;
  const double height = m_port_top - m_port_bottom;
  aspect = ( m_bValidPort && ON_IsValid(height) && ON_IsValid(width) && height != 0.0 )
         ? fabs(width/height) 
         : 0.0;
  return (m_bValidPort && aspect != 0.0);
}

bool ON_ViewportFromRhinoView(
        ON::view_projection projection,
        const ON_3dPoint& rhvp_target, // 3d point
        double rhvp_angle1, double rhvp_angle2, double rhvp_angle3, // radians
        double rhvp_viewsize,     // > 0
        double rhvp_cameradist,   // > 0
        int screen_width, int screen_height,
        ON_Viewport& vp
        )
/*****************************************************************************
Compute canonical view projection information from Rhino viewport settings
INPUT:
  projection
  rhvp_target
    Rhino viewport target point (3d point that is center of view rotations)
  rhvp_angle1, rhvp_angle2, rhvp_angle3
    Rhino viewport angle settings
  rhvp_viewsize 
    In perspective, rhvp_viewsize = tangent(half lens angle).
    In parallel, rhvp_viewsize = 1/2 * minimum(frustum width,frustum height)
  rhvp_cameradistance ( > 0 )
    Distance from camera location to Rhino's "target" point
  screen_width, screen_height (0,0) if not known
*****************************************************************************/
{
  vp.SetProjection( projection );
  /*
  width, height
    width and height of viewport  
    ( = RhinoViewport->width, RhinoViewport->height )
  z_buffer_depth
    depth for the z buffer.  0xFFFF is currently used for Rhino
    quick rendering.
  */

  // In the situation where there is no physical display device, assume a
  // 1000 x 1000 "screen" and set the parameters accordingly.  Toolkit users
  // that are using this class to actually draw a picture, can make a subsequent
  // call to SetScreenPort().

  const double height = (screen_width < 1 || screen_height < 1) 
                      ? 1000.0 : (double)screen_height;
  const double width  = (screen_width < 1 || screen_height < 1) 
                      ? 1000.0 : (double)screen_width;
  //const int z_buffer_depth = 0xFFFF; // value Rhino "Shade" command uses

  // Use this function to obtain standard view information from a Rhino VIEWPORT
  // view. The Rhino viewport has many entries.  As of 17 October, 1997 all Rhino
  // world to clipping transformation information is derived from the VIEWPORT 
  // fields:
  //
  //   target, angle1, angle2, angle3, viewsize, and cameradist.
  //
  // The width, height and zbuffer_depth arguments are used to determing the
  // clipping to screen transformation.

  ON_Xform R1, R2, R3, RhinoRot;
  double frustum_left, frustum_right, frustum_bottom, frustum_top;
  double near_clipping_distance, far_clipping_distance;

  // Initialize default view in case input is garbage.
  if (height < 1)
    return false;
  if ( width < 1 )
    return false;
  if ( rhvp_viewsize <= 0.0 )
    return false;
  if ( rhvp_cameradist <= 0.0 )
    return false;

  // A Rhino 1.0 VIEWPORT structure describes the camera's location, direction,
  // and  orientation by specifying a rotation transformation that is
  // applied to an initial frame.  The rotation transformation is defined
  // as a sequence of 3 rotations abount fixed axes.  The initial frame
  // has the camera located at (0,0,cameradist), pointed in the direction
  // (0,0,-1), and oriented so that up is (0,1,0).

  R1.Rotation( rhvp_angle1, ON_zaxis, ON_origin ); // so called "twist"
  R2.Rotation( rhvp_angle2, ON_xaxis, ON_origin ); // so called "elevation"
  R3.Rotation( rhvp_angle3, ON_zaxis, ON_origin ); // so called "fudge factor"
  RhinoRot = R3 * R2 * R1;

  vp.SetCameraUp( RhinoRot*ON_yaxis );
  vp.SetCameraDirection( -(RhinoRot*ON_zaxis) );
  vp.SetCameraLocation( rhvp_target - rhvp_cameradist*vp.CameraDirection() );
  vp.SetTargetPoint( rhvp_target );
  //vp.SetTargetDistance( rhvp_cameradist );

  // Camera coordinates "X" = CameraRight = CameraDirection x CameraUp
  // Camera coordinates "Y" = CameraUp
  // Camera coordinates "Z" = -CameraDirection

  // Rhino 1.0 did not support skew projections.  In other words, the
  // view frustum is symmetric and ray that begins at CameraLocation and 
  // goes along CameraDirection runs along the frustum's central axis.
  // The aspect ratio of the view frustum equals 
  // (screen port width)/(screen port height)
  // This means frus_left = -frus_right, frus_bottom = -frus_top, and
  // frus_top/frus_right = height/width

  // Set near and far clipping planes to some reasonable values.  If
  // the depth of the pixel is important, then the near and far clipping
  // plane will need to be adjusted later.
  // Rhino 1.0 didn't have a far clipping plane in wire frame (which explains
  // why you can get perspective views reversed through the origin by using 
  // the SetCameraTarget() command.  It's near clipping plane is set to 
  // a miniscule value.  For mesh rendering, it must come up with some
  // sort of reasonable near and far clipping planes because the zbuffer
  // is used correctly.  When time permits, I'll dig through the rendering
  // code and determine what values are being used.
  //
  near_clipping_distance = rhvp_cameradist/64.0;
  if ( near_clipping_distance > 1.0 )
    near_clipping_distance = 1.0;
  far_clipping_distance = 4.0*rhvp_cameradist;


  if ( width <= height )
  {
    frustum_right = rhvp_viewsize;
    frustum_top = frustum_right*height/width;
  }
  else
  {
    frustum_top = rhvp_viewsize;
    frustum_right = frustum_top*width/height;
  }
  if ( vp.IsPerspectiveProjection() )
  {
    frustum_right *= near_clipping_distance;
    frustum_top   *= near_clipping_distance;
  }
  frustum_left   = -frustum_right;
  frustum_bottom = -frustum_top;


  vp.SetFrustum( 
         frustum_left,   frustum_right, 
         frustum_bottom, frustum_top, 
         near_clipping_distance, far_clipping_distance );

  // Windows specific stuff that requires knowing size of client area in pixels
  vp.SetScreenPort( 0, (int)width, // windows has screen X increasing accross
                    (int)height,  0, // windows has screen Y increasing downwards
                    0, 0xFFFF );

  return (vp.IsValid()?true:false);
}

bool ON_Viewport::GetCameraAngle( 
       double* angle,
       double* angle_h, 
       double* angle_w
       ) const
{
  bool rc = false;
  if ( angle )
    *angle = 0.0;
  if ( angle_h )
    *angle_h = 0.0;
  if ( angle_w )
    *angle_w = 0.0;
  double half_w, half_h, left, right, bot, top, near_dist;
  if ( GetFrustum( &left, &right, &bot, &top, &near_dist, NULL ) ) 
  {
    half_w = ( right > -left ) ? right : -left;
    half_h = ( top   > -bot  ) ? top   : -bot;
    if ( near_dist > 0.0 && ON_IsValid(near_dist) )
    {
      if ( angle )
        *angle = atan( sqrt(half_w*half_w + half_h*half_h)/near_dist );
      if ( angle_h )
        *angle_h = atan( half_h/near_dist );
      if ( angle_w )
        *angle_w = atan( half_w/near_dist );
    }
    rc = true;
  }
  return rc;
}

bool ON_Viewport::GetCameraAngle( 
       double* angle
       ) const
{
  double angle_h = 0.0;
  double angle_w = 0.0;
  bool rc = GetCameraAngle( NULL, &angle_h, &angle_w );
  if ( angle && rc ) {
    *angle = (angle_h < angle_w) ? angle_h : angle_w;
  }
  return rc;
}

bool ON_Viewport::SetCameraAngle( double angle )
{
  bool rc = false;
  double r, d, aspect, half_w, half_h, near_dist, far_dist;
  if ( angle > 0.0  && angle < 0.5*ON_PI*(1.0-ON_SQRT_EPSILON) ) {
    if ( GetFrustum( NULL, NULL, NULL, NULL, &near_dist, &far_dist ) && GetFrustumAspect( aspect) ) {
      r = near_dist*tan(angle);
      // d = r/sqrt(1.0+aspect*aspect); // if angle is 1/2 diagonal angle
      d = r; // angle is 1/2 smallest angle
      if ( aspect >= 1.0 ) {
        // width >= height
        half_w = d*aspect;
        half_h = d;
      }
      else {
        // height > width
        half_w = d;
        half_h = d/aspect;
      }
      rc = SetFrustum( -half_w, half_w, -half_h, half_h, near_dist, far_dist );
    }
  }
  return rc;
}

// This version of the function has "lens" misspelled.
bool ON_Viewport::GetCamera35mmLenseLength( double* lens_length ) const
{
  return GetCamera35mmLensLength( lens_length );
}

bool ON_Viewport::GetCamera35mmLensLength( double* lens_length ) const
{
  // 35 mm film has a height of 24 mm and a width of 36 mm
  double film_r, view_r, half_w, half_h;
  double frus_left, frus_right, frus_bottom, frus_top, frus_near, frus_far;
  if ( !lens_length )
    return false;
  *lens_length = 0.0;
  if ( !GetFrustum( &frus_left, &frus_right, &frus_bottom, &frus_top, 
                     &frus_near, &frus_far ) )
    return false;
  if ( frus_near <= 0.0 )
    return false;
  half_w = ( frus_right > -frus_left ) ? frus_right : -frus_left;
  half_h = ( frus_top   > -frus_bottom ) ? frus_top : -frus_bottom;

  // 2009 May 8 Dale Lear - always use width in two point perspective
  view_r = (half_w <= half_h || IsTwoPointPerspectiveProjection()) ? half_w : half_h;
  film_r = 12.0;
  if ( view_r <= 0.0 )
    return false;

  *lens_length = frus_near*film_r/view_r;
  return true;
}

// This version of the function has "lens" misspelled.
bool ON_Viewport::SetCamera35mmLenseLength( double lens_length )
{
  return SetCamera35mmLensLength( lens_length );
}

bool ON_Viewport::SetCamera35mmLensLength( double lens_length )
{
  // 35 mm film has a height of 24 mm and a width of 36 mm
  double film_r, view_r, half_w, half_h, s;
  double frus_left, frus_right, frus_bottom, frus_top, frus_near, frus_far;
  if ( !ON_IsValid(lens_length) || lens_length <= 0.0 )
    return false;
  if ( !GetFrustum( &frus_left, &frus_right, &frus_bottom, &frus_top, 
                     &frus_near, &frus_far ) )
    return false;
  if ( frus_near <= 0.0 )
    return false;
  half_w = ( frus_right > -frus_left ) ? frus_right : -frus_left;
  half_h = ( frus_top   > -frus_bottom  ) ? frus_top   : -frus_bottom;

  // 2009 May 8 Dale Lear - always use width in two point perspective
  view_r = (half_w <= half_h || IsTwoPointPerspectiveProjection()) ? half_w : half_h;
  film_r = 12.0;
  if ( view_r <= 0.0 )
    return false;

  s = (film_r/view_r)*(frus_near/lens_length);
  if ( fabs(s-1.0) < 1.0e-6 )
    return true;

  frus_left *= s;
  frus_right *= s;
  frus_bottom *= s;
  frus_top *= s;
  return SetFrustum( frus_left, frus_right, frus_bottom, frus_top, frus_near, frus_far );
}

bool ON_Viewport::GetXform( 
       ON::coordinate_system srcCS,
       ON::coordinate_system destCS,
       ON_Xform& xform
       ) const
{
  bool rc = false;
  ON_Xform x0, x1;

  xform.Identity();

  switch( srcCS ) 
  {
  case ON::world_cs:
  case ON::camera_cs:
  case ON::clip_cs:
  case ON::screen_cs:
    break;
  default:
    return false;
  }
  switch( destCS ) 
  {
  case ON::world_cs:
  case ON::camera_cs:
  case ON::clip_cs:
  case ON::screen_cs:
    break;
  default:
    return false;
  }

  if (srcCS == destCS)
    return true;


  switch ( srcCS ) 
  {

  case ON::world_cs:
    if ( !m_bValidCamera )
      break;

    switch ( destCS ) 
    {
    case ON::camera_cs:
      xform.WorldToCamera( m_CamLoc, m_CamX, m_CamY, m_CamZ );
      rc = true;
      break;

    case ON::clip_cs:
      rc = GetXform( ON::world_cs,  ON::camera_cs, x0 );
      if (rc)
        rc = GetXform( ON::camera_cs, ON::clip_cs,   x1 );
      if (rc)
        xform = x1*x0;
      break;

    case ON::screen_cs:
      rc = GetXform( ON::world_cs,  ON::clip_cs,   x0 );
      if (rc)
        rc = GetXform( ON::clip_cs,   ON::screen_cs, x1 );
      if (rc)
        xform = x1*x0;
      break;

    case ON::world_cs:
      // Never happens.  This is here to quiet g++ warnings.
      break;
    }
    break;

  case ON::camera_cs:
    if ( !m_bValidCamera )
      break;

    switch ( destCS ) 
    {
    case ON::world_cs:
      xform.CameraToWorld( m_CamLoc, m_CamX, m_CamY, m_CamZ );
      rc = true;
      break;

    case ON::clip_cs:
      if ( m_bValidFrustum ) 
      {
        ON_Xform cam2clip;
        cam2clip.CameraToClip( 
          IsPerspectiveProjection(),
          m_frus_left, m_frus_right,
          m_frus_bottom, m_frus_top,
          m_frus_near, m_frus_far );
        xform = m_clip_mods*cam2clip;
        rc = true;
      }
      break;

    case ON::screen_cs:
      rc = GetXform( ON::camera_cs,  ON::clip_cs,  x0 );
      if (rc)
        rc = GetXform( ON::clip_cs,   ON::screen_cs, x1 );
      if (rc)
        xform = x1*x0;
      break;

    case ON::camera_cs:
      // Never happens.  This is here to quiet g++ warnings.
      break;
    }
    break;

  case ON::clip_cs:
    switch ( destCS ) 
    {
    case ON::world_cs:
      rc = GetXform( ON::clip_cs,   ON::camera_cs, x0 );
      if (rc)
        rc = GetXform( ON::camera_cs,  ON::world_cs, x1 );
      if (rc)
        xform = x1*x0;
      break;

    case ON::camera_cs:
      if ( m_bValidFrustum ) 
      {
        ON_Xform clip2cam;
        clip2cam.ClipToCamera(
          IsPerspectiveProjection(),
          m_frus_left, m_frus_right,
          m_frus_bottom, m_frus_top,
          m_frus_near, m_frus_far );
        xform = clip2cam*m_clip_mods_inverse;
        rc = true;
      }
      break;

    case ON::screen_cs:
      if ( m_bValidPort ) 
      {
        xform.ClipToScreen( 
          m_port_left, m_port_right, 
          m_port_bottom, m_port_top,
          m_port_near, m_port_far );
        rc = true;
      }
      break;

    case ON::clip_cs:
      // Never happens.  This is here to quiet g++ warnings.
      break;
    }
    break;

  case ON::screen_cs:
    switch ( destCS ) 
    {
    case ON::world_cs:
      rc = GetXform( ON::screen_cs, ON::camera_cs, x0 );
      if (rc)
        rc = GetXform( ON::camera_cs, ON::world_cs,  x1 );
      if (rc)
        xform = x1*x0;
      break;
    case ON::camera_cs:
      rc = GetXform( ON::screen_cs, ON::clip_cs,   x0 );
      if (rc)
        rc = GetXform( ON::clip_cs,   ON::camera_cs, x1 );
      if (rc)
        xform = x1*x0;
      break;
    case ON::clip_cs:
      if ( m_bValidPort ) {
        xform.ScreenToClip(
          m_port_left, m_port_right, 
          m_port_bottom, m_port_top,
          m_port_near, m_port_far );
        rc = true;
      }
      break;
    case ON::screen_cs:
      // Never happens.  This is here to quiet g++ warnings.
      break;
    }
    break;

  }

  return rc;
}

bool ON_Viewport::GetFrustumLine( double screenx, double screeny, ON_Line& world_line ) const
{
  ON_Xform s2c, c2w;
  ON_3dPoint c;
  ON_Line line;
  bool rc;

  rc = GetXform( ON::screen_cs, ON::clip_cs, s2c );
  if ( rc )
    rc = GetXform( ON::clip_cs, ON::world_cs, c2w );
  if (rc )
  {
    // c = mouse point on near clipping plane
    c.x = s2c.m_xform[0][0]*screenx + s2c.m_xform[0][1]*screeny + s2c.m_xform[0][3];
    c.y = s2c.m_xform[1][0]*screenx + s2c.m_xform[1][1]*screeny + s2c.m_xform[1][3];
    c.z = 1.0;
    line.to = c2w*c;   // line.to = near plane mouse point in world coords
    c.z = -1.0;
    line.from = c2w*c; // line.from = far plane mouse point in world coords

    world_line = line;
  }
  return rc;
}

static double clipDist( const double* camLoc, const double* camZ, const double* P )
{
  return (camLoc[0]-P[0])*camZ[0]+(camLoc[1]-P[1])*camZ[1]+(camLoc[2]-P[2])*camZ[2];
}


bool ON_Viewport::SetFrustumNearFar( 
       const double* box_min,
       const double* box_max
       )
{
  bool rc = false;
  const double* box[2];
  int i,j,k;
  double n, f, d;
  double camLoc[3], camZ[3], P[3];

  if ( !box_min )
    box_min = box_max;
  if ( !box_max )
    box_max = box_min;
  if ( !box_min )
    return false;

  // 31 May 2007 Dale Lear RR 25980
  //    Add validation of box_min and box_max.
  if ( !ON_IsValid(box_min[0]) || !ON_IsValid(box_min[1]) || !ON_IsValid(box_min[2]) )
    return false;
  if ( !ON_IsValid(box_max[0]) || !ON_IsValid(box_max[1]) || !ON_IsValid(box_max[2]) )
    return false;
  if (    box_min[0] > box_max[0]
       || box_min[1] > box_max[1]
       || box_min[2] > box_max[2]
     )
  {
    return false;
  }
  box[0] = box_min;
  box[1] = box_max;

  if ( GetCameraFrame( camLoc, NULL, NULL, camZ ) ) {
    n = f = -1.0;
    for(i=0;i<2;i++)for(j=0;j<2;j++)for(k=0;k<2;k++) {
      P[0] = box[i][0];
      P[1] = box[j][1];
      P[2] = box[k][2];
      d = clipDist(camLoc,camZ,P);
      if (!i&&!j&&!k)
        n=f=d;
      else if ( d < n )
        n = d;
      else if ( d > f )
        f = d;
    }
    if ( !ON_IsValid(f) || !ON_IsValid(n) )
      return false;
    if ( f <= 0.0 )
      return false; // box is behind camera
    n *= 0.9375;
    f *= 1.0625;
    if ( n <= 0.0 )
      n = m__MIN_NEAR_OVER_FAR*f;
    if ( IsPerspectiveProjection() )
      rc = SetFrustumNearFar( n, f, m__MIN_NEAR_DIST, m__MIN_NEAR_OVER_FAR, 0.5*(n+f) );
    else
      rc = SetFrustumNearFar( n, f );
  }
  return rc;
}

bool ON_Viewport::SetFrustumNearFar( 
       const double* center,
       double        radius
       )
{
  bool rc = false;
  double n, f, d;
  double camLoc[3], camZ[3], P[3];

  if ( !center 
       || !ON_IsValid(center[0]) 
       || !ON_IsValid(center[1]) 
       || !ON_IsValid(center[2])
       || !ON_IsValid(radius)
     )
  {
    return false;
  }

  if ( GetCameraFrame( camLoc, NULL, NULL, camZ ) ) 
  {
    d = fabs(radius);
    P[0] = center[0] + d*camZ[0];
    P[1] = center[1] + d*camZ[0];
    P[2] = center[2] + d*camZ[0];
    n = clipDist(camLoc,camZ,P);
    P[0] = center[0] - d*camZ[0];
    P[1] = center[1] - d*camZ[0];
    P[2] = center[2] - d*camZ[0];
    f = clipDist(camLoc,camZ,P);
    if ( !ON_IsValid(f) || !ON_IsValid(n) )
      return false;
    if ( f <= 0.0 )
      return false; // sphere is behind camera
    n *= 0.9375;
    f *= 1.0625;
    if ( n <= 0.0 )
      n = m__MIN_NEAR_OVER_FAR*f;
    if ( IsPerspectiveProjection() )
      rc = SetFrustumNearFar( n, f, m__MIN_NEAR_DIST, m__MIN_NEAR_OVER_FAR, 0.5*(n+f) );
    else
      rc = SetFrustumNearFar( n, f );
  }
  return rc;
}

bool ON_Viewport::SetFrustumNearFar( double n, double f )
{
  // This is a bare bones setter.  Except for the perspective 0 < n < f
  // requirement, do not add checking here.
  //
  // Use the ON_Viewport::SetFrustumNearFar( near_dist, 
  //                                         far_dist, 
  //                                         min_near_dist, 
  //                                         min_near_over_far,
  //                                         target_dist );
  //
  // version if you need lots of validation and automatic fixing.

  double d, frus_left, frus_right, frus_bottom, frus_top, frus_near, frus_far;
  bool rc = false;
  if ( ON_IsValid(n) && ON_IsValid(f) && n > 0.0 && f > n ) 
  {
    if ( GetFrustum( &frus_left,   &frus_right, 
                      &frus_bottom, &frus_top, 
                      &frus_near,   &frus_far ) ) 
    {
      // preserve valid frustum
      if ( IsPerspectiveProjection() ) 
      {
        d = n/frus_near;
        frus_left *= d;
        frus_right *= d;
        frus_bottom *= d;
        frus_top *= d;
      }
      frus_near = n;
      frus_far = f;
      rc = SetFrustum( frus_left,   frus_right, 
                       frus_bottom, frus_top, 
                       frus_near,   frus_far );
    }
    else 
    {
      if ( IsPerspectiveProjection() && (n <= 1.0e-8 || f > 1.0001e8*n) )
      {
        ON_ERROR("ON_Viewport::SetFrustum - bogus perspective m_frus_near/far values - will crash MS OpenGL");
      }
      m_frus_near = n;
      m_frus_far = f;
      rc = true;
    }
  }
  return rc;
}


bool ON_Viewport::ChangeToSymmetricFrustum( 
    bool bLeftRightSymmetric, 
    bool bTopBottomSymmetric,
    double target_distance
    )
{
  if ( bLeftRightSymmetric && m_frus_left == -m_frus_right )
    bLeftRightSymmetric = false; // no left/right chnages required.

  if ( bTopBottomSymmetric && m_frus_bottom == -m_frus_top )
    bTopBottomSymmetric = false; // no top/bottom chagnes required.

  if ( !bLeftRightSymmetric && !bTopBottomSymmetric )
    return true; // no changes required

  if ( !m_bValidFrustum )
    return false;

  const double half_w = 0.5*(m_frus_right-m_frus_left);
  const double half_h = 0.5*(m_frus_top-m_frus_bottom);
  double dx = bLeftRightSymmetric ? (m_frus_right - half_w) : 0.0;
  double dy = bTopBottomSymmetric ? (m_frus_top   - half_h) : 0.0;
  if ( bLeftRightSymmetric )
  {
    m_frus_right = half_w;
    m_frus_left = -m_frus_right;
  }
  if ( bTopBottomSymmetric )
  {
    m_frus_top = half_h;
    m_frus_bottom = -m_frus_top;
  }

  // if possible, dolly the camera so the original
  // target plane is still visible.
  if ( m_bValidCamera && (dx != 0.0 || dy != 0.0 ) )
  {
    if ( ON::perspective_view == m_projection )
    {
      if ( m_frus_near > 0.0 )
      {
        if ( ON_UNSET_VALUE == target_distance )
          target_distance = TargetDistance(true);
        if ( ON_IsValid(target_distance) && target_distance > 0.0 )
        {
          double s = target_distance/m_frus_near;
          dx *= s;
          dy *= s;
        }
      }
      else
      {
        dx=dy = 0.0;
      }
    }
    if ( dx != 0.0 || dy != 0.0 )
    {
      ON_3dPoint cam_loc = m_CamLoc + dx*m_CamX + dy*m_CamY;
      SetCameraLocation(cam_loc);
    }
  } 

  return true;
}


bool ON_Viewport::GetWorldToScreenScale( const ON_3dPoint& P, double* scale ) const
{
  if ( scale ) {
    ON_Xform w2s;
    ON_3dVector X;
    ON_3dPoint Q, ScrC, ScrQ;
    if (!GetCameraFrame( NULL, X, NULL, NULL )) 
      return false;
    if (!GetXform( ON::world_cs, ON::screen_cs, w2s )) 
      return false;
    Q = P+X;
    ScrC = w2s*P;
    ScrQ = w2s*Q;
    *scale = fabs(ScrC.x - ScrQ.x);
  }
  return true;
}

bool ON_Viewport::GetCoordinateSprite( 
                     int size, 
                     int scrx, int scry,
                     int indx[3], // axis order by depth
                     double scr_coord[3][2] ) const
{
  // size = length of axes in pixels

  indx[0] = 0; indx[1] = 1; indx[2] = 2;
  scr_coord[0][0] = scr_coord[1][0] = scr_coord[2][0] = scrx;
  scr_coord[0][1] = scr_coord[1][1] = scr_coord[2][1] = scry;

  ON_3dPoint C, XP, YP, ZP, ScrC, ScrXP;
  ON_3dVector X, Z, Scr[3];
  ON_Xform w2s;
  if (!GetFrustumCenter( C ) )
    return false;
  if (!GetCameraFrame( NULL, X, NULL, Z )) 
    return false;
  if (!GetXform( ON::world_cs, ON::screen_cs, w2s )) 
    return false;

  // indx[] determines order that axes are drawn
  // sorted from back to front
  int i,j,k;
  for (i = 0; i < 2; i++) for (j = i+1; j <= 2; j++) {
    if (Z[indx[i]] > Z[indx[j]])
      {k = indx[i]; indx[i] = indx[j]; indx[j] = k;}
  }

  // determine world length that corresponds to size pixels
  XP = C+X;
  ScrC = w2s*C;
  ScrXP = w2s*XP;
  if (ScrC.x == ScrXP.x)
    return false;
  double s = size/fabs( ScrC.x - ScrXP.x );

  // transform world coord axes to screen
  XP = C;
  XP.x += s;
  YP = C;
  YP.y += s;
  ZP = C;
  ZP.z += s;
  Scr[0] = w2s*XP;
  Scr[1] = w2s*YP;
  Scr[2] = w2s*ZP;
  
  double dx = scrx - ScrC.x;
  double dy = scry - ScrC.y;
  for (i=0;i<3;i++) {
    scr_coord[i][0] = dx + Scr[i].x; 
    scr_coord[i][1] = dy + Scr[i].y;
  }

  return true;
}

static ON_BOOL32 GetRelativeScreenCoordinates(
          int port_left, int port_right,
          int port_bottom, int port_top,
          ON_BOOL32 bSortPoints,
          int& x0, int& y0, int& x1, int& y1,
          double& s0, double& t0, double& s1, double& t1
          )
{
  // convert screen rectangle into relative rectangle
  if ( bSortPoints ) {
    int i;
    if ( x0 > x1 ) {
      i = x0; x0 = x1; x1 = i;
    }
    if ( port_left > port_right ) {
      i = x0; x0 = x1; x1 = i;
    }
    if ( y0 > y1 ) {
      i = y0; y0 = y1; y1 = i;
    }
    if ( port_bottom > port_top ) {
      i = y0; y0 = y1; y1 = i;
    }
  }

  s0 = ((double)(x0 - port_left))/((double)(port_right - port_left));
  s1 = ((double)(x1 - port_left))/((double)(port_right - port_left));
  t0 = ((double)(y0 - port_bottom))/((double)(port_top - port_bottom));
  t1 = ((double)(y1 - port_bottom))/((double)(port_top - port_bottom));
  
  double tol = 0.001;
  if ( fabs(s0) <= tol ) s0 = 0.0; else if (fabs(s0-1.0) <= tol ) s0 = 1.0;
  if ( fabs(s1) <= tol ) s1 = 0.0; else if (fabs(s1-1.0) <= tol ) s1 = 1.0;
  if ( fabs(t0) <= tol ) t0 = 0.0; else if (fabs(t0-1.0) <= tol ) t0 = 1.0;
  if ( fabs(t1) <= tol ) t1 = 0.0; else if (fabs(t1-1.0) <= tol ) t1 = 1.0;
  if ( fabs(s0-s1) <= tol )
    return false;
  if ( fabs(t0-t1) <= tol )
    return false;
  return true;
}

bool ON_Viewport::ZoomToScreenRect( int x0, int y0, int x1, int y1 )
{
  int port_left, port_right, port_bottom, port_top, port_near, port_far;
  if ( !GetScreenPort( &port_left, &port_right, 
                       &port_bottom, &port_top, 
                       &port_near, &port_far ) )
    return false;

  // dolly camera sideways so it's looking at center of rectangle
  int dx = (x0+x1)/2;
  int dy = (y0+y1)/2;
  int cx = (port_left+port_right)/2;
  int cy = (port_bottom+port_top)/2;

  ON_3dVector dolly_vector;
  if ( !GetDollyCameraVector( dx, dy, cx, cy, 0.5*(FrustumNear()+FrustumFar()), dolly_vector ) )
    return false;
  if ( !DollyCamera( dolly_vector ) )
    return false;

  // adjust frustum
  dx = cx - dx;
  dy = cy - dy;
  x0 += dx;
  x1 += dx;
  y0 += dy;
  y1 += dy;
  double frus_left, frus_right, frus_bottom, frus_top, frus_near, frus_far;
  if ( !GetFrustum( &frus_left,   &frus_right, 
                     &frus_bottom, &frus_top, 
                     &frus_near,   &frus_far ) )
    return false;
  double s0,t0,s1,t1;
  if ( !GetRelativeScreenCoordinates(port_left, port_right, port_bottom, port_top,
                              true,
                              x0,y0,x1,y1,
                              s0,t0,s1,t1) )
    return false;
  double w = frus_right - frus_left;
  double h = frus_top - frus_bottom;
  double a0 = (1.0-s0)*frus_left   + s0*frus_right;
  double a1 = (1.0-s1)*frus_left   + s1*frus_right;
  double b0 = (1.0-t0)*frus_bottom + t0*frus_top;
  double b1 = (1.0-t1)*frus_bottom + t1*frus_top;
  if ( -a0 > a1 ) a1 = -a0; else a0 = -a1;
  if ( -b0 > b1 ) b1 = -b0; else b0 = -b1;
  double d;
  if ( (b1-b0)*w < (a1-a0)*h ) {
    d = (a1-a0)*h/w;
    d = 0.5*(d - (b1-b0));
    b0 -= d;
    b1 += d;
  }
  else {
    d = (b1-b0)*w/h;
    d = 0.5*(d - (a1-a0));
    a0 -= d;
    a1 += d;
  }

  return SetFrustum( a0, a1, b0, b1, frus_near, frus_far );
}

/*
ON_BOOL32 ON_Viewport::DollyToScreenRect( double view_plane_distance,
                                        int x0, int y0, int x1, int y1 )
{
  // Only makes sense in a perspective projection. In a parallel projection,
  // I resort to ZoomToScreenRect(0 and the visual result is the same.
  if ( !IsPerspectiveProjection() )
    return ZoomToScreenRect( x0, y0, x1, y1 );

  int port_left, port_right, port_bottom, port_top;
  if ( !GetScreenPort( &port_left, &port_right, &port_bottom, &port_top, NULL, NULL ) )
    return false;
  int dx = (x0+x1)/2;
  int dy = (y0+y1)/2;
  int cx = (port_left+port_right)/2;
  int cy = (port_bottom+port_top)/2;
  if ( !DollyAlongScreenChord( dx, dy, cx, cy ) )
    return false;
  dx = cx - dx;
  dy = cy - dy;
  x0 += dx;
  x1 += dx;
  y0 += dy;
  y1 += dy;

  double frus_left, frus_right, frus_bottom, frus_top, frus_near, frus_far;
  if ( !GetFrustum( &frus_left,   &frus_right, 
                     &frus_bottom, &frus_top, 
                     &frus_near,   &frus_far ) )
    return false;

  double s0, t0, s1, t1;
  if ( !GetRelativeScreenCoordinates(port_left, port_right, port_bottom, port_top,
                              true,
                              x0,y0,x1,y1,
                              s0,t0,s1,t1) )
    return false;

  double w = frus_right - frus_left;
  double h = frus_top - frus_bottom;
  double a0 = (1.0-s0)*frus_left   + s0*frus_right;
  double a1 = (1.0-s1)*frus_left   + s1*frus_right;
  double b0 = (1.0-t0)*frus_bottom + t0*frus_top;
  double b1 = (1.0-t1)*frus_bottom + t1*frus_top;
  if ( -a0 > a1 ) a1 = -a0; else a0 = -a1;
  if ( -b0 > b1 ) b1 = -b0; else b0 = -b1;
  double d;
  if ( (b1-b0)*w < (a1-a0)*h ) {
    d = (a1-a0)*h/w;
    d = 0.5*(d - h);
    b0 -= d;
    b1 += d;
  }
  else {
    d = (b1-b0)*w/h;
    d = 0.5*(d - w);
    a0 -= d;
    a1 += d;
  }

  d = 0.5*((a1-a0)/w + (b1-b0)/h)*view_plane_distance;
  double delta = d - view_plane_distance;

  frus_near += delta;
  frus_far  += delta;
  if ( frus_near <= 0.0 ) {
    if ( frus_far <= 0.0 )
      frus_far = 100.0;
    frus_near = 0.001*frus_far;
  }
  if ( !SetFrustumNearFar( frus_near, frus_far ) )
    return false;

  double camLoc[3], camY[3], camZ[3];
  if ( !GetCameraFrame( camLoc, NULL, camY, camZ ) )
    return false;
  camLoc[0] += delta*camZ[0];
  camLoc[1] += delta*camZ[1];
  camLoc[2] += delta*camZ[2];
  camZ[0] = -camZ[0];
  camZ[1] = -camZ[1];
  camZ[2] = -camZ[2];
  if ( !SetCamera( camLoc, camZ, camY ) )
    return false;

  return true;
}
*/

bool ON_Viewport::Extents( double angle, const ON_BoundingBox& bbox )
{
  double radius;
  double x, y, xmin, xmax, ymin, ymax;
  int i,j,k;

  if ( !bbox.IsValid() || !IsValid() )
    return false;
  ON_3dVector camX = CameraX();
  ON_3dVector camY = CameraY();
  ON_3dPoint center = bbox.Center();
  xmin=xmax=ymin=ymax=0.0;
  for (i=0;i<2;i++) for (j=0;j<2;j++) for (k=0;k<2;k++) {
    ON_3dVector box_corner = bbox.Corner(i,j,k);
    x = camX*box_corner;
    y = camY*box_corner;
    if ( i==0&&j==0&&k==0) {
      xmin=xmax=x;
      ymin=ymax=y;
    }
    else {
      if ( x > xmax) xmax=x; else if (x < xmin) xmin = x;
      if ( y > ymax) ymax=y; else if (y < ymin) ymin = y;
    }
  }
  radius = xmax-xmin;
  if ( ymax-ymin > radius )
    radius = ymax-ymin;
  if ( radius <= ON_SQRT_EPSILON ) {
    radius = bbox.Diagonal().MaximumCoordinate();
  }
  radius *= 0.5;
  if ( radius <= ON_SQRT_EPSILON )
    radius = 1.0;
  return Extents( angle, center, radius );
}

bool ON_Viewport::Extents( double angle, const ON_3dPoint& center, double radius )
{
  if ( !IsValid() )
    return false;

  double target_dist, near_dist, far_dist;

  if ( radius <= 0.0 || 
       angle  <= 0.0 || 
       angle  >= 0.5*ON_PI*(1.0-ON_SQRT_EPSILON) )
    return false;
  
  target_dist = radius/sin(angle);
  if ( !IsPerspectiveProjection() )
  {
    target_dist += 1.0625*radius;
  }
  near_dist = target_dist - 1.0625*radius;
  if ( near_dist < 0.0625*radius )
    near_dist = 0.0625*radius;
  if ( near_dist < m__MIN_NEAR_DIST )
    near_dist = m__MIN_NEAR_DIST;
  far_dist = target_dist +  1.0625*radius;

  SetCameraLocation( center + target_dist*CameraZ() );
  if ( !SetFrustumNearFar( near_dist, far_dist ) )
    return false;
  if ( !SetCameraAngle( angle ) )
    return false;

  return IsValid()?true:false;
}

void ON_Viewport::Dump( ON_TextLog& dump ) const
{
  dump.Print("ON_Viewport\n");
  dump.PushIndent();

  dump.Print("Projection: ");
  switch(m_projection)
  {
  case ON::parallel_view:
    dump.Print("parallel\n");
    break;
  case ON::perspective_view:
    dump.Print("perspective\n");
    break;
  default:
    dump.Print("invalid\n");
    break;
  }
  dump.Print("Camera: (m_bValidCamera = %s)\n",(m_bValidCamera?"true":"false"));
  dump.PushIndent();
  dump.Print("Location: "); if ( CameraLocationIsLocked() ) dump.Print("(locked) "); dump.Print(m_CamLoc); dump.Print("\n");
  dump.Print("Direction: "); if ( CameraDirectionIsLocked() ) dump.Print("(locked) "); dump.Print(m_CamDir); dump.Print("\n");
  dump.Print("Up: "); if ( CameraUpIsLocked() ) dump.Print("(locked) "); dump.Print(m_CamUp); dump.Print("\n");
  dump.Print("X: "); dump.Print(m_CamX); dump.Print("\n");
  dump.Print("Y: "); dump.Print(m_CamY); dump.Print("\n");
  dump.Print("Z: "); dump.Print(m_CamZ); dump.Print("\n");
  dump.PopIndent();
  dump.Print("Target Point: "); dump.Print(m_target_point); dump.Print("\n");
  dump.Print("target distance %g\n",TargetDistance(true));

  double frus_aspect=0.0;
  GetFrustumAspect(frus_aspect);
  dump.Print("Frustum: (m_bValidFrustum = %s)\n",(m_bValidFrustum?"true":"false"));
  dump.PushIndent();
  dump.Print("left/right symmetry locked = %s\n",FrustumIsLeftRightSymmetric()?"true":"false");
  dump.Print("top/bottom symmetry locked = %s\n",FrustumIsTopBottomSymmetric()?"true":"false");
  dump.Print("left: "); dump.Print(m_frus_left); dump.Print("\n");
  dump.Print("right: "); dump.Print(m_frus_right); dump.Print("\n");
  dump.Print("bottom: "); dump.Print(m_frus_bottom); dump.Print("\n");
  dump.Print("top: "); dump.Print(m_frus_top); dump.Print("\n");
  dump.Print("near: "); dump.Print(m_frus_near); dump.Print("\n");
  dump.Print("far: "); dump.Print(m_frus_far); dump.Print("\n");
  dump.Print("aspect (width/height): "); dump.Print(frus_aspect); dump.Print("\n");
  if ( ON::perspective_view == m_projection )
  {
    dump.PushIndent();
    dump.Print("near/far: %g\n",m_frus_near/m_frus_far);
    dump.Print("suggested minimum near: = %g\n",m__MIN_NEAR_DIST);
    dump.Print("suggested minimum near/far: = %g\n",m__MIN_NEAR_OVER_FAR);
    dump.PopIndent();
  }
  dump.PopIndent();

  double port_aspect=0.0;
  GetScreenPortAspect(port_aspect);
  dump.Print("Port: (m_bValidPort = %s\n",(m_bValidPort?"true":"false"));
  dump.PushIndent();
  dump.Print("left: %d\n",m_port_left);
  dump.Print("right: %d\n",m_port_right);
  dump.Print("bottom: %d\n",m_port_bottom);
  dump.Print("top: %d\n",m_port_top);
  dump.Print("near: %d\n",m_port_near);
  dump.Print("far: %d\n",m_port_far);
  dump.Print("aspect (width/height): "); dump.Print(port_aspect); dump.Print("\n");
  dump.PopIndent();

  dump.PopIndent();
}

bool ON_Viewport::GetPointDepth(       
       ON_3dPoint point,
       double* near_dist,
       double* far_dist,
       bool bGrowNearFar
       ) const
{
  bool rc = false;
  if ( point.x != ON_UNSET_VALUE )
  {
    double depth = (m_CamLoc - point)*m_CamZ;
    if ( 0 != near_dist && (*near_dist == ON_UNSET_VALUE || !bGrowNearFar || *near_dist > depth) )
      *near_dist = depth;
    if ( 0 != far_dist && (*far_dist == ON_UNSET_VALUE || !bGrowNearFar || *far_dist < depth) )
      *far_dist = depth;
    rc = true;
  }
  return rc;
}

bool ON_Viewport::GetPointDepth(       
       ON_3dPoint point,
       double* view_plane_depth
       ) const
{
  bool rc = false;
  if ( point.x != ON_UNSET_VALUE )
  {
    double depth = (m_CamLoc - point)*m_CamZ;
    if ( 0 != view_plane_depth )
      *view_plane_depth = depth;
    rc = true;
  }
  return rc;
}

bool ON_Viewport::GetBoundingBoxDepth(       
       ON_BoundingBox bbox,
       double* near_dist,
       double* far_dist,
       bool bGrowNearFar
       ) const
{
  // The Xbuffer[] stuff is to skip wasting time in unneeded constructors.
  // The buffers are double arrays to insure alignments are correct.
  ON_3dPoint* C;
  ON_3dPoint* P;
  ON_PlaneEquation* S;
  ON_Line* L;
  ON_3dPoint Q;
  double Pbuffer[(8+8+8+48)*(sizeof(P[0])/sizeof(double))];
  double Sbuffer[5*(sizeof(S[0])/sizeof(double))];
  double Lbuffer[4*(sizeof(L[0])/sizeof(double))];
  double d, t[2], v[4][8], v0, v1;
  const double tol = ON_SQRT_EPSILON*(1.0 + m_CamLoc.MaximumCoordinate());
  C = (ON_3dPoint*)Pbuffer;
  P = C+8;
  S = (ON_PlaneEquation*)Sbuffer;
  L = (ON_Line*)Lbuffer;
  unsigned int i, j, k, Pin, Pout, Pcount;
  bool rc;
  const bool bPerspectiveProjection = (ON::perspective_view == m_projection);

  for (;;)
  {
    rc = bbox.GetCorners(C);
    if (!rc)
      break;
    rc = GetFrustumLeftPlaneEquation(S[0]);
    if (!rc)
      break;
    rc = GetFrustumRightPlaneEquation(S[1]);
    if (!rc)
      break;
    rc = GetFrustumBottomPlaneEquation(S[2]);
    if (!rc)
      break;
    rc = GetFrustumTopPlaneEquation(S[3]);
    if (!rc)
      break;

    S[4].ON_3dVector::operator=(-m_CamZ);
    S[4].d = -S[4].ON_3dVector::operator*(m_CamLoc);
    
    Pcount = 0;
    Pin = 0;
    Pout = 0;

    for ( i = 0; i < 8; i++ )
    {
      k = 0;
      if ( (v[0][i] = S[0].ValueAt(C[i])) >= -tol )
        k |= 1;      
      else
        Pout |= 1;
      if ( (v[1][i] = S[1].ValueAt(C[i])) >= -tol )
        k |= 2;
      else
        Pout |= 2;
      if ( (v[2][i] = S[2].ValueAt(C[i])) >= -tol )
        k |= 4;
      else
        Pout |= 4;
      if ( (v[3][i] = S[3].ValueAt(C[i])) >= -tol )
        k |= 8;
      else
        Pout |= 8;

      if ( !bPerspectiveProjection || S[4].ValueAt(C[i]) > 0.0 )
        k |= 16;

      Pin |= k;
      if ( (1|2|4|8|16) == k )
      {
        // C[i] is inside the infinte frustum
        P[Pcount++] = C[i];
      }
    }

    if ( Pcount < 8 )
    {
      // some portion of bbox is outside the infinte frustum
      if ( (1|2|4|8|16) != Pin )
      {
        // bbox does not intersect the infinite frustum.
        rc = false;
        break;
      }

      j = 0;
      if ( bPerspectiveProjection )
      {
        if ( bbox.MinimumDistanceTo(m_CamLoc) <= 0.0 )
        {
          // camera location is in the bounding box
          P[Pcount++] = m_CamLoc;
          j = 1; // j = 1 indicates m_CamLoc has been added to P[].
        }
        L[0].from = m_CamLoc;
        L[1].from = m_CamLoc;
        L[2].from = m_CamLoc;
        L[3].from = m_CamLoc;
      }
      else
      {
        rc = GetNearRect(L[0].from,L[1].from,L[2].from,L[3].from);
        if (!rc)
          break;
      }

      rc = GetFarRect(L[0].to,L[1].to,L[2].to,L[3].to);
      if (!rc)
        break;

      const unsigned int Linout[4] = {
        1|4, // intersection of left and bottom frustum sides
        2|4, // intersection of right and bottom frustum sides
        1|8, // intersection of left and top frustum sides
        2|8  // intersection of right and top frustum sides
        };

      k = Pin & Pout;
      for ( i = 0; i < 4; i++ )
      {
        // The Linout[i] == ... test is true if bbox is on both sides
        // of both planes whose intersection defines the line L[i].
        // The fast integer test helps cull unnecessary calls to
        // the expensive ON_Intersect() function.
        if (    Linout[i] == (k & Linout[i]) 
             && ON_Intersect(bbox,L[i],tol,(ON_Interval*)t) 
           )
        {
          if ( bPerspectiveProjection )
          {
            if ( t[1] < 0.0 )
              continue;
            if ( t[0] < 0.0 )
            {
              if ( 0 == j )
              {
                P[Pcount++] = m_CamLoc;
                j = 1; // j = 1 indicates m_CamLoc has been added to P[].
              }
              t[0] = t[1];
            }
          }
          P[Pcount++] = L[i].PointAt(t[0]);
          if ( t[1] > t[0] )
            P[Pcount++] = L[i].PointAt(t[1]);
        }
      }

      // intersect box edges with frustum sides
      // The 12 bbox edges have endpoints
      // C[e[*][0]] and C[E[*][1]]
      const unsigned int e[12][2] = {
        {0,1},{2,3},{4,5},{6,7},
        {0,2},{1,3},{4,6},{5,7},
        {0,4},{1,5},{2,6},{3,7}};

      for ( i = 0; i < 4; i++ )
      {
        for ( j = 0; j < 12; j++ )
        {
          v0 = v[i][e[j][0]];
          v1 = v[i][e[j][1]];
          if ( v0*v1 < 0.0 )
          {
            // this box edge crosses the frustum side plane
            d = v0/(v0-v1);
            P[Pcount++] = Q = (1.0-d)*C[e[j][0]] + d*C[e[j][1]];
            // verify that Q is in the frustum
            for ( k = 0; k < 4; k++ )
            {
              if ( i != k && S[k].ValueAt(Q) <= -tol )
              {
                // Q is not in the view frustum
                Pcount--;
                break;
              }
            }
          }
        }
      }
      if ( 0 == Pcount )
      {
        rc = false;
        break;
      }
    }

    t[0] = t[1] = (m_CamLoc - P[0])*m_CamZ;
    for ( i = 1; i < Pcount; i++ )
    {
      d = (m_CamLoc - P[i])*m_CamZ;
      if ( d < t[0] )
        t[0] = d;
      else if ( d > t[1] )
        t[1] = d;
    }

    if ( bPerspectiveProjection )
    {
      if ( t[1] < 0.0 )
      {
        rc = false;
        break;
      }
      if ( t[0] < 0.0 )
        t[0] = 0.0;
    }

    if ( 0 != near_dist && (!bGrowNearFar || !ON_IsValid(*near_dist) || t[0] < *near_dist) )
      *near_dist = t[0];
    if ( 0 != far_dist && (!bGrowNearFar || !ON_IsValid(*far_dist) || t[1] > *far_dist) )
      *far_dist = t[1];

    rc = true;
    break;
  }

  return rc;
}

bool ON_Viewport::GetSphereDepth( 
       ON_Sphere sphere,
       double* near_dist,
       double* far_dist,
       bool bGrowNearFar
       ) const
{
  bool rc = GetPointDepth( sphere.Center(), near_dist, far_dist, bGrowNearFar );
  if ( rc && sphere.Radius() > 0.0 )
  {
    if ( 0 != near_dist )
      *near_dist -= sphere.Radius();
    if ( 0 != far_dist )
      *far_dist += sphere.Radius();
  }
  return rc;
}

bool ON_Viewport::SetFrustumNearFar( 
       double near_dist,
       double far_dist,
       double min_near_dist,
       double min_near_over_far,
       double target_dist
       )
{
  double relative_depth_bias = 0.0;
  return SetFrustumNearFar( 
            near_dist,
            far_dist,
            min_near_dist,
            min_near_over_far,
            target_dist,
            relative_depth_bias
            );
}

bool ON_Viewport::SetFrustumNearFar( 
       double near_dist,
       double far_dist,
       double min_near_dist,
       double min_near_over_far,
       double target_dist,
       double relative_depth_bias
       )
{
  if (    !ON_IsValid(near_dist)
       || !ON_IsValid(far_dist)
       || near_dist > far_dist )
  {
    return false;
  }

  // min_near_over_far needs to be < 1 and should be in the
  // range 1e-6 to 1e-2.  By setting negative min's to zero,
  // the code below is simplified but still ignores a negative
  // input.
  const double tiny = ON_ZERO_TOLERANCE;
  const double MIN_NEAR_DIST = ( ON_IsValid(m__MIN_NEAR_DIST) &&  m__MIN_NEAR_DIST <= tiny )
                            ? m__MIN_NEAR_DIST
                            : ON_Viewport::DefaultMinNearDist;
  const double MIN_NEAR_OVER_FAR = (    ON_IsValid(m__MIN_NEAR_OVER_FAR)
                                     && m__MIN_NEAR_OVER_FAR > tiny 
                                     && m__MIN_NEAR_OVER_FAR < 1.0-tiny )
                                 ? m__MIN_NEAR_OVER_FAR
                                 : ON_Viewport::DefaultMinNearOverFar;

  // 30 May Dale Lear
  //    Add checks for validity of min_near_dist and min_near_over_far
  if ( !ON_IsValid(min_near_dist) || min_near_dist <= tiny )
  {
    min_near_dist = MIN_NEAR_DIST;
  }

  if (    !ON_IsValid(min_near_over_far) 
       || min_near_over_far <= tiny 
       || min_near_over_far >= 1.0-tiny )
  {
    min_near_over_far = MIN_NEAR_OVER_FAR;
  }

  if ( IsPerspectiveProjection() )
  {
    // make sure 0 < near_dist < far_dist
    if ( near_dist < min_near_dist )
      near_dist = min_near_dist;

    if ( far_dist <= near_dist+tiny )
    {
      far_dist =  100.0*near_dist;
      if ( target_dist > near_dist+min_near_dist && far_dist <= target_dist+min_near_dist )
      {
        far_dist =  2.0*target_dist - near_dist;
      }
      if ( near_dist < min_near_over_far*far_dist ) 
        far_dist = near_dist/min_near_over_far;
    }
    // The 1.0001 fudge factor is to ensure successive calls to this function
    // give identical results.
    while ( near_dist < 1.0001*min_near_over_far*far_dist )
    {
      // need to move near and far closer together
      if ( ON_IsValid(target_dist) && near_dist < target_dist && target_dist < far_dist )
      {
        // STEP 1
        // If near and far are a long ways from the target
        // point, move them towards the target so the
        // fine tuning in step 2 makes sense.
        if ( target_dist/far_dist < min_near_over_far )
        {
          if ( near_dist/target_dist >= sqrt(min_near_over_far) )
          {
            // assume near_dist is good and just pull back far_dist
            far_dist = near_dist/min_near_over_far;
            break;
          }
          else
          {
            // move far_dist to within striking distance of the target
            // and let STEP 2 fine tune things.
            far_dist = target_dist/min_near_over_far;
          }
        }

        if ( near_dist/target_dist < min_near_over_far )
        {
          if ( target_dist/far_dist <= sqrt(min_near_over_far) 
               && far_dist <= 4.0*target_dist )
          {
            // assume far_dist is good and just move up near_dist
            near_dist = far_dist*min_near_over_far;
            break;
          }
          else
          {
            // move near_dist to within striking distance of the target
            // and let STEP 2 fine tune things.
            near_dist = target_dist*min_near_over_far;
          }
        }

        // STEP 2
        // Move near and far towards target by
        // an amount proportional to current
        // distances from the target.

        double b = (far_dist - target_dist)*min_near_over_far + (target_dist - near_dist);
        if ( b > 0.0)
        {
          double s = target_dist*(1.0 - min_near_over_far)/b;
          if ( s > 1.0 || s <= ON_ZERO_TOLERANCE || !ON_IsValid(s) )
          {
            if ( s > 1.00001 || s <= ON_ZERO_TOLERANCE )
            {
              // should never happen
              ON_ERROR("ON_Viewport::SetFrustumNearFar arithmetic problem 1.");
            }
            s = 1.0; 
          }

          // 19 Jan 2010, Mikko:
          // Reordered the operations to guarantee n==near_dist and f==far_dist
          // when s==1.0. The old system generated bogus problem reports when the dist
          // difference was big.
          double n = s*near_dist + target_dist*(1.0-s);
          double f = s*far_dist + target_dist*(1.0-s);
          //double n = target_dist + s*(near_dist-target_dist);
          //double f = target_dist + s*(far_dist-target_dist);

#if defined(ON_DEBUG)
          double m = ((f != 0.0) ? n/f : 0.0)/min_near_over_far;
          if ( m < 0.95 || m > 1.05 )
          {
            ON_ERROR("ON_Viewport::SetFrustumNearFar arithmetic problem 2.");
          }
#endif

          if ( n < near_dist || n >= target_dist)
          {
            ON_ERROR("ON_Viewport::SetFrustumNearFar arithmetic problem 3.");
            if ( target_dist < f && f < far_dist )
              n = min_near_over_far*f;
            else
              n = near_dist;
          }
          if ( f > far_dist || f <= target_dist )
          {
            ON_ERROR("ON_Viewport::SetFrustumNearFar arithmetic problem 4.");
            if ( near_dist < n && n < target_dist )
              f = n/min_near_over_far;
            else
              f = far_dist;
          }

          if ( n < min_near_over_far*f )
            n = min_near_over_far*f;
          else 
            f = n/min_near_over_far;

          near_dist = n;
          far_dist = f;
        }
        else
        {
          near_dist = min_near_over_far*far_dist;
        }
      }
      else if ( ON_IsValid(target_dist) && fabs(far_dist-target_dist) > fabs(near_dist-target_dist) )
      {
        far_dist = near_dist/min_near_over_far;
      }
      else
      {
        near_dist = min_near_over_far*far_dist;
      }
      break;
    }
  }
  else
  {
    // parallel projection
    if ( far_dist <= near_dist+tiny)
    {
      double d = fabs(near_dist)*0.125;
      if ( d <= MIN_NEAR_DIST || d < tiny || d < min_near_dist )
        d = 1.0;
      near_dist -= d;
      far_dist += d;
    }

    if ( near_dist < min_near_dist || near_dist < MIN_NEAR_DIST )
    {
      if ( !m_bValidCamera )
        return false;
      // move camera back in parallel projection so everything shows
      double h = fabs(m_frus_top - m_frus_bottom);
      double w = fabs(m_frus_right - m_frus_left);
      double r = 0.5*((h > w) ? h : w);
      double n = 3.0*r;
      if (n < 2.0*min_near_dist )
        n = 2.0*min_near_dist;
      if ( n < 2.0*MIN_NEAR_DIST )
        n = 2.0*MIN_NEAR_DIST;
      double d = n-near_dist;
      ON_3dPoint new_loc = CameraLocation() + d*CameraZ();
      SetCameraLocation(new_loc);
      if ( m_bValidFrustum && fabs(m_frus_near) >= d*ON_SQRT_EPSILON )
      {
        m_frus_near += d;
        m_frus_far += d;
      }
      near_dist = n;
      far_dist += d;
      target_dist += d;
      if ( far_dist < near_dist )
      {
        // could happen if d is < ON_EPSILON*far_dist
        far_dist = 1.125*near_dist;
      }
    }
  }

  // call bare bones setter
  bool rc = SetFrustumNearFar( near_dist, far_dist );

  // if depth bias will be applied, then make an attempt
  // to adust the frustum's near plane to prevent
  // clipping biased objects.  This post-adjustment
  // fixes display bugs like # 87514.
  if ( rc 
       && relative_depth_bias > 0.0 && relative_depth_bias <= 0.5 
       && m_frus_near > min_near_dist
       && m_frus_far > m_frus_near
       && m_frus_near > MIN_NEAR_DIST
       )
  {
    const double near0 = m_frus_near;
    const double far0 = m_frus_far;
    double bias_3d = 1.001*relative_depth_bias*(m_frus_far - m_frus_near);
    double near1 = m_frus_near - bias_3d;
    if ( IsPerspectiveProjection() )
    {
      if ( near1 < min_near_over_far*far0 || near1 < MIN_NEAR_OVER_FAR*far0 )
      {
        if (near0 - near1 > 0.01*near0)
          near1 = 0.99*near0;
      }
    }

    // It is important that this test be applied in perspective
    // and parallel views.  Otherwise the camera location in
    // parallel view will creep back when SetFrustumNearFar()
    // is called multiple times.
    if ( !(near1 >= min_near_dist && near1 >= MIN_NEAR_DIST) )
    {
      near1 = (min_near_dist >= MIN_NEAR_DIST)
            ? min_near_dist
            : MIN_NEAR_DIST;
    }

    if ( near1 < near0 )
    {
#if defined(ON_DEBUG)
      const ON_3dPoint debug_camloc0(m_CamLoc);
#endif
      if ( IsPerspectiveProjection() )
      {
        rc = SetFrustumNearFar( near1, far0 );
        if (!rc)
          rc = SetFrustumNearFar( near0, far0 );
      }
      else
      {
        // call this function again with relative_depth_bias = 0.0
        // to get cameral location positioned correctly when near1 
        // is too small or negative.
        rc = SetFrustumNearFar( 
          near1, far0,
          min_near_dist, min_near_over_far,
          target_dist,
          0.0
          );
        if (!rc)
          rc = SetFrustumNearFar( 
            near0, far0,
            min_near_dist, min_near_over_far,
            target_dist,
            0.0
            );
      }
#if defined(ON_DEBUG)
      if ( debug_camloc0 != m_CamLoc )
      {
        ON_WARNING("Relative depth bias changed camera location.");
      }
#endif
    }
  }

  return rc;
}


bool ON_Viewport::GetFrustumLeftPlane( 
  ON_Plane& left_plane 
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {
    if ( IsPerspectiveProjection() )
    {
      ON_2dVector v(m_frus_near,m_frus_left);
      rc = v.Unitize();
      left_plane.origin = m_CamLoc;
      left_plane.xaxis =  v.y*m_CamX - v.x*m_CamZ;
      left_plane.yaxis =  m_CamY;
      left_plane.zaxis =  v.x*m_CamX + v.y*m_CamZ;
    }
    else
    {
      left_plane.origin = m_CamLoc + m_frus_left*m_CamX;
      left_plane.xaxis = -m_CamZ;
      left_plane.yaxis =  m_CamY;
      left_plane.zaxis =  m_CamX;
    }
    left_plane.UpdateEquation();
  }
  return rc;
}

bool ON_Viewport::GetFrustumLeftPlaneEquation( 
  ON_PlaneEquation& left_plane_equation
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {

    if ( IsPerspectiveProjection() )
    {
      ON_2dVector v(m_frus_near,m_frus_left);
      if ( 0 != (rc = v.Unitize()) )
      {
        left_plane_equation.ON_3dVector::operator=(v.x*m_CamX + v.y*m_CamZ);
        left_plane_equation.d = -left_plane_equation.ON_3dVector::operator*(m_CamLoc);
      }
    }
    else
    {
      left_plane_equation.ON_3dVector::operator=(m_CamX);
      left_plane_equation.d = -left_plane_equation.ON_3dVector::operator*(m_CamLoc + m_frus_left*m_CamX);
    }
  }
  return rc;
}


bool ON_Viewport::GetFrustumRightPlane( 
  ON_Plane& right_plane 
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {
    if ( IsPerspectiveProjection() )
    {
      ON_2dVector v(m_frus_near,-m_frus_right);
      rc = v.Unitize();
      right_plane.origin = m_CamLoc;
      right_plane.xaxis =  v.y*m_CamX + v.x*m_CamZ;
      right_plane.yaxis =  m_CamY;
      right_plane.zaxis = -v.x*m_CamX + v.y*m_CamZ;
    }
    else
    {
      right_plane.origin = m_CamLoc + m_frus_right*m_CamX;
      right_plane.xaxis =  m_CamZ;
      right_plane.yaxis =  m_CamY;
      right_plane.zaxis = -m_CamX;
    }
    right_plane.UpdateEquation();
  }
  return rc;
}

bool ON_Viewport::GetFrustumRightPlaneEquation( 
  ON_PlaneEquation& right_plane_equation
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {

    if ( IsPerspectiveProjection() )
    {
      ON_2dVector v(m_frus_near,-m_frus_right);
      if ( 0 != (rc = v.Unitize()) )
      {
        right_plane_equation.ON_3dVector::operator=(-v.x*m_CamX + v.y*m_CamZ);
        right_plane_equation.d = -right_plane_equation.ON_3dVector::operator*(m_CamLoc);
      }
    }
    else
    {
      right_plane_equation.ON_3dVector::operator=(-m_CamX);
      right_plane_equation.d = -right_plane_equation.ON_3dVector::operator*(m_CamLoc + m_frus_right*m_CamX);
    }
  }
  return rc;
}


bool ON_Viewport::GetFrustumBottomPlane( 
  ON_Plane& bottom_plane 
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {
    if ( IsPerspectiveProjection() )
    {
      ON_2dVector v(m_frus_near,m_frus_bottom);
      rc = v.Unitize();
      bottom_plane.origin = m_CamLoc;
      bottom_plane.xaxis = -v.y*m_CamY + v.x*m_CamZ;
      bottom_plane.yaxis =  m_CamX;
      bottom_plane.zaxis =  v.x*m_CamY + v.y*m_CamZ;
    }
    else
    {
      bottom_plane.origin = m_CamLoc + m_frus_bottom*m_CamY;
      bottom_plane.xaxis =  m_CamZ;
      bottom_plane.yaxis =  m_CamX;
      bottom_plane.zaxis =  m_CamY;
    }
    bottom_plane.UpdateEquation();
  }
  return rc;
}


bool ON_Viewport::GetFrustumBottomPlaneEquation( 
  ON_PlaneEquation& bottom_plane_equation
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {

    if ( IsPerspectiveProjection() )
    {
      ON_2dVector v(m_frus_near,m_frus_bottom);
      if ( 0 != (rc = v.Unitize()) )
      {
        bottom_plane_equation.ON_3dVector::operator=(v.x*m_CamY + v.y*m_CamZ);
        bottom_plane_equation.d = -bottom_plane_equation.ON_3dVector::operator*(m_CamLoc);
      }
    }
    else
    {
      bottom_plane_equation.ON_3dVector::operator=(m_CamY);
      bottom_plane_equation.d = -bottom_plane_equation.ON_3dVector::operator*(m_CamLoc + m_frus_bottom*m_CamY);
    }
  }
  return rc;
}


bool ON_Viewport::GetFrustumTopPlane( 
  ON_Plane& top_plane 
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {
    if ( IsPerspectiveProjection() )
    {
      ON_2dVector v(m_frus_near,-m_frus_top);
      rc = v.Unitize();
      top_plane.origin = m_CamLoc;
      top_plane.xaxis = -v.y*m_CamY - v.x*m_CamZ;
      top_plane.yaxis =  m_CamX;
      top_plane.zaxis = -v.x*m_CamY + v.y*m_CamZ;
    }
    else
    {
      top_plane.origin = m_CamLoc + m_frus_top*m_CamY;
      top_plane.xaxis = -m_CamZ;
      top_plane.yaxis =  m_CamX;
      top_plane.zaxis = -m_CamY;
    }
    top_plane.UpdateEquation();
  }
  return rc;
}

bool ON_Viewport::GetFrustumTopPlaneEquation( 
  ON_PlaneEquation& top_plane_equation
  ) const
{
  bool rc = m_bValidCamera && m_bValidFrustum;
  if (rc)
  {

    if ( IsPerspectiveProjection() )
    {
      ON_2dVector v(m_frus_near,-m_frus_top);
      if ( 0 != (rc = v.Unitize()) )
      {
        top_plane_equation.ON_3dVector::operator=(-v.x*m_CamY + v.y*m_CamZ);
        top_plane_equation.d = -top_plane_equation.ON_3dVector::operator*(m_CamLoc);
      }
    }
    else
    {
      top_plane_equation.ON_3dVector::operator=(-m_CamY);
      top_plane_equation.d = -top_plane_equation.ON_3dVector::operator*(m_CamLoc + m_frus_top*m_CamY);
    }
  }
  return rc;
}

void ON_Viewport::GetViewScale( double* x, double* y ) const
{
  if ( x ) *x = 1.0;
  if ( y ) *y = 1.0;
  if ( !m_clip_mods.IsIdentity()
       && 0.0 == m_clip_mods.m_xform[3][0]
       && 0.0 == m_clip_mods.m_xform[3][1]
       && 0.0 == m_clip_mods.m_xform[3][2]
       && 1.0 == m_clip_mods.m_xform[3][3]
     )
  {
    // 04 November 2011 S. Baer (RR93636)
    //   See comments in SetViewScale about why we are ignoring the test for 1
    //   on either sx or sy
    double sx = m_clip_mods.m_xform[0][0];
    double sy = m_clip_mods.m_xform[1][1];
    if (    sx > ON_ZERO_TOLERANCE
         && sy > ON_ZERO_TOLERANCE
         && 0.0 == m_clip_mods.m_xform[0][1]
         && 0.0 == m_clip_mods.m_xform[0][2]
         && 0.0 == m_clip_mods.m_xform[1][0]
         && 0.0 == m_clip_mods.m_xform[1][2]
         // && (1.0 == sx || 1.0 == sy )
        )
    {
      if ( x ) *x = sx;
      if ( y ) *y = sy;
    }
  }
}

//bool ON_Viewport::ScaleView( double x, double y, double z )
//{
//  // z ignored on purpose - it was a mistake to include z
//  return (!IsPerspectiveProjection()) ? SetViewScale(x,y) : false;
//}

bool ON_Viewport::SetViewScale( double x, double y )
{
  // 22 May Dale Lear
  //   View scaling should have been done by adjusting the 
  //   frustum left/right top/bottom but I was stupid and added a clipmodxform 
  //   that is more trouble than it is worth.
  //   Someday I will fix this.  In the mean time, I want all scaling requests
  //   to flow through SetViewScale/GetViewScale so I can easly find and fix
  //   things when I have time to do it right.
  // 04 November 2011 S. Baer (RR93636)
  //   This function is used for printer calibration and it is commonly possible
  //   to need to apply a scale in both x and y.  The reason for the need of x
  //   or y to be one is because the view scale is encoded in the clip mod xform
  //   and it is hard to be sure that we could accurately extract these values
  //   when calling GetViewScale.  Removing the requirement to have one of the
  //   values == 1
  bool rc = false;
  if (    !IsPerspectiveProjection() 
       && x > ON_ZERO_TOLERANCE && ON_IsValid(x) 
       && y > ON_ZERO_TOLERANCE && ON_IsValid(y) 
       // && (1.0 == x || 1.0 == y)
       )
  {
    ON_Xform xform(1.0);
    xform.m_xform[0][0] = x;
    xform.m_xform[1][1] = y;
    rc = SetClipModXform(xform);
  }
  return rc;
}

double ON_Viewport::ClipCoordDepthBias( double relative_depth_bias, double clip_z, double clip_w ) const
{
  double d;
  if ( m_frus_far > m_frus_near 
       && 0.0 != relative_depth_bias 
       && 0.0 != clip_w
     )
  {
    if ( ON::perspective_view == m_projection )
    {
      // To get the formula for the code in this claus:
      //
      // Set M = [Camera2Clip]*[translation by (0,0,relative_depth_bias*(f-n)]*[Clip2Camera]
      // Note that M maps clipping coordinates to clipping coordinates.
      //
      // Calculate M([x,y,z,w]) = [p,q,r,s]
      //
      // This function returns (r/s - z/w)*w
      //
      // If you are actually doing this calculation and trying to 
      // get the formula used in the code below, it helps to notice
      // that (f+n)/(f-n) = a/b.
      //
      // Note that there "should" be a small adjustment to the
      // x and y coordinates that is not performed by tweaking
      // the z clipping coordiante
      //    z += vp->ClipCoordDepthBias( rel_bias, z, w );
      // but the effect is actually better when the goal is to
      // make wires that are on shaded surfaces appear because
      // their horizons are not altered.
      // 
      // This method is more complicated that adding a constant
      // depth buffer bias but is required for high quality images
      // when values of far/near get to be around 1e4 or larger.
      //
      double a = m_frus_far + m_frus_near;
      double b = m_frus_far - m_frus_near;
      double c = 0.5*relative_depth_bias/(m_frus_far*m_frus_near);
      double t = a + b*clip_z/clip_w;
      d = c*t*t*clip_w/(1.0 - c*b*t);
    }
    else
    {
      // The "2.0*" is here because clipping coordinates run from
      // -1 to +1, a distance of 2 units.
      d = 2.0*relative_depth_bias*clip_w;
    }
  }
  else
  {
    d = 0.0;
  }
  return d;
}

bool ON_Viewport::GetClipCoordDepthBiasXform( 
    double relative_depth_bias,
    ON_Xform& clipbias 
    ) const
{
  bool rc = false;

  while ( 0.0 != relative_depth_bias
       && m_frus_far > m_frus_near
       )
  {
    if ( ON::perspective_view == m_projection )
    {
      ON_Xform clip2cam, cam_delta(1.0), cam2clip;
      if ( !cam2clip.CameraToClip(true,m_frus_left,m_frus_right,m_frus_bottom,m_frus_top,m_frus_near,m_frus_far) )
        break;
      if ( !clip2cam.ClipToCamera(true,m_frus_left,m_frus_right,m_frus_bottom,m_frus_top,m_frus_near,m_frus_far) )
        break;
      cam_delta.m_xform[2][3] = relative_depth_bias*(m_frus_far-m_frus_near);
      clipbias = cam2clip*cam_delta*clip2cam;
    }
    else
    {
      clipbias.Identity();
      clipbias.m_xform[2][3] = 2.0*relative_depth_bias;
    }
    rc = true;
    break;
  }

  if (!rc)
    clipbias.Identity();

  return rc;
}

bool ON_Viewport::SetClipModXform( ON_Xform clip_mod_xform )
{
  bool rc = false;
  ON_Xform clip_mod_inverse_xform = clip_mod_xform;
  rc = clip_mod_inverse_xform.Invert();
  if ( rc )
  {
    ON_Xform id = clip_mod_inverse_xform*clip_mod_xform;
    double e;
    int i, j;
    for ( i = 0; i < 4 && rc; i++ ) for ( j = 0; j < 4 && rc; j++ )
    {
      e = ( i == j ) ? 1.0 : 0.0;
      if ( fabs(id.m_xform[i][j] - e) > ON_SQRT_EPSILON )
      {
        rc = false;
      }
    }
    if (rc)
    {
      m_clip_mods = clip_mod_xform;
      m_clip_mods_inverse = clip_mod_inverse_xform;
    }
  }
  return rc;
}

bool ON_Viewport::ClipModXformIsIdentity() const
{
  return m_clip_mods.IsIdentity();
}

ON_Xform ON_Viewport::ClipModXform() const
{
  return m_clip_mods;
}

ON_Xform ON_Viewport::ClipModInverseXform() const
{
  return m_clip_mods_inverse;
}

bool ON_Viewport::SetTargetPoint( ON_3dPoint target_point )
{
  bool rc = (target_point.IsValid() || (ON_UNSET_POINT == target_point));
  if (rc)
    m_target_point = target_point;
  return rc;
}

ON_3dPoint ON_Viewport::FrustumCenterPoint( double target_distance ) const
{
  double s,dx,dy,dz;
  ON_3dPoint target_point = ON_3dPoint::UnsetPoint;

  if (!m_bValidCamera || !m_bValidFrustum)
    return target_point;

  if ( ON_UNSET_VALUE == target_distance && m_bValidFrustum 
       && m_frus_near > 0.0 && m_frus_far >= m_frus_near
     )
  {
    target_distance = 0.5*(m_frus_near+m_frus_far);
    if ( target_distance < m_frus_near )
      target_distance = m_frus_near;
    else if ( target_distance > m_frus_far )
      target_distance = m_frus_far;
  }

  if ( !ON_IsValid(target_distance) || target_distance <= 0.0 )
    return target_point;

  if ( m_bValidFrustum )
  {
    s  = (ON::perspective_view == m_projection && m_frus_near > 0.0)
       ? 0.5*target_distance/m_frus_near
       : 0.5;
    dx = FrustumIsLeftRightSymmetric() 
       ?  0.0
       : s*(m_frus_right+m_frus_left);
    dy = FrustumIsTopBottomSymmetric() 
       ?  0.0
       : s*(m_frus_top+m_frus_bottom);
  }
  else
  {
    dx = dy = 0.0;
  }
  dz = -target_distance;

  // Done this way instead of using ON_3dPoint/ON_3dVector arithmetic so the
  // optimizer can generate maximum precision when using 64 bit mantissas.
  target_point.x = (m_CamLoc.x + dx*m_CamX.x + dy*m_CamY.x + dz*m_CamZ.x);
  target_point.y = (m_CamLoc.y + dx*m_CamX.y + dy*m_CamY.y + dz*m_CamZ.y);
  target_point.z = (m_CamLoc.z + dx*m_CamX.z + dy*m_CamY.z + dz*m_CamZ.z);

  return target_point;
}

ON_3dPoint ON_Viewport::TargetPoint() const
{
  return m_target_point;
}


double ON_Viewport::TargetDistance( bool bUseFrustumCenterFallback ) const
{
  double d = ON_UNSET_VALUE;
  if ( m_bValidCamera )
  {
    if ( bUseFrustumCenterFallback && !m_bValidFrustum )
      bUseFrustumCenterFallback = false;
    if ( m_target_point.IsValid() )
    {
      d = (m_CamLoc - m_target_point)*m_CamZ;
      if ( bUseFrustumCenterFallback && (!ON_IsValid(d) || d <= 0.0) )
        d = ON_UNSET_VALUE;
    }
    if ( bUseFrustumCenterFallback 
        && ON_UNSET_VALUE == d 
        && m_frus_far >= m_frus_near
        )
    {
      d = 0.5*(m_frus_near+m_frus_far);
      if ( d < m_frus_near ) d = m_frus_near; else if (d > m_frus_far) d = m_frus_far;
      if ( d <= 0.0 )
        d = ON_UNSET_VALUE;
    }
  }
  return d;
}

bool ON_Viewport::SetViewportId( const ON_UUID& id)
{
  // Please discuss any code changes with Dale Lear.
  // You should NEVER change the viewport id once
  // it is set.
  bool rc = (0 == memcmp(&m_viewport_id,&id,sizeof(m_viewport_id)));
  if ( !rc && m_viewport_id == ON_nil_uuid )
  {  
    m_viewport_id = id;
    rc = true;
  }
  return rc;
}

void ON_Viewport::ChangeViewportId(const ON_UUID& viewport_id)
{
  m_viewport_id = viewport_id; // <- good place for a breakpoint
}


ON_UUID  ON_Viewport::ViewportId(void) const
{
  return m_viewport_id;
}


class ON_PgonPt
{
public:
  ON_3dPoint m_P;
  ON_2dVector m_Q;
  double m_negcotangle;
};

static 
int comparePptAngle( const void* pa, const void* pb )
{
  double a = ((const ON_PgonPt*)pa)->m_negcotangle;
  double b = ((const ON_PgonPt*)pb)->m_negcotangle;
  if ( a == b )
  {
    a = ((const ON_PgonPt*)pa)->m_Q.LengthSquared();
    b = ((const ON_PgonPt*)pb)->m_Q.LengthSquared();
  }
  return ((a>b) ? 1 : ((a==b) ? 0 : -1));
}

bool ON_IntersectViewFrustumPlane( 
    const ON_Viewport& vp,
    const ON_PlaneEquation& plane_equation, 
    ON_SimpleArray<ON_3dPoint>& points 
    )
{

  double left, right, bottom, top, near_dist, far_dist;
  double v[8], v0, v1, s;
  ON_PgonPt ppt, ppt_list[24];
  ON_3dPoint F[8], P0, P1, P;
  ON_2dVector D;
  const ON_3dPoint  C = vp.CameraLocation();
  const ON_3dVector X = vp.CameraX();
  const ON_3dVector Y = vp.CameraY();
  const ON_3dVector Z = -vp.CameraZ();
  int e[12][2] = {{0,1},{1,2},{2,3},{3,0},
                  {4,5},{5,6},{6,7},{7,4},
                  {0,4},{1,5},{2,6},{3,7}};
  int i, i0, i1;
  int ppt_count = 0;

  if ( !vp.IsValidCamera() || !vp.GetFrustum(&left,&right,&bottom,&top,&near_dist,&far_dist) )
    return false;

  const ON_Plane plane(plane_equation);
  if ( !plane.IsValid() )
    return false;

  s = ON::perspective_view == vp.Projection() 
    ? far_dist/near_dist
    : 1.0;
  F[0] = C + left*X  + bottom*Y + near_dist*Z;
  F[1] = C + right*X + bottom*Y + near_dist*Z;
  F[2] = C + right*X + top*Y    + near_dist*Z;
  F[3] = C + left*X  + top*Y    + near_dist*Z;

  F[4] = C + s*left*X  + s*bottom*Y + far_dist*Z;
  F[5] = C + s*right*X + s*bottom*Y + far_dist*Z;
  F[6] = C + s*right*X + s*top*Y    + far_dist*Z;
  F[7] = C + s*left*X  + s*top*Y    + far_dist*Z;

  for ( i = 0; i < 8; i++ )
  {
    v[i] = plane_equation.ValueAt(F[i]);
  }

  for ( i = 0; i < 12; i++ )
  {
    v0 = v[e[i][0]];
    v1 = v[e[i][1]];
    P0 = F[e[i][0]];
    P1 = F[e[i][1]];
    if ( (v0 <= 0.0 && v1 >= 0.0) || (v0 >= 0.0 && v1 <= 0.0) )
    {
      if ( v0 == v1 )
      {
        ppt_list[ppt_count++].m_P = P0;
        ppt_list[ppt_count++].m_P = P1;
      }
      else
      {
        s = v1/(v1-v0);
        P = s*P0 + (1.0-s)*P1;
        ppt_list[ppt_count++].m_P = P;
      }
    }
  }

  if ( ppt_count <= 0 )
    return true; // plane misses frustum

  i0 = 0;
  for ( i = 0; i < ppt_count; i++ )
  {
    plane.ClosestPointTo( ppt_list[i].m_P, &ppt_list[i].m_Q.x, &ppt_list[i].m_Q.y );
    if (     ppt_list[i].m_Q.y < ppt_list[i0].m_Q.y 
         || (ppt_list[i].m_Q.y == ppt_list[i0].m_Q.y && ppt_list[i].m_Q.x < ppt_list[i0].m_Q.x) )
      i0 = i;
  }

  // Use Gram scan to get the convex hull and save it in points[].
  // See http://en.wikipedia.org/wiki/Graham_scan for details.

  // put point with smallest m_Q.y coordinate in ppt_list[0].
  ppt = ppt_list[i0];
  if ( i0 )
  {
    ppt_list[i0] = ppt_list[0];
    ppt_list[0] = ppt;
    i0 = 0;
  }

  // sort points by the angle (ppt_list[i].m_Q = ppt_list[0].m_Q) makes
  // with the positve x axis.  This is the same as sorting them by
  // -cot(angle) = -deltax/deltay.
  ppt_list[0].m_negcotangle = -ON_DBL_MAX; // -cot(0) = - infinity
  for ( i = 1; i < ppt_count; i++ )
  {
    ppt_list[i].m_Q.x -= ppt_list[0].m_Q.x;
    ppt_list[i].m_Q.y -= ppt_list[0].m_Q.y;
    ppt_list[i].m_negcotangle = (0.0 >= ppt_list[i].m_Q.y) ? -ON_DBL_MAX : -ppt_list[i].m_Q.x/ppt_list[i].m_Q.y;
  }
  ppt_list[0].m_Q.x = 0.0;
  ppt_list[0].m_Q.y = 0.0;
  ON_qsort(ppt_list+1,ppt_count-1,sizeof(ppt_list[0]),comparePptAngle);

  points.Append(ppt_list[0].m_P);
  i0 = 0;
  i1 = 1;
  D = ppt_list[i1].m_Q - ppt_list[i0].m_Q;

  for ( i = 2; i < ppt_count; i++ )
  {
    if ( (ppt_list[i].m_Q.y - ppt_list[i0].m_Q.y)*D.x <= (ppt_list[i].m_Q.x - ppt_list[i0].m_Q.x)*D.y )
    {
      // ppt_list[i0], ppt_list[i1], ppt_list[i] is a "right" turn or colinear.
      // Drop ppt_list[i1].
      i1 = i;
    }
    else
    {
      // ppt_list[i0], ppt_list[i1], ppt_list[i] is a "left" turn.
      points.Append(ppt_list[i1].m_P);
      i0 = i1;
      i1 = i;
    }
    D = ppt_list[i1].m_Q - ppt_list[i0].m_Q;
  }

  if ( i1 > i0 )
    points.Append(ppt_list[i1].m_P);

  return true;
}

void ON_Viewport::GetPerspectiveClippingPlaneConstraints( 
  ON_3dPoint camera_location,
  unsigned int depth_buffer_bit_depth,
  double* min_near_dist,
  double* min_near_over_far
  )
{
  double nof, n, d;

  if ( camera_location.IsValid() )
  {
    /*

    // This code was used prior to 14 July 2011.
    //
    d = camera_location.DistanceTo(ON_3dPoint::Origin);
    if ( d >= 1.0e5 )
    {
      if ( depth_buffer_bit_depth >= 32 )
        depth_buffer_bit_depth -= 24;
      else
        depth_buffer_bit_depth = 8;
    }
    else if ( d >= 1.0e4 )
    {
      if ( depth_buffer_bit_depth >= 24 )
        depth_buffer_bit_depth -= 16;
      else
        depth_buffer_bit_depth = 8;
    }
    else if ( d >= 1.0e3 )
    {
      if ( depth_buffer_bit_depth >= 16 )
        depth_buffer_bit_depth -= 8;
      else
        depth_buffer_bit_depth = 8;
    }
    */

    // 14 July 2011 - Dale Lear
    //   The reductions above were too harsh and were
    //   generating clipping artifacts in the perspective
    //   view in bug report 88216.  Changing to
    //   to the code below gets rid of those
    //   artifacts at the risk of having a meaninless
    //   view to clip transform if the transformation is
    //   calculated with single precision numbers.
    //   If these values require further tuning, please
    //   discuss changes with me and attach example files
    //   to bug report 88216.
    d = camera_location.MaximumCoordinate();
    if ( d > 1.0e6 && depth_buffer_bit_depth >= 16 )
      depth_buffer_bit_depth -= 8;
  }

  if ( depth_buffer_bit_depth >= 32 )
  {
    nof = 0.0001;
    n = 0.001;
  }
  else if ( depth_buffer_bit_depth >= 24 )
  {
    nof = 0.0005;
    n = 0.005;
  }
  else if ( depth_buffer_bit_depth >= 16 )
  {
    nof = 0.005;
    n = 0.005;
  }
  else
  {
    nof = 0.01;
    n = 0.01;
  }

  if ( min_near_dist )
    *min_near_dist = n;
  if ( min_near_over_far )
    *min_near_over_far = nof;
}
